import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import TransformStamped
import tf2_ros

import cv2
import numpy as np
from cv_bridge import CvBridge

ARUCO_DICTS = {
    50:   cv2.aruco.DICT_7X7_50,
    100:  cv2.aruco.DICT_7X7_100,
    250:  cv2.aruco.DICT_7X7_250,
    1000: cv2.aruco.DICT_7X7_1000,
}

ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()


def rvec_to_quaternion(rvec):
    R, _ = cv2.Rodrigues(rvec)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class BasicArucoCvTransformation(Node):

    def __init__(self):
        super().__init__('basic_aruco_cv_transformation')
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        self.declare_parameter('dictionary', 250)
        self.declare_parameter('marker_size', 0.1)  # metres

        dict_size = self.get_parameter('dictionary').get_parameter_value().integer_value
        if dict_size not in ARUCO_DICTS:
            self.get_logger().warn(f'Unknown dictionary {dict_size}, falling back to 250')
            dict_size = 250
        self.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICTS[dict_size])
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.get_logger().info(f'Using DICT_7X7_{dict_size}, marker size {self.marker_size} m')

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self._camera_info_callback,
            reliable_qos,
        )
        self.create_subscription(
            CompressedImage,
            '/camera/camera/color/image_raw/compressed',
            self._image_callback,
            best_effort_qos,
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.publisher = self.create_publisher(
            CompressedImage,
            '/camera/camera/color/aruco/compressed',
            best_effort_qos,
        )

    def _camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_frame = msg.header.frame_id
            self.get_logger().info(f'Camera info received, frame: {self.camera_frame}')

    def _image_callback(self, msg: CompressedImage):
        if self.camera_matrix is None:
            self.get_logger().warn('Waiting for camera_info...', throttle_duration_sec=2.0)
            return

        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=ARUCO_PARAMS)

        if ids is None:
            out_msg = self.bridge.cv2_to_compressed_imgmsg(img)
            out_msg.header = msg.header
            self.publisher.publish(out_msg)
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )

        cv2.aruco.drawDetectedMarkers(img, corners, ids)

        stamp = msg.header.stamp
        axis_length = self.marker_size * 0.5

        for i, marker_id in enumerate(ids.flatten()):
            cv2.aruco.drawAxis(img, self.camera_matrix, self.dist_coeffs,
                               rvecs[i], tvecs[i], axis_length)

            x, y, z, w = rvec_to_quaternion(rvecs[i])

            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.camera_frame
            t.child_frame_id = f'aruco_{marker_id}'
            t.transform.translation.x = float(tvecs[i][0][0])
            t.transform.translation.y = float(tvecs[i][0][1])
            t.transform.translation.z = float(tvecs[i][0][2])
            t.transform.rotation.x = x
            t.transform.rotation.y = y
            t.transform.rotation.z = z
            t.transform.rotation.w = w

            self.tf_broadcaster.sendTransform(t)

        out_msg = self.bridge.cv2_to_compressed_imgmsg(img)
        out_msg.header = msg.header
        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BasicArucoCvTransformation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
