import math

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

MARKER_IDS = [5, 10, 15]
MIN_READINGS = 10
MAX_READINGS = 15
PROXIMITY_THRESHOLD = 0.7  # metres
ROBOT_FRAME = 'base_footprint'
MAP_FRAME = 'map'

MARKER_COLORS = {
    5:  (1.0, 0.0, 0.0),
    10: (0.0, 1.0, 0.0),
    15: (0.0, 0.0, 1.0),
}


class ArucoMapMarker(Node):

    def __init__(self):
        super().__init__('aruco_map_marker')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, '/aruco_map_markers', 10)

        self.states = {mid: 'waiting' for mid in MARKER_IDS}
        self.readings = {mid: [] for mid in MARKER_IDS}
        self.confirmed_poses = {}

        self.create_timer(0.1, self._update)

    def _lookup(self, target, source):
        try:
            return self.tf_buffer.lookup_transform(
                target, source,
                rclpy.time.Time(),
                Duration(seconds=0.1),
            )
        except tf2_ros.LookupException as e:
            self.get_logger().warn(
                f'TF lookup failed {source}->{target}: {e}',
                throttle_duration_sec=5.0,
            )
            return None
        except Exception as e:
            self.get_logger().debug(f'TF error {source}->{target}: {e}')
            return None

    def _dist(self, t1, t2):
        # 2D distance in xy — base_footprint z=0 but marker z varies with camera height
        dx = t1.transform.translation.x - t2.transform.translation.x
        dy = t1.transform.translation.y - t2.transform.translation.y
        return math.sqrt(dx * dx + dy * dy)

    def _update(self):
        robot_tf = self._lookup(MAP_FRAME, ROBOT_FRAME)
        if robot_tf is None:
            return

        for mid in MARKER_IDS:
            if self.states[mid] == 'confirmed':
                continue

            marker_tf = self._lookup(MAP_FRAME, f'aruco_{mid}')
            if marker_tf is None:
                continue

            dist = self._dist(robot_tf, marker_tf)

            if dist < PROXIMITY_THRESHOLD:
                if self.states[mid] == 'waiting':
                    self.states[mid] = 'collecting'
                    self.readings[mid] = []
                    self.get_logger().info(f'aruco_{mid}: collecting (dist={dist:.3f} m)')

                tr = marker_tf.transform
                self.readings[mid].append((
                    tr.translation.x, tr.translation.y, tr.translation.z,
                    tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w,
                ))

                n = len(self.readings[mid])
                self.get_logger().info(
                    f'aruco_{mid}: reading {n}/{MAX_READINGS}',
                    throttle_duration_sec=0.5,
                )

                if n >= MAX_READINGS:
                    self._confirm(mid)

            else:
                if self.states[mid] == 'collecting':
                    n = len(self.readings[mid])
                    if n >= MIN_READINGS:
                        self._confirm(mid)
                    else:
                        self.get_logger().warn(
                            f'aruco_{mid}: left range with {n} readings — resetting'
                        )
                        self.states[mid] = 'waiting'
                        self.readings[mid] = []

        self._publish()

    def _confirm(self, mid):
        data = np.array(self.readings[mid])
        pos = data[:, :3].mean(axis=0)
        quat = data[:, 3:].mean(axis=0)
        quat /= np.linalg.norm(quat)
        self.confirmed_poses[mid] = (*pos, *quat)
        self.states[mid] = 'confirmed'
        self.get_logger().info(
            f'aruco_{mid}: confirmed at map ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})'
            f' from {len(self.readings[mid])} readings'
        )

    def _publish(self):
        if not self.confirmed_poses:
            return

        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for mid, pose in self.confirmed_poses.items():
            x, y, z, qx, qy, qz, qw = pose
            r, g, b = MARKER_COLORS.get(mid, (1.0, 1.0, 0.0))

            cube = Marker()
            cube.header.frame_id = MAP_FRAME
            cube.header.stamp = stamp
            cube.ns = 'aruco_markers'
            cube.id = mid
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = x
            cube.pose.position.y = y
            cube.pose.position.z = z
            cube.pose.orientation.x = qx
            cube.pose.orientation.y = qy
            cube.pose.orientation.z = qz
            cube.pose.orientation.w = qw
            cube.scale.x = 0.10
            cube.scale.y = 0.10
            cube.scale.z = 0.01
            cube.color.r = r
            cube.color.g = g
            cube.color.b = b
            cube.color.a = 0.8
            cube.lifetime.sec = 0
            array.markers.append(cube)

            label = Marker()
            label.header.frame_id = MAP_FRAME
            label.header.stamp = stamp
            label.ns = 'aruco_labels'
            label.id = mid
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + 0.15
            label.pose.orientation.w = 1.0
            label.scale.z = 0.08
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'aruco_{mid}'
            label.lifetime.sec = 0
            array.markers.append(label)

        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMapMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
