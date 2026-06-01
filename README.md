# lab4_aruco_exercise

ROS2 node that detects ArUco markers via TF2 and records their confirmed positions in the map frame as RViz visualization markers.

## Setup

```bash
mkdir -p astro_ws/src
cd astro_ws/src
git clone https://github.com/CRTA-Lab/ASTRO.git
git clone https://github.com/CRTA-Lab/lab4_aruco_exercise.git
cd ..
colcon build --symlink-install
```

## Usage

### 1. Start the robot state publisher

```bash
ros2 launch astro rsp.launch.py
```

### 2. Start localization

```bash
cd src/lab4_aruco_exercise
ros2 launch astro_navigation localization.launch.py map:=maps/aruco_mapa.yaml
```

### 3. Open RViz

```bash
ros2 run rviz2 rviz2 -d ~/astro_ws/src/lab4_aruco_exercise/config/nav2_default_view.rviz
```

### 4. Run the ArUco TF broadcaster

```bash
ros2 run lab4_aruco_exercise basic_aruco_cv_transformation
```

### 5. Run the ArUco map marker node

```bash
ros2 run lab4_aruco_exercise aruco_map_marker
```

Drive the robot within **70 cm** of each marker. Once 10–15 TF readings are collected, the marker is confirmed and drawn in RViz on the `/aruco_map_markers` topic.

## Topics

| Name | Type | Description |
|------|------|-------------|
| `/aruco_map_markers` | `visualization_msgs/msg/MarkerArray` | Confirmed marker poses visualized in the map frame |
