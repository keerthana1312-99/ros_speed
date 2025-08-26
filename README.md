# ROS Speed Control
This ROS2 node, SpeedControlNode, dynamically controls the speed of an autonomous vehicle using Lidar and Radar data. It subscribes to Lidar point clouds and Radar detections to compute the nearest obstacles and calculates a safe target speed, considering parameters like stop distance, time headway, and emergency braking. The node then publishes the target speed and velocity commands (cmd_vel) and visualizes obstacles and speed information in RViz using markers. It also applies low-pass filtering to smooth the speed changes for safe navigation.

# The LidarDummyPublisher and RadarDummyPublisher nodes are used to simulate sensor inputs for your speed control system:

### LidarDummyPublisher

Publishes random point cloud data on /lidar/points.

Simulates obstacles detected by a Lidar sensor.

Allows the speed control node to calculate distances to nearby objects without real hardware.

### RadarDummyPublisher

Publishes dummy radar detections on /radar_detections.

Each detection includes a distance and relative velocity.

Lets the speed control node respond to moving obstacles (like vehicles) and compute safe speeds.

The main purpose was these dummy nodes to test and debug SpeedControlNode safely in simulation, without needing actual Lidar or Radar hardware.

# Radar messages

RadarDetection represents one object detected by the radar.
RadarDetections is a container for all objects detected in a single radar scan, with a timestamp.

# launch file
These messages are what your SpeedControlNode subscribes to in order to calculate the nearest obstacles and adjust the vehicle speed.

radar_dummy_publisher: Publishes fake radar data on /radar_detections.
lidar_dummy_publisher: Publishes fake Lidar point clouds on /lidar/points.
speed_control_node: Subscribes to Lidar and Radar, computes safe speed, publishes /target_speed and /cmd_vel.
rviz2: Launches RViz with the configured view to visualize obstacles and speed markers (optional)

This launch file lets you run the full simulation of your speed control system, including sensor simulation, control computation, and visualization, with one command:

# Rviz
This RViz setup allows you to see the vehicle’s sensor data, obstacles, and computed target speed in real-time, helping debug and understand the behavior of the speed control node.

### Displays
MarkerArray (Topic: /speed_control/markers) → Shows the visual markers published by the speed control node, such as:
Target speed text
Nearest Lidar obstacle (sphere)
Nearest Radar obstacle (cube)
TF → Visualizes the coordinate frames with arrows and axes.
