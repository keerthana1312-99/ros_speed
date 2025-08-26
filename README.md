1. Prerequisites
- Ubuntu 22.04 (or compatible)
- ROS2 Humble/Galactic installed
- colcon build tool
- Python 3
```
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-pip
```
Install ROS2 and dependencies if not done already:

2. Create a ROS2 workspace
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

3.Clone your repository
```
git clone https://github.com/<your-username>/speed_control.git
cd ..
```

4.Build the workspace
```
colcon build --packages-select speed_control
```

5. Source the workspace
```
source install/setup.bash
```

Add this to ~/.bashrc to source automatically on each terminal:
```
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
6.Run the launch file
```
ros2 launch speed_control speed_control.launch.py
```
- radar_dummy_publisher → Publishes fake radar detections
- lidar_dummy_publisher → Publishes fake Lidar point clouds
- speed_control_node → Computes target speed and publishes /cmd_vel
- RViz → Visualizes obstacles and target speed markers

7. Visualize with RViz

RViz shows the vehicle’s environment, obstacles, and speed:
Markers → /speed_control/markers
Text: Target speed
Sphere: Nearest Lidar obstacle
Cube: Nearest Radar obstacle
TF → Shows the vehicle frame (base_link) and coordinate axes.
View → Top-down orthographic view (TopDownOrtho)

# Summary

- The dummy Lidar publishes 50 random points to simulate obstacles.
- The dummy Radar publishes detections with distance and relative velocity.
- The SpeedControlNode subscribes to these, computes the nearest obstacle, applies a low-pass filter, and publishes the safe target speed and velocity commands.
- RViz visualizes the obstacles and the target speed in real-time for easy debugging.

# ROS Speed Control
This ROS2 node, SpeedControlNode, dynamically controls the speed of an autonomous vehicle using Lidar and Radar data. It subscribes to Lidar point clouds and Radar detections to compute the nearest obstacles and calculates a safe target speed, considering parameters like stop distance, time headway, and emergency braking. The node then publishes the target speed and velocity commands (cmd_vel) and visualizes obstacles and speed information in RViz using markers. It also applies low-pass filtering to smooth the speed changes for safe navigation.

# Lidar and Radar Inputs
The LidarDummyPublisher and RadarDummyPublisher nodes are used to simulate sensor inputs for your speed control system:
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
