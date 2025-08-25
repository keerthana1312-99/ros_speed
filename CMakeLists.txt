cmake_minimum_required(VERSION 3.5)
project(speed_control)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ============================
# Custom Messages
# ============================
set(msg_files
  "msg/RadarDetection.msg"
  "msg/RadarDetections.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ============================
# Include Directories
# ============================
include_directories(include)

# ============================
# Executables
# ============================
# Speed Control Node
add_executable(speed_control_node src/speed_control_node.cpp)
ament_target_dependencies(speed_control_node
  rclcpp std_msgs geometry_msgs sensor_msgs visualization_msgs
)

# Lidar Dummy Publisher (NO PCL needed)
add_executable(lidar_dummy_publisher src/lidar_dummy_publisher.cpp)
ament_target_dependencies(lidar_dummy_publisher rclcpp sensor_msgs)

# Radar Dummy Publisher
add_executable(radar_dummy_publisher src/radar_dummy_publisher.cpp)
ament_target_dependencies(radar_dummy_publisher rclcpp std_msgs)

# ============================
# Link generated typesupport
# ============================
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(speed_control_node ${cpp_typesupport_target})
target_link_libraries(radar_dummy_publisher ${cpp_typesupport_target})

# ============================
# Install Targets
# ============================
install(TARGETS
  speed_control_node
  lidar_dummy_publisher
  radar_dummy_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch/config/rviz files
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# ============================
# Package Export
# ============================
ament_package()

