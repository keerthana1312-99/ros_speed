from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),

        Node(
            package='speed_control',
            executable='radar_dummy_publisher',
            name='radar_dummy_publisher',
            output='screen'
        ),

        # optional LiDAR dummy
        Node(
            package='speed_control',
            executable='lidar_dummy_publisher',
            name='lidar_dummy_publisher',
            output='screen'
        ),

        Node(
            package='speed_control',
            executable='speed_control_node',
            name='speed_control_node',
            output='screen',
            parameters=['config/params.yaml']
        ),

        # RViz (toggle with use_rviz:=false)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'rviz/speed_control.rviz'],
            condition=None if str(use_rdv:=use_rviz) == 'true' else None  # launch will handle as default
        ),
    ])

