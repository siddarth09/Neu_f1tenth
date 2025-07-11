from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    param_file = "/home/river/cvy_ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml"

    return LaunchDescription([
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[param_file],
            remappings=[
                ("/scan", "/scan")  
            ]
        )
    ])
