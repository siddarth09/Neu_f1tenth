from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    mercedes_dir = get_package_share_directory('f1tenth_stack')
    map_dir = '/home/river/cvy_ws/src/f1tenth_system/maps/isec_basement.yaml'
    xacro_file = os.path.join(mercedes_dir, 'launch', 'racecar.xacro')
    cartographer_config_dir = "/home/river/cvy_ws/src/f1tenth_system/f1tenth_stack/config"

    # AMCL parameter file
    amcl_params = PathJoinSubstitution([
        FindPackageShare('f1tenth_stack'),
        'config',
        'amcl_config.yaml'
    ])

    return LaunchDescription([
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_dir,
                'topic': 'map',
                'frame_id': 'map',
                'use_sim_time': True
            }]
        ),

        # AMCL Node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_params]
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        ),

        

        # Cartographer Node (live SLAM, not localization)
        # Node(
        #     package="cartographer_ros",
        #     executable="cartographer_node",
        #     name="cartographer_node",
        #     output="screen",
        #     arguments=[
        #         "-configuration_directory", cartographer_config_dir,
        #         "-configuration_basename", "lidar.lua"
        #     ],
        #     parameters=[{'use_sim_time': True}]
        # )
    ])
