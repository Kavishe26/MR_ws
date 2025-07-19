import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "nav2_amcl"
    amcl_params_file = os.path.join(get_package_share_directory(package_name), "config", "amcl_params.yaml")
    map_file = "/home/kavishe/ros2_ws/maps/house_map.yaml"

    urdf_path = "/home/kavishe/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/urdf/turtlebot3_burger.urdf"
    robot_description = open(urdf_path).read()

    return LaunchDescription([
        # Map Server
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename":map_file,
            "use_sim_time": True}]
        ),

        # AMCL Node
        Node(
            package="nav2_amcl",
            executable="nav2_amcl",
            name="amcl",
            output="screen",
            parameters=[amcl_params_file]
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": True
            }]
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
        )
    ])

