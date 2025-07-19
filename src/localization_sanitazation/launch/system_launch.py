import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = {"use_sim_time": True}

    map_yaml_file = "/home/kavishe/ros2_ws/maps/house_map.yaml"
    amcl_params_file = os.path.join(
        "/home/kavishe/ros2_ws/src/navigation2/nav2_amcl/config/amcl_params.yaml"
    )
    urdf_path = "/home/kavishe/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/urdf/turtlebot3_burger.urdf"
    robot_description = open(urdf_path).read()

    return LaunchDescription([
        # Map Server
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": map_yaml_file, "use_sim_time": True}]
        ),

        # AMCL Localization
        Node(
            package="nav2_amcl",
            executable="nav2_amcl",
            name="amcl",
            output="screen",
            parameters=[amcl_params_file, {"use_sim_time": True}]
        ),

        # NAV2 Controller
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=["/home/kavishe/ros2_ws/src/localization_sanitazation/config/controller_server.yaml", {"use_sim_time": True}]
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
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server", "amcl", "controller_server"]
            }]
        ),

        # Static TF: map -> odom
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_static_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "1", "map", "odom"],
            parameters=[use_sim_time]
        ),

        # Static TF: base_footprint -> base_link
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_static_tf",
            arguments=["0", "0", "0.01", "0", "0", "0", "1", "base_footprint", "base_link"],
            parameters=[use_sim_time]
        ),

        # Custom Nodes
    #    Node(
    #        package="localization_sanitazation",
    #        executable="frontier_node",
    #        name="frontier_node",
    #        output="screen",
    #        parameters=[use_sim_time]
    #    ),
    #    Node(
    #        package="localization_sanitazation",
    #        executable="localization_node",
    #        name="localization_node",
    #        output="screen",
    #        parameters=[use_sim_time]
    #    ),
    #    Node(
    #        package="localization_sanitazation",
    #        executable="navigation_node",
    #        name="navigation_node",
    #        output="screen",
    #        parameters=[use_sim_time]
    #    ),
    #    Node(
    #        package="localization_sanitazation",
    #        executable="sanitization_node",
    #        name="sanitization_node",
    #        output="screen",
    #        parameters=[use_sim_time]
    #    ),

        # Optional FSM node (commented out)
        # Node(
        #     package="localization_sanitazation",
        #     executable="FSMcontroller_node",
        #     name="FSMcontroller_node",
        #     output="screen",
        #     parameters=[use_sim_time]
        # )
    ])
