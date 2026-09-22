from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    slam_share = get_package_share_directory("slam_toolbox")
    bringup_share = get_package_share_directory("my_fast_bringup")
    
    try:
        nav2_share = get_package_share_directory("nav2_bringup")
        nav2_installed = True
    except:
        nav2_installed = False
    
    controller_params = os.path.join(bringup_share, "config", "diff_params.yaml")
    ekf_params = os.path.join(bringup_share, "config", "ekf.yaml")
    nav2_params = os.path.join(bringup_share, "config", "nav2_params.yaml")
    default_map = os.path.join(bringup_share, "maps", "map_downsampled_005.yaml")

    declare_localization_cmd = DeclareLaunchArgument(
        "localization",
        default_value="true",
        description="Whether to run in localization mode with AMCL to SLAM handover or mapping"
    )

    declare_map_cmd = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Full path to map yaml file to load for AMCL localization"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true"
    )

    localization = LaunchConfiguration("localization")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")

    container = ComposableNodeContainer(
        name="fast_controller_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="my_fast_controller",
                plugin="my_fast_controller::DiffDriveNode",
                name="diff_drive_node",
                parameters=[controller_params]),
            ComposableNode(
                package="my_fast_controller",
                plugin="my_fast_controller::ImuNode",
                name="imu_node"),
            ComposableNode(
                package="my_fast_controller",
                plugin="my_fast_controller::BumperNode",
                name="bumper_node",
                parameters=[controller_params]),
            ComposableNode(
                package="my_fast_controller",
                plugin="my_fast_controller::BumpMapperNode",
                name="bump_mapper_node"),
            ComposableNode(
                package="my_fast_controller",
                plugin="my_fast_controller::PointToPoseNode",
                name="point_to_pose_translator"),
            ComposableNode(
                package="my_fast_controller",
                plugin="my_fast_controller::PosePreserverNode",
                name="pose_preserver_node",
                parameters=[{"params_file": os.path.join(bringup_share, "config", "mapper_params_localization.yaml")}]),
        ],
        output="screen",
    )

    launch_actions = [
        declare_localization_cmd,
        declare_map_cmd,
        declare_use_sim_time_cmd,
        Node(
            package="ldlidar_stl_ros2",
            executable="ldlidar_stl_ros2_node",
            name="LD19",
            output="screen",
            parameters=[
                {"product_name": "LDLiDAR_LD19"},
                {"topic_name": "scan"},
                {"frame_id": "base_laser"},
                {"port_name": "/dev/ttyUSB0"},
                {"port_baudrate": 230400},
                {"laser_scan_dir": True},
                {"enable_angle_crop_func": False},
                {"bins": 500}
            ]
        ),     
        container,
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_params]
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="base_link_to_laser",
            arguments=["--x", "0.02", "--y", "-0.04", "--z", "0.21", "--yaw", "1.53", "--pitch", "0", "--roll", "0", "--frame-id", "base_link", "--child-frame-id", "base_laser"]
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="base_link_to_imu",
            arguments=["--x", "0.03", "--y", "-0.06", "--z", "0.07", "--yaw", "1.53", "--pitch", "0", "--roll", "0", "--frame-id", "base_link", "--child-frame-id", "imu_link"]
        ),
        Node(
            package="foxglove_bridge", executable="foxglove_bridge",
            parameters=[{"port": 8765, "capabilities": ["clientPublish", "services", "assets"]}]
        ),
        # Mapping Mode: SLAM Toolbox online async (only active if localization:=false)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, "launch", "online_async_launch.py")
            ),
            launch_arguments={
                "slam_params_file": os.path.join(bringup_share, "config", "mapper_params_online_async.yaml"),
                "use_sim_time": use_sim_time
            }.items(),
            condition=UnlessCondition(localization)
        ),
        # Localization Mode: SLAM Toolbox Pure Localization (seeded at Teppich Ecke)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, "launch", "localization_launch.py")
            ),
            launch_arguments={
                "slam_params_file": os.path.join(bringup_share, "config", "mapper_params_localization.yaml"),
                "use_sim_time": use_sim_time
            }.items(),
            condition=IfCondition(localization)
        ),
    ]

    if nav2_installed:
        # Nav2 Navigation (planner, controller, bt_navigator, recoveries)
        launch_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "params_file": nav2_params,
                    "use_sim_time": use_sim_time
                }.items()
            )
        )
    return LaunchDescription(launch_actions)
