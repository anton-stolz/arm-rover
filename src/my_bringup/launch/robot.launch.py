from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    slam_share = get_package_share_directory("slam_toolbox")
    bringup_share = get_package_share_directory("my_bringup")
    
    # Check if nav2_bringup is available
    try:
        nav2_share = get_package_share_directory("nav2_bringup")
        nav2_installed = True
    except:
        nav2_installed = False
    
    controller_params = os.path.join(bringup_share, "config", "diff_params.yaml")
    ekf_params = os.path.join(bringup_share, "config", "ekf.yaml")
    nav2_params = os.path.join(bringup_share, "config", "nav2_params.yaml")

    launch_actions = [
        # 1. LiDAR Node (Extracted from ld19.launch.py to prevent TF conflicts)
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
                {"binning": 500}  # <--- ADD THIS LINE: Forces exactly 360 points per scan (1° per bin)
            ]
        ),     
        # 2. Diff Drive Node
        Node(
            package="my_controller",
            executable="diff_drive_node",
            parameters=[controller_params],
            output="screen"
        ),
         
        # 3. IMU Node
        Node(
            package="my_controller",
            executable="imu_node",
            output="screen"
        ),
        
        # 3. Bumper Node
        Node(
            package="my_controller",
            executable="bumper_node",
            output="screen"
        ),
        
        # 4. EKF Node
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_params]
        ),
        
        # 5. LiDAR Mount TF
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="base_link_to_laser",
            arguments=["--x", "0.02", "--y", "-0.04", "--z", "0.21", "--yaw", "1.53", "--pitch", "0", "--roll", "0", "--frame-id", "base_link", "--child-frame-id", "base_laser"]
        ),
        
        # 6. IMU Mount TF
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="base_link_to_imu",
            arguments=["--x", "0.03", "--y", "-0.06", "--z", "0.07", "--yaw", "1.53", "--pitch", "0", "--roll", "0", "--frame-id", "base_link", "--child-frame-id", "imu_link"]
        ),
        
        # 7. Foxglove Bridge
        Node(
            package="foxglove_bridge", executable="foxglove_bridge",
            parameters=[{"port": 8765, "capabilities": ["clientPublish", "services", "assets"]}]
        ),
        
        # 8. SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, "launch", "online_async_launch.py")
            ),
            launch_arguments={
                "slam_params_file": os.path.join(bringup_share, "config", "mapper_params_online_async.yaml"),
                "use_sim_time": "false"
            }.items()
        ),
    ]

    if nav2_installed:
        # 9. Nav2 Bringup (Navigation only, SLAM provides the map)
        launch_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_share, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "params_file": nav2_params,
                    "use_sim_time": "false"
                }.items()
            )
        )

    return LaunchDescription(launch_actions)
