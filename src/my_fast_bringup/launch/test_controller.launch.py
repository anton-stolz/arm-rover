from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_share = get_package_share_directory("my_fast_bringup")
    controller_params = os.path.join(bringup_share, "config", "diff_params.yaml")
    
    container = ComposableNodeContainer(
        name="fast_controller_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(package="my_fast_controller", plugin="my_fast_controller::DiffDriveNode", name="diff_drive_node", parameters=[controller_params]),
            ComposableNode(package="my_fast_controller", plugin="my_fast_controller::ImuNode", name="imu_node"),
            ComposableNode(package="my_fast_controller", plugin="my_fast_controller::BumperNode", name="bumper_node"),
        ],
        output="screen",
    )
    return LaunchDescription([container])
