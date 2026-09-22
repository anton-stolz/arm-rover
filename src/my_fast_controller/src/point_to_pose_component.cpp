#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>

namespace my_fast_controller {

class PointToPoseNode : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;

    explicit PointToPoseNode(const rclcpp::NodeOptions & options)
    : Node("point_to_pose_translator", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        rclcpp::QoS qos_tl(10);
        qos_tl.reliable();
        qos_tl.transient_local();

        rclcpp::QoS qos_vol(10);
        qos_vol.reliable();
        qos_vol.durability_volatile();

        sub_pt_tl_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/goal_point", qos_tl,
            std::bind(&PointToPoseNode::onPointMsg, this, std::placeholders::_1));

        sub_pt_vol_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/goal_point", qos_vol,
            std::bind(&PointToPoseNode::onPointMsg, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PointToPose C++ Component ready on /goal_point.");
    }

private:
    void onPointMsg(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), ">>> [TRANSLATOR] Got Point on /goal_point: frame=%s, x=%.2f, y=%.2f",
                    msg->header.frame_id.c_str(), msg->point.x, msg->point.y);

        std::string target_frame = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
        double yaw = 0.0;
        try {
            auto transform = tf_buffer_.lookupTransform(
                target_frame, "base_link", tf2::TimePointZero, tf2::durationFromSec(0.2));
            double rx = transform.transform.translation.x;
            double ry = transform.transform.translation.y;
            double dx = msg->point.x - rx;
            double dy = msg->point.y - ry;
            if (std::hypot(dx, dy) > 0.05) {
                yaw = std::atan2(dy, dx);
            }
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup fallback: %s", ex.what());
        }

        geometry_msgs::msg::PoseStamped goal_stamped;
        goal_stamped.header.stamp = this->now();
        goal_stamped.header.frame_id = target_frame;
        goal_stamped.pose.position = msg->point;
        goal_stamped.pose.orientation.x = 0.0;
        goal_stamped.pose.orientation.y = 0.0;
        goal_stamped.pose.orientation.z = std::sin(yaw / 2.0);
        goal_stamped.pose.orientation.w = std::cos(yaw / 2.0);

        pose_pub_->publish(goal_stamped);
        RCLCPP_INFO(this->get_logger(), ">>> Dispatched /goal_pose to Nav2: frame=%s, x=%.2f, y=%.2f",
                    target_frame.c_str(), goal_stamped.pose.position.x, goal_stamped.pose.position.y);

        if (action_client_->action_server_is_ready()) {
            NavigateToPose::Goal goal_msg;
            goal_msg.pose = goal_stamped;
            action_client_->async_send_goal(goal_msg);
        }
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_pt_tl_, sub_pt_vol_;
};

} // namespace my_fast_controller

RCLCPP_COMPONENTS_REGISTER_NODE(my_fast_controller::PointToPoseNode)
