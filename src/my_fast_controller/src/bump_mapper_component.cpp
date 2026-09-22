#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp_components/register_node_macro.hpp>

#include <vector>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

namespace my_fast_controller {

struct BumpPoint {
    float x_map;
    float y_map;
    double stamp;
    float intensity;
};

class BumpMapperNode : public rclcpp::Node {
public:
    explicit BumpMapperNode(const rclcpp::NodeOptions & options)
    : Node("bump_mapper_node", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/bumper_obstacles", 10);

        front_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "bumper/front_hit", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) onHit(true);
            });

        back_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "bumper/back_hit", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) onHit(false);
            });

        timer_ = this->create_wall_timer(1000ms, std::bind(&BumpMapperNode::tick, this));

        RCLCPP_INFO(this->get_logger(), "BumpMapper C++ Component ready.");
    }

private:
    void onHit(bool front) {
        double now_sec = this->now().seconds();
        if (front) {
            if (now_sec - last_front_hit_ < 0.8) return;
            last_front_hit_ = now_sec;
            recordSegment(0.15f, -0.14f, 0.14f, 5, now_sec);
        } else {
            if (now_sec - last_back_hit_ < 0.8) return;
            last_back_hit_ = now_sec;
            recordSegment(-0.15f, -0.14f, 0.14f, 5, now_sec);
        }
    }

    void recordSegment(float offset_x, float y_min, float y_max, int num_pts, double now_sec) {
        std::string frame_id = "map";
        try {
            if (!tf_buffer_.canTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.05))) {
                frame_id = "odom";
            }
        } catch (...) {
            frame_id = "odom";
        }

        try {
            auto t = tf_buffer_.lookupTransform(frame_id, "base_link", tf2::TimePointZero);
            float tx = static_cast<float>(t.transform.translation.x);
            float ty = static_cast<float>(t.transform.translation.y);
            tf2::Quaternion q(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w
            );
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            float cos_yaw = std::cos(static_cast<float>(yaw));
            float sin_yaw = std::sin(static_cast<float>(yaw));

            float step = (num_pts > 1) ? (y_max - y_min) / (num_pts - 1) : 0.0f;
            for (int i = 0; i < num_pts; ++i) {
                float y_loc = y_min + i * step;
                float gx = tx + (offset_x * cos_yaw - y_loc * sin_yaw);
                float gy = ty + (offset_x * sin_yaw + y_loc * cos_yaw);
                points_.push_back({gx, gy, now_sec, 100.0f});
            }
            RCLCPP_INFO(this->get_logger(), "Recorded bumper hit in frame %s. Active points: %zu",
                        frame_id.c_str(), points_.size());
            publishCloud();
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed for bumper hit: %s", ex.what());
        }
    }

    void tick() {
        if (points_.empty()) return;

        double now_sec = this->now().seconds();
        std::vector<BumpPoint> alive;
        alive.reserve(points_.size());

        for (auto & p : points_) {
            double age = now_sec - p.stamp;
            if (age < 60.0) {
                p.intensity = 100.0f;
                alive.push_back(p);
            } else if (age < 300.0) {
                p.intensity = 100.0f - static_cast<float>((age - 60.0) / 240.0 * 75.0);
                alive.push_back(p);
            }
        }
        points_ = std::move(alive);

        if (!points_.empty()) {
            publishCloud();
        }
    }

    void publishCloud() {
        if (points_.empty()) return;

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = this->now();
        cloud.header.frame_id = "map";
        cloud.height = 1;
        cloud.width = points_.size();
        cloud.is_dense = true;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(points_.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_i(cloud, "intensity");

        for (const auto & p : points_) {
            *iter_x = p.x_map;
            *iter_y = p.y_map;
            *iter_z = 0.05f;
            *iter_i = p.intensity;

            ++iter_x; ++iter_y; ++iter_z; ++iter_i;
        }

        cloud_pub_->publish(cloud);
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr front_sub_, back_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<BumpPoint> points_;
    double last_front_hit_ = 0.0;
    double last_back_hit_ = 0.0;
};

} // namespace my_fast_controller

RCLCPP_COMPONENTS_REGISTER_NODE(my_fast_controller::BumpMapperNode)
