#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp_components/register_node_macro.hpp>

#include <fstream>
#include <sstream>
#include <cmath>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

namespace my_fast_controller {

class PosePreserverNode : public rclcpp::Node {
public:
    explicit PosePreserverNode(const rclcpp::NodeOptions & options)
    : Node("pose_preserver_node", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        yaml_path_ = this->declare_parameter<std::string>(
            "params_file",
            "/home/anton/ros2_ws/src/my_fast_bringup/config/mapper_params_localization.yaml");

        timer_ = this->create_wall_timer(2000ms, std::bind(&PosePreserverNode::checkAndSave, this));
        RCLCPP_INFO(this->get_logger(), "PosePreserver C++ Component ready. Target: %s", yaml_path_.c_str());
    }

    ~PosePreserverNode() override {
        savePose();
    }

private:
    void checkAndSave() {
        try {
            if (!tf_buffer_.canTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.1))) {
                return;
            }
            auto t = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            double x = t.transform.translation.x;
            double y = t.transform.translation.y;

            tf2::Quaternion q(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w
            );
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            double d_dist = std::hypot(x - last_x_, y - last_y_);
            double d_yaw = std::abs(yaw - last_yaw_);
            if (d_yaw > M_PI) d_yaw = 2.0 * M_PI - d_yaw;

            if (d_dist < 0.03 && d_yaw < 0.05) {
                // Robot is still
                if (!is_still_) {
                    still_start_time_ = this->now().seconds();
                    is_still_ = true;
                } else if (!saved_ && (this->now().seconds() - still_start_time_ > 3.0)) {
                    current_x_ = x;
                    current_y_ = y;
                    current_yaw_ = yaw;
                    has_valid_pose_ = true;
                    savePose();
                    saved_ = true;
                }
            } else {
                // Robot is moving
                is_still_ = false;
                saved_ = false;
                last_x_ = x;
                last_y_ = y;
                last_yaw_ = yaw;
                current_x_ = x;
                current_y_ = y;
                current_yaw_ = yaw;
                has_valid_pose_ = true;
            }
        } catch (const tf2::TransformException &) {
            // TF not ready or dropped
        }
    }

    void savePose() {
        if (!has_valid_pose_) return;

        std::ifstream fin(yaml_path_);
        if (!fin.is_open()) return;

        std::string line;
        std::vector<std::string> lines;
        bool in_start_pose = false;

        while (std::getline(fin, line)) {
            if (line.find("map_start_pose:") != std::string::npos) {
                lines.push_back("    map_start_pose:");
                char buf[64];
                snprintf(buf, sizeof(buf), "    - %.3f", current_x_);
                lines.push_back(buf);
                snprintf(buf, sizeof(buf), "    - %.3f", current_y_);
                lines.push_back(buf);
                snprintf(buf, sizeof(buf), "    - %.3f", current_yaw_);
                lines.push_back(buf);
                in_start_pose = true;
            } else if (in_start_pose) {
                if (line.find("    - ") == 0 || line.find("  - ") == 0 || line.find("- ") == 0) {
                    continue; // skip old entries
                } else {
                    in_start_pose = false;
                    lines.push_back(line);
                }
            } else {
                lines.push_back(line);
            }
        }
        fin.close();

        std::ofstream fout(yaml_path_);
        if (!fout.is_open()) return;
        for (const auto & l : lines) {
            fout << l << "\n";
        }
        fout.close();

        RCLCPP_INFO(this->get_logger(), "Preserved robot pose to %s: [%.3f, %.3f, %.3f]",
                    yaml_path_.c_str(), current_x_, current_y_, current_yaw_);
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string yaml_path_;
    double last_x_ = 0.0, last_y_ = 0.0, last_yaw_ = 0.0;
    double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0;
    bool has_valid_pose_ = false;
    bool is_still_ = false;
    bool saved_ = false;
    double still_start_time_ = 0.0;
};

} // namespace my_fast_controller

RCLCPP_COMPONENTS_REGISTER_NODE(my_fast_controller::PosePreserverNode)
