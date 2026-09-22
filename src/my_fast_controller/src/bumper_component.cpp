#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

namespace my_fast_controller {

class BumperNode : public rclcpp::Node {
public:
    explicit BumperNode(const rclcpp::NodeOptions & options) : Node("bumper_node", options) {
        av_bumper_l_pub_ = this->create_publisher<std_msgs::msg::Float32>("bumper_left_averaged", 10);
        av_bumper_r_pub_ = this->create_publisher<std_msgs::msg::Float32>("bumper_right_averaged", 10);
        front_hit_pub_ = this->create_publisher<std_msgs::msg::Bool>("bumper/front_hit", 10);
        back_hit_pub_ = this->create_publisher<std_msgs::msg::Bool>("bumper/back_hit", 10);
        safe_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe", 10);

        raw_l_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
            "/bumper_left_raw", 10, [this](const std_msgs::msg::UInt16::SharedPtr msg) {
                if (current_left_ < 0) current_left_ = msg->data;
                else current_left_ = decay_rate_ * current_left_ + msg->data * (1.0 - decay_rate_);
            });
            
        raw_r_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
            "/bumper_right_raw", 10, [this](const std_msgs::msg::UInt16::SharedPtr msg) {
                if (current_right_ < 0) current_right_ = msg->data;
                else current_right_ = decay_rate_ * current_right_ + msg->data * (1.0 - decay_rate_);
            });

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                last_user_twist_ = *msg;
                last_user_twist_time_ = this->now().seconds();
            });

        decay_rate_ = this->declare_parameter("decay_rate", 0.85);
        thresh_back_ = this->declare_parameter("thresh_back", 480.0);
        thresh_front_ = this->declare_parameter("thresh_front", 540.0);
        recovery_speed_ = this->declare_parameter("recovery_speed", 0.10);
        recovery_drive_sec_ = this->declare_parameter("recovery_drive_sec", 0.6);
        recovery_pause_sec_ = this->declare_parameter("recovery_pause_sec", 0.5);

        param_cb_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & params) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto & param : params) {
                    if (param.get_name() == "decay_rate") {
                        decay_rate_ = param.as_double();
                        RCLCPP_INFO(this->get_logger(), "Updated parameter decay_rate: %.4f", decay_rate_);
                    } else if (param.get_name() == "thresh_back") {
                        thresh_back_ = param.as_double();
                        RCLCPP_INFO(this->get_logger(), "Updated parameter thresh_back: %.1f", thresh_back_);
                    } else if (param.get_name() == "thresh_front") {
                        thresh_front_ = param.as_double();
                        RCLCPP_INFO(this->get_logger(), "Updated parameter thresh_front: %.1f", thresh_front_);
                    } else if (param.get_name() == "recovery_speed") {
                        recovery_speed_ = param.as_double();
                    } else if (param.get_name() == "recovery_drive_sec") {
                        recovery_drive_sec_ = param.as_double();
                    } else if (param.get_name() == "recovery_pause_sec") {
                        recovery_pause_sec_ = param.as_double();
                    }
                }
                return result;
            });

        timer_ = this->create_wall_timer(100ms, std::bind(&BumperNode::tick, this));
        RCLCPP_INFO(this->get_logger(), "Bumper Node Component started (decay=%.4f, thresh_front=%.1f, thresh_back=%.1f)",
                    decay_rate_, thresh_front_, thresh_back_);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr av_bumper_l_pub_, av_bumper_r_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr front_hit_pub_, back_hit_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr raw_l_sub_, raw_r_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    double current_left_ = -1.0, current_right_ = -1.0;
    double decay_rate_;
    double thresh_back_;   // Left sensor: triggers back bump when < thresh_back_
    double thresh_front_;  // Right sensor: triggers front bump when > thresh_front_
    double recovery_speed_;
    double recovery_drive_sec_;
    double recovery_pause_sec_;

    int recovery_phase_ = 0;
    int recovery_dir_ = 0;
    double recovery_start_time_ = 0;

    geometry_msgs::msg::Twist last_user_twist_;
    double last_user_twist_time_ = 0;

    int checkCollision() {
        if (current_left_ < 0 || current_right_ < 0) return 0;
        bool front = (current_right_ > thresh_front_); // right is front
        bool back = (current_left_ < thresh_back_);    // left is back
        if (front) return 1;
        if (back) return 2;
        return 0;
    }

    void tick() {
        double now_sec = this->now().seconds();
        if (current_left_ >= 0) {
            std_msgs::msg::Float32 l_msg, r_msg;
            l_msg.data = current_left_; r_msg.data = current_right_;
            av_bumper_l_pub_->publish(l_msg); av_bumper_r_pub_->publish(r_msg);
        }

        if (recovery_phase_ != 0) {
            double elapsed = now_sec - recovery_start_time_;
            if (recovery_phase_ == 1) { // driving
                if (elapsed < recovery_drive_sec_) {
                    geometry_msgs::msg::Twist t;
                    t.linear.x = (recovery_dir_ == 1) ? -recovery_speed_ : recovery_speed_;
                    safe_vel_pub_->publish(t);
                    return;
                } else {
                    recovery_phase_ = 2; // pausing
                    recovery_start_time_ = now_sec;
                }
            }
            if (recovery_phase_ == 2) { // pausing
                if (elapsed < recovery_pause_sec_) {
                    safe_vel_pub_->publish(geometry_msgs::msg::Twist());
                    return;
                } else {
                    recovery_phase_ = 0;
                }
            }
        }

        int collision = checkCollision();
        std_msgs::msg::Bool f_msg, b_msg; f_msg.data = (collision == 1); b_msg.data = (collision == 2);
        front_hit_pub_->publish(f_msg); back_hit_pub_->publish(b_msg);

        if (collision != 0) {
            RCLCPP_WARN(this->get_logger(), "Collision detected! (dir=%d, L_back=%.1f/%.1f, R_front=%.1f/%.1f)",
                        collision, current_left_, thresh_back_, current_right_, thresh_front_);
            recovery_phase_ = 1; recovery_dir_ = collision; recovery_start_time_ = now_sec;
            safe_vel_pub_->publish(geometry_msgs::msg::Twist());
            return;
        }

        if (now_sec - last_user_twist_time_ > 0.5) {
            last_user_twist_ = geometry_msgs::msg::Twist();
        }
        safe_vel_pub_->publish(last_user_twist_);
    }
};

} // namespace my_fast_controller

RCLCPP_COMPONENTS_REGISTER_NODE(my_fast_controller::BumperNode)
