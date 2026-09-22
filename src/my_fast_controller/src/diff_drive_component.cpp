#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp_components/register_node_macro.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <thread>
#include <mutex>
#include <atomic>

using namespace std::chrono_literals;

#pragma pack(push, 1)
struct TelemetryPacket {
    uint8_t   magic1;
    uint8_t   magic2;
    uint32_t  time_ms;
    float     left_angle_deg;
    float     right_angle_deg;
    float     left_vel_deg_s;
    float     right_vel_deg_s;
    int16_t   left_target_dps10;
    int16_t   right_target_dps10;
    uint8_t   left_pwm;
    uint8_t   right_pwm;
    uint8_t   state_left;
    uint8_t   state_right;
    uint16_t  raw_bumper_left;
    uint16_t  raw_bumper_right;
    uint8_t   checksum;
};
#pragma pack(pop)

const uint8_t CMD_MAGIC1 = 0xAA;
const uint8_t CMD_MAGIC2 = 0x55;
const uint8_t TEL_MAGIC1 = 0xAB;
const uint8_t TEL_MAGIC2 = 0xCD;

namespace my_fast_controller {

class DiffDriveNode : public rclcpp::Node {
public:
    explicit DiffDriveNode(const rclcpp::NodeOptions & options) : Node("diff_drive_node", options) {
        port_ = this->declare_parameter("port", "/dev/ttyACM0");
        baud_ = this->declare_parameter("baud", 115200);
        wheel_radius_ = this->declare_parameter("wheel_radius", 0.031);
        wheel_separation_ = this->declare_parameter("wheel_separation", 0.190);
        gear_ratio_ = this->declare_parameter("gear_ratio", 1.0);
        max_wheel_deg_s_ = this->declare_parameter("max_wheel_deg_s", 360.0);
        publish_rate_ = this->declare_parameter("publish_rate", 20.0);
        cmd_timeout_s_ = this->declare_parameter("cmd_timeout_s", 0.5);
        publish_tf_ = this->declare_parameter("publish_tf", true);
        odom_frame_ = this->declare_parameter("odom_frame", "odom");
        base_frame_ = this->declare_parameter("base_frame", "base_link");
        connect_delay_s_ = this->declare_parameter("connect_delay_s", 0.5);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel/odom", 10);
        bumper_left_pub_ = this->create_publisher<std_msgs::msg::UInt16>("bumper_left_raw", 10);
        bumper_right_pub_ = this->create_publisher<std_msgs::msg::UInt16>("bumper_right_raw", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_safe", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                last_lin_ = msg->linear.x; last_ang_ = msg->angular.z; last_cmd_time_ = this->now().seconds();
            });

        tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        stop_reader_ = false;
        serial_thread_ = std::thread(&DiffDriveNode::serialReaderLoop, this);

        double period = 1.0 / (publish_rate_ > 0.0 ? publish_rate_ : 20.0);
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(period),
            std::bind(&DiffDriveNode::timerCb, this));

        RCLCPP_INFO(this->get_logger(), "Diff Drive Node Component started");
    }

    ~DiffDriveNode() {
        stop_reader_ = true;
        if (serial_thread_.joinable()) serial_thread_.join();
        if (ser_fd_ >= 0) {
            uint8_t pkt[7] = {CMD_MAGIC1, CMD_MAGIC2, 0, 0, 0, 0, 0};
            pkt[6] = (pkt[0] + pkt[1]) & 0xFF;
            auto res = ::write(ser_fd_, pkt, 7); (void)res;
            close(ser_fd_);
        }
    }

private:
    std::string port_; int baud_; double wheel_radius_; double wheel_separation_; double gear_ratio_;
    double max_wheel_deg_s_; double publish_rate_; double cmd_timeout_s_; bool publish_tf_;
    std::string odom_frame_; std::string base_frame_; double connect_delay_s_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr bumper_left_pub_, bumper_right_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;
    rclcpp::TimerBase::SharedPtr timer_;

    int ser_fd_ = -1; std::mutex ser_mutex_; std::thread serial_thread_; std::atomic<bool> stop_reader_;
    std::mutex tel_mutex_; TelemetryPacket latest_tel_; double last_tel_time_ = 0;
    
    bool has_last_angles_ = false; double last_angles_l_ = 0; double last_angles_r_ = 0;
    double last_odom_time_ = 0; double odom_x_ = 0, odom_y_ = 0, odom_yaw_ = 0;
    double last_lin_ = 0, last_ang_ = 0; double last_cmd_time_ = 0;

    bool ensureSerial() {
        std::lock_guard<std::mutex> lock(ser_mutex_);
        if (ser_fd_ >= 0) return true;
        ser_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (ser_fd_ < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Serial open failed: %s", port_.c_str());
            return false;
        }
        struct termios tty;
        if (tcgetattr(ser_fd_, &tty) != 0) { close(ser_fd_); ser_fd_ = -1; return false; }
        cfsetospeed(&tty, B115200); cfsetispeed(&tty, B115200);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK; tty.c_lflag = 0; tty.c_oflag = 0;
        tty.c_cc[VMIN]  = 0; tty.c_cc[VTIME] = 1;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
        if (tcsetattr(ser_fd_, TCSANOW, &tty) != 0) { close(ser_fd_); ser_fd_ = -1; return false; }
        if (connect_delay_s_ > 0.0) std::this_thread::sleep_for(std::chrono::duration<double>(connect_delay_s_));
        tcflush(ser_fd_, TCIFLUSH);
        RCLCPP_INFO(this->get_logger(), "Serial connected: %s", port_.c_str());
        return true;
    }

    void sendCommand(double left_dps, double right_dps) {
        if (!ensureSerial()) return;
        int16_t l10 = std::clamp((int)(left_dps * 10.0), -32000, 32000);
        int16_t r10 = std::clamp((int)(right_dps * 10.0), -32000, 32000);
        uint8_t pkt[7] = {CMD_MAGIC1, CMD_MAGIC2, (uint8_t)(l10 & 0xFF), (uint8_t)((l10 >> 8) & 0xFF), (uint8_t)(r10 & 0xFF), (uint8_t)((r10 >> 8) & 0xFF), 0};
        pkt[6] = (pkt[0] + pkt[1] + pkt[2] + pkt[3] + pkt[4] + pkt[5]) & 0xFF;
        std::lock_guard<std::mutex> lock(ser_mutex_);
        if (::write(ser_fd_, pkt, 7) != 7) { close(ser_fd_); ser_fd_ = -1; }
    }

    void serialReaderLoop() {
        std::vector<uint8_t> buf; buf.reserve(256); uint8_t chunk[64];
        while (!stop_reader_.load()) {
            if (!ensureSerial()) { std::this_thread::sleep_for(100ms); continue; }
            ssize_t n = ::read(ser_fd_, chunk, sizeof(chunk));
            if (n <= 0) { std::this_thread::sleep_for(1ms); continue; }
            buf.insert(buf.end(), chunk, chunk + n);
            while (buf.size() >= sizeof(TelemetryPacket)) {
                bool found = false;
                for (size_t i = 0; i <= buf.size() - sizeof(TelemetryPacket); ++i) {
                    if (buf[i] == TEL_MAGIC1 && buf[i+1] == TEL_MAGIC2) {
                        if (i > 0) buf.erase(buf.begin(), buf.begin() + i);
                        found = true; break;
                    }
                }
                if (!found) { if (buf.size() > sizeof(TelemetryPacket) * 4) buf.clear(); break; }
                if (buf.size() < sizeof(TelemetryPacket)) break;
                uint8_t cksum = 0;
                for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; ++i) cksum += buf[i];
                if (cksum == buf[sizeof(TelemetryPacket) - 1]) {
                    TelemetryPacket pkt; memcpy(&pkt, buf.data(), sizeof(TelemetryPacket));
                    {
                        std::lock_guard<std::mutex> lock(tel_mutex_);
                        latest_tel_ = pkt; last_tel_time_ = this->now().seconds();
                    }
                    buf.erase(buf.begin(), buf.begin() + sizeof(TelemetryPacket));
                } else { buf.erase(buf.begin(), buf.begin() + 1); }
            }
        }
    }

    void timerCb() {
        double now_sec = this->now().seconds();
        
        double send_lin = 0.0, send_ang = 0.0;
        if (now_sec - last_cmd_time_ > cmd_timeout_s_) { send_lin = 0.0; send_ang = 0.0; }
        else { send_lin = last_lin_; send_ang = last_ang_; }

        double v_l = -send_lin - send_ang * wheel_separation_ / 2.0;
        double v_r = -send_lin + send_ang * wheel_separation_ / 2.0;
        double dps_l = (v_l / wheel_radius_) * (180.0 / M_PI) * gear_ratio_;
        double dps_r = (v_r / wheel_radius_) * (180.0 / M_PI) * gear_ratio_;
        dps_l = std::clamp(dps_l, -max_wheel_deg_s_, max_wheel_deg_s_);
        dps_r = std::clamp(dps_r, -max_wheel_deg_s_, max_wheel_deg_s_);
        sendCommand(dps_r, dps_l); // swapped

        TelemetryPacket tel; double tel_time = 0;
        { std::lock_guard<std::mutex> lock(tel_mutex_); tel = latest_tel_; tel_time = last_tel_time_; }
        if (tel_time == 0) return;
        if (now_sec - tel_time > 1.0) { has_last_angles_ = false; return; }

        std_msgs::msg::UInt16 rl_msg, rr_msg; rl_msg.data = tel.raw_bumper_left; rr_msg.data = tel.raw_bumper_right;
        bumper_left_pub_->publish(rl_msg); bumper_right_pub_->publish(rr_msg);

        if (!has_last_angles_) {
            last_angles_l_ = tel.left_angle_deg; last_angles_r_ = tel.right_angle_deg;
            last_odom_time_ = now_sec; has_last_angles_ = true; return;
        }

        double dt = now_sec - last_odom_time_; if (dt <= 0.0) return;
        double dl = -(tel.left_angle_deg - last_angles_l_) * (M_PI / 180.0) / gear_ratio_ * wheel_radius_;
        double dr = -(tel.right_angle_deg - last_angles_r_) * (M_PI / 180.0) / gear_ratio_ * wheel_radius_;
        last_angles_l_ = tel.left_angle_deg; last_angles_r_ = tel.right_angle_deg; last_odom_time_ = now_sec;

        double d_center = (dl + dr) / 2.0;
        double d_theta = (dl - dr) / wheel_separation_ * 0.6;
        odom_x_ += d_center * cos(odom_yaw_ + d_theta / 2.0);
        odom_y_ += d_center * sin(odom_yaw_ + d_theta / 2.0);
        odom_yaw_ += d_theta;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now(); odom.header.frame_id = odom_frame_; odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = odom_x_; odom.pose.pose.position.y = odom_y_;
        odom.pose.pose.orientation.w = cos(odom_yaw_ / 2.0); odom.pose.pose.orientation.z = sin(odom_yaw_ / 2.0);
        odom.pose.covariance[0] = 0.8; odom.pose.covariance[7] = 0.8; odom.pose.covariance[35] = 3.0;
        odom.twist.twist.linear.x = d_center / dt; odom.twist.twist.angular.z = d_theta / dt;
        odom.twist.covariance[0] = 0.8; odom.twist.covariance[7] = 0.8; odom.twist.covariance[35] = 3.0;
        odom_pub_->publish(odom);

        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = odom.header.stamp; tf.header.frame_id = odom_frame_; tf.child_frame_id = base_frame_;
            tf.transform.translation.x = odom_x_; tf.transform.translation.y = odom_y_;
            tf.transform.rotation.w = odom.pose.pose.orientation.w; tf.transform.rotation.z = odom.pose.pose.orientation.z;
            tf_br_->sendTransform(tf);
        }
    }
};

} // namespace my_fast_controller

RCLCPP_COMPONENTS_REGISTER_NODE(my_fast_controller::DiffDriveNode)
