#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <atomic>

using namespace std::chrono_literals;

namespace my_fast_controller {

class ImuNode : public rclcpp::Node {
public:
    explicit ImuNode(const rclcpp::NodeOptions & options) : Node("imu_node", options) {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "wheel/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                bool moving = (std::abs(msg->twist.twist.linear.x) > 0.005 || 
                               std::abs(msg->twist.twist.angular.z) > 0.01);
                wheels_moving_.store(moving);
            });

        initIMU();
        timer_ = this->create_wall_timer(50ms, std::bind(&ImuNode::timerCb, this));
        RCLCPP_INFO(this->get_logger(), "IMU Node Component started");
    }

    ~ImuNode() {
        if (i2c_fd_ >= 0) close(i2c_fd_);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    int i2c_fd_ = -1;
    int current_bank_ = -1;
    std::atomic<bool> wheels_moving_{false};

    bool i2cWrite(uint8_t reg, uint8_t val) {
        if (i2c_fd_ < 0) return false;
        uint8_t buf[2] = {reg, val};
        return ::write(i2c_fd_, buf, 2) == 2;
    }

    bool i2cRead(uint8_t reg, uint8_t *buf, size_t len) {
        if (i2c_fd_ < 0) return false;
        if (::write(i2c_fd_, &reg, 1) != 1) return false;
        return ::read(i2c_fd_, buf, len) == (ssize_t)len;
    }

    void i2cBank(int bank) {
        if (current_bank_ != bank) {
            i2cWrite(0x7F, bank << 4);
            current_bank_ = bank;
        }
    }

    void initIMU() {
        i2c_fd_ = open("/dev/i2c-1", O_RDWR);
        if (i2c_fd_ < 0) { RCLCPP_ERROR(this->get_logger(), "Failed to open I2C"); return; }
        if (ioctl(i2c_fd_, I2C_SLAVE, 0x68) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set I2C address");
            close(i2c_fd_); i2c_fd_ = -1; return;
        }
        i2cBank(0);
        uint8_t who_am_i;
        if (i2cRead(0x00, &who_am_i, 1) && who_am_i == 0xEA) {
            RCLCPP_INFO(this->get_logger(), "ICM20948 IMU detected");
        } else {
            RCLCPP_ERROR(this->get_logger(), "ICM20948 not found");
            close(i2c_fd_); i2c_fd_ = -1; return;
        }

        i2cWrite(0x06, 0x80);
        rclcpp::sleep_for(10ms);
        i2cWrite(0x06, 0x01);
        i2cWrite(0x07, 0x00);
        i2cBank(2);
        i2cWrite(0x00, 10);
        uint8_t cfg1; i2cRead(0x01, &cfg1, 1); i2cWrite(0x01, (cfg1 & 0x8E) | 1 | (5 << 4));
        i2cRead(0x01, &cfg1, 1); i2cWrite(0x01, (cfg1 & 0xF9));
        i2cWrite(0x10, 0); i2cWrite(0x11, 8);
        uint8_t acfg; i2cRead(0x14, &acfg, 1); i2cWrite(0x14, (acfg & 0x8E) | 1 | (5 << 4));
        i2cRead(0x14, &acfg, 1); i2cWrite(0x14, (acfg & 0xF9) | (3 << 1));
        i2cBank(0); i2cWrite(0x0F, 0x30);
    }

    void timerCb() {
        if (i2c_fd_ < 0) return;
        i2cBank(0);
        uint8_t data[12];
        if (!i2cRead(0x2D, data, 12)) return;

        int16_t ax = (data[0] << 8) | data[1];
        int16_t ay = (data[2] << 8) | data[3];
        int16_t az = (data[4] << 8) | data[5];
        int16_t gx = (data[6] << 8) | data[7];
        int16_t gy = (data[8] << 8) | data[9];
        int16_t gz = (data[10] << 8) | data[11];

        sensor_msgs::msg::Imu msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";
        
        if (wheels_moving_.load()) {
            msg.angular_velocity.x = (gx / 131.0) * (M_PI / 180.0);
            msg.angular_velocity.y = (gy / 131.0) * (M_PI / 180.0);
            msg.angular_velocity.z = (gz / 131.0) * (M_PI / 180.0);
        } else {
            msg.angular_velocity.x = 0; msg.angular_velocity.y = 0; msg.angular_velocity.z = 0;
        }
        msg.linear_acceleration.x = (ax / 2048.0) * 9.81;
        msg.linear_acceleration.y = (ay / 2048.0) * 9.81;
        msg.linear_acceleration.z = (az / 2048.0) * 9.81;
        msg.orientation_covariance[0] = -1.0;

        imu_pub_->publish(msg);
    }
};

} // namespace my_fast_controller

RCLCPP_COMPONENTS_REGISTER_NODE(my_fast_controller::ImuNode)
