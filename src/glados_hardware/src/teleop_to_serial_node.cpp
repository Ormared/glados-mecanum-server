#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "glados_hardware/msg/int8_array.hpp"
#include <cmath>

// Odometry
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

// JointState
#include "sensor_msgs/msg/joint_state.hpp"

// uint16_t crc16_modbus(const std::vector<uint8_t>& data)
static uint16_t crc16_modbus(const std::vector<uint8_t>& buf) {
    // Initialize the CRC lookup table using a std::vector
    static const std::vector<uint16_t> table = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 
    };

    uint8_t xr1 = 0;
    uint16_t crc = 0xFFFF;

    // Use the size of the vector instead of len
    for (const auto& bufferItem : buf) {
        xr1 = bufferItem ^ crc;
        crc >>= 8;
        crc ^= table[xr1];
    }

    return crc;
}

double getThresholdedValue(const double &value, const double &threshold)
{
    if (value > threshold)
    {
        return threshold;
    }
    else if (value < -threshold)
    {
        return -threshold;
    }
    return value;
}

const int N_JOINTS = 4;

class TeleopToSerialNode : public rclcpp::Node {
public:
    TeleopToSerialNode() : Node("teleop_to_serial_node"),
                           first_message_(true),
                           x_(0.0),
                           y_(0.0),
                           theta_(0.0) {
        joint_names_.resize(N_JOINTS);
        accumulated_angles_ = std::vector<double>(4, 0.0);

        // STM
        this->declare_parameter<int>("from_stm_message_size_bytes", 39);
        this->declare_parameter<int>("to_stm_message_size_bytes", 20);
        this->declare_parameter<int>("device_id", 0xa8);
        this->declare_parameter<int>("message_id", 1);
        this->declare_parameter<double>("frequency_constant", 10000.0);
        this->declare_parameter<double>("max_frequency", 32767.0);
        this->get_parameter("from_stm_message_size_bytes", from_stm_message_size_bytes_);
        this->get_parameter("to_stm_message_size_bytes", to_stm_message_size_bytes_);
        this->get_parameter("device_id", device_id_);
        this->get_parameter("message_id", message_id_);
        this->get_parameter("frequency_constant", frequency_constant_);
        this->get_parameter("max_frequency", max_frequency_);

        // Measurements and properties
        this->declare_parameter<std::string>("fl_joint_name", "front_left_wheel_joint");
        this->declare_parameter<std::string>("fr_joint_name", "front_right_wheel_joint");
        this->declare_parameter<std::string>("rl_joint_name", "rear_left_wheel_joint");
        this->declare_parameter<std::string>("rr_joint_name", "rear_right_wheel_joint");
        this->declare_parameter<double>("r", 0.1);
        this->declare_parameter<double>("lx", 0.1625);
        this->declare_parameter<double>("ly", 0.2);
        this->get_parameter("fl_joint_name", joint_names_[0]);
        this->get_parameter("fr_joint_name", joint_names_[1]);
        this->get_parameter("rl_joint_name", joint_names_[2]);
        this->get_parameter("rr_joint_name", joint_names_[3]);
        this->get_parameter("r", r_);
        this->get_parameter("lx", lx_);
        this->get_parameter("ly", ly_);

        // Velocity
        twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&TeleopToSerialNode::twist_callback, this, std::placeholders::_1));

        // Serial
        serial_publisher_ = this->create_publisher<glados_hardware::msg::Int8Array>("serial_write", 10);
        serial_subscription_ = this->create_subscription<glados_hardware::msg::Int8Array>(
            "serial_read", 10, std::bind(&TeleopToSerialNode::serial_callback, this, std::placeholders::_1));

        // Odometry
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // JointState publisher
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    }

private:
    std::vector<int16_t> velocitiesToFrequencies(const double &v_x,
                                                 const double &v_y,
                                                 const double &omega) {
        std::vector<int16_t> frequencies(4);

        frequencies[0] = static_cast<int16_t>(
            getThresholdedValue(frequency_constant_ * (1/r_) * (v_x - v_y - (lx_ + ly_) * omega) / (2 * M_PI), max_frequency_));
        frequencies[1] = static_cast<int16_t>(
            getThresholdedValue(frequency_constant_ * (1/r_) * (v_x + v_y + (lx_ + ly_) * omega) / (2 * M_PI), max_frequency_));
        frequencies[2] = static_cast<int16_t>(
            getThresholdedValue(frequency_constant_ * (1/r_) * (v_x + v_y - (lx_ + ly_) * omega) / (2 * M_PI), max_frequency_));
        frequencies[3] = static_cast<int16_t>(
            getThresholdedValue(frequency_constant_ * (1/r_) * (v_x - v_y + (lx_ + ly_) * omega) / (2 * M_PI), max_frequency_));
        
        return frequencies;
    }

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto frequencies = velocitiesToFrequencies(msg->linear.x, msg->linear.y, msg->angular.z);

        // Prepare the serial data message
        std::vector<uint8_t> serial_data = {
            static_cast<uint8_t>(device_id_),
            static_cast<uint8_t>(to_stm_message_size_bytes_),
            static_cast<uint8_t>(message_id_),
            0, 0, 0, 0,
            static_cast<uint8_t>((frequencies[3] >> 8) & 0xFF),
            static_cast<uint8_t>(frequencies[3] & 0xFF),
            static_cast<uint8_t>((frequencies[2] >> 8) & 0xFF),
            static_cast<uint8_t>(frequencies[2] & 0xFF),
            static_cast<uint8_t>((frequencies[0] >> 8) & 0xFF),
            static_cast<uint8_t>(frequencies[0] & 0xFF),
            static_cast<uint8_t>((frequencies[1] >> 8) & 0xFF),
            static_cast<uint8_t>(frequencies[1] & 0xFF),
            0, 0, 0
        };

        // Calculate and append CRC to the message
        uint16_t crc = crc16_modbus(serial_data);
        serial_data.push_back(static_cast<uint8_t>(crc & 0xFF));        // CRC low byte
        serial_data.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF)); // CRC high byte

        // Create the Int8Array message and publish it
        auto serial_msg = glados_hardware::msg::Int8Array();
        serial_msg.data = serial_data;
        serial_publisher_->publish(serial_msg);
    }

    void serial_callback(const glados_hardware::msg::Int8Array::SharedPtr msg)
    {
        const auto current_time = this->now();

        if (first_message_) {
            last_time_ = current_time;
            first_message_ = false;
            return;
        }

        // Extract frequencies in rad/s
        auto data = msg->data;

        static const double twoPi = 2 * M_PI;

        // Frequencies in rad/s
        const std::vector<double> frequencies = {
            static_cast<int16_t>(data[17] * 256 + data[18]) / frequency_constant_ * twoPi,
            static_cast<int16_t>(data[19] * 256 + data[20]) / frequency_constant_ * twoPi,
            static_cast<int16_t>(data[15] * 256 + data[16]) / frequency_constant_ * twoPi,
            static_cast<int16_t>(data[13] * 256 + data[14]) / frequency_constant_ * twoPi
        };

        publishOdometry(frequencies, current_time);
        publishJointStates(frequencies, current_time);

        last_time_ = current_time;
    }

    void publishJointStates(const std::vector<double> &frequencies,
                            const rclcpp::Time &current_time) {
        const double dt = (current_time - last_time_).seconds();
        
        // Compute delta angles and accumulate them
        for (int i = 0; i < N_JOINTS; ++i) {
            accumulated_angles_[i] += frequencies[i] * dt;
        }

        // Publish joint states
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = current_time;
        joint_state_msg.name = joint_names_;
        joint_state_msg.position = accumulated_angles_;

        joint_state_publisher_->publish(joint_state_msg);
    }

    void publishOdometry(const std::vector<double> &frequencies,
                         const rclcpp::Time &current_time) {
        const double dt = (current_time - last_time_).seconds();
        
        // Compute velocities in m/s
        const double vx = r_ * (frequencies[0] + frequencies[1] + frequencies[2] + frequencies[3]) / 4.0;
        const double vy = r_ * (-frequencies[0] + frequencies[1] + frequencies[2] - frequencies[3]) / 4.0;
        const double omega = r_ * (-frequencies[0] + frequencies[1] - frequencies[2] + frequencies[3]) / (4.0 * (lx_ + ly_));

        // Update pose using velocities
        const double dx = (vx * std::cos(theta_) - vy * std::sin(theta_)) * dt;
        const double dy = (vx * std::sin(theta_) + vy * std::cos(theta_)) * dt;
        const double dtheta = omega * dt;
        x_ += dx;
        y_ += dy;
        theta_ += dtheta;

        // Publish Odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.angular.z = omega;

        odom_publisher_->publish(odom_msg);

        // Publish Transform
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = current_time;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_footprint";

        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = odom_msg.pose.pose.orientation;

        // tf_broadcaster_->sendTransform(transform);
    }

    // STM constants
    int from_stm_message_size_bytes_;
    int to_stm_message_size_bytes_;
    int device_id_;
    int message_id_;
    double frequency_constant_;
    double max_frequency_;
    double r_;
    double lx_;
    double ly_;

    bool first_message_;

    // JointState
    std::vector<std::string> joint_names_;
    std::vector<double> accumulated_angles_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

    // Odometry
    double x_;
    double y_;
    double theta_;
    rclcpp::Time last_time_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
    rclcpp::Publisher<glados_hardware::msg::Int8Array>::SharedPtr serial_publisher_;
    rclcpp::Subscription<glados_hardware::msg::Int8Array>::SharedPtr serial_subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopToSerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}