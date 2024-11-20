#include "serial_gz_adapter_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "glados_hardware/msg/int8_array.hpp"

SerialGzAdapterNode::SerialGzAdapterNode() : Node("serial_gz_adapter_node") {
    serial_write_sub_ = create_subscription<glados_hardware::msg::Int8Array>(
        "serial_write", 10, 
        std::bind(&SerialGzAdapterNode::serialWriteCallback, this, std::placeholders::_1)
    );

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states_gz", 10, 
        std::bind(&SerialGzAdapterNode::jointStateCallback, this, std::placeholders::_1)
    );

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_gz", 10);
    serial_read_pub_ = create_publisher<glados_hardware::msg::Int8Array>("serial_read", 10);
}

void SerialGzAdapterNode::serialWriteCallback(const glados_hardware::msg::Int8Array::SharedPtr msg) {
    if (msg->data.size() != 20) return;

    const double r = 0.1;
    const double lx = 0.1575;
    const double ly = 0.215;

    // Extract frequencies from specific byte pairs
    double freq3 = static_cast<int16_t>(
        (static_cast<int16_t>(msg->data[7]) << 8) & 0xFFFF |
        (static_cast<int16_t>(msg->data[8]) & 0xFF)) / 10000.;
    double freq4 = static_cast<int16_t>(
        (static_cast<int16_t>(msg->data[9]) << 8) & 0xFFFF |
        (static_cast<int16_t>(msg->data[10]) & 0xFF)) / 10000.;
    double freq2 = static_cast<int16_t>(
        (static_cast<int16_t>(msg->data[11]) << 8) & 0xFFFF |
        (static_cast<int16_t>(msg->data[12]) & 0xFF)) / 10000.;    
    double freq1 = static_cast<int16_t>(
        (static_cast<int16_t>(msg->data[13]) << 8) & 0xFFFF |
        (static_cast<int16_t>(msg->data[14]) & 0xFF)) / 10000.;    
            
    // Mecanum wheel kinematics conversion
    geometry_msgs::msg::Twist twist;
    twist.linear.x = r * (freq1 + freq2 + freq3 + freq4) / 4.0;
    twist.linear.y = r * (-freq1 + freq2 + freq3 - freq4) / 4.0;
    twist.angular.z = r * (-freq1 + freq2 - freq3 + freq4) / (4.0 * (lx + ly));

    cmd_vel_pub_->publish(twist);
}

void SerialGzAdapterNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (prev_joint_state_.name.empty()) {
        prev_joint_state_ = *msg;
        prev_time_ = now();
        return;
    }

    auto current_time = now();
    double dt = (current_time - prev_time_).seconds();

    std::vector<int16_t> wheel_velocities; // in rev/s
    for (size_t i = 1; i <= 4; ++i) {
        // double velocity = (msg->position[i] - prev_joint_state_.position[i]) / (dt * 2 * M_PI);
        wheel_velocities.push_back(msg->velocity[i] / (2 * M_PI) * 10000);
    }

    // char data[39];
    // data[13] = static_cast<char>((wheel_velocities[2] >> 8) & 0xFF);
    // data[14] = static_cast<char>(wheel_velocities[2] & 0xFF);
    // data[15] = static_cast<char>((wheel_velocities[3] >> 8) & 0xFF);
    // data[16] = static_cast<char>(wheel_velocities[3] & 0xFF);
    // data[17] = static_cast<char>((wheel_velocities[1] >> 8) & 0xFF);
    // data[18] = static_cast<char>(wheel_velocities[1] & 0xFF);
    // data[19] = static_cast<char>((wheel_velocities[0] >> 8) & 0xFF);
    // data[20] = static_cast<char>(wheel_velocities[0] & 0xFF);

    // auto serial_msg = glados_hardware::msg::Int8Array();
    // serial_msg.data = std::string(data);

    auto serial_msg = glados_hardware::msg::Int8Array();    
    serial_msg.data = std::vector<uint8_t>{
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        static_cast<uint8_t>((wheel_velocities[2] >> 8) & 0xFF),
        static_cast<uint8_t>(wheel_velocities[2] & 0xFF),
        static_cast<uint8_t>((wheel_velocities[3] >> 8) & 0xFF),
        static_cast<uint8_t>(wheel_velocities[3] & 0xFF),
        static_cast<uint8_t>((wheel_velocities[1] >> 8) & 0xFF),
        static_cast<uint8_t>(wheel_velocities[1] & 0xFF),
        static_cast<uint8_t>((wheel_velocities[0] >> 8) & 0xFF),
        static_cast<uint8_t>(wheel_velocities[0] & 0xFF),
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    serial_read_pub_->publish(serial_msg);

    prev_joint_state_ = *msg;
    prev_time_ = current_time;
}

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialGzAdapterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}