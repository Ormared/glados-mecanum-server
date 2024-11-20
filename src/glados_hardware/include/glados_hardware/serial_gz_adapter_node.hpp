#ifndef SERIAL_GZ_ADAPTER_NODE_HPP
#define SERIAL_GZ_ADAPTER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "glados_hardware/msg/int8_array.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>

class SerialGzAdapterNode : public rclcpp::Node {
public:
    SerialGzAdapterNode();

private:
    void serialWriteCallback(const glados_hardware::msg::Int8Array::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Subscription<glados_hardware::msg::Int8Array>::SharedPtr serial_write_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<glados_hardware::msg::Int8Array>::SharedPtr serial_read_pub_;
};

#endif // SERIAL_GZ_ADAPTER_NODE_HPP