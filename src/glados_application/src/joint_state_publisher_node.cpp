#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>
#include <cmath>

#define _USE_MATH_DEFINES

class JointStatePublisherNode : public rclcpp::Node
{
public:
    JointStatePublisherNode() : Node("joint_state_publisher_node"), last_time_(this->now())
    {
        // Subscription to receive wheel velocities in rev/s
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "wheel_frequencies", 10,
            std::bind(&JointStatePublisherNode::convertAndPublishJointStates, this, std::placeholders::_1));
        
        // Publisher for joint states
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Initialize joint names
        joint_names_ = {"front_left_wheel_joint", "front_right_wheel_joint", 
                        "rear_left_wheel_joint", "rear_right_wheel_joint"};

        // Initialize wheel angles
        accumulated_angles_ = std::vector<double>(4, 0.0);

        last_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Wheel to Joint State Node initialized.");
    }

private:
    void convertAndPublishJointStates(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 4)
        {
            RCLCPP_WARN(this->get_logger(), "Expected 4 wheel velocities, but got %zu", msg->data.size());
            return;
        }

        auto current_time = this->now();
        double dt = (current_time - last_time_).nanoseconds(); // Time difference in seconds
        last_time_ = current_time;

        // Convert linear velocities (m/s) to angular velocities (rad/s)
        std::vector<double> angular_velocities(4);
        for (size_t i = 0; i < msg->data.size(); ++i)
        {
            angular_velocities[i] = msg->data[i] * 2 * M_PI;

            // Accumulate the rotation angle (angular velocity * time delta)
            accumulated_angles_[i] += angular_velocities[i] * 1e-9 * dt;
        }

        // Publish joint states
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = current_time;
        joint_state_msg.name = joint_names_;
        joint_state_msg.position = accumulated_angles_; // Accumulated angles

        joint_state_publisher_->publish(joint_state_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

    std::vector<std::string> joint_names_;
    std::vector<double> accumulated_angles_; // Store accumulated angles for each wheel

    rclcpp::Time last_time_; // Last time stamp for delta time calculation
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisherNode>());
    rclcpp::shutdown();
    return 0;
}
