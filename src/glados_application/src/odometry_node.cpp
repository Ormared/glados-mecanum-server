#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <string>

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode() : Node("odometry_node"), x_(0.0), y_(0.0), theta_(0.0)
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "wheel_frequencies", 10,
            std::bind(&OdometryNode::processWheelFrequencies, this, std::placeholders::_1));
        
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        last_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Odometry Node initialized.");
    }

private:
    void processWheelFrequencies(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto data = msg->data;

        // Extract and calculate frequencies
        double freq1 = data[2]*2*M_PI;
        double freq2 = data[3]*2*M_PI;
        double freq3 = data[1]*2*M_PI;
        double freq4 = data[0]*2*M_PI;

        // Constants for Mecanum Kinematics
        const double r = 0.1;  // Wheel radius
        const double l_x = 0.1575;  // Distance from the center of the robot to the wheels along the x-axis
        const double l_y = 0.215;   // Distance from the center of the robot to the wheels along the y-axis



        // Calculate linear and angular velocities
        double vx = r * (freq1 + freq2 + freq3 + freq4) / 4.0;
        double vy = r * (-freq1 + freq2 + freq3 - freq4) / 4.0;
        RCLCPP_INFO(this->get_logger(), "vx: %f, vy: %f", vx, vy);
        double omega = r * (-freq1 + freq2 - freq3 + freq4) / (4.0 * (l_x + l_y));

        updateOdometry(vx, vy, omega);
    }

    void updateOdometry(double vx, double vy, double omega)
    {
        auto current_time = this->now();
        RCLCPP_INFO(this->get_logger(), "current_time: %f, last_time %f", current_time.seconds(), last_time_.seconds());
        // double dt = (current_time - last_time_).nanoseconds();
        double dt = (current_time - last_time_).seconds();

        // Update pose using velocity and time
        // double delta_x = (vx * cos(theta_) - vy * sin(theta_)) * 1e-9 * dt;
        // double delta_y = (vx * sin(theta_) + vy * cos(theta_)) * 1e-9 * dt;
        double delta_x = (vx * cos(theta_) - vy * sin(theta_)) *  dt;
        double delta_y = (vx * sin(theta_) + vy * cos(theta_)) *  dt;
        double delta_theta = omega / dt;
        
        x_ += delta_x;
        y_ += delta_y;
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, theta: %f", x_, y_, theta_);
        theta_ += delta_theta;

        publishOdometry(vx, vy, omega, current_time);
        last_time_ = current_time;
    }

    void publishOdometry(double vx, double vy, double omega, rclcpp::Time current_time)
    {
        // Create odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";

        // Set the position
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        // Set the orientation (quaternion from theta_)
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // Set the velocities
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.angular.z = omega;

        // Publish odometry
        odom_publisher_->publish(odom_msg);

        // Publish the transform
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = current_time;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_footprint";

        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        // transform.transform.translation.x = 0;
        // transform.transform.translation.y = 0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = odom_msg.pose.pose.orientation;

        tf_broadcaster_->sendTransform(transform);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double x_, y_, theta_; // Robot's pose
    rclcpp::Time last_time_; // Last time stamp for odometry calculation
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}
