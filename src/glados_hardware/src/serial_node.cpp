#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "glados_hardware/msg/int8_array.hpp"  // Custom message header
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <mutex>

class SerialNode : public rclcpp::Node
{
public:
    SerialNode() : Node("serial_node")
    {
        this->declare_parameter<std::string>("port_name", "/dev/ttyS1");
        this->declare_parameter<int>("baud_rate", 57600);
        this->get_parameter("port_name", port_name_);
        this->get_parameter("baud_rate", baud_rate_);

        serial_port_ = open_serial_port(port_name_, baud_rate_);
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_name_.c_str());
            return;
        }

        subscription_ = this->create_subscription<glados_hardware::msg::Int8Array>(
            "serial_write", 10, std::bind(&SerialNode::on_message_received, this, std::placeholders::_1));

        publisher_ = this->create_publisher<glados_hardware::msg::Int8Array>("serial_read", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&SerialNode::write_read_to_serial, this));

        // write_timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(10), std::bind(&SerialNode::write_to_serial, this));

        // read_timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(30), std::bind(&SerialNode::read_from_serial, this));
    }

    ~SerialNode()
    {
        close(serial_port_);
    }

private:
    int open_serial_port(const std::string &port, int baud_rate)
    {
        int fd = open(port.c_str(), O_RDWR | O_NOCTTY);
        if (fd == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial port %s", port.c_str());
            return -1;
        }

        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error getting attributes for %s", port.c_str());
            close(fd);
            return -1;
        }

        // Set baud rate
        cfsetospeed(&tty, baud_rate);
        cfsetispeed(&tty, baud_rate);

        // Configure 8N1 mode
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        // Save tty settings
        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error setting attributes for %s", port.c_str());
            close(fd);
            return -1;
        }

        return fd;
    }
    
    void on_message_received(const glados_hardware::msg::Int8Array::SharedPtr msg)
    {
        // std::lock_guard<std::mutex> lock(write_mutex_);
        // if (current_message_.data != msg->data) {
        current_message_ = *msg; // Copy the new message
        has_new_message_ = true;
        // }
    }

    
    void write_read_to_serial(){
        // write_to_serial();
        // read_from_serial();

        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error writing to serial port");
            return;
        }

        if (current_message_.data.empty()) return; // Nothing to write
        
        if(has_new_message_ == false && current_message_.data.size() != 0){
            ssize_t bytes_written = write(serial_port_, current_message_.data.data(), current_message_.data.size());
            if (bytes_written < 0) {
                RCLCPP_ERROR(this->get_logger(), "Error writing to serial port");
            }
        }
        has_new_message_ = false;

        uint8_t buffer[39]; // uint8_t buffer to match the custom message
        ssize_t bytes_read = read(serial_port_, buffer, sizeof(buffer));
        if (bytes_read == 39) {
            auto message = glados_hardware::msg::Int8Array();
            message.data = std::vector<uint8_t>(buffer, buffer + bytes_read);
            publisher_->publish(message);
        }
        // return;
    }

    void write_to_serial()
    {
        if (serial_port_ < 0) return;

        // std::lock_guard<std::mutex> lock(write_mutex_);
        if (current_message_.data.empty()) return; // Nothing to write
        
        if(has_new_message_ == false && current_message_.data.size() != 0){
            ssize_t bytes_written = write(serial_port_, current_message_.data.data(), current_message_.data.size());
            if (bytes_written < 0) {
                RCLCPP_ERROR(this->get_logger(), "Error writing to serial port");
            }
        }
        has_new_message_ = false;
    }

    void read_from_serial() {
        if (serial_port_ < 0) return;

        uint8_t buffer[39]; // uint8_t buffer to match the custom message
        ssize_t bytes_read = read(serial_port_, buffer, sizeof(buffer));
        if (bytes_read == 39) {
            auto message = glados_hardware::msg::Int8Array();
            message.data = std::vector<uint8_t>(buffer, buffer + bytes_read);
            publisher_->publish(message);
        }
        return;
    }

    int serial_port_;
    std::string port_name_;
    int baud_rate_;

    rclcpp::Subscription<glados_hardware::msg::Int8Array>::SharedPtr subscription_;
    rclcpp::Publisher<glados_hardware::msg::Int8Array>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr read_timer_;
    rclcpp::TimerBase::SharedPtr write_timer_;

    glados_hardware::msg::Int8Array current_message_;
    std::mutex write_mutex_;
    bool has_new_message_ = true;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}