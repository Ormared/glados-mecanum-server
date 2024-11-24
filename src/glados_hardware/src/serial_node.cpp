#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "glados_hardware/msg/int8_array.hpp"  // Custom message header
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <chrono>

class SerialNode : public rclcpp::Node
{
public:
    SerialNode() : Node("serial_node")
    {
        // Load parameters
        this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 9600);
        this->get_parameter("port_name", port_name_);
        this->get_parameter("baud_rate", baud_rate_);

        // Open the serial port
        serial_port_ = open_serial_port(port_name_, baud_rate_);
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_name_.c_str());
            return;
        }

        // Subscribe to a topic
        subscription_ = this->create_subscription<glados_hardware::msg::Int8Array>(
            "serial_write", 10, std::bind(&SerialNode::save_msg, this, std::placeholders::_1));

        // Publish data from the serial port
        publisher_ = this->create_publisher<glados_hardware::msg::Int8Array>("serial_read", 10);

        // Timer to periodically check the serial port for data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&SerialNode::send_to_serial, this));
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

    void save_msg(const glados_hardware::msg::Int8Array::SharedPtr msg)
    {
        last_time_ = std::chrono::high_resolution_clock::now();

        message_ = msg;
    }

    void send_to_serial() {
        if (serial_port_ < 0) return;
        if (!message_)return;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - last_time_).count() > 100) {
            RCLCPP_WARN(this->get_logger(), "Message timeout");
            message_->data.data()[7] = 0;
            message_->data.data()[8] = 0;
            message_->data.data()[9] = 0;
            message_->data.data()[10] = 0;
            message_->data.data()[11] = 0;
            message_->data.data()[12] = 0;
            message_->data.data()[13] = 0;
            // return;
        }

        ssize_t bytes_written = write(serial_port_, message_->data.data(), message_->data.size());
        if (bytes_written < message_->data.size()) {
            RCLCPP_ERROR(this->get_logger(), "Error writing to serial port");
            return;
        }
        uint8_t buffer[39]; // uint8_t buffer to match the custom message
        ssize_t bytes_read = read(serial_port_, buffer, sizeof(buffer));
        RCLCPP_INFO(this->get_logger(), "Bytes read: %d", bytes_read);

        if (bytes_read > 0) {
            auto message = glados_hardware::msg::Int8Array(); // Replace with your package name
            message.data = std::vector<uint8_t>(buffer, buffer + bytes_read); // Populate uint8[] field
            publisher_->publish(message);
            // message_.reset();
        }
        return;
    }

    int serial_port_;
    std::string port_name_;
    int baud_rate_;

    rclcpp::Subscription<glados_hardware::msg::Int8Array>::SharedPtr subscription_;
    rclcpp::Publisher<glados_hardware::msg::Int8Array>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    glados_hardware::msg::Int8Array::SharedPtr message_;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time_ = std::chrono::high_resolution_clock::now();
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

