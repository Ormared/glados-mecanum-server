#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial_communication/msg/int8_array.hpp"  // Custom message header
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <vector>

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
        subscription_ = this->create_subscription<serial_communication::msg::Int8Array>(
            "serial_write", 10, std::bind(&SerialNode::write_to_serial, this, std::placeholders::_1));

        // Publish data from the serial port
        publisher_ = this->create_publisher<std_msgs::msg::String>("serial_read", 10);

        // Timer to periodically check the serial port for data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&SerialNode::read_from_serial, this));
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

    void write_to_serial(const serial_communication::msg::Int8Array::SharedPtr msg)
    {
        if (serial_port_ < 0) return;

        ssize_t bytes_written = write(serial_port_, msg->data.data(), msg->data.size());
        if (bytes_written < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error writing to serial port");
        }
    }

    void read_from_serial()
    {
        if (serial_port_ < 0) return;

        char buffer[256];
        ssize_t bytes_read = read(serial_port_, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            auto message = std_msgs::msg::String();
            message.data = std::string(buffer, bytes_read);
            publisher_->publish(message);
        }
    }

    int serial_port_;
    std::string port_name_;
    int baud_rate_;

    rclcpp::Subscription<serial_communication::msg::Int8Array>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

