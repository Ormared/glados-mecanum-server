#include <iostream>
#include <cstring>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>
#include <memory>
#include <chrono>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

class VoltagePublisher : public rclcpp::Node {
public:
    explicit VoltagePublisher(const std::string& serial_port = "/dev/ttyUSB0");
    ~VoltagePublisher();

private:
    void timer_callback();
    bool setup_serial();
    float read_voltage();
    void cleanup_serial();

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    int serial_fd_;
    std::string serial_port_;
    const int BAUD_RATE = B9600;
    static constexpr int BUFFER_SIZE = 256;
    bool is_serial_connected_ = false;
};

VoltagePublisher::VoltagePublisher(const std::string& serial_port)
    : Node("voltage_publisher"), serial_port_(serial_port) {
    
    // Create publisher
    publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "voltage_measurements", 10);

    // Setup serial connection
    if (setup_serial()) {
        RCLCPP_INFO(this->get_logger(), "Serial connection established on %s", 
            serial_port_.c_str());
        
        // Create timer for reading and publishing data (10Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&VoltagePublisher::timer_callback, this));
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to establish serial connection");
    }
}

VoltagePublisher::~VoltagePublisher() {
    cleanup_serial();
}

bool VoltagePublisher::setup_serial() {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error opening serial port: %s", 
            strerror(errno));
        return false;
    }

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error getting serial port attributes: %s", 
            strerror(errno));
        return false;
    }

    // Configure serial port settings
    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);
    
    tty.c_cflag |= (CLOCAL | CREAD);    // Enable receiver, ignore modem controls
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                 // 8-bit characters
    tty.c_cflag &= ~PARENB;             // No parity bit
    tty.c_cflag &= ~CSTOPB;             // One stop bit
    tty.c_cflag &= ~CRTSCTS;            // No hardware flowcontrol

    // Configure for raw input
    tty.c_lflag &= ~ICANON;             // Disable canonical mode
    tty.c_lflag &= ~ECHO;               // Disable echo
    tty.c_lflag &= ~ECHOE;              // Disable erasure
    tty.c_lflag &= ~ECHONL;             // Disable new-line echo
    tty.c_lflag &= ~ISIG;               // Disable interpretation of INTR, QUIT and SUSP

    // Configure for raw output
    tty.c_oflag &= ~OPOST;              // Prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR;              // Prevent conversion of newline to carriage return/line feed

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error setting serial port attributes: %s", 
            strerror(errno));
        return false;
    }

    is_serial_connected_ = true;
    return true;
}

float VoltagePublisher::read_voltage() {
    char buffer[BUFFER_SIZE];
    int n = read(serial_fd_, buffer, sizeof(buffer) - 1);
    
    if (n > 0) {
        buffer[n] = '\0';
        try {
            return std::stof(buffer);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to convert serial data to float: %s", 
                e.what());
        }
    }
    return -1.0f;  // Return invalid value if read fails
}

void VoltagePublisher::timer_callback() {
    if (!is_serial_connected_) {
        return;
    }

    float voltage = read_voltage();
    if (voltage >= 0.0f) {  // Valid reading
        auto message = std_msgs::msg::Float32();
        message.data = voltage;
        publisher_->publish(message);
        RCLCPP_DEBUG(this->get_logger(), "Published voltage: %.2fV", voltage);
    }
}

void VoltagePublisher::cleanup_serial() {
    if (is_serial_connected_) {
        close(serial_fd_);
        is_serial_connected_ = false;
    }
}

// Main function
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VoltagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}