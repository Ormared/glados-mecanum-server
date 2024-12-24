#include <iostream>
#include <cstring>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <string>
#include <memory>
#include <chrono>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

class BatteryStatePublisher : public rclcpp::Node {
public:
    explicit BatteryStatePublisher(const std::string& serial_port = "/dev/ttyUSB1");
    ~BatteryStatePublisher();

private:
    void timer_callback();
    bool setup_serial();
    float read_voltage();
    void cleanup_serial();

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    int serial_fd_;
    std::string serial_port_;
    const int BAUD_RATE = B9600;
    static constexpr int BUFFER_SIZE = 256;
    bool is_serial_connected_ = false;
};

BatteryStatePublisher::BatteryStatePublisher(const std::string& serial_port)
    : Node("battery_state_publisher"), serial_port_(serial_port) {
    
    // Create publisher for BatteryState
    publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
        "battery_state", 10);

    // Setup serial connection
    // if (setup_serial()) {
    if (true) {
        RCLCPP_INFO(this->get_logger(), "Serial connection established on %s", 
            serial_port_.c_str());
        
        // Create timer for reading and publishing data (10Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&BatteryStatePublisher::timer_callback, this));
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to establish serial connection");
    }
}

BatteryStatePublisher::~BatteryStatePublisher() {
    cleanup_serial();
}

bool BatteryStatePublisher::setup_serial() {
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
    
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error setting serial port attributes: %s", 
            strerror(errno));
        return false;
    }

    is_serial_connected_ = true;
    return true;
}

float BatteryStatePublisher::read_voltage() {
    float voltage;
    int n = read(serial_fd_, &voltage, sizeof(float));
    
    if (n == sizeof(float)) {
        return voltage;
    }
    
    RCLCPP_WARN(this->get_logger(), "Failed to read voltage");
    return -1.0f;
}

void BatteryStatePublisher::timer_callback() {
    // if (!is_serial_connected_) {
    //     return;
    // }
    float max_voltage = 14.3;

    float voltage = 5.4;         // Voltage in Volts (Mandatory)
    float temperature = NAN;      // Temperature in Degrees Celsius (If unmeasured NaN)
    float current = NAN;          // Negative when discharging (A)  (If unmeasured NaN)
    float design_capacity = 60;  // Capacity in Ah (design capacity)  (If unmeasured NaN)
    float capacity = 58;         // Capacity in Ah (last full capacity)  (If unmeasured NaN)
    float charge = (voltage / max_voltage) * capacity;           // Current charge in Ah  (If unmeasured NaN)
    float percentage = charge / design_capacity;       // Charge percentage on 0 to 1 range  (If unmeasured NaN)
    bool present = true;          // True if the battery is present
    
    // float voltage = read_voltage();
    // if (voltage >= 0.0f) {  // Valid reading
    if (true) {
        auto battery_msg = sensor_msgs::msg::BatteryState();
        
        // Mandatory fields
        battery_msg.voltage = voltage;
        
        // Optional fields set to NaN
        battery_msg.current = current;
        battery_msg.temperature = temperature;
        
        // Capacity-related fields
        battery_msg.charge = charge;
        battery_msg.capacity = capacity;
        battery_msg.design_capacity = design_capacity;
        battery_msg.percentage = percentage;
        
        // Status and health fields
        battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        battery_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        battery_msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
        
        // Additional fields
        battery_msg.present = present;
        
        // Publish the message
        publisher_->publish(battery_msg);
        
        // RCLCPP_DEBUG(this->get_logger(), 
        //     "Published Battery State: V=%.2f", voltage);
    }
}

void BatteryStatePublisher::cleanup_serial() {
    if (is_serial_connected_) {
        close(serial_fd_);
        is_serial_connected_ = false;
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryStatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}