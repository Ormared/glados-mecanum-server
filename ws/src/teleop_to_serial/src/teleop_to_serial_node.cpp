#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "serial_communication/msg/int8_array.hpp"  // Custom message for serial communication
#include <cmath>

constexpr uint32_t CRC_POLYNOMIAL = 0x18005;  // Polynomial for CRC calculation
constexpr uint32_t CRC_INITIAL = 0xFFFF;     // Initial value for CRC
constexpr uint32_t CRC_XOR_OUT = 0x0000;     // Final XOR value

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

// Convert Twist (velocity) to frequencies for the motors
std::vector<int16_t> toFreqs(float v_x, float v_y, float omega)
{
    const int MAX_FREQ = 10000;
    const float r = 0.1;  // Wheel radius
    const float l_x = 0.1575;  // Distance from the center of the robot to the wheels along the x-axis
    const float l_y = 0.215;   // Distance from the center of the robot to the wheels along the y-axis

    std::vector<int16_t> freqs(4);

    // Calculate the frequencies for each wheel (simplified kinematics)
    freqs[0] = static_cast<int16_t>(MAX_FREQ * round((1/r) * (v_x - v_y - (l_x + l_y) * omega) / (2 * M_PI)));
    freqs[1] = static_cast<int16_t>(MAX_FREQ * round((1/r) * (v_x + v_y + (l_x + l_y) * omega) / (2 * M_PI)));
    freqs[2] = static_cast<int16_t>(MAX_FREQ * round((1/r) * (v_x + v_y - (l_x + l_y) * omega) / (2 * M_PI)));
    freqs[3] = static_cast<int16_t>(MAX_FREQ * round((1/r) * (v_x - v_y + (l_x + l_y) * omega) / (2 * M_PI)));

    return freqs;
}

class TeleopToSerialNode : public rclcpp::Node
{
public:
    TeleopToSerialNode() : Node("teleop_to_serial_node")
    {
        // Subscription to Twist message (from teleop-twist-joy)
        twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&TeleopToSerialNode::twist_callback, this, std::placeholders::_1));

        // Publisher for Int8Array message
        serial_publisher_ = this->create_publisher<serial_communication::msg::Int8Array>("serial_write", 10);
    }

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const uint8_t DEVICE_ID = 0xa8;
        const uint8_t MESSAGE_LENGTH = 20;
        const uint8_t MESSAGE_ID = 1;

        // Convert Twist data to motor frequencies
        auto freqs = toFreqs(msg->linear.x, msg->linear.y, msg->angular.z);
	// auto freqs = std::vector<uint16_t>( {5000, 5000, 0, 0} );

        // Prepare the serial data message
        std::vector<uint8_t> serial_data = {
            DEVICE_ID, MESSAGE_LENGTH, MESSAGE_ID,
	    0, 0, 0, 0,
            static_cast<uint8_t>((freqs[2] >> 8) & 0xFF), static_cast<uint8_t>(freqs[2] & 0xFF),
            static_cast<uint8_t>((freqs[3] >> 8) & 0xFF), static_cast<uint8_t>(freqs[3] & 0xFF),
            static_cast<uint8_t>((freqs[1] >> 8) & 0xFF), static_cast<uint8_t>(freqs[1] & 0xFF),
            static_cast<uint8_t>((freqs[0] >> 8) & 0xFF), static_cast<uint8_t>(freqs[0] & 0xFF),
            0, 0, 0  // Placeholder bytes for future use
        };

        // Calculate and append CRC to the message
        uint16_t crc = crc16_modbus(serial_data);
        serial_data.push_back(static_cast<uint8_t>(crc & 0xFF));        // CRC low byte
        serial_data.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF)); // CRC high byte

        // Create the Int8Array message and publish it
        auto serial_msg = serial_communication::msg::Int8Array();
        serial_msg.data = serial_data;
        serial_publisher_->publish(serial_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
    rclcpp::Publisher<serial_communication::msg::Int8Array>::SharedPtr serial_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopToSerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

