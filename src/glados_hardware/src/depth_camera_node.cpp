#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <librealsense2/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class RealsenseNode : public rclcpp::Node {
private:
    // RealSense pipeline and configuration
    rs2::pipeline pipeline_;
    rs2::config config_;
    rs2::align align_;

    // Publishers for images and camera info
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;

    // Timer for periodic frame publishing
    rclcpp::TimerBase::SharedPtr timer_;

    // Configurable parameters
    int width_ = 640;
    int height_ = 480;
    int fps_ = 30;

    sensor_msgs::msg::CameraInfo createCameraInfoMsg(
        const rs2_intrinsics& intrinsics, 
        const std::string& frame_id) 
    {
        sensor_msgs::msg::CameraInfo camera_info;
        
        camera_info.header.frame_id = frame_id;
        camera_info.height = intrinsics.height;
        camera_info.width = intrinsics.width;
        
        // Camera matrix (K)
        camera_info.k[0] = intrinsics.fx;  // focal length x
        camera_info.k[2] = intrinsics.ppx; // principal point x
        camera_info.k[4] = intrinsics.fy;  // focal length y
        camera_info.k[5] = intrinsics.ppy; // principal point y
        camera_info.k[8] = 1.0;
        
        // Distortion model and coefficients
        camera_info.distortion_model = "plumb_bob";
        camera_info.d.resize(5);
        for (int i = 0; i < 5; ++i) {
            camera_info.d[i] = intrinsics.coeffs[i];
        }
        
        return camera_info;
    }

    void publishFrames() {
        try {
            // Wait for frames
            rs2::frameset frames = pipeline_.wait_for_frames();
            
            // Align color to depth
            rs2::frameset aligned_frames = align_.process(frames);
            
            // Get depth and color frames
            rs2::frame depth_frame = aligned_frames.get_depth_frame();
            rs2::frame color_frame = aligned_frames.get_color_frame();
            
            if (!depth_frame || !color_frame) {
                RCLCPP_WARN(this->get_logger(), "Failed to get frames");
                return;
            }
            
            // Convert RealSense frames to OpenCV
            cv::Mat depth_image(
                cv::Size(depth_frame.get_width(), depth_frame.get_height()), 
                CV_16UC1, 
                (void*)depth_frame.get_data()
            );
            
            cv::Mat color_image(
                cv::Size(color_frame.get_width(), color_frame.get_height()), 
                CV_8UC3, 
                (void*)color_frame.get_data()
            );
            
            // Current timestamp
            auto current_time = this->get_clock()->now();
            
            // Publish depth image
            auto depth_msg = cv_bridge::CvImage(
                std_msgs::msg::Header(), 
                "16UC1", 
                depth_image
            ).toImageMsg();
            depth_msg->header.stamp = current_time;
            depth_msg->header.frame_id = "camera_depth_optical_frame";
            depth_pub_->publish(*depth_msg);
            
            // Publish depth camera info
            auto depth_intrinsics = depth_frame.get_profile()
                .as<rs2::video_stream_profile>().get_intrinsics();
            auto depth_info = createCameraInfoMsg(
                depth_intrinsics, 
                "camera_depth_optical_frame"
            );
            depth_info.header.stamp = current_time;
            depth_info_pub_->publish(depth_info);
            
            // Publish color image
            auto color_msg = cv_bridge::CvImage(
                std_msgs::msg::Header(), 
                "bgr8", 
                color_image
            ).toImageMsg();
            color_msg->header.stamp = current_time;
            color_msg->header.frame_id = "camera_color_optical_frame";
            color_pub_->publish(*color_msg);
            
            // Publish color camera info
            auto color_intrinsics = color_frame.get_profile()
                .as<rs2::video_stream_profile>().get_intrinsics();
            auto color_info = createCameraInfoMsg(
                color_intrinsics, 
                "camera_color_optical_frame"
            );
            color_info.header.stamp = current_time;
            color_info_pub_->publish(color_info);
        }
        catch (const rs2::error& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "RealSense error: %s", e.what());
        }
        catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "CV Bridge error: %s", e.what());
        }
    }

public:
    RealsenseNode() 
        : Node("realsense_node"),
          align_(RS2_STREAM_COLOR)
    {
        // Declare parameters
        this->declare_parameter("width", 640);
        this->declare_parameter("height", 480);
        this->declare_parameter("fps", 30);
        
        // Get parameters
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_ = this->get_parameter("fps").as_int();

        // Configure RealSense pipeline
        config_.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);
        config_.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_);
        
        try {
            // Start pipeline
            pipeline_.start(config_);
        }
        catch (const rs2::error & e) {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to start RealSense pipeline: %s", e.what());
            rclcpp::shutdown();
            return;
        }
        
        // Create publishers
        color_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "camera/color/image_raw", 10);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "camera/depth/image_raw", 10);
        color_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "camera/color/camera_info", 10);
        depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "camera/depth/camera_info", 10);
        
        // Create timer for periodic frame publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / fps_)),
            std::bind(&RealsenseNode::publishFrames, this)
        );
    }
};

int main(int argc, char** argv) {
    // Initialize ROS2 node
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<RealsenseNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
            "Unhandled exception: %s", e.what());
        return -1;
    }
    
    rclcpp::shutdown();
    return 0;
}