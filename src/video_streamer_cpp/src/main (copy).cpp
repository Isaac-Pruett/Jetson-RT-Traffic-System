#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VideoStreamer : public rclcpp::Node
{
public:
    VideoStreamer() : Node("video_streamer")
    {
        declare_parameter<std::string>("video_filepath", "");
        declare_parameter<int>("fps", 20);      // How fast we PUBLISH (Processing Speed)
        declare_parameter<int>("frame_skip", 0); // How many frames to BURN (Playback Speed Fix)
        declare_parameter("width", 960);
        declare_parameter("height", 544);
        
        width_ = get_parameter("width").as_int();
        height_ = get_parameter("height").as_int();
        target_fps_ = get_parameter("fps").as_int();
        manual_skip_ = get_parameter("frame_skip").as_int();

        std::string video_path = get_parameter("video_filepath").as_string();
        cap_.open(video_path);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open video file: %s", video_path.c_str());
            throw std::runtime_error("Failed to open video file");
        }

        RCLCPP_INFO(get_logger(), "Streaming %s", video_path.c_str());
        RCLCPP_INFO(get_logger(), "Target FPS: %d | Skipping: %d frames per loop", target_fps_, manual_skip_);

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "image_raw", 
            rclcpp::SensorDataQoS() // Best Effort
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000/target_fps_),  
            std::bind(&VideoStreamer::publish_frame, this)
        );
    }

private:
    void publish_frame()
    {
        // 1. SKIP (Fast Forward)
        // We burn 'manual_skip_' frames to catch up to real time
        for (int i = 0; i < manual_skip_; i++) {
            if (!cap_.grab()) { 
                handle_eof(); 
                return; 
            }
        }

        // 2. READ (Decode)
        cv::Mat frame;
        if (!cap_.read(frame)) {
            handle_eof();
            return;
        }

        cv::Mat output_frame;
        cv::resize(frame, output_frame, cv::Size(width_, height_));
        
        cv::Mat rgb_frame;
        cv::cvtColor(output_frame, rgb_frame, cv::COLOR_BGR2RGB);

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_link";

        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
           header, "rgb8", rgb_frame
        ).toImageMsg();

        image_pub_->publish(*msg);
    }

    void handle_eof() {
        RCLCPP_INFO(this->get_logger(), "Video finished. Looping...");
        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int width_;
    int height_;
    int target_fps_;
    int manual_skip_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoStreamer>());
    rclcpp::shutdown();
    return 0;
}
