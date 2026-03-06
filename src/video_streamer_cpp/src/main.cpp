#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VideoStreamerNode : public rclcpp::Node {
public:
    VideoStreamerNode() : Node("video_streamer_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);
        
        // 30 FPS Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), 
            std::bind(&VideoStreamerNode::timer_callback, this));
            
        cap_.open("/home/nvidia/Jetson-RT-Traffic-System/video/training_vid.mp4");
    }

private:
    void timer_callback() {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0); // Loop
            return;
        }
        
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now(); // Sync Timestamp
        publisher_->publish(*msg);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoStreamerNode>());
    rclcpp::shutdown();
    return 0;
}
