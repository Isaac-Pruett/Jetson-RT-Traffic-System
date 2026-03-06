#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <map>
#include <vector>
#include <deque>
#include <set>

class DrawingNode : public rclcpp::Node {
public:
    DrawingNode() : Node("detections_img_node") {
        RCLCPP_INFO(this->get_logger(), "Drawing node started. Listening for /image_raw and /traffic_detections");

        this->declare_parameter<int>("trail_length", 30);
        max_trail_length_ = this->get_parameter("trail_length").as_int();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&DrawingNode::image_callback, this, std::placeholders::_1));

        box_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/traffic_detections", 10, std::bind(&DrawingNode::box_callback, this, std::placeholders::_1));

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_with_bboxes", 10);
    }

private:
    cv_bridge::CvImagePtr latest_image_;
    vision_msgs::msg::Detection2DArray::SharedPtr latest_boxes_;
    std::map<std::string, std::deque<cv::Point2f>> object_trails_;
    int max_trail_length_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            latest_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            try_render();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void box_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        latest_boxes_ = msg;
        // Don't call try_render here; let the image_callback drive the framerate
    }

    void try_render() {
        if (!latest_image_) return;

        cv::Mat img = latest_image_->image.clone();
        std::set<std::string> current_ids;

        if (latest_boxes_) {
            for (const auto &box : latest_boxes_->detections) {
                if (box.results.empty()) continue;
                
                // --- DEBUG LOG: WHAT IS THE AI SENDING? ---
                auto b_id = box.results[0].hypothesis.class_id;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "AI detected: %s", b_id.c_str());

                const auto ctr = box.bbox.center.position;
                cv::Point2f center(ctr.x, ctr.y);
                current_ids.insert(box.id);
                
                auto& trail = object_trails_[box.id];
                trail.push_back(center);
                if (trail.size() > max_trail_length_) trail.pop_front();

                // Draw Logic (Matches "car", "0", "1", etc.)
                cv::Scalar color = cv::Scalar(0, 255, 0); // Default Green
                if (b_id == "car" || b_id == "0") color = cv::Scalar(0, 0, 255);
                else if (b_id == "person" || b_id == "2") color = cv::Scalar(255, 0, 0);

                cv::Point pt1(ctr.x - box.bbox.size_x/2, ctr.y - box.bbox.size_y/2);
                cv::Point pt2(ctr.x + box.bbox.size_x/2, ctr.y + box.bbox.size_y/2);
                cv::rectangle(img, pt1, pt2, color, 2);
                cv::putText(img, b_id + " #" + box.id, cv::Point(pt1.x, pt1.y - 5), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            }
        }

        auto msg = cv_bridge::CvImage(latest_image_->header, "bgr8", img).toImageMsg();
        image_pub_->publish(*msg);
        
        // IMPORTANT: We do NOT reset latest_boxes_ here so that 
        // detections persist until the next AI update.
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr box_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawingNode>());
    rclcpp::shutdown();
    return 0;
}
