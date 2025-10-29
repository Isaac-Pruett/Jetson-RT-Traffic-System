#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <map>
#include <cmath>

class DrawingNode : public rclcpp::Node {
public:
    DrawingNode() : Node("detections_img_node"), next_id_(0) {
        RCLCPP_INFO(this->get_logger(), "Drawing node has started up!");

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 4,
            std::bind(&DrawingNode::image_callback, this, std::placeholders::_1)
        );

        box_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/traffic_detections", 4,
            std::bind(&DrawingNode::box_callback, this, std::placeholders::_1)
        );

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_with_bboxes", 10);
    }

private:
    cv_bridge::CvImagePtr latest_image_;
    vision_msgs::msg::Detection2DArray::SharedPtr latest_boxes_;
    
    // Tracking variables
    std::map<int, cv::Point2f> tracked_objects_;  // ID -> last center position
    std::map<int, std::string> tracked_classes_;  // ID -> class name
    int next_id_;
    const float DISTANCE_THRESHOLD = 50.0f;  // pixels - adjust based on your needs

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
        try_render();
    }

    float distance(const cv::Point2f& p1, const cv::Point2f& p2) {
        float dx = p1.x - p2.x;
        float dy = p1.y - p2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    int assign_id(const cv::Point2f& center, const std::string& class_id) {
        // Try to find a matching tracked object
        int best_id = -1;
        float best_distance = DISTANCE_THRESHOLD;

        for (const auto& [id, prev_center] : tracked_objects_) {
            // Only match if same class
            if (tracked_classes_[id] != class_id) continue;
            
            float dist = distance(center, prev_center);
            if (dist < best_distance) {
                best_distance = dist;
                best_id = id;
            }
        }

        if (best_id == -1) {
            // New object - assign new ID
            best_id = next_id_++;
        }

        // Update tracking info
        tracked_objects_[best_id] = center;
        tracked_classes_[best_id] = class_id;

        return best_id;
    }

    void try_render() {
        if (!latest_boxes_ || !latest_image_){
            return;
        }

        cv::Mat img = latest_image_->image.clone();

        // Clear old tracking data and prepare for new frame
        std::map<int, cv::Point2f> new_tracked_objects;
        std::map<int, std::string> new_tracked_classes;

        for (const auto &box : latest_boxes_->detections){
            const auto ctr = box.bbox.center.position;
            float w = box.bbox.size_x;
            float h = box.bbox.size_y;
            cv::Point pt1(static_cast<int>(ctr.x - w/2.0f), static_cast<int>(ctr.y - h/2.0f));
            cv::Point pt2(static_cast<int>(ctr.x + w/2.0f), static_cast<int>(ctr.y + h/2.0f));
            
            if (box.results.empty()) continue;
            
            auto b_id = box.results[0].hypothesis.class_id;
            cv::Point2f center(ctr.x, ctr.y);

            // Assign unique ID to this detection
            int unique_id = assign_id(center, b_id);
            new_tracked_objects[unique_id] = center;
            new_tracked_classes[unique_id] = b_id;

            cv::Scalar color;

            if (b_id == "car") {
                color = cv::Scalar(0, 0, 255);
            } else if (b_id == "bicycle"){
                color = cv::Scalar(255, 0, 0);
            } else if (b_id == "person"){
                color = cv::Scalar(40, 255, 0);
            } else if (b_id == "road_sign"){
                color = cv::Scalar(60, 187, 255);
            } else {
                continue;
            }

            int thickness_scalar = std::max(1, static_cast<int>(std::floor(latest_image_->image.cols / 960.0f)));

            cv::rectangle(img, pt1, pt2, color, thickness_scalar);

            // Create label with ID
            std::string label = b_id + " #" + box.id;  // <-- CHANGED: Use the ID from the detection

            cv::putText(
                img, label, cv::Point(pt1.x, pt1.y - 5), 
                cv::FONT_HERSHEY_SIMPLEX, 
                0.5 * thickness_scalar,
                cv::Scalar(255, 255, 255), 
                1 * thickness_scalar
            );
            
            cv::circle(img, cv::Point(ctr.x, ctr.y), 2 * thickness_scalar, cv::Scalar(255, 255, 255), -1);
        }

        // Update tracked objects for next frame
        tracked_objects_ = new_tracked_objects;
        tracked_classes_ = new_tracked_classes;

        auto msg = cv_bridge::CvImage(
            latest_image_->header, sensor_msgs::image_encodings::BGR8, img
        ).toImageMsg();

        image_pub_->publish(*msg);  

        latest_image_.reset();
        latest_boxes_.reset();
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr box_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawingNode>());
    rclcpp::shutdown();
    return 0;
}