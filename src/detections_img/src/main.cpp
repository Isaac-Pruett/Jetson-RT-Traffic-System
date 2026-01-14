#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <map>
#include <vector>
#include <deque>

class DrawingNode : public rclcpp::Node {
public:
    DrawingNode() : Node("detections_img_node") {
        RCLCPP_INFO(this->get_logger(), "Drawing node has started up!");

        // Declare parameter for trail length
        this->declare_parameter<int>("trail_length", 30);  // number of points to keep
        max_trail_length_ = this->get_parameter("trail_length").as_int();

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
    
    // Trail tracking: map from object ID to deque of center points
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
        try_render();
    }

    void try_render() {
        if (!latest_boxes_ || !latest_image_){
            return;
        }

        cv::Mat img = latest_image_->image.clone();

        // Track which IDs are present in this frame
        std::set<std::string> current_ids;

        // First pass: update trails and collect current IDs
        for (const auto &box : latest_boxes_->detections){
            if (box.results.empty()) continue;
            
            const auto ctr = box.bbox.center.position;
            cv::Point2f center(ctr.x, ctr.y);
            std::string obj_id = box.id;
            
            current_ids.insert(obj_id);
            
            // Add center point to trail
            auto& trail = object_trails_[obj_id];
            trail.push_back(center);
            
            // Limit trail length
            if (trail.size() > max_trail_length_) {
                trail.pop_front();
            }
        }

        // Remove trails for IDs that are no longer present
        for (auto it = object_trails_.begin(); it != object_trails_.end(); ) {
            if (current_ids.find(it->first) == current_ids.end()) {
                it = object_trails_.erase(it);
            } else {
                ++it;
            }
        }

        // Draw all trails first (so they appear behind bounding boxes)
        for (const auto& [obj_id, trail] : object_trails_) {
            if (trail.size() < 2) continue;
            
            // Draw trail as connected line segments
            for (size_t i = 1; i < trail.size(); ++i) {
                // Optional: fade older points by making them more transparent/thinner
                float alpha = static_cast<float>(i) / trail.size();
                int thickness = std::max(1, static_cast<int>(alpha * 3));
                
                cv::line(img, trail[i-1], trail[i], cv::Scalar(255, 255, 0), thickness);
            }
        }

        // Second pass: draw bounding boxes and labels
        for (const auto &box : latest_boxes_->detections){
            const auto ctr = box.bbox.center.position;
            float w = box.bbox.size_x;
            float h = box.bbox.size_y;
            cv::Point pt1(static_cast<int>(ctr.x - w/2.0f), static_cast<int>(ctr.y - h/2.0f));
            cv::Point pt2(static_cast<int>(ctr.x + w/2.0f), static_cast<int>(ctr.y + h/2.0f));
            
            if (box.results.empty()) continue;
            
            auto b_id = box.results[0].hypothesis.class_id;

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

            // Create label with tracking ID
            std::string label = b_id + " #" + box.id;

            cv::putText(
                img, label, cv::Point(pt1.x, pt1.y - 5), 
                cv::FONT_HERSHEY_SIMPLEX, 
                0.5 * thickness_scalar,
                cv::Scalar(255, 255, 255), 
                1 * thickness_scalar
            );
            
            cv::circle(img, cv::Point(ctr.x, ctr.y), 2 * thickness_scalar, cv::Scalar(255, 255, 255), -1);
        }

        // Draw coordinate reference markers
        // Marker at (0, 0) - Red with white border
        cv::rectangle(img, cv::Point(0, 0), cv::Point(5, 5), cv::Scalar(255, 255, 255), -1);  // White border
        cv::rectangle(img, cv::Point(1, 1), cv::Point(4, 4), cv::Scalar(0, 0, 255), -1);      // Red center
        cv::putText(img, "(0,0)", cv::Point(8, 12), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        
        // Marker at (100, 200) - Green with white border
        cv::rectangle(img, cv::Point(98, 198), cv::Point(103, 203), cv::Scalar(255, 255, 255), -1);  // White border
        cv::rectangle(img, cv::Point(99, 199), cv::Point(102, 202), cv::Scalar(0, 255, 0), -1);      // Green center
        cv::putText(img, "(100,200)", cv::Point(106, 202), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

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