#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class DrawingNode : public rclcpp::Node {
public:
    DrawingNode() : Node("detections_img_node") {
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

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        //RCLCPP_INFO(get_logger(), "got a new image");

        try {
            latest_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            try_render();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void box_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        //RCLCPP_INFO(get_logger(), "got a new box arr");
        
        latest_boxes_ = msg;
        
        try_render();
    }

    void try_render() {
        if (!latest_boxes_ || !latest_image_){
            return;
        }

        //RCLCPP_INFO(get_logger(), "try to draw");
        cv::Mat img = latest_image_->image.clone();


        for (const auto &box : latest_boxes_->detections){
            const auto ctr = box.bbox.center.position;
            float w = box.bbox.size_x;
            float h = box.bbox.size_y;
            cv::Point pt1(static_cast<int>(ctr.x - w/2.0f), static_cast<int>(ctr.y - h/2.0f));
            cv::Point pt2(static_cast<int>(ctr.x + w/2.0f), static_cast<int>(ctr.y + h/2.0f));
            //RCLCPP_INFO(get_logger(), "results size: %d", box.results.size());
            // if (box.results.empty()) continue;
            auto b_id = box.results[0].hypothesis.class_id;

            cv::Scalar color;

            //RCLCPP_INFO(get_logger(), "skipped?");

            if (b_id == "car") {
                color = cv::Scalar(0, 0, 255);
            } else if (b_id == "bicycle"){
                color = cv::Scalar(255, 0, 0);
            } else if (b_id == "person"){
                color = cv::Scalar(40, 255, 0);
            } else if (b_id == "road_sign"){
                color = cv::Scalar(60, 187, 255);
            } else {
                return;
            }

            //RCLCPP_INFO(get_logger(), "returned?");

            int thickness_scalar = std::max(1, static_cast<int>(std::floor(latest_image_->image.cols / 960.0f)));

            cv::rectangle(img, pt1, pt2, color, thickness_scalar);

            cv::putText(                                    //adding box text
                img, b_id, cv::Point(pt1.x, pt1.y - 5), 
                cv::FONT_HERSHEY_SIMPLEX, 
                0.5 * thickness_scalar,
                cv::Scalar(255, 255, 255), 
                1 * thickness_scalar
            );
            
            
            cv::circle(img, cv::Point(ctr.x, ctr.y), 2 * thickness_scalar, cv::Scalar(255, 255, 255), -1);  // center dot
            


        }

        auto msg = cv_bridge::CvImage(
            latest_image_->header, sensor_msgs::image_encodings::BGR8, img
        ).toImageMsg();

        //RCLCPP_INFO(get_logger(), "sending img from drawing node");
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
