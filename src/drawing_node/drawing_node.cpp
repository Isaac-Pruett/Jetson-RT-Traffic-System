#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

struct Zone {
    cv::Rect rect;
    bool active;
};

class DrawingNode : public rclcpp::Node {
public:
    DrawingNode() : Node("drawing_node"), drawing(false) {
        RCLCPP_INFO(this->get_logger(), "Drawing node started!");

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 4,
            std::bind(&DrawingNode::image_callback, this, std::placeholders::_1)
        );

        box_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/traffic_detections", 4,
            std::bind(&DrawingNode::box_callback, this, std::placeholders::_1)
        );

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "image_with_zones", 10
        );

        cv::namedWindow("Drawing Node", cv::WINDOW_NORMAL);
        cv::setMouseCallback("Drawing Node", on_mouse_static, this);
    }

private:
    cv_bridge::CvImagePtr latest_image_;
    vision_msgs::msg::Detection2DArray::SharedPtr latest_boxes_;
    std::vector<Zone> zones;

    bool drawing;
    cv::Point start_pt, end_pt;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr box_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    static void on_mouse_static(int event, int x, int y, int, void *userdata) {
        static_cast<DrawingNode*>(userdata)->on_mouse(event, x, y);
    }

    void on_mouse(int event, int x, int y) {
        if (event == cv::EVENT_LBUTTONDOWN) {
            drawing = true;
            start_pt = cv::Point(x, y);
        } else if (event == cv::EVENT_MOUSEMOVE && drawing) {
            end_pt = cv::Point(x, y);
        } else if (event == cv::EVENT_LBUTTONUP) {
            drawing = false;
            end_pt = cv::Point(x, y);
            cv::Rect rect(start_pt, end_pt);
            if (rect.width > 10 && rect.height > 10) {
                zones.push_back({rect, false});
                RCLCPP_INFO(this->get_logger(), "Added zone: (%d,%d)-(%d,%d)", rect.x, rect.y, rect.x+rect.width, rect.y+rect.height);
            }
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            latest_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            render();
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void box_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        latest_boxes_ = msg;
        render();
    }

    void render() {
        if (!latest_image_) return;

        cv::Mat frame = latest_image_->image.clone();

        // Draw existing zones
        for (auto &zone : zones) {
            cv::Scalar color = zone.active ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 255, 0);
            cv::rectangle(frame, zone.rect, color, 2);
        }

        // Draw and check car detections
        if (latest_boxes_) {
            for (const auto &det : latest_boxes_->detections) {
                auto &b = det.bbox;
                cv::Point center(b.center.position.x, b.center.position.y);
                cv::circle(frame, center, 3, cv::Scalar(255, 255, 255), -1);

                // Check overlaps
                for (auto &zone : zones) {
                    if (zone.rect.contains(center)) {
                        zone.active = true;
                        RCLCPP_INFO(this->get_logger(),
                                    "Object inside zone (%d,%d,%d,%d)",
                                    zone.rect.x, zone.rect.y,
                                    zone.rect.width, zone.rect.height);
                    } else {
                        zone.active = false;
                    }
                }
            }
        }

        // Draw preview while dragging
        if (drawing) {
            cv::rectangle(frame, start_pt, end_pt, cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow("Drawing Node", frame);
        cv::waitKey(1);

        // Publish the rendered image
        auto msg = cv_bridge::CvImage(latest_image_->header, "bgr8", frame).toImageMsg();
        image_pub_->publish(*msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DrawingNode>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
