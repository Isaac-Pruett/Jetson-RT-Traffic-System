#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include <cstdlib>
#include <chrono>

float hypot(float x1, float y1, float x2, float y2){
    return std::hypot(x1-x2, y1-y2);
}

float angle(float x1, float y1, float x2, float y2){
    return std::atan2(y1-y2, x1-x2);
}



class BoxVelocityNode : public rclcpp::Node
{
public:
    BoxVelocityNode() : Node("box_velocity_node")
    {
        RCLCPP_INFO(this->get_logger(), "BoxVelocityNode has been started.");
        box_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/traffic_detections", 10,
            std::bind(&BoxVelocityNode::box_callback, this, std::placeholders::_1)
        );
        
        
    }
private:

    void box_callback(const vision_msgs::msg::Detection2DArray::SharedPtr m){

        // first loop count init stuff
        auto msg_time = rclcpp::Time(m->header.stamp);
        if (!prev_time.nanoseconds()){
            prev_time = msg_time;
            
        }

        double dt = (msg_time - prev_time).seconds();
        prev_time = msg_time;

        
        if (chains.size() == 0){
            for (auto &det : m->detections){
                std::vector<vision_msgs::msg::Detection2D> v;
                v.push_back(det);
                chains.push_back(v);
            }
            
            
            return;
        }
        // end first loop
        

        
        // link all chains with new detections, 
        // we parse through all detections in new frame, 
        // and compare the dists to the latest member of each chain
        // then we link our detection to closest chain
        for (auto &c : chains){
            
            auto latest = c.back();
            float d = 999999999999.0f; // impossibly large number by default. 
            vision_msgs::msg::Detection2D* closest = nullptr;

            for (auto &det : m->detections){
                if (det.id != latest.id) continue;
                
                float dist = hypot(latest.bbox.center.position.x, latest.bbox.center.position.y,
                     det.bbox.center.position.x, det.bbox.center.position.y);


                if ((dist < d)){
                    d = dist;
                    closest = &det;
                    
                }




            }

            
            

            if (closest) {
                // add closest to chain
                c.push_back(*closest);

                //erase the closest from the detection pool. we found the match and linked it.
                auto it = std::find_if(m->detections.begin(), m->detections.end(),
                    [closest](const vision_msgs::msg::Detection2D& det) {
                        return &det == closest;
                    });
                if (it != m->detections.end()) {
                    m->detections.erase(it);
                }

            } else {
                RCLCPP_WARN(this->get_logger(), "No matching detection found for ID: %s", latest.id.c_str());
                continue; // skip velocity calc
            }

            // limit the number of items in a chain.
            const size_t max_chain_length = 10; // TODO make this a cli param
            if (c.size() > max_chain_length) {
                c.erase(c.begin());  // Remove oldest
            }

        }

        for (auto &c : chains){
            // get last -> cur velo and track the obj across camera
            if (c.size() >= 2 && dt > 0.0) {
                auto &prev = c[c.size() - 2];
                auto &curr = c[c.size() - 1];

                float dx = curr.bbox.center.position.x - prev.bbox.center.position.x;
                float dy = curr.bbox.center.position.y - prev.bbox.center.position.y;
                float vx = dx / dt;
                float vy = dy / dt;
                float speed = std::hypot(vx, vy);
                float dir = std::atan2(vy, vx);  

                RCLCPP_INFO(this->get_logger(),
                    "Object ID: %s | v = (%.2f, %.2f) px/s | speed = %.2f px/s | angle = %.2f rad",
                    curr.id.c_str(), vx, vy, speed, dir);
            }
        }        







        
    }

    rclcpp::Time prev_time;

    std::vector<std::vector<vision_msgs::msg::Detection2D>> chains;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr box_sub_;
};


int main(int argc, char * argv[]){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoxVelocityNode>());
    rclcpp::shutdown();
    return 0;

}