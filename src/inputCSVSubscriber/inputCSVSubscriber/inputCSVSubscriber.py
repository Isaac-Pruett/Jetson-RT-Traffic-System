import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose

# ----- To Run the node -----
# ros2 run inputCSVSubscriber inputCSVSubscriber 

# A node that subscribes to /tracked_object_summary to put data into CSV
class ObjectSubscriber(Node):
    def __init__(self):
        super().__init__('object_subscriber')

        # Subscriber to /tracked_object_summary
        self.sub_ = self.create_subscription(Detection2DArray, '/tracked_object_summary', self.callback, 10)
        
    def callback(self, msg):
        for detection in msg.detections:
            class_id = detection.results[0].hypothesis.class_id
            confidence = detection.results[0].hypothesis.score
            
            # Grabbing coordinates from the published topic /tracked_object_summary
            x_initial = detection.bbox.center.position.x
            y_initial = detection.bbox.center.position.y
    
            x_final = detection.results[1].pose.pose.position.x
            y_final = detection.results[1].pose.pose.position.y

            object_id = detection.id 

            if confidence >= 0.5:
                self.get_logger().info(f"Detected {class_id} ID#{object_id} at initial: ({x_initial:.1f}, {y_initial:.1f}) | final ({x_final}, {y_final})")

def main():
    rclpy.init()
    node = ObjectSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

