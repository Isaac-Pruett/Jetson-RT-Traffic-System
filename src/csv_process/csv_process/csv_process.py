import sys
import csv
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose
from math import sqrt

# ----- To Run the node -----
# ros2 run inputCSVSubscriber inputCSVSubscriber 

# A node that subscribes to /tracked_object_summary to put data into CSV
class ObjectSubscriber(Node):
    def __init__(self, name):
        super().__init__('tracked_object_subscriber')
        self.name = name
        
        # Subscriber to /tracked_object_summary
        self.sub_ = self.create_subscription(Detection2DArray, '/tracked_object_summary', self.callback, 10)
        self.count = 0

        # Open csv once and add the header
        #self.file = open("csv/test3.csv", "w", newline="")
        self.file = open(f"csv/{self.name}.csv", "w", newline="")
        self.write = csv.writer(self.file)
        self.write.writerow(["Class ID", "ID", "Confidence LV", "x_initial", "y_initial", "x_final", "y_final"])
    
    def callback(self, msg):
        # Shut down the node once the first 500 moving objects are collected
        if (self.count >= 100):
            self.get_logger("100 objects detected! Shutting down...")
            self.file.close()
            rclpy.shutdown()
            return 
        
        # Loop through the detected topic message and write them to the csv
        for detection in msg.detections:
            class_id = detection.results[0].hypothesis.class_id
            confidence = detection.results[0].hypothesis.score
            
            # Grabbing coordinates from the published topic /tracked_object_summary 
            x_initial = detection.results[0].pose.pose.position.x
            y_initial = detection.results[0].pose.pose.position.y

            x_final = detection.bbox.center.position.x
            y_final = detection.bbox.center.position.y
            object_id = detection.id 
        
            # Calculate the Euclidian Distance
            pixel_dist = sqrt((x_final - x_initial)**2 + (y_final - y_initial)**2) 

            # Only write the objects with confidence level greater than 50%
            if (confidence >= 0.5) and (pixel_dist > 100):
                self.count+= 1
                self.get_logger().info(f"Current count: {self.count}")
                self.get_logger().info(f"Detected {class_id} ID#{object_id} at initial: ({x_initial:.1f}, {y_initial:.1f}) | final ({x_final:.1f}, {y_final:.1f})\n")
                self.write.writerow([class_id, object_id, f"{confidence:.4f}", int(x_initial), int(y_initial), int(x_final), int(y_final)])
                # Get the first 100 objects as reference
                if (self.count >= 100):
                    self.get_logger().info("100 moving objects detected!")
                    self.file.close()
                    rclpy.shutdown()
                    return
def main():
    # Parse command-line argument
    if (len(sys.argv) > 1):
        filename = sys.argv[1] # Grab the filename from cmd line
    else:
        filename = "default"

    rclpy.init()
    node = ObjectSubscriber(filename)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
