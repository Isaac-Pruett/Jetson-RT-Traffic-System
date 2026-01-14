import rclpy
from rclpy.node import Node

import numpy as np
import pandas as pd
from sklearn.cluster import KMeans

from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32MultiArray
import os

class KMeansClusterNode(Node):
    def __init__(self):
        super().__init__('kmeans_cluster_node')

        # ---- PARAMETERS ----
        self.declare_parameter('input_csv', 'training_data.csv')
        self.declare_parameter('output_csv', 'clustered_output.csv')
        self.declare_parameter('num_clusters', 2)
        self.declare_parameter('topic_name', '/new_positions')

        input_csv = self.get_parameter('input_csv').value
        self.output_csv = self.get_parameter('output_csv').value
        num_clusters = self.get_parameter('num_clusters').value
        topic_name = self.get_parameter('topic_name').value

        # ---- LOAD TRAINING DATA ----
        self.get_logger().info(f'Loading training data from {input_csv}')
        df = pd.read_csv(input_csv)

        self.features = df[['x_initial', 'y_initial', 'x_final', 'y_final']].values

        # ---- TRAIN KMEANS ----
        # Initialize the KMeans model
        self.kmeans = KMeans(
            n_clusters=num_clusters,
            random_state=42,
            n_init=10
        )

        # Expect a 2D array
        self.kmeans.fit(self.features) # finds the cluster centers that minimizes the sum of distances between data points and their assigned cluster centers

        self.get_logger().info('KMeans model trained')

        # ---- PREP OUTPUT CSV ----
        if not os.path.exists(self.output_csv):
            out_df = pd.DataFrame(
                columns=[
                    'x_initial',
                    'y_initial',
                    'x_final',
                    'y_final',
                    'cluster_id'
                ]
            )
            out_df.to_csv(self.output_csv, index=False)

        # ---- ROS SUBSCRIBER ----
        self.subscription = self.create_subscription(
            Float32MultiArray,
            topic_name,
            self.callback,
            10
        )

        self.get_logger().info(f'Subscribed to {topic_name}')

    def callback(self, msg: Float32MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn('Expected 4 values: x_i, y_i, x_f, y_f')
            return

        point = np.array(msg.data).reshape(1, -1)

        # ---- PREDICT CLUSTER ----
        cluster_id = int(self.kmeans.predict(point)[0])

        self.get_logger().info(
            f'Point {msg.data} assigned to cluster {cluster_id}'
        )

        # ---- SAVE TO CSV ----
        new_row = pd.DataFrame([{
            'x_initial': msg.data[0],
            'y_initial': msg.data[1],
            'x_final': msg.data[2],
            'y_final': msg.data[3],
            'cluster_id': cluster_id
        }])

        new_row.to_csv(
            self.output_csv,
            mode='a',
            header=False,
            index=False
        )


def main(args=None):
    rclpy.init(args=args)
    node = KMeansClusterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
