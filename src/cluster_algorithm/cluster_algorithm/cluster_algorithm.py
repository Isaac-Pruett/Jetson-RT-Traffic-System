#!/usr/bin/env python3

import os
import sys
import rclpy
import csv
import pandas as pd
import numpy as np
from rclpy.node import Node
from sklearn.cluster import KMeans
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose
from sklearn.metrics import silhouette_samples, silhouette_score

class KMeansClusteringNode(Node):
    def __init__(self, inputFile, outputFile, ): #n_clusters):
        super().__init__('kmeans_clustering_node')
        self.inputFile = inputFile
        self.outputFile = outputFile
        self.n_clusters = 2
        self.ready = False # Variable to mark "ready to add the next object into cluster(s)"

        # Declare parameters
        self.declare_parameter('input_csv', f"csv/{self.inputFile}.csv")
        self.declare_parameter('output_csv', f"csv/{self.outputFile}.csv")
        
        #self.declare_parameter('n_clusters', int(self.n_clusters))
        self.declare_parameter('random_state', 42)
        
        # Get parameters
        self.input_csv = self.get_parameter('input_csv').get_parameter_value().string_value
        self.output_csv = self.get_parameter('output_csv').get_parameter_value().string_value
        #self.n_clusters = self.get_parameter('n_clusters').get_parameter_value().integer_value
        self.random_state = self.get_parameter('random_state').get_parameter_value().integer_value
        
        self.get_logger().info(f'KMeans Clustering Node started')
        self.get_logger().info(f'Input CSV: {self.input_csv}')
        self.get_logger().info(f'Output CSV: {self.output_csv}')
        #self.get_logger().info(f'Number of clusters: {self.n_clusters}')
        
        # Initialize K-mean Algorithm
        self.kmeans = KMeans(n_clusters=self.n_clusters, random_state=self.random_state)
        
    def run_clustering(self):
        try:
            # Check if input file exists
            if not os.path.exists(self.input_csv):
                self.get_logger().error(f'Input file not found: {self.input_csv}')
                return
            
            # Read the CSV file
            self.get_logger().info(f'Reading CSV file: {self.input_csv}')
            df = pd.read_csv(self.input_csv)
            
            # Validate required columns
            required_columns = ['ID', 'x_initial', 'y_initial', 'x_final', 'y_final']
            missing_columns = [col for col in required_columns if col not in df.columns]
            
            if missing_columns:
                self.get_logger().error(f'Missing required columns: {missing_columns}')
                self.get_logger().error(f'Available columns: {list(df.columns)}')
                return
            
            self.get_logger().info(f'Loaded {len(df)} rows from CSV')
            
            # Prepare features for clustering (initial x, initial y, final x, final y)
            features = df[['x_initial', 'y_initial', 'x_final', 'y_final']].values
            
            # Number of clusters to run to find the highest silhouette_score
            highest_silhouette_avg = -1
            highest_n_clusters = 2

            range_n_clusters = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
            for n_cluster in range_n_clusters:
                self.get_logger().info(f'Running KMeans clustering with k={n_cluster}')
                
                # Perform KMeans clustering
                kmeans_x = KMeans(n_cluster, random_state=self.random_state)
                #cluster_labels = self.kmeans.fit_predict(features)
                #cluster_labels = self.kmeans.fit_predict(features)
                cluster_labels = kmeans_x.fit_predict(features)
                
                # Compute the silhoutte_score for the average value for all the samples
                # ==> The density and separation of the formed clusters
                silhouette_avg = silhouette_score(features, cluster_labels)
                self.get_logger().info(f"For n_clusters = {n_cluster}, The average silhouette_score is : {silhouette_avg}")
                
                if (silhouette_avg > highest_silhouette_avg):
                    highest_silhouette_avg = silhouette_avg
                    highest_n_clusters = n_cluster
                    self.n_cluster = n_cluster 
            
            # Run cluster_algorithm with the highest silhouette average score
            # Perform KMeans clustering
            self.kmeans = KMeans(highest_n_clusters, random_state=self.random_state)
            cluster_labels = self.kmeans.fit_predict(features)

            # Add cluster assignments to dataframe
            df['cluster'] = cluster_labels
            
            # Save to output CSV
            df.to_csv(self.output_csv, index=False)
            
            self.get_logger().info(f'Clustering complete! Output saved to: {self.output_csv}')
            self.get_logger().info(f'Cluster distribution:')
            
            # Print cluster distribution
            for i in range(highest_n_clusters):
                count = (cluster_labels == i).sum()
                self.get_logger().info(f'  Cluster {i}: {count} objects')
            
            # Print cluster centers
            self.get_logger().info(f'Cluster centers (initial_x, initial_y, final_x, final_y):')
            for i, center in enumerate(self.kmeans.cluster_centers_):
                self.get_logger().info(f'  Cluster {i}: [{center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}, {center[3]:.2f}]') 
            
            # Mark it ready to continously add more vehicles past 100th
            self.ready = True
            self.get_logger().info("Ready to group clusterings...")

        except Exception as e:
            self.get_logger().error(f'Error during clustering: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    # A function to start subscribing
    def start_subscriber(self):
        # Subscribe to /tracked_object_summary
        self.sub_ = self.create_subscription(Detection2DArray, '/tracked_object_summary', self.callback, 10)

    def callback(self, msg):
        if not self.ready:
            return # ignore early messages
        
        # Loop through the detected topic message and write them to csdv
        for detection in msg.detections:
            class_id  = detection.results[0].hypothesis.class_id
            object_id = detection.id
            confidence = detection.results[0].hypothesis.score

            # Grab x,y initials and finals
            x_initial = detection.results[0].pose.pose.position.x
            y_initial = detection.results[0].pose.pose.position.y
            x_final = detection.bbox.center.position.x
            y_final = detection.bbox.center.position.y
            
            # Only write the objets with confidence level greater than 50%
            if (confidence >= 0.5):
                self.get_logger().info(f"Adding {class_id} ID #{object_id}")
                new_data = np.array([[x_initial, y_initial, x_final, y_final]])
                new_cluster_label = self.kmeans.predict(new_data)
                new_csv_data = [class_id, object_id, f"{confidence:.4f}", int(x_initial), int(y_initial), int(x_final), int(y_final), new_cluster_label[0]]
                with open(self.output_csv, 'a', newline='') as file:
                    write = csv.writer(file)
                    write.writerow(new_csv_data)


def main(args=None):
    # Parse command-line
    if (len(sys.argv) > 1):
        print(f"length of argv {len(sys.argv)}")
        inputFile = sys.argv[1]
        outputFile = sys.argv[2]
    
    rclpy.init(args=args)
    node = KMeansClusteringNode(inputFile, outputFile)#, n_clusters)
    node.run_clustering() # run the clutering algo
    node.start_subscriber() # start subscibing
    rclpy.spin(node) # run callback

    # Since this is a one-time processing node, we can just let it complete
    # and then shutdown
    node.get_logger().info('Clustering task completed. Shutting down node.')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
