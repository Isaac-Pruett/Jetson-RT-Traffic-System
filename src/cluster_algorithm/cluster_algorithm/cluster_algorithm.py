#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pandas as pd
from sklearn.cluster import KMeans
import os

class KMeansClusteringNode(Node):
    def __init__(self):
        super().__init__('kmeans_clustering_node')
        
        # Declare parameters
        self.declare_parameter('input_csv', 'csv/base3.csv')
        self.declare_parameter('output_csv', 'csv/base_clustered3.csv')
        self.declare_parameter('n_clusters', 2)
        self.declare_parameter('random_state', 42)
        
        # Get parameters
        self.input_csv = self.get_parameter('input_csv').get_parameter_value().string_value
        self.output_csv = self.get_parameter('output_csv').get_parameter_value().string_value
        self.n_clusters = self.get_parameter('n_clusters').get_parameter_value().integer_value
        self.random_state = self.get_parameter('random_state').get_parameter_value().integer_value
        
        self.get_logger().info(f'KMeans Clustering Node started')
        self.get_logger().info(f'Input CSV: {self.input_csv}')
        self.get_logger().info(f'Output CSV: {self.output_csv}')
        self.get_logger().info(f'Number of clusters: {self.n_clusters}')
        
        # Run clustering
        self.run_clustering()
        
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
            
            self.get_logger().info(f'Running KMeans clustering with k={self.n_clusters}')
            
            # Perform KMeans clustering
            kmeans = KMeans(n_clusters=self.n_clusters, random_state=self.random_state)
            cluster_labels = kmeans.fit_predict(features)
            
            # Add cluster assignments to dataframe
            df['cluster'] = cluster_labels
            
            # Save to output CSV
            df.to_csv(self.output_csv, index=False)
            
            self.get_logger().info(f'Clustering complete! Output saved to: {self.output_csv}')
            self.get_logger().info(f'Cluster distribution:')
            
            # Print cluster distribution
            for i in range(self.n_clusters):
                count = (cluster_labels == i).sum()
                self.get_logger().info(f'  Cluster {i}: {count} objects')
            
            # Print cluster centers
            self.get_logger().info(f'Cluster centers (initial_x, initial_y, final_x, final_y):')
            for i, center in enumerate(kmeans.cluster_centers_):
                self.get_logger().info(f'  Cluster {i}: [{center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}, {center[3]:.2f}]')
                
        except Exception as e:
            self.get_logger().error(f'Error during clustering: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    
    node = KMeansClusteringNode()
    
    # Since this is a one-time processing node, we can just let it complete
    # and then shutdown
    node.get_logger().info('Clustering task completed. Shutting down node.')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
