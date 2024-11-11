#!/usr/bin/env python3

import rclpy
import os
from rclpy.node import Node
from rclpy.parameter import Parameter

def verify_directories():
    # ROS 2 uses rclpy for node and parameters
    # Get the ROS package path using rclpy
    node = rclpy.create_node('directory_verification_node')
    default_dir = node.get_package_share_directory('insta360_ros2_driver')
    default_dir = os.path.join(default_dir, 'bag')

    # ROS 2 parameter handling
    raw_bag_folder = node.get_parameter('raw_bag_folder').get_parameter_value().string_value if node.has_parameter('raw_bag_folder') else os.path.join(default_dir, 'raw')
    compressed_bag_folder = node.get_parameter('compressed_bag_folder').get_parameter_value().string_value if node.has_parameter('compressed_bag_folder') else os.path.join(default_dir, 'compressed')
    undistorted_bag_folder = node.get_parameter('undistorted_bag_folder').get_parameter_value().string_value if node.has_parameter('undistorted_bag_folder') else os.path.join(default_dir, 'undistorted')
    
    folders = [raw_bag_folder, compressed_bag_folder, undistorted_bag_folder]

    for folder in folders:
        if not os.path.exists(folder):
            os.makedirs(folder)
            node.get_logger().info(f"Created folder {folder}")

    # Shutdown the node after verifying directories
    rclpy.shutdown()

def main():
    # Initialize the ROS 2 Python client library (rclpy)
    rclpy.init()

    # Call the function to verify directories
    verify_directories()

if __name__ == '__main__':
    main()
