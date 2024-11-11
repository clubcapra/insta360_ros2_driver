#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rosbag2_py
import os
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm
from insta360_ros2_driver.tools import split_image, compress_image_to_msg
from insta360_ros2_driver.directory_verification import verify_directories
import rospkg

class CompressionNode(Node):
    def __init__(self):
        super().__init__('compression_node')
        self.bridge = CvBridge()
        
        # Parameters and topic names
        self.topic_name = '/insta_image_yuv'
        
        verify_directories()

        # Retrieve directories from parameters or set default paths
        default_dir = rospkg.RosPack().get_path('insta360_ros2_driver')
        default_dir = os.path.join(default_dir, 'bag')
        
        raw_bag_folder = self.declare_parameter('raw_bag_folder', os.path.join(default_dir, 'raw')).value
        compressed_bag_folder = self.declare_parameter('compressed_bag_folder', os.path.join(default_dir, 'compressed')).value

        self.get_logger().info(f"Raw Bag Folder: {raw_bag_folder}")
        self.get_logger().info(f"Compressed Bag Folder: {compressed_bag_folder}")
        
        bag_filenames = [f for f in os.listdir(raw_bag_folder) if f.endswith('.db3') and not f.startswith('.')]
        bag_filenames.sort()
        bag_filenames.sort(key=len)

        bag_paths = [os.path.join(raw_bag_folder, filename) for filename in bag_filenames]
        outbag_filenames = [filename.split('.')[0] + '_compressed.db3' for filename in bag_filenames]
        outbag_paths = [os.path.join(compressed_bag_folder, outbag_filename) for outbag_filename in outbag_filenames]
        
        for i in tqdm(range(len(bag_paths))):
            try:
                self.process_bag(bag_paths[i], outbag_paths[i])
            except Exception as e:
                self.get_logger().error(f"Error processing bag file {bag_paths[i]}: {e}")
                continue

    def process_bag(self, input_bag_path, output_bag_path):
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=input_bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        
        reader.open(storage_options, converter_options)
        
        writer = rosbag2_py.SequentialWriter()
        writer_options = rosbag2_py.StorageOptions(uri=output_bag_path, storage_id='sqlite3')
        writer.open(writer_options, converter_options)
        
        topic_types = reader.get_all_topics_and_types()
        for topic_type in topic_types:
            writer.create_topic(topic_type)
        
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            
            if topic == self.topic_name:
                try:
                    msg = self.bridge.deserialize(data, topic_types[self.topic_name])
                    image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                    
                    # Convert YUV image to BGR format and split front and back images
                    bgr_image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
                    front_image, back_image = split_image(bgr_image)
                    
                    # Compress images and create messages
                    front_compressed_msg = compress_image_to_msg(front_image, timestamp)
                    back_compressed_msg = compress_image_to_msg(back_image, timestamp)
                    
                    # Write compressed images to bag
                    writer.write('/front_camera_image/compressed', front_compressed_msg, timestamp)
                    writer.write('/back_camera_image/compressed', back_compressed_msg, timestamp)
                except Exception as e:
                    self.get_logger().error(f"Error processing image data: {e}")
            else:
                writer.write(topic, data, timestamp)

def main(args=None):
    rclpy.init(args=args)
    node = CompressionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
