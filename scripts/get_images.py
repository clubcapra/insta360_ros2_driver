#!/usr/bin/env python3
import numpy as np
import cv2
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import ament_index_python.packages as ament
import rclpy
from rclpy.parameter import Parameter

class GetImages(Node):    
    def __init__(self):
        super().__init__('image_capture_node')
        self.bridge = CvBridge()
        
        # Get package path using ROS 2's ament package API
        pkg_dir = ament.get_package_share_directory('insta360_ros2_driver')
        self.img_path = os.path.join(pkg_dir, 'image_capture')
        if not os.path.exists(self.img_path):
            os.makedirs(self.img_path)
        
        # Use the ROS 2 parameter server to get the topic name
        self.declare_parameter('topic', '/back_camera_image/compressed')
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        
        # Create subscription for compressed images
        self.sub = self.create_subscription(
            CompressedImage,
            self.topic,
            self.sub_callback,
            10  # queue size
        )
        
        self.img_counter = 0

    def sub_callback(self, msg):
        try:
            # Decode the image from the compressed message
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            window_title = "Image Capture - Press 'q' to quit, 'SPACE' to capture"
            cv2.imshow(window_title, image)

            k = cv2.waitKey(1)
            if k == ord('q'):  # 'q' Pressed
                self.get_logger().info("Closing...")
                rclpy.shutdown()

            elif k % 256 == 32:  # SPACE Pressed
                img_name = "frame_{}.jpg".format(self.img_counter)
                cv2.imwrite(os.path.join(self.img_path, img_name), image)
                self.get_logger().info(f"{img_name} captured!")
                self.img_counter += 1

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
            return

def main(args=None):
    rclpy.init(args=args)
    image_capture = GetImages()
    try:
        rclpy.spin(image_capture)
    except KeyboardInterrupt:
        pass
    finally:
        image_capture.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
