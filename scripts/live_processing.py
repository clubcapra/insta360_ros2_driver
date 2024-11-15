#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

from insta360_ros2_driver.tools import *

class LiveProcessing(Node):
    def __init__(self):
        super().__init__('live_processing_node')

        self.bridge = CvBridge()
        
        K_matrix = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        D_matrix = [0.0, 0.0, 0.0, 0.0]
        
        undistort = self.declare_parameter('undistort', False)
        K = self.declare_parameter("K", K_matrix)  # Default 3x3 matrix flattened to a list
        D = self.declare_parameter("D", D_matrix)

        K_value = K.get_parameter_value().double_array_value
        D_value = D.get_parameter_value().double_array_value
        undistort_value = undistort.get_parameter_value().bool_value
        
        self.get_logger().info(f"Retrieved 'K' parameter: {K_value}")
        self.get_logger().info(f"Retrieved 'D' parameter: {D_value}")
        self.get_logger().info(f"Retrieved 'undistort' parameter: {undistort_value}")
        

        # Parameters for topics, undistortion, and frame IDs
        self.topic_name = '/insta_image_yuv'
        self.undistort = undistort_value
        self.K = np.asarray(K_value).reshape(3, 3)
        self.D = np.asarray(D_value)


        # Define frame IDs for front and back cameras
        self.front_frame_id = 'front_camera_optical_frame'
        self.back_frame_id = 'back_camera_optical_frame'

        distortion_with_balance = False
        balance = 1.0
        image_scale = 1.0

        h, w = 1152, 2304  # Hardcoded image size for Insta360 X3
        if not distortion_with_balance:
            # Generate undistortion maps for front and back cameras with the same resolution
            self.map1_front, self.map2_front = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, np.eye(3), self.K, (w // 2, h), cv2.CV_32FC1)
            self.map1_back, self.map2_back = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, np.eye(3), self.K, (w // 2, h), cv2.CV_32FC1)
            width, height = w // 2, h
        else:
            # Original width and height
            original_width, original_height = w // 2, h
            # Increased resolution
            new_width, new_height = int(original_width * image_scale), int(original_height * image_scale)
            # Estimate new camera matrix for front and back cameras with increased resolution
            new_camera_matrix_front = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(self.K, self.D, (original_width, original_height), np.eye(3), balance=balance, new_size=(new_width, new_height))
            new_camera_matrix_back = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(self.K, self.D, (original_width, original_height), np.eye(3), balance=balance, new_size=(new_width, new_height))
            # Generate undistortion maps with the new 4x resolution
            self.map1_front, self.map2_front = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, np.eye(3), new_camera_matrix_front, (new_width, new_height), cv2.CV_32FC1)
            self.map1_back, self.map2_back = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, np.eye(3), new_camera_matrix_back, (new_width, new_height), cv2.CV_32FC1)
            self.K = new_camera_matrix_front
            self.D = np.zeros((4, 1))
            width, height = new_width, new_height

        # Precompute camera info messages
        self.front_camera_info_msg = self.get_camera_info(width, height, self.K, self.D, self.front_frame_id)
        self.back_camera_info_msg = self.get_camera_info(width, height, self.K, self.D, self.back_frame_id)

        # Image subscribers and publishers
        self.image_sub = self.create_subscription(
            Image,
            self.topic_name,
            self.processing,
            10
        )

        self.front_image_pub = self.create_publisher(Image, 'front_camera_image/image_raw', 10)
        self.back_image_pub = self.create_publisher(Image, 'back_camera_image/image_raw', 10)

        # Camera info publishers
        self.front_camera_info_pub = self.create_publisher(CameraInfo, 'front_camera/camera_info', 10)
        self.back_camera_info_pub = self.create_publisher(CameraInfo, 'back_camera/camera_info', 10)

    def get_camera_info(self, width, height, K, D, frame_id):
        camera_info_msg = CameraInfo()

        # Camera resolution
        camera_info_msg.width = width
        camera_info_msg.height = height

        # Camera intrinsic matrix (K)
        camera_info_msg.k = K.flatten().tolist()

        # Distortion coefficients (D) - Explicitly cast to float
        camera_info_msg.d = [float(d) for d in D.flatten()]

        # Distortion model
        camera_info_msg.distortion_model = "plumb_bob"

        # Rectification matrix (identity for now)
        camera_info_msg.r = np.eye(3).flatten().tolist()

        # Projection matrix (P)
        # In this case, we can set it to [K, [0, 0, 0]]
        camera_info_msg.p = np.hstack((K, np.zeros((3, 1)))).flatten().tolist()

        # Set the frame ID for the camera
        camera_info_msg.header.frame_id = frame_id

        return camera_info_msg

    def processing(self, msg):
        try:
            current_timestamp = msg.header.stamp  # rclpy.time.Time.now()

            # Convert ROS Image message to OpenCV image
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Convert the YUV image to BGR format
            bgr_image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420).astype(np.uint8)

            # Assuming the image is horizontally split for Front | Back
            height, width = bgr_image.shape[:2]
            mid_point = width // 2

            front_image = bgr_image[:, :mid_point]
            back_image = bgr_image[:, mid_point:]

            # Live Undistortion
            if self.undistort:
                interpolation_method = cv2.INTER_LINEAR
                front_image = cv2.remap(front_image, self.map1_front, self.map2_front, interpolation=interpolation_method)
                back_image = cv2.remap(back_image, self.map1_back, self.map2_back, interpolation=interpolation_method)

            # Publish processed images
            self.front_image_pub.publish(self.bridge.cv2_to_imgmsg(front_image, encoding="bgr8"))
            self.back_image_pub.publish(self.bridge.cv2_to_imgmsg(back_image, encoding="bgr8"))
            self.front_camera_info_pub.publish(self.front_camera_info_msg)
            self.back_camera_info_pub.publish(self.back_camera_info_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")

def main(args=None):
    rclpy.init(args=args)

    lp = LiveProcessing()

    rclpy.spin(lp)

    # Clean up and shut down
    lp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
