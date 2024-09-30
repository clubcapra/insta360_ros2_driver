#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from insta360_ros_driver.tools import *

class LiveProcessing():
    def __init__(self):
        rospy.init_node('live_processing_node')
        self.bridge = CvBridge()
        
        self.topic_name = '/insta_image_yuv'
        self.undistort = rospy.get_param('undistort', False)
        self.K = np.asarray(rospy.get_param("K", [[1, 0, 0], [0, 1, 0], [0, 0, 1]])).astype(np.float32)
        self.D = np.asarray(rospy.get_param("D", [0, 0, 0, 0])).astype(np.float32)

        self.image_sub = rospy.Subscriber(self.topic_name, Image, self.processing)
        self.front_image_pub = rospy.Publisher('front_camera_image/compressed', CompressedImage, queue_size=10)
        self.back_image_pub = rospy.Publisher('back_camera_image/compressed', CompressedImage, queue_size=10)

        h, w = 1152, 2304  # Hardcoded image size for Insta360 X3
        self.map1_front, self.map2_front = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, np.eye(3), self.K, (w//2, h), cv2.CV_32FC1)
        self.map1_back, self.map2_back = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, np.eye(3), self.K, (w//2, h), cv2.CV_32FC1)

    def processing(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Convert the YUV image to BGR format
            bgr_image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
            print(f"Image Size: ({bgr_image.shape[1]}, {bgr_image.shape[0]})")

            # Assuming the image is horizontally split for Front | Back
            height, width = bgr_image.shape[:2]
            mid_point = width // 2

            front_image = bgr_image[:, :mid_point]
            back_image = bgr_image[:, mid_point:]
        
            #Live Undistortion
            if self.undistort:
                front_image = cv2.remap(front_image, self.map1_front, self.map2_front, interpolation=cv2.INTER_LINEAR)
                back_image = cv2.remap(back_image, self.map1_back, self.map2_back, interpolation=cv2.INTER_LINEAR)

            #Convert to compressed image message
            front_compressed_msg = compress_image_to_msg(front_image, msg.header.stamp)
            back_compressed_msg = compress_image_to_msg(back_image, msg.header.stamp)

            # Publish the compressed images
            self.front_image_pub.publish(front_compressed_msg)
            self.back_image_pub.publish(back_compressed_msg)

        except CvBridgeError as e:
            rospy.logerr("CvBridgeError: %s", e)
        except Exception as e:
            rospy.logerr("Failed to process image: %s", e)

if __name__ == '__main__':
    try:
        live_processing = LiveProcessing()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass