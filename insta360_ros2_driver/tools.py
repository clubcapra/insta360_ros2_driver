import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from builtin_interfaces.msg import Time

bridge = CvBridge()

# Function to split an image into front and back parts
def split_image(image):
    height, width = image.shape[:2]
    mid_point = width // 2
    front_image = image[:, :mid_point]
    back_image = image[:, mid_point:]
    return front_image, back_image

# Function to undistort an image based on camera matrix (K) and distortion coefficients (D)
def undistort_image(image, K, D):
    h, w = image.shape[:2]
    new_K = K.copy()
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, (w, h), cv2.CV_32FC1)
    undistorted_img = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR)
    return undistorted_img

# Function to convert a raw OpenCV image into a CompressedImage message
def compress_image_to_msg(image, timestamp):
    # Compress the image using JPEG encoding
    _, buffer = cv2.imencode('.jpg', image)
    image_msg = CompressedImage()
    
    # Ensure timestamp is in ROS2 format
    if isinstance(timestamp, Time):
        image_msg.header.stamp = timestamp
    else:
        # If timestamp is in another format, convert it to ROS2 Time (if necessary)
        time_msg = Time()
        time_msg.sec = int(timestamp)
        time_msg.nanosec = int((timestamp - time_msg.sec) * 1e9)
        image_msg.header.stamp = time_msg
    
    image_msg.format = 'jpeg'
    image_msg.data = buffer.tobytes()
    return image_msg
