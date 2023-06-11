#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point
import time
from sensor_msgs.msg import LaserScan
import numpy as np
import math

obstacle_pub = None

def image_callback(msg):
    # Convert ROS Image message to OpenCV image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # Get image dimensions
    height, width = cv_image.shape

    # Calculate the center coordinates
    center_x = width // 2
    center_y = height // 2

    # Define the pixel window size
    window_size = 10

    # Calculate the window boundaries
    xmin = max(0, center_x - window_size // 2)
    ymin = max(0, center_y - window_size // 2)
    xmax = min(width, center_x + window_size // 2)
    ymax = min(height, center_y + window_size // 2)

    # Extract the pixel values within the window
    window_pixels = cv_image[ymin:ymax, xmin:xmax]

    # Flatten the window pixels into a 1D array
    flattened_window = window_pixels.flatten()

    # Calculate the median value
    median_value = np.median(flattened_window)

    if median_value > 0.6:
        rospy.loginfo("Obstacle detected!")
        publishObstacle()
    else:
        rospy.loginfo("Obstacle not detected.")
        
    # Print the median value of the center pixel
    rospy.loginfo("Median value of the center pixel: {}".format(median_value))

def publishObstacle():
    laser_scan_msg = LaserScan()
    
    start_angle = -0.35
    end_angle = 0.35
    # Set the necessary fields of the LaserScan message
    laser_scan_msg.header.stamp = rospy.Time.now()
    laser_scan_msg.header.frame_id = "base_link"
    laser_scan_msg.angle_min = start_angle
    laser_scan_msg.angle_max = end_angle
    laser_scan_msg.angle_increment = 0.01  # Angular resolution in radians

    laser_scan_msg.range_min = 0.0  # Minimum range value
    laser_scan_msg.range_max = 10.0  # Maximum range value

    laser_scan_msg.ranges = []
    laser_scan_msg.intensities = []

    distance = 0.55  # Distance from origin in cm

    # Populate scan ranges
    for angle in np.arange(start_angle, end_angle, 0.01):
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        range_value = math.sqrt(x**2 + y**2)
        laser_scan_msg.ranges.append(range_value)
        laser_scan_msg.intensities.append(1.0)
    
    # Publish the LaserScan message
    obstacle_pub.publish(laser_scan_msg)

def listener():
    global obstacle_pub
    rospy.init_node('depth_image_listener', anonymous=True)
    obstacle_pub = rospy.Publisher("/scan", LaserScan, queue_size=1)
    rospy.Subscriber("/front_cam/image_depth", Image, image_callback)

    while not rospy.is_shutdown():
        publishObstacle()
        time.sleep(2)
    rospy.spin()

if __name__ == '__main__':
    listener()

