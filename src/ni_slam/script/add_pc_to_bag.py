#!/usr/bin/env python

import rospy
import rosbag
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

def depth_image_to_point_cloud(depth_image, color_image, intrinsic_matrix):
    # Get the shape of the depth image
    height, width = depth_image.shape

    # Get the focal lengths and the principal point from the intrinsic matrix
    fx, fy = intrinsic_matrix[0, 0], intrinsic_matrix[1, 1]
    cx, cy = intrinsic_matrix[0, 2], intrinsic_matrix[1, 2]

    # Create an empty list to store the point cloud data
    point_cloud_data = []

    # Iterate through the depth image
    for v in range(height):
        for u in range(width):
            Z = depth_image[v, u]

            if Z > 0:
                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy

                R, G, B = color_image[v, u]
                point_cloud_data.append([X, Y, Z, R, G, B])

    return point_cloud_data

def main():
    # Define the camera intrinsic matrix
    intrinsic_matrix = np.array([
        [481.20, 0, 319.50],
        [0, -480.00, 239.50],
        [0, 0, 1]
    ])

    # Define the input and output rosbag files
    input_rosbag_file = 'icl_ofkt1.bag'
    output_rosbag_file = 'icl_ofkt1_points.bag'

    # Read the input rosbag file
    input_bag = rosbag.Bag(input_rosbag_file, 'r')

    # Create a new rosbag file to store the output data
    output_bag = rosbag.Bag(output_rosbag_file, 'w')

    # Initialize the CV Bridge
    bridge = CvBridge()

    # Iterate through the input rosbag file
    for topic, msg, t in input_bag.read_messages():
        # If the current topic is the depth image
        if topic == '/camera/depth/image':
            depth_image_msg = msg
            depth_image = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding='32FC1')

        # If the current topic is the color image
        elif topic == '/camera/rgb/image_color':
            color_image_msg = msg
            color_image = bridge.imgmsg_to_cv2(color_image_msg, desired_encoding='bgr8')

            # Convert the depth and color images to a point cloud
            point_cloud_data = depth_image_to_point_cloud(depth_image, color_image, intrinsic_matrix)

            # Create a PointCloud2 message
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'camera_link'
            point_cloud_msg = pc2.
