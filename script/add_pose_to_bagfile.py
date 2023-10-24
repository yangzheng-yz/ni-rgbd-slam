#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""  For tum data set
This scripts reads a bag file containing RGBD data, adds the corresponding
PointCloud2 messages, and saves it again into a bag file. Optional arguments
allow to select only a portion of the original bag file.
"""

import argparse
import sys
import os

import rospy
import rosbag
import sensor_msgs.msg
from sensor_msgs.msg import Imu
import cv2
from cv_bridge import CvBridge, CvBridgeError
import struct
import tf
from numpy import *
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':
    
    folder = "/home/eee/drones/src/ni_slam/bag/tum/"

    imufile = folder+"fr1_desk2.txt"
    
    inputbag = folder+"fr1_desk2.bag"
    

    file = open(imufile, 'r')
    results = array([map(float, line.split(' ')) for line in file if line.strip() !=""])
    t = results[:,0]
    pos = results[:,1:4]
    q = results[:,4:8]

    bag = rosbag.Bag(inputbag,'a')

    for i in xrange(0,q.shape[0]):

        pose = PoseStamped()

        pose.pose.position.x = pos[i, 0]
        pose.pose.position.y = pos[i, 1]
        pose.pose.position.z = pos[i, 2]

        pose.pose.orientation.x = q[i,0]
        pose.pose.orientation.y = q[i,1]
        pose.pose.orientation.z = q[i,2]
        pose.pose.orientation.w = q[i,3]

        pose.header.frame_id="world"
        pose.header.stamp.secs = int(t[i])
        pose.header.stamp.nsecs = int((t[i]-int(t[i]))*1000000000)

        bag.write('/ground_truth/pose', pose, pose.header.stamp)

        print i
    bag.close()