#!/usr/bin/python
# This is a script for write a sequence of depth images, 
# color images, and corresponding color pointcloud
# into a rosbag file. 
# This is generated by GPT-4 through the following question:
# "我现在有一个数据集，数据集中包含一个序列的深度图和彩色图片，
# 我现在希望把这写深度图和彩色图片写入到rosbag中，
# 并且我还要写入深度图投影后的点云，也就是说rosbag中需要有三个topic，
# 分别是/camera/depth/image, /camera/rgb/image, /camera/rgb/pointcloud，
# 其中点云中的每个点应该即包含xyz的坐标也包含rgb的数值。应该如何用代码实现，python代码"

#!/usr/bin/env python

import rospy
import rosbag
import sensor_msgs.msg
import cv2
import os
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import struct

import open3d as o3d
import time

# def create_point_cloud(depth_image, rgb_image, fx, fy, cx, cy):
#     depth_scaled = depth_image.astype(np.float32)
#     height, width = depth_image.shape

#     x_range = np.arange(0, width, 1)
#     y_range = np.arange(0, height, 1)
#     xv, yv = np.meshgrid(x_range, y_range)
#     x = (xv - cx) * depth_scaled / fx
#     y = (yv - cy) * depth_scaled / fy
#     z = depth_scaled

#     xyz = np.column_stack((x.flatten(), y.flatten(), z.flatten()))
#     # Convert RGB values to a single 32-bit integer
#     rgb = np.zeros((height * width, 1), dtype=np.float32)
#     rgb_view = rgb.view(dtype=np.uint32)
#     rgb_view[:, 0] = rgb_image[..., 0].flatten() << 16 | rgb_image[..., 1].flatten() << 8 | rgb_image[..., 2].flatten()

#     # Create an ordered point cloud with shape (height, width, 6)
#     ordered_point_cloud = np.zeros((height, width, 6), dtype=np.float32)
#     ordered_point_cloud[..., :3] = xyz.reshape(height, width, 3)
#     ordered_point_cloud[..., 3] = rgb.reshape(height, width)
#     # # fx, fy, cx, cy 是相机内参
#     # height, width = depth_image.shape

#     # # 创建点云数据
#     # points_ = np.zeros((height, width, 6)).astype(np.float32)
#     # colors = []
#     # points = []
    
#     # visualizer = o3d.visualization.Visualizer()
#     # visualizer.create_window()
#     # pcd = o3d.geometry.PointCloud()

#     # for v in range(height):
#     #     for u in range(width):
#     #         z = depth_image[v, u] # 如果深度图单位是毫米，需要转换为米
#     #         if z == 0: # 忽略无效深度值
#     #             points_[v,u] = [np.nan, np.nan, np.nan, np.nan]
#     #             continue
            
#     #         x = (u - cx) * z / fx
#     #         y = (v - cy) * z / fy
#     #         color = color_image[v, u]
#     #         rgb = int(color[2]) << 16 | int(color[1]) << 8 | int(color[0])
#     #         points_[v,u] = [x, y, z, rgb]
#     #         points.append([x,y,z,rgb])
#     #         colors.append(color)

#     # points = np.array(points, dtype=np.float32)
#     # colors = np.asarray(colors, dtype=np.float32) / 255.0  # 归一化颜色值

#     # pcd.points = o3d.utility.Vector3dVector(points[:, :3])  # 只取 x, y, z 坐标
#     # pcd.colors = o3d.utility.Vector3dVector(colors)

#     # visualizer.add_geometry(pcd)
#     # visualizer.run()
#     # visualizer.destroy_window()
#     # 定义点云的字段
#     # fields = [
#     #     sensor_msgs.msg.PointField(name='x', offset=0, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1),
#     #     sensor_msgs.msg.PointField(name='y', offset=4, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1),
#     #     sensor_msgs.msg.PointField(name='z', offset=8, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1),
#     #     sensor_msgs.msg.PointField(name='rgb', offset=12, datatype=sensor_msgs.msg.PointField.UINT32, count=1),
#     # ]
#     fields = [
#         pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
#         pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
#         pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
#         pc2.PointField('rgb', 12, pc2.PointField.FLOAT32, 1)
#     ]
#     flattened_points = ordered_point_cloud.reshape(-1, 4)
#     # 创建带有颜色信息的点云
#     return pc2.create_cloud(rospy.Header(frame_id='camera_link'), fields, flattened_points)


# def write_to_rosbag(depth_image_files, color_image_files, output_bag, fx, fy, cx, cy):
#     bridge = CvBridge()
    
#     start_time = 1678417386.01981187
    
#     with rosbag.Bag(output_bag, 'w') as bag:
#         for i, (depth_image_file, color_image_file) in enumerate(zip(depth_image_files, color_image_files)):
            
#             depth_image = cv2.imread(depth_image_file, cv2.IMREAD_UNCHANGED)
#             color_image = cv2.imread(color_image_file)

#             depth_msg = bridge.cv2_to_imgmsg(depth_image, '16UC1')
#             color_msg = bridge.cv2_to_imgmsg(color_image, 'bgr8')
                        
#             depth_msg.header.stamp = rospy.Time(start_time + i * 1.0 / 30)
#             depth_msg.header.frame_id = 'camera_link'
#             color_msg.header.stamp = rospy.Time(start_time + i * 1.0 / 30)
#             color_msg.header.frame_id = 'camera_link'

#             point_cloud_msg = create_point_cloud(depth_image, color_image, fx, fy, cx, cy)
#             point_cloud_msg.header.stamp = rospy.Time(start_time + i * 1.0 / 30)

#             bag.write('/camera/depth/image', depth_msg, depth_msg.header.stamp)
#             bag.write('/camera/rgb/image', color_msg, color_msg.header.stamp)
#             bag.write('/camera/rgb/pointcloud', point_cloud_msg, point_cloud_msg.header.stamp)
#             print("processed %s" % i)


def write_to_rosbag(depth_image_files, color_image_files, output_bag_name, fx, fy, centerX, centerY):
    start_time = 1678417386.01981187
    bridge = CvBridge()
    
    with rosbag.Bag(output_bag_name, 'w') as outbag:
        for i, (depth_image_file, color_image_file) in enumerate(zip(depth_image_files, color_image_files)):
            depth_image = cv2.imread(depth_image_file, cv2.IMREAD_UNCHANGED)
            rgb_image_color = cv2.imread(color_image_file)
            
            depth_msg = bridge.cv2_to_imgmsg(depth_image, '16UC1')
            color_msg = bridge.cv2_to_imgmsg(rgb_image_color, 'bgr8')
                        
            depth_msg.header.stamp = rospy.Time(start_time + i * 1.0 / 30)
            depth_msg.header.frame_id = 'camera_link'
            color_msg.header.stamp = rospy.Time(start_time + i * 1.0 / 30)
            color_msg.header.frame_id = 'camera_link'


            rgb_points = sensor_msgs.msg.PointCloud2()
            rgb_points.header.stamp = rospy.Time(start_time + i * 1.0 / 30)
            rgb_points.header.frame_id = "map"
            rgb_points.width = depth_image.shape[1]
            rgb_points.height  = depth_image.shape[0]
            rgb_points.fields.append(sensor_msgs.msg.PointField(
                name = "x",offset = 0,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
            rgb_points.fields.append(sensor_msgs.msg.PointField(
                name = "y",offset = 4,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
            rgb_points.fields.append(sensor_msgs.msg.PointField(
                name = "z",offset = 8,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
            rgb_points.fields.append(sensor_msgs.msg.PointField(
                name = "rgb",offset = 16,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
            rgb_points.point_step = 32 
            rgb_points.row_step = rgb_points.point_step * rgb_points.width
            buffer = []
            for v in range(rgb_points.height):
                for u in range(rgb_points.width):
                    d = depth_image[v,u] * 0.0002
                    rgb = rgb_image_color[v,u]
                    ptx = (u - centerX) * d / fx
                    pty = (v - centerY) * d / fy
                    ptz = d
                    buffer.append(struct.pack('ffffBBBBIII',
                        ptx,pty,ptz,1.0,
                        rgb[0],rgb[1],rgb[2],0,
                        0,0,0))
            rgb_points.data = b"".join(buffer)
            outbag.write('/camera/depth/image', depth_msg, depth_msg.header.stamp)
            outbag.write('/camera/rgb/image', color_msg, color_msg.header.stamp)
            outbag.write('/camera/rgb/pointcloud', rgb_points, rgb_points.header.stamp)

def GetFilesFromDir(dir):
    # Generates a list of files from the directory
    print( "Searching directory %s" % dir )
    rgb_files = []
    depth_files = []
    if os.path.exists(dir):
        for path, names, files in os.walk(dir + '/rgb'):
            files = ['%06d.png' % float(i[:-4]) for i in files]
            for f in sorted(files):
                rgb_files.append( os.path.join( path, '%s.png' % str(int(f[:-4])) ) )
        for path, names, files in os.walk(dir + '/depth'):
            files = ['%06d.png' % float(i[:-4]) for i in files]
            for f in sorted(files):
                depth_files.append( os.path.join( path, '%s.png' % str(int(f[:-4])) ) )
    return rgb_files, depth_files

if __name__ == '__main__':
    dir = '/home/zheng/datasets/public_dataset/ICL-NUIM/of_kt3'
    rgb_files, depth_files = GetFilesFromDir(dir)
    print(len(rgb_files))
    print(len(depth_files))
    # depth_image_files = [os.path.join(depth_dir, i) for i in depth_files[300:]]
    # color_image_files = [os.path.join(rgb_dir, i) for i in rgb_files[300:]]

    output_bag = 'icl_ofkt3.bag'
    fx, fy, cx, cy = 481.20, -480.00, 319.50, 239.50  # 替换为您的相机内参
    # print(depth_files)
    write_to_rosbag(depth_files[1:], rgb_files[1:], output_bag, fx, fy, cx, cy)