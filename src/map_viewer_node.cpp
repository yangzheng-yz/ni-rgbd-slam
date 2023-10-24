// Copyright (c) <2016>, <Nanyang Technological University> All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ni_slam/MapInfo.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "lib.h"

using namespace std;
typedef pcl::PointXYZI GrayPoint;
typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<pcl::PointXYZRGB> ColorCloud;
pcl::visualization::PCLVisualizer viewer ("Map of Non-Iterative SLAM");
float resolution = std::numeric_limits<float>::quiet_NaN();
ColorCloud::Ptr final_map(new ColorCloud());
pcl::VoxelGrid<ColorPoint> voxel;
GrayPoint viewpoint;
Jeffsan::CPPTimer timer;

void callback(const sensor_msgs::ImageConstPtr& color_msg, 
              const sensor_msgs::ImageConstPtr& depth_msg, 
              const ni_slam::MapInfoConstPtr& info)
{
    timer.tic();
    Eigen::Affine3d transform;

    GrayPoint newpoint;
    newpoint.x = info->origin.position.x;
    newpoint.y = info->origin.position.y;
    newpoint.z = info->origin.position.z;
    viewer.addLine(viewpoint, newpoint, 0, 0, 1, to_string(info->header.seq));
    viewpoint = newpoint;

    tf::poseMsgToEigen (info->origin, transform);
    cv::Mat color = cv_bridge::toCvShare(color_msg, "32FC1")->image;
    cv::Mat depth = cv_bridge::toCvShare(depth_msg, "32FC1")->image;
    ColorCloud::Ptr cloud(new ColorCloud());
    unsigned int height_center = info->height/2;
    unsigned int width_center = info->width/2;

    for(unsigned int i = 0; i < info->height; i++)
        for(unsigned int j = 0; j < info->width; j++)
        {
            ColorPoint point;
            point.z = depth.at<float>(i, j);
            if(point.z < 0.2)
                continue;
            point.y = (1.0 * i - height_center) * info->resolution;
            point.x = (1.0 * j - width_center) * info->resolution;
            point.r = color.at<float>(i, j) * 255;
            point.g = color.at<float>(i, j) * 255;
            point.b = color.at<float>(i, j) * 255;
            cloud->push_back(point);
        }

    ColorCloud::Ptr transformed_cloud(new ColorCloud());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    if (!isnan(resolution))
    {
        *final_map += *transformed_cloud;
        ColorCloud::Ptr tmp_cloud(new ColorCloud());
        voxel.setInputCloud( final_map );
        voxel.filter( *tmp_cloud );
        final_map = tmp_cloud;
        viewer.updatePointCloud(final_map, "ni_slam_map");
        cout<<"ddd"<<endl;
    }
    else
        viewer.addPointCloud(transformed_cloud, to_string(info->header.seq));

    ROS_INFO("Added refined cloud ID: %d; Using time %.4fs", info->header.seq, timer.end());
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "ni_slam_map_viewer_node");

    ros::NodeHandle n("~");

    if(n.getParam("/map_visualization/resolution", resolution))
    {
        ROS_WARN("Set fixed visualization resolution parameter: %f", resolution);
        viewer.addPointCloud(final_map, "ni_slam_map");
    }
    else
        ROS_WARN("Using dynamic visualization resolution!");

    message_filters::Subscriber<sensor_msgs::Image> color_sub(n, "/refined/image_color", 10);

    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "/refined/image_depth", 10);
    
    message_filters::Subscriber<ni_slam::MapInfo> info_sub(n, "/refined/map_info", 10);

    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, ni_slam::MapInfo> sync(color_sub, depth_sub, info_sub, 30);

    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    viewer.addCoordinateSystem(0.2, 0);
    
    viewer.setBackgroundColor(1, 1, 1, 0);

    voxel.setLeafSize(resolution, resolution, resolution);

    while(ros::ok())
    {
        ros::spinOnce();
        viewer.spinOnce();
    }

    return 0;
}