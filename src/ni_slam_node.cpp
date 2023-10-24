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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "ni_slam.h"

using namespace message_filters;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "ni_slam_node");
    
    
    ros::NodeHandle n("~");

    float psr = 70.0;
    int height = 360, width = 480;
    int depth_height = 480, depth_width = 640;
    // int height = 240, width = 360;
    string filename_prefix = "";
    string gt_file_path = "";
    double time_diff = 0.003;
    bool visualization = false, flag_publish_map = false, flag_publish_incremental_keypose_cov = false, flag_publish_twist = false;
    std::vector<double> rotation_imu;
    int queue_size_cloud = 1000, queue_size_imu = 100, queue_size_sync= 500;

    

    if(n.getParam("/axonometric/image_height", height))
        ROS_INFO("Get image_height parameter: %d", height);
    else
        ROS_WARN("Using default height: %d", height);

    if(n.getParam("/axonometric/image_width", width))
        ROS_INFO("Get image_width parameter: %d", width);
    else
        ROS_WARN("Using default width: %d", width);

    if(n.getParam("/depth/image_height", depth_height))
        ROS_INFO("Get depth image_height parameter: %d", depth_height);
    else
        ROS_WARN("Using default depth height: %d", depth_height);

    if(n.getParam("/depth/image_width", depth_width))
        ROS_INFO("Get depth image_width parameter: %d", depth_width);
    else
        ROS_WARN("Using default depth width: %d", depth_width);

    if(n.getParam("saveposes/filename_prefix", filename_prefix))
        ROS_INFO("Get filename prefix: %s", filename_prefix.c_str());
    else
        ROS_WARN("Won't save any files!");

    if(n.getParam("gtposes/file_path", gt_file_path))
        ROS_INFO("Get gt poses file: %s", gt_file_path.c_str());
    else
        ROS_WARN("Won't show any gt poses!");

    if(n.getParam("topic/time_diff", time_diff))
        ROS_INFO("Get max time diff for topics: %f", time_diff);
    else
        ROS_WARN("Using default time diff for topics: %f", time_diff);

    if(n.getParam("train/psr_bound", psr))
        ROS_INFO("Get PSR bound: %f", psr);
    else
        ROS_WARN("Using default PSR bound: %f", psr);

    if(n.getParam("debug/visualization", visualization))
        ROS_INFO("Get Visualization flag: %s",visualization? "true":"false");
    else
        ROS_WARN("Using default Visualization flag: false!");

    if(n.getParam("queue_size/cloud", queue_size_cloud))
        ROS_INFO("Get queue_size_cloud parameter: %d", queue_size_cloud);
    else
        ROS_WARN("Using default queue_size_cloud: %d", queue_size_cloud);

    if(n.getParam("queue_size/imu", queue_size_imu))
        ROS_INFO("Get queue_size_imu parameter: %d", queue_size_imu);
    else
        ROS_WARN("Using default queue_size_imu: %d", queue_size_imu);

    if(n.getParam("queue_size/sync", queue_size_sync))
        ROS_INFO("Get queue_size_sync parameter: %d", queue_size_sync);
    else
        ROS_WARN("Using default queue_size_sync: %d", queue_size_sync);

    NI_SLAM ni_slam(n, height, width, depth_height, depth_width, psr);

    if(n.getParam("debug/is_debugging", ni_slam.is_debugging))
        ROS_INFO("Is debugging: %s",ni_slam.is_debugging? "true":"false");
    else
        ROS_WARN("Using default debugging flag: false!");  

    if(n.getParam("update_modes_real_time", ni_slam.update_modes_real_time))
        ROS_INFO("update modes real time? : %s",ni_slam.update_modes_real_time? "true":"false");
    else
        ROS_WARN("Using default update_modes_real_time flag: false!");  

    if(n.getParam("/rotation/imu", rotation_imu))
    {
        ROS_INFO("Get rotation imu: [%f, %f, %f]",rotation_imu[0], rotation_imu[1], rotation_imu[2]);
        ni_slam.set_rotation_imu(rotation_imu[0], rotation_imu[1], rotation_imu[2]);
    }
    else
    {
        ROS_WARN("Using default rotation from imu: [0, 0, 0]");
        ni_slam.set_rotation_imu(0, 0, 0);
    }

    if(n.getParam("adaptive/mu", ni_slam.adaptive_mu))
        ROS_INFO("Get adaptive parameter mu: %f", ni_slam.adaptive_mu);
    else
        ROS_WARN("Using default adaptive mu: %f", ni_slam.adaptive_mu);

    if(n.getParam("frame_name/parent", ni_slam.parent_frame))
        ROS_INFO("Get parent frame: %s", ni_slam.parent_frame.c_str());
    else
        ROS_WARN("Using default parent frame: %s", ni_slam.parent_frame.c_str());

    if(n.getParam("frame_name/child", ni_slam.child_frame))
        ROS_INFO("Get child frame: %s", ni_slam.child_frame.c_str());
    else
        ROS_WARN("Using default parent frame: %s",ni_slam.child_frame.c_str());

    if(n.getParam("refined_frame/publish_flag", flag_publish_map))
        ROS_INFO("Get refined_frame/publish_flag: %s", flag_publish_map? "true":"false");
    else
        ROS_WARN("Using default refined_frame/publish_flag: false!");

    if(n.getParam("incremental_keypose_with_covariance/publish_flag", flag_publish_incremental_keypose_cov))
        ROS_INFO("Get keypose_with_covariance publish_flag: %s", flag_publish_incremental_keypose_cov?"True":"False");
    else
        ROS_WARN("Using default keypose_with_covariance publish_flag: %s", flag_publish_incremental_keypose_cov?"True":"False");

    if(n.getParam("twist_with_covariance/publish_flag", flag_publish_twist))
        ROS_INFO("Get twist_with_covariance publish_flag: %s", flag_publish_twist?"True":"False");
    else
        ROS_WARN("Using default twist_with_covariance publish_flag: %s", flag_publish_twist?"True":"False");

    if(filename_prefix != "")
        ni_slam.set_file(filename_prefix);

    if(gt_file_path != "")
        ni_slam.set_gt_poses(gt_file_path);
        
    if(visualization == true)
        ni_slam.set_visualization();

    if(flag_publish_map == true)
        ni_slam.set_publish_refined_keyframe();

    if(flag_publish_incremental_keypose_cov == true)
        ni_slam.set_publish_incremental_keypose();

    if(flag_publish_twist == true)
        ni_slam.set_publish_twist();
        
    message_filters::Subscriber<CloudType> cloud_sub(n, "/camera/rgb/pointcloud", queue_size_cloud);

    // message_filters::Subscriber<Imu> imu_sub(n, "/imu/data", queue_size_imu);
    
    message_filters::Subscriber<Image> depth_sub(n, "/camera/depth/image", queue_size_cloud);

    // message_filters::Subscriber<Image> rgb_sub(n, "/camera/rgb/image_color", queue_size_cloud);
   
    // typedef sync_policies::ApproximateTime<CloudType, Imu> SyncPolicy;
    typedef sync_policies::ApproximateTime<CloudType, Image> SyncPolicy;
    
    // Synchronizer<SyncPolicy> sync(SyncPolicy(queue_size_sync), cloud_sub, imu_sub);
    Synchronizer<SyncPolicy> sync(SyncPolicy(queue_size_sync), cloud_sub, depth_sub);
    
    sync.setMaxIntervalDuration(ros::Duration(time_diff));
    
    sync.registerCallback(boost::bind(&NI_SLAM::callback, &ni_slam, _1, _2));
    
    // start timer
    auto start = std::chrono::high_resolution_clock::now();
    
    ros::spin();
    
    // end timer    
    auto end = std::chrono::high_resolution_clock::now();

    // calculate elapsed time
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // calculate average time per image
    // Assuming image_count is the number of images processed
    // double average_time_per_image = static_cast<double>(elapsed.count()) / image_count;

    std::cout << "Total execution time: " << elapsed.count() << " milliseconds.\n";

    ni_slam.ShutDown();
    return 0;
}
