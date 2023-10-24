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

#include "ni_slam.h"
#include "debug.h"
#include <time.h>

NI_SLAM::NI_SLAM(ros::NodeHandle n, int height, int width, int depth_height, int depth_width, float psr)
    :_shutdown(false), nh(n), image_transport(n), height(height), width(width), depth_height(depth_height), depth_width(depth_width), initialized(false), train_num(0), psrbound(psr)
{
    // for debug
    flag_save_file = false;
    flag_visualization = false;
    flag_show_gt = false;

    // for mapping and graph optimazation
    flag_publish_refined_keyframe = false;
    flag_publish_incremental_keypose_cov = false;
    flag_publish_twist = false;
    sigma_x2 = 0, sigma_y2 = 0, sigma_z2 = 0;
    
    // parameters for hardware
    len_hist = 50;
    adaptive_mu = 100;
    point_cloud_sample = 5;
    histogram_sequence = ArrayXi::LinSpaced(len_hist, 1, len_hist);
    min_valid_depth = 0.2;
    max_distance = 4.0;
    set_rotation_imu(0, 0, 0);
    
    // initialization
    valid_diff = ArrayXXf::Constant(width, height, 0.1);
    color_empty = ArrayXXf::Constant(width, height, 0.5);
    inliers = ArrayXXb::Constant(width, height, true);
    depth_empty = ArrayXXf::Zero(width, height);
    labels_fft = get_labels_fft();
    child_frame = "ni_slam";
    parent_frame = "world";
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("posestamped", 1000);

    // parameters for translation estimation
    sigma  = 0.2;         // the gaussian_kernel sigma
    lambda = 0.1;         // the regulerazaion tersmall
    small  = 1e-7;        // prevent divison by zero

    // parameters for rotation estimation
    K <<   481.20, 	0, 	319.50,
        0, 	-480.00, 	239.50,
        0, 	0, 	1;  // for ICL_NUIM camera intrinsic parameters
    // K <<   605.2034912109375, 	0, 	326.1383056640625,
    //     0, 	605.503173828125, 	238.05599975585938,
    //     0, 	0, 	1;
    cellSize = 10; // 10;
    maxPyramidLevel = 2;
    normalsCV_last = cv::Mat::zeros(cv::Size(depth_height/maxPyramidLevel, depth_width/maxPyramidLevel), CV_64FC3);
    normalsCV = cv::Mat::zeros(cv::Size(depth_height/maxPyramidLevel, depth_width/maxPyramidLevel), CV_64FC3);
    v.setLinSpaced(depth_height/maxPyramidLevel, 0, 0+(depth_height/maxPyramidLevel-1)*1);
    u.setLinSpaced(depth_width/maxPyramidLevel, 0, 0+(depth_width/maxPyramidLevel-1)*1);
    vMap = v.replicate(1, u.size());
    uMap_t = u.replicate(1, v.size());
    uMap = uMap_t.transpose();
    vertexMapx = Eigen::MatrixXd::Zero(depth_height/maxPyramidLevel, depth_width/maxPyramidLevel);
    vertexMapy = Eigen::MatrixXd::Zero(depth_height/maxPyramidLevel, depth_width/maxPyramidLevel);
    vertexMapx = ((uMap - Eigen::MatrixXd::Ones(uMap.rows(), uMap.cols()) * (K(0,2)*(1.0/maxPyramidLevel)))/(K(0,0)*(1.0/maxPyramidLevel)));
    vertexMapy = ((vMap - Eigen::MatrixXd::Ones(vMap.rows(), vMap.cols()) * (K(1,2)*(1.0/maxPyramidLevel)))/(K(1,1)*(1.0/maxPyramidLevel)));
    
    // for parallel programming
    _normalMap_thread = std::thread(boost::bind(&NI_SLAM::EstimateNormalMapThread, this));
    _rotation_thread = std::thread(boost::bind(&NI_SLAM::EstimateRotationThread, this));
    _translation_thread = std::thread(boost::bind(&NI_SLAM::EstimateTranslationThread, this));

    // further parallel
    // _rotation_part1_thread = std::thread(boost::bind(&NI_SLAM::EstimateRotationPart1Thread, this));
    // _rotation_part2_thread = std::thread(boost::bind(&NI_SLAM::EstimateRotationPart2Thread, this));

}


// void NI_SLAM::callback(const CloudType::ConstPtr& pcl_cloud, const ImuConstPtr& imu, const ImageConstPtr& depth_img)
void NI_SLAM::callback(const CloudType::ConstPtr& pcl_cloud, const ImageConstPtr& depth_img)
{
    // std::cout << "Function 0 is being executed by thread " << std::this_thread::get_id() << std::endl;
    timer.hz("Callback:"); jtimer.tic();
    cout << "--------processing " << nth_frame+1 << ".png-----------" << endl;
    std_msgs::Header ros_header_new = pcl_conversions::fromPCL(pcl_cloud->header);
    time_diff = ros_header_new.stamp.toSec() - ros_header.stamp.toSec();
    ros_header = ros_header_new;
    // check_synchronization(ros_header, imu->header, 0.003);
    // check_synchronization(depth_img->header, ros_header_new, 0.003);

    // ** convert to depth image to show
    cv_bridge::CvImageConstPtr cv_ptr_depth;

    try {
        cv_ptr_depth = cv_bridge::toCvShare(depth_img, sensor_msgs::image_encodings::TYPE_32FC1);
    } 
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat imD = cv_ptr_depth->image;
    if(imD.type() != CV_64FC1)
    {
        imD.convertTo(imD, CV_64FC1);
    }
    cv::resize(imD, imD, cv::Size(), 1.0/maxPyramidLevel, 1.0/maxPyramidLevel);

    // for parallel programming
    InputDataPtr data = std::shared_ptr<InputData>(new InputData());
    data->depth_image = imD.clone();
    data->pcl_cloud = pcl_cloud;
    data->time = nth_frame;

    while(_data_buffer.size()>=10 && !_shutdown){
        usleep(2000);
    }

    // if(nth_frame==0){
    //     t1 = std::chrono::steady_clock::now();
    // }
    // if((nth_frame+1)%5 == 1){
    //     _dataBuffer_mutex.lock();
    //     _data_buffer.push(data);
    //     _dataBuffer_mutex.unlock();
    // }

    _dataBuffer_mutex.lock();
    _data_buffer.push(data);
    _dataBuffer_mutex.unlock();
    nth_frame++;
    
    // for parallel programming end


    // // time_use = jtimer.end();
    // // rotation = tf::Quaternion(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w); // current rotation relative to inital frame
    // // rotation *= rotation_imu; // for ICL_NUIM setting, rotation_imu should be (0,0,0)    
    // // For debugging, real values are stored in acceleration, can be deleted safely.
    // // tf::Vector3 position(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    // // pose_real.setOrigin(position);
    // // pose_real.setRotation(rotation);
    
    // if (initialized == false)
    // {
    //     EfficientDepth2NormalMap(imD, normalsCV, cellSize, vertexMapx, vertexMapy);
    //     rotation_rel = tf::Quaternion(0,0,0,1);
    //     translation_key_rot = rotation_rel;

    //     translation_cur_rot = (translation_key_rot * rotation_rel).normalized(); // current rotation relative to inital frame

    //     init_poses();
        
    //     adapt_field_of_view(pcl_cloud); // need rotation, so should get rotation before this
        
    //     reproject(pcl_cloud);
        
    //     train();
    //     initialized = true;
    //     if(flag_save_file)
    //         save_file();

    //     // if(flag_show_gt)
    //     // {
    //     //     gt_pose_last = gt_poses[nth_frame];
    //     // }
    //     nth_frame++;
    //     return;
    // }

    // EfficientDepth2NormalMap(imD, normalsCV, cellSize, vertexMapx, vertexMapy);
    // cv::Mat AnglesMap = cv::Mat::ones(cv::Size(depth_height/maxPyramidLevel, depth_width/maxPyramidLevel), CV_64FC3)*720;
    // Eigen::Matrix3d rotation_matrix;
    // EfficientNormal2RotationMat(normalsCV_last, normalsCV, rotation_matrix, AnglesMap);
    // Eigen::Quaterniond q_rel(rotation_matrix);
    // rotation_rel = tf::Quaternion(q_rel.coeffs()[0], q_rel.coeffs()[1], q_rel.coeffs()[2], q_rel.coeffs()[3]);
    // translation_cur_rot = (translation_key_rot * rotation_rel).normalized(); // current rotation relative to inital frame

    // reproject(pcl_cloud, tf::Transform(translation_key_rot.inverse()*translation_cur_rot)); // reproject current frame using rotation to key frame coordinates
    // // sleep(1000); 

    // get_response();

    // get_psr();

    // refine_keyclouds_poses();

    // publish_poses();

    // // if (flag_publish_twist)
    // //     publish_twists();

    // // if (flag_publish_incremental_keypose_cov)
    // //     publish_incremental_keypose_cov();

    // // if (!(psr > psrbound))
    // if (!(psr > psrbound) || valid_points<3000 || tf::Transform(translation_key_rot.inverse()*translation_cur_rot).getRotation().getAngle() >= 0.15)
    // {
    //     // cout << "!!!!!!!!!!rotation: " << tf::Transform(rotation_key.inverse()*rotation).getRotation().getAngle() << endl;
    //     // cout << "valid points: " << valid_points << endl;
    //     // if(flag_publish_refined_keyframe)
    //     //     publish_keyframe();

    //     adapt_field_of_view(pcl_cloud);

    //     reproject(pcl_cloud); // reproject current frame to own coordinate for training
        
    //     train();

    //     // if(flag_show_gt)
    //     // {
    //     //     Eigen::Matrix3d R_gt_last;
    //     //     Eigen::Quaterniond q_gt_last(gt_pose_last[6], gt_pose_last[3], gt_pose_last[4], gt_pose_last[5]);
    //     //     R_gt_last = q_gt_last.toRotationMatrix();
    //     //     Eigen::Matrix3d R_gt_cur;
    //     //     Eigen::Quaterniond q_gt_cur(gt_poses[nth_frame][6], gt_poses[nth_frame][3], gt_poses[nth_frame][4], gt_poses[nth_frame][5]);
    //     //     R_gt_cur = q_gt_cur.toRotationMatrix();
    //     //     Eigen::Matrix3d R_gt_rlt;
    //     //     R_gt_rlt = R_gt_last.transpose() * R_gt_cur;
    //     //     Eigen::AngleAxisd gt_angleaxis;
    //     //     gt_angleaxis.fromRotationMatrix(R_gt_rlt);
    //     //     cout << "gt axis-angle: \n" << gt_angleaxis.angle() * gt_angleaxis.axis() << endl;
    //     //     cout << "gt_pose_last: " << gt_pose_last[3] << ", " << gt_pose_last[4] << ", " << gt_pose_last[5] << ", " << gt_pose_last[6] << ", " << endl;
    //     //     cout << "gt_pose: " << gt_poses[nth_frame][3] << ", " << gt_poses[nth_frame][4] << ", " << gt_poses[nth_frame][5] << ", " << gt_poses[nth_frame][6] << ", " << endl;
    //     //     gt_pose_last = gt_poses[nth_frame]; // shoule make sure the nth_frameth gt pose corresponding to the nth_frame.png
    //     // }

    //     ROS_WARN("Trained. %d times with PSR: %.1f............", train_num++, psr);
    //     if(flag_save_file)
    //         save_file();
    // }

    // time_use = jtimer.end();

    // ROS_INFO("(%td,%td); Res: %5fm  Timing: %.4fs = %.2fHz; Dt: %.4fs PSR: %04.1f;", 
    //     max_index[0], max_index[1], resolution, time_use, 1/time_use, time_diff, psr);

    // // if(flag_save_file)
    // //     save_file();

    // if(flag_visualization)
    //     show();
    
    // nth_frame++;
}

void NI_SLAM::EstimateNormalMapThread(){
    while(!_shutdown){
        // std::cout << "Function 1 is being executed by thread " << std::this_thread::get_id() << std::endl;
        if(_data_buffer.empty()){
            usleep(2000);
            continue;
        }

        // count time
        auto start = std::chrono::high_resolution_clock::now();

        InputDataPtr input_data;
        _dataBuffer_mutex.lock();
        input_data = _data_buffer.front();
        _data_buffer.pop();
        _dataBuffer_mutex.unlock();

        int frame_id = input_data->time;
        cv::Mat depth_image = input_data->depth_image.clone();



        normalsCV = cv::Mat::zeros(cv::Size(depth_height/maxPyramidLevel, depth_width/maxPyramidLevel), CV_64FC3);
        EfficientDepth2NormalMap(depth_image, normalsCV, cellSize, vertexMapx, vertexMapy);

        NormalMapPtr normalMap_data = std::shared_ptr<NormalMap>(new NormalMap());
        normalMap_data->start_time = t1;
        normalMap_data->time = frame_id;
        normalMap_data->normal_map = normalsCV.clone();
        normalMap_data->pcl_cloud = input_data->pcl_cloud;


        // count time 
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "Thread [EstimateNormalMapThread] took " << elapsed.count() << " milliseconds to execute.\n";



        while(_normalMap_buffer.size()>=3){
            usleep(2000);
        }

        _normalMap_mutex.lock();
        _normalMap_buffer.push(normalMap_data);
        _normalMap_mutex.unlock();
    }
}

void NI_SLAM::EstimateRotationThread(){
    while(!_shutdown){
        // std::cout << "Function 2 is being executed by thread " << std::this_thread::get_id() << std::endl;
        if(_normalMap_buffer.empty()){
            usleep(2000);
            continue;
        }

        // count time
        auto start = std::chrono::high_resolution_clock::now();

        NormalMapPtr normal_map;
        _normalMap_mutex.lock();
        normal_map = _normalMap_buffer.front();
        _normalMap_buffer.pop();
        _normalMap_mutex.unlock();

        int frame_id = normal_map->time;
        

        FramePtr frame = std::shared_ptr<Frame>(new Frame(frame_id, normal_map->normal_map, normal_map->pcl_cloud));
        frame->start_time = normal_map->start_time;

        if(!initialized){
            rotation_rel = tf::Quaternion(0,0,0,1);
            rotation_key_rot = rotation_rel;
            normalsCV_last = normal_map->normal_map.clone();
            // pcl_cloud_last = normal_map->pcl_cloud;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*normal_map->pcl_cloud, *temp);
            pcl_cloud_last = temp;


            rotation_cur_rot = rotation_rel; // current rotation relative to inital frame
            translation_cur_rot = rotation_rel;
            init_poses();
            
            adapt_field_of_view(normal_map->pcl_cloud); // need rotation, so should get rotation before this
            // cout << frame_id << "!!!!!!!!pcl_cloud checking: " << normal_map->pcl_cloud->at(100, 100).x << endl;
            reproject(normal_map->pcl_cloud);
            // cout << frame_id << "!!!!!!!!pcl_cloud checking: " << normal_map->pcl_cloud->at(100, 100).x << endl;
            train();

            

            initialized = true;

            // cout << frame_id << "!!!!!!!!!!!!!!!!!!!initialized rotation_rot is : " << rotation_cur_rot.getAngle() << endl;

            frame->SetRotation(rotation_cur_rot);
            frame->SetPose(pose);
            // frame->start_time = normalMap->start_time;

            if(flag_save_file)
                save_file(frame);

            ROS_WARN("Trained. %d times with PSR: %.1f............", train_num++, psr);
        }
        else{

            cv::Mat AnglesMap = cv::Mat::ones(cv::Size(depth_height/maxPyramidLevel, depth_width/maxPyramidLevel), CV_64FC3)*720;
            Eigen::Matrix3d rotation_matrix;
            EfficientNormal2RotationMat(normalsCV_last, normal_map->normal_map, rotation_matrix, AnglesMap);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*normal_map->pcl_cloud, *temp);
            pcl_cloud_now = temp;
            // EfficientNormal2RotationMatLst(normalsCV_last, normal_map->normal_map, rotation_matrix, AnglesMap, pcl_cloud_last, pcl_cloud_now);
            // rotation_matrix = computeRotation(pcl_cloud_last, pcl_cloud_now);
            // cout << "2222I am here!!!!!!!!!!!!!!!" << rotation_matrix << endl;
            // cout << "rotation after ICP: " << rotation_matrix << endl;
            // EfficientNormal2RotationMat(normalsCV_last, normal_map->normal_map, rotation_matrix, AnglesMap);
            cout << "rotation after Ours: " << rotation_matrix << endl;

            Eigen::Quaterniond q_rel(rotation_matrix);
            rotation_rel = tf::Quaternion(q_rel.coeffs()[0], q_rel.coeffs()[1], q_rel.coeffs()[2], q_rel.coeffs()[3]);
            rotation_cur_rot = (rotation_key_rot * rotation_rel).normalized(); // current rotation relative to inital frame
            rotation_cur_rot = tf::Quaternion(gt_poses[frame_id][3], gt_poses[frame_id][4], gt_poses[frame_id][5], gt_poses[frame_id][6]);

            // if(frame_id+1>250){
            //     rotation_cur_rot = tf::Quaternion(gt_poses[frame_id][3], gt_poses[frame_id][4], gt_poses[frame_id][5], gt_poses[frame_id][6]);
            //     // cout << "gt_poses: !!!!!!!!!!!!!!!!!!!!!!!!!!" << gt_poses[1][3] << ", " << gt_poses[1][6] << endl;
            // }

            if((frame_id+1)%5 == 1 || valid_points<5000 || tf::Transform(rotation_key_rot.inverse()*rotation_cur_rot).getRotation().getAngle() >= 0.15){
                normalsCV_last = normal_map->normal_map.clone();
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::copyPointCloud(*normal_map->pcl_cloud, *temp);
                pcl_cloud_last = temp;
                rotation_key_rot = rotation_cur_rot;
                frame->SetKeyRot();
            }
            frame->SetRotation(rotation_cur_rot);
            // frame->start_time = normalMap->start_time;
        }

        // cout << frame_id << "!!!!!!!!!!!!!!!!!!!rotation_rot is : " << rotation_cur_rot.getAngle() << endl;

        // count time 
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "Thread [EstimateRotationThread] took " << elapsed.count() << " milliseconds to execute.\n";

        _frame_mutex.lock();
        _frame_buffer.push(frame);
        _frame_mutex.unlock();
    }
}

void NI_SLAM::EstimateRotationPart1Thread(){
    while(!_shutdown){
        // std::cout << "Function 2 is being executed by thread " << std::this_thread::get_id() << std::endl;
        if(_normalMap_buffer.empty()){
            usleep(2000);
            continue;
        }

        // count time
        auto start = std::chrono::high_resolution_clock::now();

        NormalMapPtr normal_map;
        _normalMap_mutex.lock();
        normal_map = _normalMap_buffer.front();
        _normalMap_buffer.pop();
        _normalMap_mutex.unlock();

        int frame_id = normal_map->time;
        

        // FramePtr frame = std::shared_ptr<Frame>(new Frame(frame_id, normal_map->normal_map, normal_map->pcl_cloud));
        // frame->start_time = normal_map->start_time;

        if(!initialized){
            FramePtr frame = std::shared_ptr<Frame>(new Frame(frame_id, normal_map->normal_map, normal_map->pcl_cloud));
            frame->start_time = normal_map->start_time;
            rotation_rel = tf::Quaternion(0,0,0,1);
            rotation_key_rot = rotation_rel;
            normalsCV_last = normal_map->normal_map.clone();
            rotation_cur_rot = rotation_rel; // current rotation relative to inital frame
            translation_cur_rot = rotation_rel;
            init_poses();
            
            adapt_field_of_view(normal_map->pcl_cloud); // need rotation, so should get rotation before this
            // cout << frame_id << "!!!!!!!!pcl_cloud checking: " << normal_map->pcl_cloud->at(100, 100).x << endl;
            reproject(normal_map->pcl_cloud);
            // cout << frame_id << "!!!!!!!!pcl_cloud checking: " << normal_map->pcl_cloud->at(100, 100).x << endl;
            train();

            

            initialized = true;

            // cout << frame_id << "!!!!!!!!!!!!!!!!!!!initialized rotation_rot is : " << rotation_cur_rot.getAngle() << endl;

            frame->SetRotation(rotation_cur_rot);
            frame->SetPose(pose);
            // frame->start_time = normalMap->start_time;

            if(flag_save_file)
                save_file(frame);

            ROS_WARN("Trained. %d times with PSR: %.1f............", train_num++, psr);

            _frame_mutex.lock();
            _frame_buffer.push(frame);
            _frame_mutex.unlock();
        }
        else{
            if((frame_id+1)%5 != 1){
                continue;
            }

            // cv::Mat AnglesMap = cv::Mat::ones(cv::Size(depth_height/maxPyramidLevel, depth_width/maxPyramidLevel), CV_64FC3)*720;
            DataModesPtr dataModes = std::shared_ptr<DataModes>(new DataModes());
            Eigen::Matrix3d rotation_matrix;
            dataModes = EfficientNormal2RotationMatPart1(normalsCV_last, normal_map->normal_map);
            Eigen::Quaterniond q_rel(rotation_matrix);
            rotation_rel = tf::Quaternion(q_rel.coeffs()[0], q_rel.coeffs()[1], q_rel.coeffs()[2], q_rel.coeffs()[3]);
            rotation_cur_rot = (rotation_key_rot * rotation_rel).normalized(); // current rotation relative to inital frame

            dataModes->_normal_map = normal_map->normal_map;
            dataModes->_frame_id = frame_id;
            dataModes->_pcl_cloud = normal_map->pcl_cloud;

            normalsCV_last = normal_map->normal_map.clone();

            _modesData_mutex.lock();
            _modesData_buffer.push(dataModes);
            _modesData_mutex.unlock();
        }

        // cout << frame_id << "!!!!!!!!!!!!!!!!!!!rotation_rot is : " << rotation_cur_rot.getAngle() << endl;

        // count time 
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "Thread [EstimateRotationThreadPart1] took " << elapsed.count() << " milliseconds to execute.\n";
    }
}

void NI_SLAM::EstimateRotationPart2Thread(){
    while(!_shutdown){
        // std::cout << "Function 2 is being executed by thread " << std::this_thread::get_id() << std::endl;
        if(_modesData_buffer.empty()){
            usleep(2000);
            continue;
        }

        // count time
        auto start = std::chrono::high_resolution_clock::now();

        DataModesPtr dataModes = std::shared_ptr<DataModes>(new DataModes());
        _modesData_mutex.lock();
        dataModes = _modesData_buffer.front();
        _modesData_buffer.pop();
        _modesData_mutex.unlock();

        int frame_id = dataModes->_frame_id;
        

        FramePtr frame = std::shared_ptr<Frame>(new Frame(frame_id, dataModes->_normal_map, dataModes->_pcl_cloud));
        // frame->start_time = normal_map->start_time;


        Eigen::Matrix3d rotation_matrix;
        EfficientNormal2RotationMatPart2(dataModes, rotation_matrix);
        Eigen::Quaterniond q_rel(rotation_matrix);
        rotation_rel = tf::Quaternion(q_rel.coeffs()[0], q_rel.coeffs()[1], q_rel.coeffs()[2], q_rel.coeffs()[3]);
        rotation_cur_rot = (rotation_key_rot * rotation_rel).normalized(); // current rotation relative to inital frame

        rotation_key_rot = rotation_cur_rot;
        frame->SetKeyRot();
        frame->SetRotation(rotation_cur_rot);
 

        // cout << frame_id << "!!!!!!!!!!!!!!!!!!!rotation_rot is : " << rotation_cur_rot.getAngle() << endl;

        // count time 
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "Thread [EstimateRotationThreadPart2] took " << elapsed.count() << " milliseconds to execute.\n";


        _frame_mutex.lock();
        _frame_buffer.push(frame);
        _frame_mutex.unlock();
    }
}


void NI_SLAM::EstimateTranslationThread(){
    while(!_shutdown){
        // std::cout << "Function 3 is being executed by thread " << std::this_thread::get_id() << std::endl;
        if(_frame_buffer.empty()){
            usleep(2000);
            continue;
        }

        // count time
        auto start = std::chrono::high_resolution_clock::now();

        _frame_mutex.lock();
        FramePtr frame = _frame_buffer.front();
        _frame_buffer.pop();
        _frame_mutex.unlock();

        int frame_id = frame->GetFrameId();
        if(frame_id==0){
            t1 = std::chrono::steady_clock::now();
            continue;
        }
        translation_cur_rot = frame->GetRotation();
        // cout << frame_id << "!!!!!!!!!!!!!!!!!!!rotation_trans is : " << translation_key_rot.getAngle() << endl;
        // cout << frame_id << "!!!!!!!!!!!!!!!!!!!rotation_trans is : " << translation_cur_rot.getAngle() << endl;
        CloudType::ConstPtr cur_pcl_cloud = frame->GetPclCloud();
        // cout << frame_id << "!!!!!!!!pcl_cloud checking: " << cur_pcl_cloud->at(100, 100).x << endl;

        reproject(cur_pcl_cloud, tf::Transform((translation_key_rot.inverse())*translation_cur_rot));
        // cout << frame_id << "!!!!!!!!pcl_cloud checking: " << cur_pcl_cloud->at(100, 100).x << endl;
        get_response();

        get_psr();

        refine_keyclouds_poses();

        // publish_poses();

        frame->SetPose(pose);

        bool rotation_key = frame->IsKeyRot();

        if (!(psr > psrbound)){ // || (frame_id+1)%2 == 1){ // || tf::Transform(translation_key_rot.inverse()*translation_cur_rot).getRotation().getAngle() >= 0.3){ // || rotation_key){

            adapt_field_of_view(cur_pcl_cloud);

            reproject(cur_pcl_cloud);

            train();

            ROS_WARN("Trained. %d times with PSR: %.1f............", train_num++, psr);

            if(flag_save_file)
                ROS_WARN("write %d times into file", train_num);
                save_file(frame);
                ROS_WARN("written %d times into file", train_num);
        }
    
        // count time 
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "Thread [EstimateTranslationThread] took " << elapsed.count() << " milliseconds to execute.\n";
    }
}


Eigen::Matrix3d NI_SLAM::computeRotation(const CloudType::ConstPtr& _pcl_cloud_last, const CloudType::ConstPtr& _pcl_cloud) {

    // 创建一个新的点云用于存储预处理后的点云
    CloudType::Ptr cloud_filtered_last(new CloudType);
    CloudType::Ptr cloud_filtered(new CloudType);

    // 移除无效的点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*_pcl_cloud_last, *cloud_filtered_last, indices);
    pcl::removeNaNFromPointCloud(*_pcl_cloud, *cloud_filtered, indices);

    // 创建一个VoxelGrid滤波器用于下采样
    pcl::VoxelGrid<CloudType::PointType> sor;
    sor.setInputCloud(_pcl_cloud_last);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered_last);
    
    pcl::VoxelGrid<CloudType::PointType> sor1;
    sor1.setLeafSize(0.01f, 0.01f, 0.01f);
    sor1.setInputCloud(_pcl_cloud);
    sor1.filter(*cloud_filtered);


    // // 创建一个StatisticalOutlierRemoval滤波器用于去噪
    // pcl::StatisticalOutlierRemoval<CloudType::PointType> sor2;
    // sor2.setInputCloud (cloud_filtered_last);
    // sor2.setMeanK (50);
    // sor2.setStddevMulThresh (1.0);
    // sor2.filter (*cloud_filtered_last);

    // sor2.setInputCloud (cloud_filtered);
    // sor2.filter (*cloud_filtered);
        

    // 创建ICP对象
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(cloud_filtered_last);
    icp.setInputTarget(cloud_filtered);


    // 执行ICP
    CloudType Final;
    icp.align(Final);

    // // 获取旋转和平移
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    
    // // 提取旋转矩阵
    Eigen::Matrix3f rotation = transformation.block<3,3>(0, 0);
    
    // 将旋转矩阵转换为double类型并存储在输入的旋转矩阵中   
    Eigen::Matrix3d _rotation_matrix = rotation.cast<double>();
    cout << "I am here!!!!!!!!!!!!!!!" << _rotation_matrix << endl;
    return _rotation_matrix;
    // return Eigen::Matrix3d::Identity();
    // if (icp.hasConverged()) {
    //     // 获取配准后的变换矩阵
    //     Eigen::Matrix4f transformation = icp.getFinalTransformation();

    //     // 从4x4变换矩阵中提取出3x3旋转矩阵
    //     Eigen::Matrix3d rotation = transformation.block<3,3>(0, 0).cast<double>();
    //     cout << "I am here!!!!!!!!!!!!!!!" << rotation << endl;
    //     return rotation;
    // } else {
    //     // 如果配准失败，返回一个单位矩阵
    //     return Eigen::Matrix3d::Identity();
    // }
}

Eigen::Matrix3d NI_SLAM::computeICP(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& _pcl_cloud_last, 
                           const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& _pcl_cloud) {
    // 创建一个ICP对象，用于ICP配准
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

    // 设置最大迭代次数（这是可选的，缺省值为10）
    // icp.setMaximumIterations(50);
    // icp.setMaximumIterations();

    // 给ICP设置源点云和目标点云
    icp.setInputSource(_pcl_cloud_last);
    icp.setInputTarget(_pcl_cloud);

    // 创建一个空的PointCloud用于存储ICP的结果
    pcl::PointCloud<pcl::PointXYZRGB> Final;

    // 对源点云进行ICP配准
    icp.align(Final);
    cout << "I am here!!!!!!!!!!!!!!!" << endl;
    // 检查配准是否成功
    if (icp.hasConverged()) {
        // 获取配准后的变换矩阵
        Eigen::Matrix4f transformation = icp.getFinalTransformation();

        // 从4x4变换矩阵中提取出3x3旋转矩阵
        Eigen::Matrix3d rotation = transformation.block<3,3>(0, 0).cast<double>();
        cout << "I am here!!!!!!!!!!!!!!!" << rotation << endl;
        return rotation;
    } else {
        // 如果配准失败，返回一个单位矩阵
        return Eigen::Matrix3d::Identity();
    }
}


inline bool NI_SLAM::check_synchronization(std_msgs::Header pc, std_msgs::Header imu, double max_diff)
{
    double dt = pc.stamp.toSec()-imu.stamp.toSec();
    if (!(dt <= max_diff))
    {
        ROS_ERROR("Point cloud and IMU's time diff too large!: %.6fs > %.6fs ", dt, max_diff);
        return false;
    }
    else
        return true;
}


inline void NI_SLAM::adapt_field_of_view(const CloudType::ConstPtr& pcl_cloud)
{
    float tan_theta = 1.0 * pcl_cloud->height/pcl_cloud->width;
    float cos_theta = 1.0 / sqrt(1 + tan_theta * tan_theta);
    float sin_theta = tan_theta * cos_theta;
    int adaptive_width_center = round(len_hist * cos_theta);
    int adaptive_height_center = round(adaptive_width_center * tan_theta);
    float resolution_hist = max_distance / adaptive_width_center / 2;
    histogram = ArrayXi::Zero(len_hist);
    int accept_num, index_num;

    for(size_t j = 0; j < pcl_cloud->height; j += point_cloud_sample)
        for (size_t i = 0; i < pcl_cloud->width; i+= point_cloud_sample)
        {
            if (isnan(pcl_cloud->at(i, j).z))
                continue;

            int x = abs(int(pcl_cloud->at(i, j).x/resolution_hist));
            int y = abs(int(pcl_cloud->at(i, j).y/resolution_hist));

            if(x < adaptive_width_center && y< adaptive_height_center)
            {
                if (y > x * tan_theta)
                    ++histogram(round(y/sin_theta));
                else
                    ++histogram(round(x/cos_theta));
            }
        }

    accept_num = histogram.sum() * adaptive_mu;

    for (index_num = 0; index_num != len_hist; ++index_num)
    {
        accept_num -= histogram(index_num);
        if (accept_num < 0)
            break;
    }

    resolution = 2 * (index_num + 1) * resolution_hist / width;
}


inline void NI_SLAM::reproject(const CloudType::ConstPtr& pcl_cloud, const tf::Transform& transform)
{
    int pixel_num = 0, width_center = width/2, height_center = height/2;

    color = color_empty;
    depth = depth_empty;
    weights_dep = depth_empty;

    if(is_debugging){
        cout << "translation is: " << transform.getOrigin().getX() << ", " << transform.getOrigin().getY() << ", " << transform.getOrigin().getZ() << ", " << endl;
        cout << "rotation is: " << transform.getRotation().getAngle() << endl;
    }

    for(size_t i = 0; i != pcl_cloud->points.size(); ++i)
    {
        if (isnan(pcl_cloud->points[i].z))
            continue;
        
        tf::Vector3 point(pcl_cloud->points[i].x, pcl_cloud->points[i].y, pcl_cloud->points[i].z);
        tf::Vector3 tf_point = transform * point;

        int   x = tf_point.x()/resolution + width_center;
        int   y = tf_point.y()/resolution + height_center;
        float z = tf_point.z();

        short B = pcl_cloud->points[i].b;
        short G = pcl_cloud->points[i].g;
        short R = pcl_cloud->points[i].r;

        float rgb = (0.1140*B + 0.5870*G + 0.2989*R)/255.0;

        if((0<=x && x<width && 0<=y && y<height)) // (x,y) may locate outside image
        {
            float pre_value = depth(x, y);
            if(pre_value == 0 || pre_value > z) //near points cover far points 
            {
                color(x, y) = rgb;
                depth(x, y) = z;
                weights_dep(x, y) = 1;
                ++pixel_num;
            }
        }
    }

    if (pixel_num < width * height *0.25)
        ROS_WARN("Only %d << %d points!!", pixel_num, width * height);
}


inline void NI_SLAM::train()
{
    translation_z = 0;

    translation_key_rot = translation_cur_rot;

    pose_key = pose;

    // pose_real_key = pose_real;

    color_key = color;

    depth_key = depth;

    weights = weights_dep;
   
    weights_key = weights_dep;
    
    model_fft = fft(color_key);
    
    kernel = gaussian_kernel();
    
    alpha = labels_fft/(kernel + lambda);

    // normalsCV_last = normalsCV.clone(); // parallel need to comment this


    // Eigen::MatrixXd a = Eigen::MatrixXd::Random(1000, 1000);  
    // Eigen::MatrixXd b = Eigen::MatrixXd::Random(1000, 1000);

    // double start = clock();
    // Eigen::MatrixXd c = a * b;    
    // double endd = clock();
    // double thisTime = (double)(endd - start) / CLOCKS_PER_SEC;

    // cout << "testMKL: " << thisTime << endl;

}


inline ArrayXXcf NI_SLAM::gaussian_kernel()
{
    unsigned int N = height * width;

    model_square = model_fft.square().abs().sum()/N;

    float xx = model_square;

    float yy = model_square;

    model_fft_conj = model_fft.conjugate();
    
    xyf = model_fft * model_fft_conj;
    
    xy = ifft(xyf);
    
    xxyy = (xx+yy-2*xy)/N;
    // ArrayXXF kernel = (-1/(sigma*sigma)*xxyy).exp();
    // kernel = kernel/kernel.abs().maxCoeff();

    // return fft(kernel)
    return fft((-1/(sigma*sigma)*xxyy).exp());
}


inline ArrayXXcf NI_SLAM::gaussian_kernel(const ArrayXXcf& xf)
{
    unsigned int N = height * width;

    float xx = xf.square().abs().sum()/N;

    float yy = model_square;
    
    xyf = xf * model_fft_conj;
    
    xy = ifft(xyf);

    xxyy = (xx+yy-2*xy)/N;

    return fft((-1/(sigma*sigma)*xxyy).exp());
}


inline ArrayXXcf NI_SLAM::get_labels_fft()
{
    ArrayXXf labels =  ArrayXXf::Zero(width, height);
    labels(width/2, height/2) = 1;

    return fft(labels);
}


inline void NI_SLAM::init_poses()
{
    // rotation_key = rotation;

    // tf::Vector3 position(0, 0, -2.5);
    tf::Vector3 position(0, 0, 0);

    pose_key.setOrigin(position);

    // pose_key.setOrigin(pose_real.getOrigin());
        
    // pose_key.setRotation(rotation);
    pose_key.setRotation(rotation_cur_rot);
    pose = pose_key;
}


inline void NI_SLAM::get_response()
{
    color_fft = fft(color);

    kernel = gaussian_kernel(color_fft);

    response = ifft(alpha * kernel);

    max_response = response.maxCoeff(&(max_index[0]), &(max_index[1]));
    
    trans(0) = max_index[0] - width/2;

    trans(1) = max_index[1] - height/2;
}


inline float NI_SLAM::get_psr()
{
    // if (max_index[0]>=5 && max_index[0]<width-5 && max_index[1]>=5 && max_index[1]<height-5)
        // response.block<11, 11>(max_index[0]-5, max_index[1]-5)=0;

    response(max_index[0], max_index[1]) = 0;

    response_mean = response.mean();

    float std  = sqrt((response-response_mean).square().mean());

    psr = (max_response - response_mean)/std;

    return psr;
}


inline void NI_SLAM::refine_keyclouds_poses()
{

    color_rect = shift(color, -trans(0), -trans(1));
    
    depth_rect = shift(depth, -trans(0), -trans(1));
    
    weights_rect = shift(weights_dep, -trans(0), -trans(1));

    weights_sum = weights + weights_rect + small;

    if (psr>100)
        color_key = (color_key * weights + color_rect * weights_rect)/weights_sum;    
    
    weighted = weights_key * weights_rect;

    depth_trans = depth_rect - depth_key;

    inliers = ((color_rect - color_key).abs() < valid_diff) && ((depth_trans - translation_z).abs() < valid_diff);
    
    weighted *= inliers.cast<float>();

    weighted_sum =  weighted.sum();
    
    if (weighted_sum > 1e-7)
    {
        translation_z = (depth_trans * weighted ).sum()/weighted_sum;
    }

    if (psr>100)
    {
        depth_key = (depth_key * weights + (depth_rect - translation_z) * weights_rect)/weights_sum;

        weights = weights + weights_rect;
    }

    // tf::Quaternion rotation_inc = ((rotation_key.inverse()) * rotation).normalized();
    tf::Quaternion rotation_inc = (translation_key_rot.inverse()*translation_cur_rot).normalized();

    if (max_index[0]==0 || max_index[1]==0)
    {
        ROS_WARN("Can not find peak! Setting to static!..");
        trans(0) = 0;
        trans(1) = 0;
    }
    
    tf::Vector3 translation_inc(-trans(0)*resolution, -trans(1)*resolution, -translation_z);
        
    transform_inc = tf::Transform(rotation_inc, translation_inc);

    transform_dif = pose.inverse() * pose_key * transform_inc;

    pose =  pose_key * transform_inc;

    // cout << "in refine_keyclouds_poses pose: " << pose_key.getOrigin().getZ() << endl;
}

inline void NI_SLAM::publish_twists()
{
    twiststamped.header = ros_header;

    twiststamped.header.frame_id = "keyframe_" + to_string(train_num);

    tf::vector3TFToMsg(transform_dif.getOrigin()/time_diff, twiststamped.twist.twist.linear);

    tf::Vector3 euler;

    transform_dif.getBasis().getRPY(euler[0], euler[1], euler[2]);

    tf::vector3TFToMsg(euler/time_diff, twiststamped.twist.twist.angular);

    double dt2 = time_diff * time_diff;

    double sx2 = sigma_x2, sy2 = sigma_y2, sz2 = sigma_z2;

    double integration = response_mean * width * height + max_response;

    sigma_x2 = 1.0/2/M_PI/max_response*integration*resolution*resolution;

    sigma_y2 = sigma_x2;

    sigma_z2 = ((depth_trans-translation_z) * weighted).square().sum() / weighted_sum;

    twiststamped.twist.covariance[0*7] = (sigma_x2 + sx2)/dt2;

    twiststamped.twist.covariance[1*7] = (sigma_y2 + sy2)/dt2;

    twiststamped.twist.covariance[2*7] = (sigma_z2 + sz2)/dt2;

    twiststamped.twist.covariance[3*7] = (0.0005 * 2)/dt2;

    twiststamped.twist.covariance[4*7] = (0.0005 * 2)/dt2;

    twiststamped.twist.covariance[5*7] = (0.0005 * 2)/dt2;

    twist_pub.publish(twiststamped);
}

inline void NI_SLAM::publish_poses()
{
    br.sendTransform(tf::StampedTransform(pose, ros_header.stamp, parent_frame, child_frame));

    // br.sendTransform(tf::StampedTransform(pose_real, ros_header.stamp, parent_frame, "real"));

    posestamped.header = ros_header;

    posestamped.header.frame_id = parent_frame;

    tf::poseTFToMsg(pose, posestamped.pose);

    pose_pub.publish(posestamped);
}

inline void NI_SLAM::publish_incremental_keypose_cov()
{
    tf::poseTFToMsg(transform_inc, pose_inc_cov.pose.pose);

    pose_inc_cov.header = posestamped.header;

    pose_inc_cov.header.frame_id = "keyframe_" + to_string(train_num);

    double integration = response_mean * width * height + max_response;

    sigma_x2 = 1.0/2/M_PI/max_response*integration*resolution*resolution;

    sigma_y2 = sigma_x2;

    sigma_z2 = ((depth_trans-translation_z) * weighted).square().sum() / weighted_sum;

    pose_inc_cov.pose.covariance[0*7] = sigma_x2;

    pose_inc_cov.pose.covariance[1*7] = sigma_y2;

    pose_inc_cov.pose.covariance[2*7] = sigma_z2;

    pose_cov_pub.publish(pose_inc_cov);
}

inline void NI_SLAM::EfficientDepth2NormalMap(cv::Mat &_imD, cv::Mat &_normalsCV, int &_cellSize,
                                Eigen::MatrixXd &vertexMapxx, Eigen::MatrixXd &vertexMapyy)
{
    // *0. generate mask
    cv::Mat mask;
    cv::inRange(_imD*0.0002, 0.15, 2.5, mask); // Pixels within the range are set to 255
    mask = mask / 255; // Convert mask values from 255 to 1
    // double minVal; 
    // double maxVal; 
    cv::Point minLoc; 
    cv::Point maxLoc;

    Eigen::MatrixXd maskEigen;
    cv::cv2eigen(mask, maskEigen);

    // Convert the mask to an Array
    Eigen::ArrayXXd maskArray = maskEigen.array();
    
    Eigen::MatrixXd vertexMapz = Eigen::MatrixXd::Zero(_imD.rows, _imD.cols);
    cv::cv2eigen(_imD, vertexMapz);
    Eigen::MatrixXd vertexMapx_ = Eigen::MatrixXd::Zero(_imD.rows, _imD.cols);
    Eigen::MatrixXd vertexMapy_ = Eigen::MatrixXd::Zero(_imD.rows, _imD.cols); 
    
    vertexMapx_ = vertexMapxx.cwiseProduct(vertexMapz); 
    vertexMapy_ = vertexMapyy.cwiseProduct(vertexMapz); 

    Eigen::ArrayXXd utanMap_x = Eigen::ArrayXXd::Zero(vertexMapx_.rows(), vertexMapx_.cols());
    Eigen::ArrayXXd utanMap_y = Eigen::ArrayXXd::Zero(vertexMapx_.rows(), vertexMapx_.cols());
    Eigen::ArrayXXd utanMap_z = Eigen::ArrayXXd::Zero(vertexMapx_.rows(), vertexMapx_.cols());
    Eigen::ArrayXXd vtanMap_x = Eigen::ArrayXXd::Zero(vertexMapx_.rows(), vertexMapx_.cols());
    Eigen::ArrayXXd vtanMap_y = Eigen::ArrayXXd::Zero(vertexMapx_.rows(), vertexMapx_.cols());
    Eigen::ArrayXXd vtanMap_z = Eigen::ArrayXXd::Zero(vertexMapx_.rows(), vertexMapx_.cols()); //these six lines: 0.9ms

    utanMap_x.block(1, 1, vertexMapx_.rows() - 2, vertexMapx_.cols() - 2) +=
        (vertexMapx_.array().block(1, 2, vertexMapx_.rows() - 2, vertexMapx_.cols() - 2)
            - vertexMapx_.array().block(1, 0, vertexMapx_.rows() - 2, vertexMapx_.cols() - 2));
    
    utanMap_y.block(1, 1, vertexMapy_.rows() - 2, vertexMapy_.cols() - 2) +=
        (vertexMapy_.array().block(1,2, vertexMapy_.rows() - 2, vertexMapy_.cols() - 2)
            - vertexMapy_.array().block(1,0, vertexMapy_.rows() - 2, vertexMapy_.cols() - 2));

    utanMap_z.block(1, 1, vertexMapz.rows() - 2, vertexMapz.cols() - 2) +=
        (vertexMapz.array().block(1,2, vertexMapz.rows() - 2, vertexMapz.cols() - 2)
            - vertexMapz.array().block(1,0, vertexMapz.rows() - 2, vertexMapz.cols() - 2));
    
    vtanMap_x.block(1,1, vertexMapx_.rows() - 2, vertexMapx_.cols() - 2) +=
        (vertexMapx_.array().block(2,1, vertexMapx_.rows() - 2, vertexMapx_.cols() - 2)
            - vertexMapx_.array().block(0,1, vertexMapx_.rows() - 2, vertexMapx_.cols() - 2));

    vtanMap_y.block(1,1, vertexMapy_.rows() - 2, vertexMapy_.cols() - 2) +=
        (vertexMapy_.array().block(2,1, vertexMapy_.rows() - 2, vertexMapy_.cols() - 2)
            - vertexMapy_.array().block(0,1, vertexMapy_.rows() - 2, vertexMapy_.cols() - 2));

    vtanMap_z.block(1, 1, vertexMapz.rows() - 2, vertexMapz.cols() - 2) +=
        (vertexMapz.array().block(2,1, vertexMapz.rows() - 2, vertexMapz.cols() - 2)
            - vertexMapz.array().block(0,1, vertexMapz.rows() - 2, vertexMapz.cols() - 2)); // 426-448: 0.06ms

    Eigen::ArrayXXd normalsEig_x = utanMap_y * vtanMap_z - utanMap_z * vtanMap_y;
    Eigen::ArrayXXd normalsEig_y = utanMap_z * vtanMap_x - utanMap_x * vtanMap_z;
    Eigen::ArrayXXd normalsEig_z = utanMap_x * vtanMap_y - utanMap_y * vtanMap_x;

    Eigen::MatrixXd normalsEig_x_mat = (normalsEig_x / (1e-6 + sqrt(normalsEig_x * normalsEig_x + normalsEig_y * normalsEig_y + normalsEig_z * normalsEig_z))).matrix();
    Eigen::MatrixXd normalsEig_y_mat = (normalsEig_y / (1e-6 + sqrt(normalsEig_x * normalsEig_x + normalsEig_y * normalsEig_y + normalsEig_z * normalsEig_z))).matrix();
    Eigen::MatrixXd normalsEig_z_mat = (normalsEig_z / (1e-6 + sqrt(normalsEig_x * normalsEig_x + normalsEig_y * normalsEig_y + normalsEig_z * normalsEig_z))).matrix();
    
    std::vector<cv::Mat> channels_all(3);
    cv::eigen2cv(normalsEig_x_mat, channels_all[0]);
    cv::eigen2cv(normalsEig_y_mat, channels_all[1]);
    cv::eigen2cv(normalsEig_z_mat, channels_all[2]);

    cv::merge(channels_all, _normalsCV);
    cv::blur(_normalsCV, _normalsCV, cv::Size(_cellSize, _cellSize));
    cv::split(_normalsCV, channels_all);
    cv::cv2eigen(channels_all[0], normalsEig_x_mat);
    cv::cv2eigen(channels_all[1], normalsEig_y_mat);
    cv::cv2eigen(channels_all[2], normalsEig_z_mat);
    normalsEig_x = normalsEig_x_mat.array();
    normalsEig_y = normalsEig_y_mat.array();
    normalsEig_z = normalsEig_z_mat.array();

    // Apply the mask to the normal vectors
    // normalsEig_x *= maskArray;
    // normalsEig_y *= maskArray;
    // normalsEig_z *= maskArray;

    normalsEig_x_mat = (normalsEig_x / (1e-6 + sqrt(normalsEig_x * normalsEig_x + normalsEig_y * normalsEig_y + normalsEig_z * normalsEig_z))).matrix();
    normalsEig_y_mat = (normalsEig_y / (1e-6 + sqrt(normalsEig_x * normalsEig_x + normalsEig_y * normalsEig_y + normalsEig_z * normalsEig_z))).matrix();
    normalsEig_z_mat = (normalsEig_z / (1e-6 + sqrt(normalsEig_x * normalsEig_x + normalsEig_y * normalsEig_y + normalsEig_z * normalsEig_z))).matrix();
    cv::eigen2cv(normalsEig_x_mat, channels_all[0]);
    cv::eigen2cv(normalsEig_y_mat, channels_all[1]);
    cv::eigen2cv(normalsEig_z_mat, channels_all[2]);

    cv::merge(channels_all, _normalsCV);
}

inline DataModesPtr NI_SLAM::EfficientNormal2RotationMatPart1(cv::Mat &_normalsCV_last, cv::Mat &_normalsCV)
{
    cv::Vec3d zeros{0, 0, 0};
    cv::Vec3d N{1,0,0};
    // vector<cv::Vec3d> modes; // save different plane modes
    // vector<int> mode_count; // to record the number of normals in each plane mode 
    // vector<vector<Eigen::Vector2i>> mode_coor; // save the pixel coordinates of normals for each plane mode (may be optimized later) 
    vector<Eigen::Vector2i> tmp_coor; // to create a new mode in mode_coor
    // vector<DynamicMedian> mode_normals_x;
    // vector<DynamicMedian> mode_normals_y;
    // vector<DynamicMedian> mode_normals_z;
    // vector<DynamicMedian> mode_normals_x_last;
    // vector<DynamicMedian> mode_normals_y_last;
    // vector<DynamicMedian> mode_normals_z_last;
    DynamicMedian tmp_mode_normals;
    DataModesPtr dataModes = std::shared_ptr<DataModes>(new DataModes());;


    //** this loop is to perform selection mechanism 
    for(int v=0; v<_normalsCV.rows;v++)
        for(int u=0; u<_normalsCV.cols;u++)
        {
            //** exclude invalid points (noise from depth sensor)
            if((_normalsCV.at<cv::Vec3d>(v, u) == zeros) || (_normalsCV_last.at<cv::Vec3d>(v, u) == zeros) || (v <= 10) || (u <= 10) || (abs(v-_normalsCV.rows) <= 10) || (abs(u-_normalsCV.cols) <= 10))
            {
                continue;
            }
            //** exclude non-overlapped points
            if((_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u)).val[0] < 0.9){ //0.7){
                continue;
            }
            //** aim to cancel the points on edge
            if((_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v-1, u)).val[0] < 0.999 ||
            (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v+1, u)).val[0] < 0.999 ||
            (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v, u-1)).val[0] < 0.999 ||
            (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v, u+1)).val[0] < 0.999 ||
            (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v-1, u)).val[0] < 0.999 ||
            (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v+1, u)).val[0] < 0.999 ||
            (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u-1)).val[0] < 0.999 ||
            (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u+1)).val[0] < 0.999){
                continue;
            }
            //** perform mode selection mechanism
            int sizeofMode = static_cast<int>(dataModes->modes.size());
            if(sizeofMode == 0){

                // if(debug){
                //     cout << "============find new mode[" << static_cast<int>(modes.size()) << "]!!!" << "at (" << v << ", " << u << ")===============" << endl;
                // }
                //** the below four lines are used to create a new mode
                //** include modes updating/mode_coor updating/saved_selected_mode_normals updating/mode_count updating
                dataModes->modes.push_back(_normalsCV.at<cv::Vec3d>(v, u));
                dataModes->mode_coor.push_back(tmp_coor);
                // saved_selected_mode_normals.push_back(tmp_saved_selected_mode_normals);
                // saved_selected_mode_normals_last.push_back(tmp_saved_selected_mode_normals);
                dataModes->mode_normals_x.push_back(tmp_mode_normals);
                dataModes->mode_normals_x_last.push_back(tmp_mode_normals);
                dataModes->mode_normals_y.push_back(tmp_mode_normals);
                dataModes->mode_normals_y_last.push_back(tmp_mode_normals);
                dataModes->mode_normals_z.push_back(tmp_mode_normals);
                dataModes->mode_normals_z_last.push_back(tmp_mode_normals);
                dataModes->mode_count.push_back(0);
            }
            else{
                bool found_mode = false;
                //** perform normal-mode matching
                for(int i=0; i<sizeofMode; i++){
                    if((dataModes->modes[i].t() * _normalsCV.at<cv::Vec3d>(v, u)).val[0] >= 0.999){
                        found_mode = true;
                        Eigen::Vector2i x{v, u};
                        dataModes->mode_coor[i].push_back(x);
                        dataModes->mode_normals_x[i].addNum(_normalsCV.at<cv::Vec3d>(v, u)[0]);
                        dataModes->mode_normals_x_last[i].addNum(_normalsCV_last.at<cv::Vec3d>(v, u)[0]);
                        dataModes->mode_normals_y[i].addNum(_normalsCV.at<cv::Vec3d>(v, u)[1]);
                        dataModes->mode_normals_y_last[i].addNum(_normalsCV_last.at<cv::Vec3d>(v, u)[1]);
                        dataModes->mode_normals_z[i].addNum(_normalsCV.at<cv::Vec3d>(v, u)[2]);
                        dataModes->mode_normals_z_last[i].addNum(_normalsCV_last.at<cv::Vec3d>(v, u)[2]);
                        dataModes->mode_count[i]++;
                        
                        if(update_modes_real_time){
                            dataModes->modes[i][0] = dataModes->mode_normals_x[i].findMedian();
                            dataModes->modes[i][1] = dataModes->mode_normals_y[i].findMedian();
                            dataModes->modes[i][2] = dataModes->mode_normals_z[i].findMedian();
                        }

                        break;
                    }
                }
                if(found_mode == false){
                    if((_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v-1, u)).val[0] < 0.999 ||
                    (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v+1, u)).val[0] < 0.999 ||
                    (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v, u-1)).val[0] < 0.999 ||
                    (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v, u+1)).val[0] < 0.999 ||
                    (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v-1, u)).val[0] < 0.999 ||
                    (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v+1, u)).val[0] < 0.999 ||
                    (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u-1)).val[0] < 0.999 ||
                    (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u+1)).val[0] < 0.999){
                        continue;
                    }
                    // if(debug){
                    //     cout << "============find new mode[" << static_cast<int>(modes.size()) << "]!!!" << "at (" << v << ", " << u << ")===============" << endl;
                    // }
                    dataModes->modes.push_back(_normalsCV.at<cv::Vec3d>(v, u));
                    dataModes->mode_coor.push_back(tmp_coor);
                    // saved_selected_mode_normals.push_back(tmp_saved_selected_mode_normals);
                    // saved_selected_mode_normals_last.push_back(tmp_saved_selected_mode_normals);
                    dataModes->mode_normals_x.push_back(tmp_mode_normals);
                    dataModes->mode_normals_x_last.push_back(tmp_mode_normals);
                    dataModes->mode_normals_y.push_back(tmp_mode_normals);
                    dataModes->mode_normals_y_last.push_back(tmp_mode_normals);
                    dataModes->mode_normals_z.push_back(tmp_mode_normals);
                    dataModes->mode_normals_z_last.push_back(tmp_mode_normals);
                    dataModes->mode_count.push_back(0);
                }
            }
        }
    return dataModes;
  
}

inline void NI_SLAM::EfficientNormal2RotationMatPart2(DataModesPtr _dataModes, Eigen::Matrix3d &_R)
{
    //** After finishing selecting modes
    int sizeofMode = static_cast<int>(_dataModes->modes.size()); // sizeofmode saves the amount of modes which indicates how many planes there are
    // Eigen::VectorXi mode_count_eigen = Eigen::Map<Eigen::VectorXi, Eigen::Unaligned>(mode_count.data(), mode_count.size());
    // Eigen::Array<bool, Eigen::Dynamic, 1> mask = (mode_count_eigen.array()>12000);
    // int count_valid_mode = mask.count(); // define a counter to count the modes with points more than 12000
    int count_valid_mode{0};
    vector<int> selected_mode; // define a vector to save the number of selected modes
    // cout << "how many: " << modes.size() << endl;
    // cout << count_valid_mode << endl;
    int sumPoints{0};
    for(int i=0; i<sizeofMode; i++){ 
        if(_dataModes->mode_count[i] > 12000){
            if(is_debugging){
                cout << "**************************mode" << i << ": " << _dataModes->modes[i] << endl;
                cout << "totally " << _dataModes->mode_count[i] << " points" << endl;
            }

            sumPoints+=_dataModes->mode_count[i];
            selected_mode.push_back(i);
            count_valid_mode++;
        }
    }
    if(is_debugging){
        cout << count_valid_mode << endl;
    }
    
    int count_second_max{0};
    //** if only one mode is more than 10000 points, will select one more mode if this new mode is more than 5000
    //** otherwise, will select two modes if they are less than 5000
    // if(count_valid_mode<2){
    //     cout << "=====================================================================================" << endl;
    //     cout << "mode smaller than 2, will select one (>5000 points) or more (<5000 points) mode from plane with points less than 10000" << endl;
    //     selected_mode.push_back(0);
    //     for(int i=0; i<sizeofMode; i++){
    //         if(i == selected_mode[0]){
    //             continue;
    //         }
    //         if(mode_count[i]>=3000 && mode_count[i]<=10000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_second_max){
    //             count_second_max = mode_count[i];
    //             selected_mode[1] = i;
    //         }
    //     }
    //     cout << "**************************mode" << selected_mode[1] << ": " << modes[selected_mode[1]] << endl;
    //     cout << "totally " << mode_coor[selected_mode[1]].size() << " points" << endl;
    //     if(count_second_max != 0){
    //         count_valid_mode++;
    //     }
        
    //     if(count_second_max<5000){
    //         selected_mode.push_back(0);
    //         int count_third_max{0};
    //         for(int i=0; i<sizeofMode; i++){
    //             if(i == selected_mode[0] || i == selected_mode[1]){
    //                 continue;
    //             }
    //             if(mode_count[i]>=1000 && mode_count[i]<=10000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_third_max &&
    //             (modes[i].t() * modes[selected_mode[1]]).val[0]<0.5){
    //                 count_third_max = mode_count[i];
    //                 selected_mode[2] = i;
    //             }
    //         }
    //         if(count_third_max != 0){
    //             count_valid_mode++;
    //             cout << "**************************mode" << selected_mode[2] << ": " << modes[selected_mode[2]] << endl;
    //             cout << "totally " << mode_coor[selected_mode[2]].size() << " points" << endl;
    //         }

    //     }
    //     if(count_valid_mode == 1){
    //         cout << "cannot work, because only one plane exists" << endl;
    //     }
    // }

    if(count_valid_mode<2){
        if(is_debugging){
            cout << "=====================================================================================" << endl;
            cout << "mode smaller than 2, will select one (>12000 points) or more (<12000 points) mode from plane with points less than 12000" << endl;
        }
        selected_mode.push_back(0);
        for(int i=0; i<sizeofMode; i++){
            if(i == selected_mode[0]){
                continue;
            }
            if(_dataModes->mode_count[i]>=1000 && _dataModes->mode_count[i]<=12000 && (_dataModes->modes[i].t() * _dataModes->modes[selected_mode[0]]).val[0]<0.5 && _dataModes->mode_count[i]>count_second_max){
                count_second_max = _dataModes->mode_count[i];
                selected_mode[1] = i;
                // selected_mode.push_back(i);
            }
        }

        if(count_second_max != 0){
            count_valid_mode++;
            if(is_debugging){
                cout << "**************************mode" << selected_mode[1] << ": " << _dataModes->modes[selected_mode[1]] << endl;
                cout << "totally (>1000) " << _dataModes->mode_coor[selected_mode[1]].size() << " points" << endl;
            }
        }
        else{
            selected_mode.pop_back();
        }
        
        if(count_valid_mode==2 && count_second_max<12000){
            selected_mode.push_back(0);
            int count_third_max{0};
            for(int i=0; i<sizeofMode; i++){
                if(i == selected_mode[0] || i == selected_mode[1]){
                    continue;
                }
                if(_dataModes->mode_count[i]>=1000 && _dataModes->mode_count[i]<=12000 && (_dataModes->modes[i].t() * _dataModes->modes[selected_mode[0]]).val[0]<0.5 && _dataModes->mode_count[i]>count_third_max &&
                (_dataModes->modes[i].t() * _dataModes->modes[selected_mode[1]]).val[0]<0.5){
                    count_third_max = _dataModes->mode_count[i];
                    selected_mode[2] = i;
                }
            }
            if(count_third_max != 0){
                count_valid_mode++;
                if(is_debugging){
                    cout << "**************************mode" << selected_mode[2] << ": " << _dataModes->modes[selected_mode[2]] << endl;
                    cout << "totally (>1000) " << _dataModes->mode_coor[selected_mode[2]].size() << " points" << endl;
                }
            }
            else{
                selected_mode.pop_back();
            }

        }
        else if(count_valid_mode==1){
            selected_mode.push_back(0);
            for(int i=0; i<sizeofMode; i++){
                if(i == selected_mode[0]){
                    continue;
                }
                if(_dataModes->mode_count[i]>=500 && _dataModes->mode_count[i]<=12000 && (_dataModes->modes[i].t() * _dataModes->modes[selected_mode[0]]).val[0]<0.5 && _dataModes->mode_count[i]>count_second_max){
                    count_second_max = _dataModes->mode_count[i];
                    selected_mode[1] = i;
                }
            }
            if(count_second_max != 0){
                count_valid_mode++;
                if(is_debugging){
                    cout << "**************************mode" << selected_mode[1] << ": " << _dataModes->modes[selected_mode[1]] << endl;
                    cout << "totally (>500) " << _dataModes->mode_coor[selected_mode[1]].size() << " points" << endl;
                }
            }
            else{
                selected_mode.pop_back();
                std::cerr << "only one plane detected!" << endl;
            }

        }
    }

    //** if more than 2 valid modes, we have to calculate rotation matrix from each pair and evaluate which is the best
    //** paired_modes used to store number of a pair of modes
    //** first for loop used to generate all combinations of modes
    //** traverse each paired_mode to generate a rotation matrix, and use the rest of modes to evaluate the rotation matrix
    //** note that 'paired_modes' save the No. of modes, 'selected_mode' also save the No. of modes
    if(count_valid_mode>2){
        if(is_debugging){
            cout << "count valid mode: " << count_valid_mode << endl;
            cout << "---------------------------------------------------------" << endl;
            cout << "mode more than 2, will compare which paired modes is best" << endl;
        }

        vector<vector<int>> paired_modes;
        for(int i=0; i<count_valid_mode; i++)
            for(int j=i+1; j<count_valid_mode; j++){
                paired_modes.push_back({selected_mode[i],selected_mode[j]});
        }

        Eigen::Vector3d v1;
        Eigen::Vector3d v11;
        Eigen::Vector3d v2;
        Eigen::Vector3d v22;
        Eigen::Matrix3d R_;
        int best_paired_modes{0};
        double error{999999};
        for(size_t i=0; i<(paired_modes.size()); i++){
            if(is_debugging){
                cout << "**current paired modes is: " << paired_modes[i][0] << " and " << paired_modes[i][1] << "**" << endl;
            }
            // v1 = oneVectorFromMultiVectors(normalsCV_last, mode_coor[paired_modes[i][0]]);
            // v11 = oneVectorFromMultiVectors(normalsCV, mode_coor[paired_modes[i][0]]);
            // v2 = oneVectorFromMultiVectors(normalsCV_last, mode_coor[paired_modes[i][1]]);
            // v22 = oneVectorFromMultiVectors(normalsCV, mode_coor[paired_modes[i][1]]);

            // v1 = oneVectorFromMultiVectors(mode_count[paired_modes[i][0]], saved_selected_mode_normals_last[paired_modes[i][0]]);
            // v11 = oneVectorFromMultiVectors(mode_count[paired_modes[i][0]], saved_selected_mode_normals[paired_modes[i][0]]);
            // v2 = oneVectorFromMultiVectors(mode_count[paired_modes[i][1]], saved_selected_mode_normals_last[paired_modes[i][1]]);
            // v22 = oneVectorFromMultiVectors(mode_count[paired_modes[i][1]], saved_selected_mode_normals[paired_modes[i][1]]);

            v1 = {_dataModes->mode_normals_x_last[paired_modes[i][0]].findMedian(), 
                    _dataModes->mode_normals_y_last[paired_modes[i][0]].findMedian(), 
                        _dataModes->mode_normals_z_last[paired_modes[i][0]].findMedian()};
            v11 = {_dataModes->mode_normals_x[paired_modes[i][0]].findMedian(), 
                    _dataModes->mode_normals_y[paired_modes[i][0]].findMedian(), 
                        _dataModes->mode_normals_z[paired_modes[i][0]].findMedian()};
            v2 = {_dataModes->mode_normals_x_last[paired_modes[i][1]].findMedian(), 
                    _dataModes->mode_normals_y_last[paired_modes[i][1]].findMedian(), 
                        _dataModes->mode_normals_z_last[paired_modes[i][1]].findMedian()};
            v22 = {_dataModes->mode_normals_x[paired_modes[i][1]].findMedian(), 
                    _dataModes->mode_normals_y[paired_modes[i][1]].findMedian(), 
                        _dataModes->mode_normals_z[paired_modes[i][1]].findMedian()};
            v22 = v22.normalized();
            v2 = v2.normalized();
            v11 = v11.normalized();
            v1 = v1.normalized();            

            // cout << "**v1 is : " << v1[0] << ", " << v1[1] << ", " << v1[2] << " **" << endl;
            // cout << "new v1 is : " << mode_normals_x_last[paired_modes[i][0]].findMedian() << "," <<
            //         mode_normals_y_last[paired_modes[i][0]].findMedian() << "," << 
            //             mode_normals_z_last[paired_modes[i][0]].findMedian() << endl;
            // cout << "**v11 is : " << v11[0] << ", " << v11[1] << ", " << v11[2] << " **" << endl;
            // cout << "new v11 is : " << mode_normals_x[paired_modes[i][0]].findMedian() << "," <<
            //         mode_normals_y[paired_modes[i][0]].findMedian() << "," << 
            //             mode_normals_z[paired_modes[i][0]].findMedian() << endl;
            // cout << "**v2 is : " << v2[0] << ", " << v2[1] << ", " << v2[2] << " **" << endl;
            // cout << "new v2 is : " << mode_normals_x_last[paired_modes[i][1]].findMedian() << "," <<
            //         mode_normals_y_last[paired_modes[i][1]].findMedian() << "," << 
            //             mode_normals_z_last[paired_modes[i][1]].findMedian() << endl;
            // cout << "**v22 is : " << v22[0] << ", " << v22[1] << ", " << v22[2] << " **" << endl;
            // cout << "new v22 is : " << mode_normals_x[paired_modes[i][1]].findMedian() << "," <<
            //         mode_normals_y[paired_modes[i][1]].findMedian() << "," << 
            //             mode_normals_z[paired_modes[i][1]].findMedian() << endl;
            rotationFromTwoPlanes(v1, v11, v2, v22, R_);
            double count{0.0};
            double error_cur{0};
            for(int j=0; j<count_valid_mode; j++){
                if(selected_mode[j]==paired_modes[i][1] || selected_mode[j]==paired_modes[i][0]){
                    continue;
                }
                count += 1.0;
                // float weight = float(mode_count[selected_mode[j]]) / float(sumPoints);
                // Eigen::Vector3d vTest = oneVectorFromMultiVectors(normalsCV_last, mode_coor[selected_mode[j]]);
                // Eigen::Vector3d vTTest = oneVectorFromMultiVectors(normalsCV, mode_coor[selected_mode[j]]);
                
                // Eigen::Vector3d vTest = oneVectorFromMultiVectors(mode_count[selected_mode[j]], saved_selected_mode_normals_last[selected_mode[j]]);
                // Eigen::Vector3d vTTest = oneVectorFromMultiVectors(mode_count[selected_mode[j]], saved_selected_mode_normals[selected_mode[j]]);

                Eigen::Vector3d vTest = {_dataModes->mode_normals_x_last[selected_mode[j]].findMedian(), 
                                            _dataModes->mode_normals_y_last[selected_mode[j]].findMedian(), 
                                                _dataModes->mode_normals_z_last[selected_mode[j]].findMedian()};
                Eigen::Vector3d vTTest = {_dataModes->mode_normals_x[selected_mode[j]].findMedian(), 
                                            _dataModes->mode_normals_y[selected_mode[j]].findMedian(), 
                                                _dataModes->mode_normals_z[selected_mode[j]].findMedian()};  
                vTest = vTest.normalized();
                vTTest = vTTest.normalized();
                
                Eigen::Vector3d vTTest_rotated = R_ * vTTest;
                // cout << "vTest: " << vTest << endl;
                // cout << "vTTest_rotated: " << vTest.transpose()*vTTest_rotated << endl;
                // error_cur += (weight * vTest.cross(vTTest_rotated).norm());
                error_cur += (vTest.cross(vTTest_rotated).norm());
            }
            if(is_debugging){
                cout << "**current paired modes' (weighted) error is: " << error_cur/count << " **\n" << endl;
            }
            if(error_cur/count < error){
                error = error_cur/count;
                _R = R_;
                best_paired_modes = i;
            }
        }
        if(is_debugging){
            cout << "---------------------------------------------------------" << endl;
            cout << "------------best paired mode IS: " << paired_modes[best_paired_modes][0] << " and " << paired_modes[best_paired_modes][1] << " ------------------" << endl;
        }
        valid_points = min(_dataModes->mode_count[paired_modes[best_paired_modes][0]], _dataModes->mode_count[paired_modes[best_paired_modes][1]]);
    }
    else if(count_valid_mode==2){
        Eigen::Vector3d v1{0,0,0};
        Eigen::Vector3d v11{0,0,0};
        Eigen::Vector3d v2{0,0,0};
        Eigen::Vector3d v22{0,0,0};

        if(is_debugging){
            cout << "=====================================================================================" << endl;
            cout << "mode equal to 2" << endl;
        }
        v1 = {_dataModes->mode_normals_x_last[selected_mode[0]].findMedian(), 
                _dataModes->mode_normals_y_last[selected_mode[0]].findMedian(), 
                    _dataModes->mode_normals_z_last[selected_mode[0]].findMedian()};
        v11 = {_dataModes->mode_normals_x[selected_mode[0]].findMedian(), 
                _dataModes->mode_normals_y[selected_mode[0]].findMedian(), 
                    _dataModes->mode_normals_z[selected_mode[0]].findMedian()};
        v2 = {_dataModes->mode_normals_x_last[selected_mode[1]].findMedian(), 
                _dataModes->mode_normals_y_last[selected_mode[1]].findMedian(), 
                    _dataModes->mode_normals_z_last[selected_mode[1]].findMedian()};
        v22 = {_dataModes->mode_normals_x[selected_mode[1]].findMedian(), 
                _dataModes->mode_normals_y[selected_mode[1]].findMedian(), 
                    _dataModes->mode_normals_z[selected_mode[1]].findMedian()};
        v22 = v22.normalized();
        v2 = v2.normalized();
        v11 = v11.normalized();
        v1 = v1.normalized();

        if(is_debugging){
            cout << "=====================================================================================" << endl;
        }
        valid_points = min(_dataModes->mode_count[selected_mode[0]], _dataModes->mode_count[selected_mode[1]]);
        rotationFromTwoPlanes(v1, v11, v2, v22, _R);
    }
    else{
        cerr << "only one valid plane!" << endl;
    }
     
}

inline void NI_SLAM::EfficientNormal2RotationMat(cv::Mat &_normalsCV_last, cv::Mat &_normalsCV, Eigen::Matrix3d &_R, cv::Mat &_AnglesMap)
{
    cv::Vec3d zeros{0, 0, 0};
    cv::Vec3d N{1,0,0};
    vector<cv::Vec3d> modes; // save different plane modes
    vector<int> mode_count; // to record the number of normals in each plane mode 
    vector<vector<Eigen::Vector2i>> mode_coor; // save the pixel coordinates of normals for each plane mode (may be optimized later) 
    vector<Eigen::Vector2i> tmp_coor; // to create a new mode in mode_coor
    vector<DynamicMedian> mode_normals_x;
    vector<DynamicMedian> mode_normals_y;
    vector<DynamicMedian> mode_normals_z;
    vector<DynamicMedian> mode_normals_x_last;
    vector<DynamicMedian> mode_normals_y_last;
    vector<DynamicMedian> mode_normals_z_last;
    DynamicMedian tmp_mode_normals;


    //** this loop is to perform selection mechanism 
    for(int v=0; v<_normalsCV.rows;v++)
        for(int u=0; u<_normalsCV.cols;u++)
        {
            //** exclude invalid points (noise from depth sensor)
            if((_normalsCV.at<cv::Vec3d>(v, u) == zeros) || (_normalsCV_last.at<cv::Vec3d>(v, u) == zeros) || (v <= 10) || (u <= 10) || (abs(v-_normalsCV.rows) <= 10) || (abs(u-_normalsCV.cols) <= 10))
            {
                continue;
            }
            //** exclude non-overlapped points
            if((_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u)).val[0] < 0.9){ // icl: 0.9){
                continue;
            }
            //** aim to cancel the points on edge
            if((_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v-1, u)).val[0] < 0.999 ||
            (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v+1, u)).val[0] < 0.999 ||
            (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v, u-1)).val[0] < 0.999 ||
            (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v, u+1)).val[0] < 0.999 ||
            (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v-1, u)).val[0] < 0.999 ||
            (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v+1, u)).val[0] < 0.999 ||
            (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u-1)).val[0] < 0.999 ||
            (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u+1)).val[0] < 0.999){
                continue;
            }
            //** perform mode selection mechanism
            int sizeofMode = static_cast<int>(modes.size());
            if(sizeofMode == 0){

                // if(debug){
                //     cout << "============find new mode[" << static_cast<int>(modes.size()) << "]!!!" << "at (" << v << ", " << u << ")===============" << endl;
                // }
                //** the below four lines are used to create a new mode
                //** include modes updating/mode_coor updating/saved_selected_mode_normals updating/mode_count updating
                modes.push_back(_normalsCV.at<cv::Vec3d>(v, u));
                mode_coor.push_back(tmp_coor);
                // saved_selected_mode_normals.push_back(tmp_saved_selected_mode_normals);
                // saved_selected_mode_normals_last.push_back(tmp_saved_selected_mode_normals);
                mode_normals_x.push_back(tmp_mode_normals);
                mode_normals_x_last.push_back(tmp_mode_normals);
                mode_normals_y.push_back(tmp_mode_normals);
                mode_normals_y_last.push_back(tmp_mode_normals);
                mode_normals_z.push_back(tmp_mode_normals);
                mode_normals_z_last.push_back(tmp_mode_normals);
                mode_count.push_back(0);
            }
            else{
                bool found_mode = false;
                //** perform normal-mode matching
                for(int i=0; i<sizeofMode; i++){
                    if((modes[i].t() * _normalsCV.at<cv::Vec3d>(v, u)).val[0] >= 0.95){ // icl: 0.999){
                        found_mode = true;
                        Eigen::Vector2i x{v, u};
                        mode_coor[i].push_back(x);
                        mode_normals_x[i].addNum(_normalsCV.at<cv::Vec3d>(v, u)[0]);
                        mode_normals_x_last[i].addNum(_normalsCV_last.at<cv::Vec3d>(v, u)[0]);
                        mode_normals_y[i].addNum(_normalsCV.at<cv::Vec3d>(v, u)[1]);
                        mode_normals_y_last[i].addNum(_normalsCV_last.at<cv::Vec3d>(v, u)[1]);
                        mode_normals_z[i].addNum(_normalsCV.at<cv::Vec3d>(v, u)[2]);
                        mode_normals_z_last[i].addNum(_normalsCV_last.at<cv::Vec3d>(v, u)[2]);
                        mode_count[i]++;
                        modes[i] = cv::Vec3d(mode_normals_x[i].findMedian(), mode_normals_y[i].findMedian(), mode_normals_z[i].findMedian());
                        break;
                    }
                }
                if(found_mode == false){
                    if((_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v-1, u)).val[0] < 0.999 ||
                    (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v+1, u)).val[0] < 0.999 ||
                    (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v, u-1)).val[0] < 0.999 ||
                    (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v, u+1)).val[0] < 0.999 ||
                    (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v-1, u)).val[0] < 0.999 ||
                    (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v+1, u)).val[0] < 0.999 ||
                    (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u-1)).val[0] < 0.999 ||
                    (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u+1)).val[0] < 0.999){
                        continue;
                    }
                    // if(debug){
                    //     cout << "============find new mode[" << static_cast<int>(modes.size()) << "]!!!" << "at (" << v << ", " << u << ")===============" << endl;
                    // }
                    modes.push_back(_normalsCV.at<cv::Vec3d>(v, u));
                    mode_coor.push_back(tmp_coor);
                    // saved_selected_mode_normals.push_back(tmp_saved_selected_mode_normals);
                    // saved_selected_mode_normals_last.push_back(tmp_saved_selected_mode_normals);
                    mode_normals_x.push_back(tmp_mode_normals);
                    mode_normals_x_last.push_back(tmp_mode_normals);
                    mode_normals_y.push_back(tmp_mode_normals);
                    mode_normals_y_last.push_back(tmp_mode_normals);
                    mode_normals_z.push_back(tmp_mode_normals);
                    mode_normals_z_last.push_back(tmp_mode_normals);
                    mode_count.push_back(0);
                }
            }
        }
    
    //** After finishing selecting modes
    int sizeofMode = static_cast<int>(modes.size()); // sizeofmode saves the amount of modes which indicates how many planes there are
    // Eigen::VectorXi mode_count_eigen = Eigen::Map<Eigen::VectorXi, Eigen::Unaligned>(mode_count.data(), mode_count.size());
    // Eigen::Array<bool, Eigen::Dynamic, 1> mask = (mode_count_eigen.array()>12000);
    // int count_valid_mode = mask.count(); // define a counter to count the modes with points more than 12000
    int count_valid_mode{0};
    vector<int> selected_mode; // define a vector to save the number of selected modes
    // cout << "how many: " << modes.size() << endl;
    // cout << count_valid_mode << endl;
    int sumPoints{0};
    for(int i=0; i<sizeofMode; i++){ 
        if(mode_count[i] > 12000){
            if(is_debugging){
                cout << "**************************mode" << i << ": " << modes[i] << endl;
                cout << "totally " << mode_count[i] << " points" << endl;
            }

            sumPoints+=mode_count[i];
            selected_mode.push_back(i);
            count_valid_mode++;
        }
    }
    if(is_debugging){
        cout << count_valid_mode << endl;
    }
    
    int count_second_max{0};
    //** if only one mode is more than 10000 points, will select one more mode if this new mode is more than 5000
    //** otherwise, will select two modes if they are less than 5000
    if(count_valid_mode<2){
        cout << "=====================================================================================" << endl;
        cout << "mode smaller than 2, will select one (>5000 points) or more (<5000 points) mode from plane with points less than 10000" << endl;
        selected_mode.push_back(0);
        for(int i=0; i<sizeofMode; i++){
            if(i == selected_mode[0]){
                continue;
            }
            if(mode_count[i]>=3000 && mode_count[i]<=10000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_second_max){
                count_second_max = mode_count[i];
                selected_mode[1] = i;
            }
        }
        cout << "**************************mode" << selected_mode[1] << ": " << modes[selected_mode[1]] << endl;
        cout << "totally " << mode_coor[selected_mode[1]].size() << " points" << endl;
        if(count_second_max != 0){
            count_valid_mode++;
        }
        
        if(count_second_max<5000){
            selected_mode.push_back(0);
            int count_third_max{0};
            for(int i=0; i<sizeofMode; i++){
                if(i == selected_mode[0] || i == selected_mode[1]){
                    continue;
                }
                if(mode_count[i]>=1000 && mode_count[i]<=10000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_third_max &&
                (modes[i].t() * modes[selected_mode[1]]).val[0]<0.5){
                    count_third_max = mode_count[i];
                    selected_mode[2] = i;
                }
            }
            if(count_third_max != 0){
                count_valid_mode++;
                cout << "**************************mode" << selected_mode[2] << ": " << modes[selected_mode[2]] << endl;
                cout << "totally " << mode_coor[selected_mode[2]].size() << " points" << endl;
            }

        }
        if(count_valid_mode == 1){
            cout << "cannot work, because only one plane exists" << endl;
        }
    }

    // if(count_valid_mode<2){
    //     if(is_debugging){
    //         cout << "=====================================================================================" << endl;
    //         cout << "mode smaller than 2, will select one (>12000 points) or more (<12000 points) mode from plane with points less than 12000" << endl;
    //     }
    //     selected_mode.push_back(0);
    //     for(int i=0; i<sizeofMode; i++){
    //         if(i == selected_mode[0]){
    //             continue;
    //         }
    //         if(mode_count[i]>=1000 && mode_count[i]<=12000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_second_max){
    //             count_second_max = mode_count[i];
    //             selected_mode[1] = i;
    //             // selected_mode.push_back(i);
    //         }
    //     }

    //     if(count_second_max != 0){
    //         count_valid_mode++;
    //         if(is_debugging){
    //             cout << "**************************mode" << selected_mode[1] << ": " << modes[selected_mode[1]] << endl;
    //             cout << "totally (>1000) " << mode_coor[selected_mode[1]].size() << " points" << endl;
    //         }
    //     }
    //     else{
    //         selected_mode.pop_back();
    //     }
        
    //     if(count_valid_mode==2 && count_second_max<12000){
    //         selected_mode.push_back(0);
    //         int count_third_max{0};
    //         for(int i=0; i<sizeofMode; i++){
    //             if(i == selected_mode[0] || i == selected_mode[1]){
    //                 continue;
    //             }
    //             if(mode_count[i]>=1000 && mode_count[i]<=12000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_third_max &&
    //             (modes[i].t() * modes[selected_mode[1]]).val[0]<0.5){
    //                 count_third_max = mode_count[i];
    //                 selected_mode[2] = i;
    //             }
    //         }
    //         if(count_third_max != 0){
    //             count_valid_mode++;
    //             if(is_debugging){
    //                 cout << "**************************mode" << selected_mode[2] << ": " << modes[selected_mode[2]] << endl;
    //                 cout << "totally (>1000) " << mode_coor[selected_mode[2]].size() << " points" << endl;
    //             }
    //         }
    //         else{
    //             selected_mode.pop_back();
    //         }

    //     }
    //     else if(count_valid_mode==1){
    //         selected_mode.push_back(0);
    //         for(int i=0; i<sizeofMode; i++){
    //             if(i == selected_mode[0]){
    //                 continue;
    //             }
    //             if(mode_count[i]>=500 && mode_count[i]<=12000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_second_max){
    //                 count_second_max = mode_count[i];
    //                 selected_mode[1] = i;
    //             }
    //         }
    //         if(count_second_max != 0){
    //             count_valid_mode++;
    //             if(is_debugging){
    //                 cout << "**************************mode" << selected_mode[1] << ": " << modes[selected_mode[1]] << endl;
    //                 cout << "totally (>500) " << mode_coor[selected_mode[1]].size() << " points" << endl;
    //             }
    //         }
    //         else{
    //             selected_mode.pop_back();
    //             std::cerr << "only one plane detected!" << endl;
    //         }

    //     }
    // }

    //** if more than 2 valid modes, we have to calculate rotation matrix from each pair and evaluate which is the best
    //** paired_modes used to store number of a pair of modes
    //** first for loop used to generate all combinations of modes
    //** traverse each paired_mode to generate a rotation matrix, and use the rest of modes to evaluate the rotation matrix
    //** note that 'paired_modes' save the No. of modes, 'selected_mode' also save the No. of modes
    if(count_valid_mode>2){
        if(is_debugging){
            cout << "count valid mode: " << count_valid_mode << endl;
            cout << "---------------------------------------------------------" << endl;
            cout << "mode more than 2, will compare which paired modes is best" << endl;
        }

        vector<vector<int>> paired_modes;
        for(int i=0; i<count_valid_mode; i++)
            for(int j=i+1; j<count_valid_mode; j++){
                paired_modes.push_back({selected_mode[i],selected_mode[j]});
        }

        Eigen::Vector3d v1;
        Eigen::Vector3d v11;
        Eigen::Vector3d v2;
        Eigen::Vector3d v22;
        Eigen::Matrix3d R_;
        int best_paired_modes{0};
        double error{999999};
        for(size_t i=0; i<(paired_modes.size()); i++){
            if(is_debugging){
                cout << "**current paired modes is: " << paired_modes[i][0] << " and " << paired_modes[i][1] << "**" << endl;
            }
            // v1 = oneVectorFromMultiVectors(normalsCV_last, mode_coor[paired_modes[i][0]]);
            // v11 = oneVectorFromMultiVectors(normalsCV, mode_coor[paired_modes[i][0]]);
            // v2 = oneVectorFromMultiVectors(normalsCV_last, mode_coor[paired_modes[i][1]]);
            // v22 = oneVectorFromMultiVectors(normalsCV, mode_coor[paired_modes[i][1]]);

            // v1 = oneVectorFromMultiVectors(mode_count[paired_modes[i][0]], saved_selected_mode_normals_last[paired_modes[i][0]]);
            // v11 = oneVectorFromMultiVectors(mode_count[paired_modes[i][0]], saved_selected_mode_normals[paired_modes[i][0]]);
            // v2 = oneVectorFromMultiVectors(mode_count[paired_modes[i][1]], saved_selected_mode_normals_last[paired_modes[i][1]]);
            // v22 = oneVectorFromMultiVectors(mode_count[paired_modes[i][1]], saved_selected_mode_normals[paired_modes[i][1]]);

            v1 = {mode_normals_x_last[paired_modes[i][0]].findMean(), 
                    mode_normals_y_last[paired_modes[i][0]].findMean(), 
                        mode_normals_z_last[paired_modes[i][0]].findMean()};
            v11 = {mode_normals_x[paired_modes[i][0]].findMean(), 
                    mode_normals_y[paired_modes[i][0]].findMean(), 
                        mode_normals_z[paired_modes[i][0]].findMean()};
            v2 = {mode_normals_x_last[paired_modes[i][1]].findMean(), 
                    mode_normals_y_last[paired_modes[i][1]].findMean(), 
                        mode_normals_z_last[paired_modes[i][1]].findMean()};
            v22 = {mode_normals_x[paired_modes[i][1]].findMean(), 
                    mode_normals_y[paired_modes[i][1]].findMean(), 
                        mode_normals_z[paired_modes[i][1]].findMean()};
            v22 = v22.normalized();
            v2 = v2.normalized();
            v11 = v11.normalized();
            v1 = v1.normalized();            

            // cout << "**v1 is : " << v1[0] << ", " << v1[1] << ", " << v1[2] << " **" << endl;
            // cout << "new v1 is : " << mode_normals_x_last[paired_modes[i][0]].findMean() << "," <<
            //         mode_normals_y_last[paired_modes[i][0]].findMean() << "," << 
            //             mode_normals_z_last[paired_modes[i][0]].findMean() << endl;
            // cout << "**v11 is : " << v11[0] << ", " << v11[1] << ", " << v11[2] << " **" << endl;
            // cout << "new v11 is : " << mode_normals_x[paired_modes[i][0]].findMean() << "," <<
            //         mode_normals_y[paired_modes[i][0]].findMean() << "," << 
            //             mode_normals_z[paired_modes[i][0]].findMean() << endl;
            // cout << "**v2 is : " << v2[0] << ", " << v2[1] << ", " << v2[2] << " **" << endl;
            // cout << "new v2 is : " << mode_normals_x_last[paired_modes[i][1]].findMean() << "," <<
            //         mode_normals_y_last[paired_modes[i][1]].findMean() << "," << 
            //             mode_normals_z_last[paired_modes[i][1]].findMean() << endl;
            // cout << "**v22 is : " << v22[0] << ", " << v22[1] << ", " << v22[2] << " **" << endl;
            // cout << "new v22 is : " << mode_normals_x[paired_modes[i][1]].findMean() << "," <<
            //         mode_normals_y[paired_modes[i][1]].findMean() << "," << 
            //             mode_normals_z[paired_modes[i][1]].findMean() << endl;
            rotationFromTwoPlanes(v1, v11, v2, v22, R_);
            double count{0.0};
            double error_cur{0};
            for(int j=0; j<count_valid_mode; j++){
                if(selected_mode[j]==paired_modes[i][1] || selected_mode[j]==paired_modes[i][0]){
                    continue;
                }
                count += 1.0;
                // float weight = float(mode_count[selected_mode[j]]) / float(sumPoints);
                // Eigen::Vector3d vTest = oneVectorFromMultiVectors(normalsCV_last, mode_coor[selected_mode[j]]);
                // Eigen::Vector3d vTTest = oneVectorFromMultiVectors(normalsCV, mode_coor[selected_mode[j]]);
                
                // Eigen::Vector3d vTest = oneVectorFromMultiVectors(mode_count[selected_mode[j]], saved_selected_mode_normals_last[selected_mode[j]]);
                // Eigen::Vector3d vTTest = oneVectorFromMultiVectors(mode_count[selected_mode[j]], saved_selected_mode_normals[selected_mode[j]]);

                Eigen::Vector3d vTest = {mode_normals_x_last[selected_mode[j]].findMean(), 
                                            mode_normals_y_last[selected_mode[j]].findMean(), 
                                                mode_normals_z_last[selected_mode[j]].findMean()};
                Eigen::Vector3d vTTest = {mode_normals_x[selected_mode[j]].findMean(), 
                                            mode_normals_y[selected_mode[j]].findMean(), 
                                                mode_normals_z[selected_mode[j]].findMean()};  
                vTest = vTest.normalized();
                vTTest = vTTest.normalized();
                
                Eigen::Vector3d vTTest_rotated = R_ * vTTest;
                // cout << "vTest: " << vTest << endl;
                // cout << "vTTest_rotated: " << vTest.transpose()*vTTest_rotated << endl;
                // error_cur += (weight * vTest.cross(vTTest_rotated).norm());
                error_cur += (vTest.cross(vTTest_rotated).norm());
            }
            if(is_debugging){
                cout << "**current paired modes' (weighted) error is: " << error_cur/count << " **\n" << endl;
            }
            if(error_cur/count < error){
                error = error_cur/count;
                _R = R_;
                best_paired_modes = i;
            }
        }
        if(is_debugging){
            cout << "---------------------------------------------------------" << endl;
            cout << "------------best paired mode IS: " << paired_modes[best_paired_modes][0] << " and " << paired_modes[best_paired_modes][1] << " ------------------" << endl;
        }
        valid_points = min(mode_count[paired_modes[best_paired_modes][0]], mode_count[paired_modes[best_paired_modes][1]]);
    }
    else if(count_valid_mode==2){
        Eigen::Vector3d v1{0,0,0};
        Eigen::Vector3d v11{0,0,0};
        Eigen::Vector3d v2{0,0,0};
        Eigen::Vector3d v22{0,0,0};

        if(is_debugging){
            cout << "=====================================================================================" << endl;
            cout << "mode equal to 2" << endl;
        }
        v1 = {mode_normals_x_last[selected_mode[0]].findMean(), 
                mode_normals_y_last[selected_mode[0]].findMean(), 
                    mode_normals_z_last[selected_mode[0]].findMean()};
        v11 = {mode_normals_x[selected_mode[0]].findMean(), 
                mode_normals_y[selected_mode[0]].findMean(), 
                    mode_normals_z[selected_mode[0]].findMean()};
        v2 = {mode_normals_x_last[selected_mode[1]].findMean(), 
                mode_normals_y_last[selected_mode[1]].findMean(), 
                    mode_normals_z_last[selected_mode[1]].findMean()};
        v22 = {mode_normals_x[selected_mode[1]].findMean(), 
                mode_normals_y[selected_mode[1]].findMean(), 
                    mode_normals_z[selected_mode[1]].findMean()};
        v22 = v22.normalized();
        v2 = v2.normalized();
        v11 = v11.normalized();
        v1 = v1.normalized();

        if(is_debugging){
            cout << "=====================================================================================" << endl;
        }
        valid_points = min(mode_count[selected_mode[0]], mode_count[selected_mode[1]]);
        rotationFromTwoPlanes(v1, v11, v2, v22, _R);
    }
    else{
        cerr << "only one valid plane!" << endl;
    }
     
}

// inline void NI_SLAM::EfficientNormal2RotationMat(cv::Mat &_normalsCV_last, cv::Mat &_normalsCV, Eigen::Matrix3d &_R, cv::Mat &_AnglesMap)
// {
//     cv::Vec3d zeros{0, 0, 0};
//     cv::Vec3d N{1,0,0};
//     vector<cv::Vec3d> modes; // save different plane modes
//     vector<int> mode_count; // to record the number of normals in each plane mode 
//     vector<vector<Eigen::Vector2i>> mode_coor; // save the pixel coordinates of normals for each plane mode (may be optimized later) 
//     vector<Eigen::Vector2i> tmp_coor; // to create a new mode in mode_coor
//     vector<DynamicMedian> mode_normals_x;
//     vector<DynamicMedian> mode_normals_y;
//     vector<DynamicMedian> mode_normals_z;
//     vector<DynamicMedian> mode_normals_x_last;
//     vector<DynamicMedian> mode_normals_y_last;
//     vector<DynamicMedian> mode_normals_z_last;
//     DynamicMedian tmp_mode_normals;

    
//     //** this loop is to perform selection mechanism 
//     //** 第一阶段：遍历 _normalsCV 来找出modes
//     for (int v = 0; v < _normalsCV.rows; v++) {
//         for (int u = 0; u < _normalsCV.cols; u++) {

//             //** 排除无效点
//             if (_normalsCV.at<cv::Vec3d>(v, u) == zeros || 
//                 (v <= 10) || (u <= 10) || 
//                 (abs(v - _normalsCV.rows) <= 10) || (abs(u - _normalsCV.cols) <= 10)) {
//                 continue;
//             }

//             int sizeofMode = static_cast<int>(modes.size());
//             if (sizeofMode == 0) {
//                 modes.push_back(_normalsCV.at<cv::Vec3d>(v, u));
//                 mode_coor.push_back(tmp_coor);
//                 mode_normals_x.push_back(tmp_mode_normals);
//                 mode_normals_y.push_back(tmp_mode_normals);
//                 mode_normals_z.push_back(tmp_mode_normals);
//                 mode_normals_x_last.push_back(tmp_mode_normals);
//                 mode_normals_y_last.push_back(tmp_mode_normals);
//                 mode_normals_z_last.push_back(tmp_mode_normals);
//                 mode_count.push_back(1);
//             } else {
//                 bool found_mode = false;
//                 for (int i = 0; i < sizeofMode; i++) {
//                     if ((modes[i].t() * _normalsCV.at<cv::Vec3d>(v, u)).val[0] >= 0.95) {
//                         found_mode = true;
//                         mode_normals_x[i].addNum(_normalsCV.at<cv::Vec3d>(v, u)[0]);
//                         mode_normals_y[i].addNum(_normalsCV.at<cv::Vec3d>(v, u)[1]);
//                         mode_normals_z[i].addNum(_normalsCV.at<cv::Vec3d>(v, u)[2]);
//                         mode_count[i]++;
//                         break;
//                     }
//                 }
//                 if (found_mode == false) {
//                     modes.push_back(_normalsCV.at<cv::Vec3d>(v, u));
//                     mode_coor.push_back(tmp_coor);
//                     mode_normals_x.push_back(tmp_mode_normals);
//                     mode_normals_y.push_back(tmp_mode_normals);
//                     mode_normals_z.push_back(tmp_mode_normals);
//                     mode_normals_x_last.push_back(tmp_mode_normals);
//                     mode_normals_y_last.push_back(tmp_mode_normals);
//                     mode_normals_z_last.push_back(tmp_mode_normals);
//                     mode_count.push_back(1);
//                 }
//             }
//         }
//     }
//     //** 第二阶段：遍历 _normalsCV_last 来找出与第一阶段modes相似的法线
//     for (int v = 0; v < _normalsCV_last.rows; v++) {
//         for (int u = 0; u < _normalsCV_last.cols; u++) {

//             //** 排除无效点
//             if (_normalsCV_last.at<cv::Vec3d>(v, u) == zeros ||
//                 (v <= 10) || (u <= 10) ||
//                 (abs(v - _normalsCV_last.rows) <= 10) || (abs(u - _normalsCV_last.cols) <= 10)) {
//                 continue;
//             }
//             // cout << "here!!!!!!!!!!!!! 1" << endl;
//             for (int i = 0; i < static_cast<int>(modes.size()); i++) {
//                 if ((modes[i].t() * _normalsCV_last.at<cv::Vec3d>(v, u)).val[0] >= 0.95) {
//                     mode_normals_x_last[i].addNum(_normalsCV_last.at<cv::Vec3d>(v, u)[0]);
//                     mode_normals_y_last[i].addNum(_normalsCV_last.at<cv::Vec3d>(v, u)[1]);
//                     mode_normals_z_last[i].addNum(_normalsCV_last.at<cv::Vec3d>(v, u)[2]);
//                     mode_count[i]++;
//                     break;
//                 }
//             }
//             // cout << "here!!!!!!!!!!!!! 2" << endl;
//         }
//     }

//     // cout << "here!!!!!!!!!!!!! 3" << endl;
    
//     //** After finishing selecting modes
//     int sizeofMode = static_cast<int>(modes.size()); // sizeofmode saves the amount of modes which indicates how many planes there are
//     // Eigen::VectorXi mode_count_eigen = Eigen::Map<Eigen::VectorXi, Eigen::Unaligned>(mode_count.data(), mode_count.size());
//     // Eigen::Array<bool, Eigen::Dynamic, 1> mask = (mode_count_eigen.array()>12000);
//     // int count_valid_mode = mask.count(); // define a counter to count the modes with points more than 12000
//     int count_valid_mode{0};
//     vector<int> selected_mode; // define a vector to save the number of selected modes
//     // cout << "how many: " << modes.size() << endl;
//     // cout << count_valid_mode << endl;
//     int sumPoints{0};
//     for(int i=0; i<sizeofMode; i++){ 
//         if(mode_count[i] > 12000){
//             if(is_debugging){
//                 cout << "**************************mode" << i << ": " << modes[i] << endl;
//                 cout << "totally " << mode_count[i] << " points" << endl;
//             }

//             sumPoints+=mode_count[i];
//             selected_mode.push_back(i);
//             count_valid_mode++;
//         }
//     }
//     if(is_debugging){
//         cout << count_valid_mode << endl;
//     }
    
//     int count_second_max{0};
//     //** if only one mode is more than 10000 points, will select one more mode if this new mode is more than 5000
//     //** otherwise, will select two modes if they are less than 5000
//     if(count_valid_mode<2){
//         cout << "=====================================================================================" << endl;
//         cout << "mode smaller than 2, will select one (>5000 points) or more (<5000 points) mode from plane with points less than 10000" << endl;
//         selected_mode.push_back(0);
//         for(int i=0; i<sizeofMode; i++){
//             if(i == selected_mode[0]){
//                 continue;
//             }
//             if(mode_count[i]>=3000 && mode_count[i]<=10000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_second_max){
//                 count_second_max = mode_count[i];
//                 selected_mode[1] = i;
//             }
//         }
//         cout << "**************************mode" << selected_mode[1] << ": " << modes[selected_mode[1]] << endl;
//         cout << "totally " << mode_coor[selected_mode[1]].size() << " points" << endl;
//         if(count_second_max != 0){
//             count_valid_mode++;
//         }
        
//         if(count_second_max<5000){
//             selected_mode.push_back(0);
//             int count_third_max{0};
//             for(int i=0; i<sizeofMode; i++){
//                 if(i == selected_mode[0] || i == selected_mode[1]){
//                     continue;
//                 }
//                 if(mode_count[i]>=1000 && mode_count[i]<=10000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_third_max &&
//                 (modes[i].t() * modes[selected_mode[1]]).val[0]<0.5){
//                     count_third_max = mode_count[i];
//                     selected_mode[2] = i;
//                 }
//             }
//             if(count_third_max != 0){
//                 count_valid_mode++;
//                 cout << "**************************mode" << selected_mode[2] << ": " << modes[selected_mode[2]] << endl;
//                 cout << "totally " << mode_coor[selected_mode[2]].size() << " points" << endl;
//             }

//         }
//         if(count_valid_mode == 1){
//             cout << "cannot work, because only one plane exists" << endl;
//         }
//     }

//     // if(count_valid_mode<2){
//     //     if(is_debugging){
//     //         cout << "=====================================================================================" << endl;
//     //         cout << "mode smaller than 2, will select one (>12000 points) or more (<12000 points) mode from plane with points less than 12000" << endl;
//     //     }
//     //     selected_mode.push_back(0);
//     //     for(int i=0; i<sizeofMode; i++){
//     //         if(i == selected_mode[0]){
//     //             continue;
//     //         }
//     //         if(mode_count[i]>=1000 && mode_count[i]<=12000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_second_max){
//     //             count_second_max = mode_count[i];
//     //             selected_mode[1] = i;
//     //             // selected_mode.push_back(i);
//     //         }
//     //     }

//     //     if(count_second_max != 0){
//     //         count_valid_mode++;
//     //         if(is_debugging){
//     //             cout << "**************************mode" << selected_mode[1] << ": " << modes[selected_mode[1]] << endl;
//     //             cout << "totally (>1000) " << mode_coor[selected_mode[1]].size() << " points" << endl;
//     //         }
//     //     }
//     //     else{
//     //         selected_mode.pop_back();
//     //     }
        
//     //     if(count_valid_mode==2 && count_second_max<12000){
//     //         selected_mode.push_back(0);
//     //         int count_third_max{0};
//     //         for(int i=0; i<sizeofMode; i++){
//     //             if(i == selected_mode[0] || i == selected_mode[1]){
//     //                 continue;
//     //             }
//     //             if(mode_count[i]>=1000 && mode_count[i]<=12000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_third_max &&
//     //             (modes[i].t() * modes[selected_mode[1]]).val[0]<0.5){
//     //                 count_third_max = mode_count[i];
//     //                 selected_mode[2] = i;
//     //             }
//     //         }
//     //         if(count_third_max != 0){
//     //             count_valid_mode++;
//     //             if(is_debugging){
//     //                 cout << "**************************mode" << selected_mode[2] << ": " << modes[selected_mode[2]] << endl;
//     //                 cout << "totally (>1000) " << mode_coor[selected_mode[2]].size() << " points" << endl;
//     //             }
//     //         }
//     //         else{
//     //             selected_mode.pop_back();
//     //         }

//     //     }
//     //     else if(count_valid_mode==1){
//     //         selected_mode.push_back(0);
//     //         for(int i=0; i<sizeofMode; i++){
//     //             if(i == selected_mode[0]){
//     //                 continue;
//     //             }
//     //             if(mode_count[i]>=500 && mode_count[i]<=12000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_second_max){
//     //                 count_second_max = mode_count[i];
//     //                 selected_mode[1] = i;
//     //             }
//     //         }
//     //         if(count_second_max != 0){
//     //             count_valid_mode++;
//     //             if(is_debugging){
//     //                 cout << "**************************mode" << selected_mode[1] << ": " << modes[selected_mode[1]] << endl;
//     //                 cout << "totally (>500) " << mode_coor[selected_mode[1]].size() << " points" << endl;
//     //             }
//     //         }
//     //         else{
//     //             selected_mode.pop_back();
//     //             std::cerr << "only one plane detected!" << endl;
//     //         }

//     //     }
//     // }

//     //** if more than 2 valid modes, we have to calculate rotation matrix from each pair and evaluate which is the best
//     //** paired_modes used to store number of a pair of modes
//     //** first for loop used to generate all combinations of modes
//     //** traverse each paired_mode to generate a rotation matrix, and use the rest of modes to evaluate the rotation matrix
//     //** note that 'paired_modes' save the No. of modes, 'selected_mode' also save the No. of modes
//     if(count_valid_mode>2){
//         if(is_debugging){
//             cout << "count valid mode: " << count_valid_mode << endl;
//             cout << "---------------------------------------------------------" << endl;
//             cout << "mode more than 2, will compare which paired modes is best" << endl;
//         }

//         vector<vector<int>> paired_modes;
//         for(int i=0; i<count_valid_mode; i++)
//             for(int j=i+1; j<count_valid_mode; j++){
//                 paired_modes.push_back({selected_mode[i],selected_mode[j]});
//         }

//         Eigen::Vector3d v1;
//         Eigen::Vector3d v11;
//         Eigen::Vector3d v2;
//         Eigen::Vector3d v22;
//         Eigen::Matrix3d R_;
//         int best_paired_modes{0};
//         double error{999999};
//         for(size_t i=0; i<(paired_modes.size()); i++){
//             if(is_debugging){
//                 cout << "**current paired modes is: " << paired_modes[i][0] << " and " << paired_modes[i][1] << "**" << endl;
//             }
//             // v1 = oneVectorFromMultiVectors(normalsCV_last, mode_coor[paired_modes[i][0]]);
//             // v11 = oneVectorFromMultiVectors(normalsCV, mode_coor[paired_modes[i][0]]);
//             // v2 = oneVectorFromMultiVectors(normalsCV_last, mode_coor[paired_modes[i][1]]);
//             // v22 = oneVectorFromMultiVectors(normalsCV, mode_coor[paired_modes[i][1]]);

//             // v1 = oneVectorFromMultiVectors(mode_count[paired_modes[i][0]], saved_selected_mode_normals_last[paired_modes[i][0]]);
//             // v11 = oneVectorFromMultiVectors(mode_count[paired_modes[i][0]], saved_selected_mode_normals[paired_modes[i][0]]);
//             // v2 = oneVectorFromMultiVectors(mode_count[paired_modes[i][1]], saved_selected_mode_normals_last[paired_modes[i][1]]);
//             // v22 = oneVectorFromMultiVectors(mode_count[paired_modes[i][1]], saved_selected_mode_normals[paired_modes[i][1]]);

//             v1 = {mode_normals_x_last[paired_modes[i][0]].findMean(), 
//                     mode_normals_y_last[paired_modes[i][0]].findMean(), 
//                         mode_normals_z_last[paired_modes[i][0]].findMean()};
//             v11 = {mode_normals_x[paired_modes[i][0]].findMean(), 
//                     mode_normals_y[paired_modes[i][0]].findMean(), 
//                         mode_normals_z[paired_modes[i][0]].findMean()};
//             v2 = {mode_normals_x_last[paired_modes[i][1]].findMean(), 
//                     mode_normals_y_last[paired_modes[i][1]].findMean(), 
//                         mode_normals_z_last[paired_modes[i][1]].findMean()};
//             v22 = {mode_normals_x[paired_modes[i][1]].findMean(), 
//                     mode_normals_y[paired_modes[i][1]].findMean(), 
//                         mode_normals_z[paired_modes[i][1]].findMean()};
//             v22 = v22.normalized();
//             v2 = v2.normalized();
//             v11 = v11.normalized();
//             v1 = v1.normalized();            

//             // cout << "**v1 is : " << v1[0] << ", " << v1[1] << ", " << v1[2] << " **" << endl;
//             // cout << "new v1 is : " << mode_normals_x_last[paired_modes[i][0]].findMean() << "," <<
//             //         mode_normals_y_last[paired_modes[i][0]].findMean() << "," << 
//             //             mode_normals_z_last[paired_modes[i][0]].findMean() << endl;
//             // cout << "**v11 is : " << v11[0] << ", " << v11[1] << ", " << v11[2] << " **" << endl;
//             // cout << "new v11 is : " << mode_normals_x[paired_modes[i][0]].findMean() << "," <<
//             //         mode_normals_y[paired_modes[i][0]].findMean() << "," << 
//             //             mode_normals_z[paired_modes[i][0]].findMean() << endl;
//             // cout << "**v2 is : " << v2[0] << ", " << v2[1] << ", " << v2[2] << " **" << endl;
//             // cout << "new v2 is : " << mode_normals_x_last[paired_modes[i][1]].findMean() << "," <<
//             //         mode_normals_y_last[paired_modes[i][1]].findMean() << "," << 
//             //             mode_normals_z_last[paired_modes[i][1]].findMean() << endl;
//             // cout << "**v22 is : " << v22[0] << ", " << v22[1] << ", " << v22[2] << " **" << endl;
//             // cout << "new v22 is : " << mode_normals_x[paired_modes[i][1]].findMean() << "," <<
//             //         mode_normals_y[paired_modes[i][1]].findMean() << "," << 
//             //             mode_normals_z[paired_modes[i][1]].findMean() << endl;
//             rotationFromTwoPlanes(v1, v11, v2, v22, R_);
//             double count{0.0};
//             double error_cur{0};
//             for(int j=0; j<count_valid_mode; j++){
//                 if(selected_mode[j]==paired_modes[i][1] || selected_mode[j]==paired_modes[i][0]){
//                     continue;
//                 }
//                 count += 1.0;
//                 // float weight = float(mode_count[selected_mode[j]]) / float(sumPoints);
//                 // Eigen::Vector3d vTest = oneVectorFromMultiVectors(normalsCV_last, mode_coor[selected_mode[j]]);
//                 // Eigen::Vector3d vTTest = oneVectorFromMultiVectors(normalsCV, mode_coor[selected_mode[j]]);
                
//                 // Eigen::Vector3d vTest = oneVectorFromMultiVectors(mode_count[selected_mode[j]], saved_selected_mode_normals_last[selected_mode[j]]);
//                 // Eigen::Vector3d vTTest = oneVectorFromMultiVectors(mode_count[selected_mode[j]], saved_selected_mode_normals[selected_mode[j]]);

//                 Eigen::Vector3d vTest = {mode_normals_x_last[selected_mode[j]].findMean(), 
//                                             mode_normals_y_last[selected_mode[j]].findMean(), 
//                                                 mode_normals_z_last[selected_mode[j]].findMean()};
//                 Eigen::Vector3d vTTest = {mode_normals_x[selected_mode[j]].findMean(), 
//                                             mode_normals_y[selected_mode[j]].findMean(), 
//                                                 mode_normals_z[selected_mode[j]].findMean()};  
//                 vTest = vTest.normalized();
//                 vTTest = vTTest.normalized();
                
//                 Eigen::Vector3d vTTest_rotated = R_ * vTTest;
//                 // cout << "vTest: " << vTest << endl;
//                 // cout << "vTTest_rotated: " << vTest.transpose()*vTTest_rotated << endl;
//                 // error_cur += (weight * vTest.cross(vTTest_rotated).norm());
//                 error_cur += (vTest.cross(vTTest_rotated).norm());
//             }
//             if(is_debugging){
//                 cout << "**current paired modes' (weighted) error is: " << error_cur/count << " **\n" << endl;
//             }
//             if(error_cur/count < error){
//                 error = error_cur/count;
//                 _R = R_;
//                 best_paired_modes = i;
//             }
//         }
//         if(is_debugging){
//             cout << "---------------------------------------------------------" << endl;
//             cout << "------------best paired mode IS: " << paired_modes[best_paired_modes][0] << " and " << paired_modes[best_paired_modes][1] << " ------------------" << endl;
//         }
//         valid_points = min(mode_count[paired_modes[best_paired_modes][0]], mode_count[paired_modes[best_paired_modes][1]]);
//     }
//     else if(count_valid_mode==2){
//         Eigen::Vector3d v1{0,0,0};
//         Eigen::Vector3d v11{0,0,0};
//         Eigen::Vector3d v2{0,0,0};
//         Eigen::Vector3d v22{0,0,0};

//         if(is_debugging){
//             cout << "=====================================================================================" << endl;
//             cout << "mode equal to 2" << endl;
//         }
//         v1 = {mode_normals_x_last[selected_mode[0]].findMean(), 
//                 mode_normals_y_last[selected_mode[0]].findMean(), 
//                     mode_normals_z_last[selected_mode[0]].findMean()};
//         v11 = {mode_normals_x[selected_mode[0]].findMean(), 
//                 mode_normals_y[selected_mode[0]].findMean(), 
//                     mode_normals_z[selected_mode[0]].findMean()};
//         v2 = {mode_normals_x_last[selected_mode[1]].findMean(), 
//                 mode_normals_y_last[selected_mode[1]].findMean(), 
//                     mode_normals_z_last[selected_mode[1]].findMean()};
//         v22 = {mode_normals_x[selected_mode[1]].findMean(), 
//                 mode_normals_y[selected_mode[1]].findMean(), 
//                     mode_normals_z[selected_mode[1]].findMean()};
//         v22 = v22.normalized();
//         v2 = v2.normalized();
//         v11 = v11.normalized();
//         v1 = v1.normalized();

//         if(is_debugging){
//             cout << "=====================================================================================" << endl;
//         }
//         valid_points = min(mode_count[selected_mode[0]], mode_count[selected_mode[1]]);
//         rotationFromTwoPlanes(v1, v11, v2, v22, _R);
//     }
//     else{
//         cerr << "only one valid plane!" << endl;
//     }
     
// }


inline void NI_SLAM::EfficientNormal2RotationMatLst(cv::Mat &_normalsCV_last, cv::Mat &_normalsCV, Eigen::Matrix3d &_R, cv::Mat &_AnglesMap, const CloudType::ConstPtr& _pcl_cloud_last, const CloudType::ConstPtr& _pcl_cloud)
{
    cv::Vec3d zeros{0, 0, 0};
    cv::Vec3d N{1,0,0};
    vector<cv::Vec3d> modes; // save different plane modes
    vector<int> mode_count; // to record the number of normals in each plane mode 
    vector<vector<Eigen::Vector2i>> mode_coor; // save the pixel coordinates of normals for each plane mode (may be optimized later) 
    vector<Eigen::Vector2i> tmp_coor; // to create a new mode in mode_coor
    vector<DynamicMedian> mode_normals_x;
    vector<DynamicMedian> mode_normals_y;
    vector<DynamicMedian> mode_normals_z;
    vector<DynamicMedian> mode_normals_x_last;
    vector<DynamicMedian> mode_normals_y_last;
    vector<DynamicMedian> mode_normals_z_last;
    DynamicMedian tmp_mode_normals;
    vector<vector<PlanePoint>> mode_planepoints_last;
    vector<vector<PlanePoint>> mode_planepoints;
    vector<PlanePoint> tmp_mode_planepoints;



    //** this loop is to perform selection mechanism 
    for(int v=0; v<_normalsCV.rows;v++)
        for(int u=0; u<_normalsCV.cols;u++)
        {
            //** exclude invalid points (noise from depth sensor)
            if((_normalsCV.at<cv::Vec3d>(v, u) == zeros) || (_normalsCV_last.at<cv::Vec3d>(v, u) == zeros) || (v <= 10) || (u <= 10) || (abs(v-_normalsCV.rows) <= 10) || (abs(u-_normalsCV.cols) <= 10))
            {
                continue;
            }
            //** exclude non-overlapped points
            if((_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u)).val[0] < 0.9){ // icl: 0.9){
                continue;
            }
            //** aim to cancel the points on edge
            if((_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v-1, u)).val[0] < 0.999 ||
            (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v+1, u)).val[0] < 0.999 ||
            (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v, u-1)).val[0] < 0.999 ||
            (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v, u+1)).val[0] < 0.999 ||
            (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v-1, u)).val[0] < 0.999 ||
            (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v+1, u)).val[0] < 0.999 ||
            (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u-1)).val[0] < 0.999 ||
            (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u+1)).val[0] < 0.999){
                continue;
            }
            //** perform mode selection mechanism
            int sizeofMode = static_cast<int>(modes.size());
            if(sizeofMode == 0){

                // if(debug){
                //     cout << "============find new mode[" << static_cast<int>(modes.size()) << "]!!!" << "at (" << v << ", " << u << ")===============" << endl;
                // }
                //** the below four lines are used to create a new mode
                //** include modes updating/mode_coor updating/saved_selected_mode_normals updating/mode_count updating
                modes.push_back(_normalsCV.at<cv::Vec3d>(v, u));
                mode_coor.push_back(tmp_coor);
                // saved_selected_mode_normals.push_back(tmp_saved_selected_mode_normals);
                // saved_selected_mode_normals_last.push_back(tmp_saved_selected_mode_normals);
                mode_normals_x.push_back(tmp_mode_normals);
                mode_normals_x_last.push_back(tmp_mode_normals);
                mode_normals_y.push_back(tmp_mode_normals);
                mode_normals_y_last.push_back(tmp_mode_normals);
                mode_normals_z.push_back(tmp_mode_normals);
                mode_normals_z_last.push_back(tmp_mode_normals);
                mode_planepoints.push_back(tmp_mode_planepoints);
                mode_planepoints_last.push_back(tmp_mode_planepoints);
                mode_count.push_back(0);
            }
            else{
                bool found_mode = false;
                //** perform normal-mode matching
                for(int i=0; i<sizeofMode; i++){
                    if((modes[i].t() * _normalsCV.at<cv::Vec3d>(v, u)).val[0] >= 0.999){ // icl: 0.999){
                        found_mode = true;
                        Eigen::Vector2i x{v, u};
                        mode_coor[i].push_back(x);
                        mode_normals_x[i].addNum(_normalsCV.at<cv::Vec3d>(v, u)[0]);
                        mode_normals_x_last[i].addNum(_normalsCV_last.at<cv::Vec3d>(v, u)[0]);
                        mode_normals_y[i].addNum(_normalsCV.at<cv::Vec3d>(v, u)[1]);
                        mode_normals_y_last[i].addNum(_normalsCV_last.at<cv::Vec3d>(v, u)[1]);
                        mode_normals_z[i].addNum(_normalsCV.at<cv::Vec3d>(v, u)[2]);
                        mode_normals_z_last[i].addNum(_normalsCV_last.at<cv::Vec3d>(v, u)[2]);
                        mode_planepoints[i].push_back({_pcl_cloud->at(u, v).x, _pcl_cloud->at(u, v).y, _pcl_cloud->at(u, v).z});
                        mode_planepoints_last[i].push_back({_pcl_cloud_last->at(u, v).x, _pcl_cloud_last->at(u, v).y, _pcl_cloud_last->at(u, v).z});
                        mode_count[i]++;
                        break;
                    }
                }
                if(found_mode == false){
                    if((_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v-1, u)).val[0] < 0.999 ||
                    (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v+1, u)).val[0] < 0.999 ||
                    (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v, u-1)).val[0] < 0.999 ||
                    (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v, u+1)).val[0] < 0.999 ||
                    (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v-1, u)).val[0] < 0.999 ||
                    (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v+1, u)).val[0] < 0.999 ||
                    (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u-1)).val[0] < 0.999 ||
                    (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u+1)).val[0] < 0.999){
                        continue;
                    }
                    // if(debug){
                    //     cout << "============find new mode[" << static_cast<int>(modes.size()) << "]!!!" << "at (" << v << ", " << u << ")===============" << endl;
                    // }
                    modes.push_back(_normalsCV.at<cv::Vec3d>(v, u));
                    mode_coor.push_back(tmp_coor);
                    // saved_selected_mode_normals.push_back(tmp_saved_selected_mode_normals);
                    // saved_selected_mode_normals_last.push_back(tmp_saved_selected_mode_normals);
                    mode_normals_x.push_back(tmp_mode_normals);
                    mode_normals_x_last.push_back(tmp_mode_normals);
                    mode_normals_y.push_back(tmp_mode_normals);
                    mode_normals_y_last.push_back(tmp_mode_normals);
                    mode_normals_z.push_back(tmp_mode_normals);
                    mode_normals_z_last.push_back(tmp_mode_normals);
                    mode_planepoints.push_back(tmp_mode_planepoints);  
                    mode_planepoints_last.push_back(tmp_mode_planepoints);
                    mode_count.push_back(0);
                }
            }
        }
    
    //** After finishing selecting modes
    int sizeofMode = static_cast<int>(modes.size()); // sizeofmode saves the amount of modes which indicates how many planes there are
    // Eigen::VectorXi mode_count_eigen = Eigen::Map<Eigen::VectorXi, Eigen::Unaligned>(mode_count.data(), mode_count.size());
    // Eigen::Array<bool, Eigen::Dynamic, 1> mask = (mode_count_eigen.array()>12000);
    // int count_valid_mode = mask.count(); // define a counter to count the modes with points more than 12000
    int count_valid_mode{0};
    vector<int> selected_mode; // define a vector to save the number of selected modes
    // cout << "how many: " << modes.size() << endl;
    // cout << count_valid_mode << endl;
    int sumPoints{0};
    for(int i=0; i<sizeofMode; i++){ 
        if(mode_count[i] > 12000){
            if(is_debugging){
                cout << "**************************mode" << i << ": " << modes[i] << endl;
                cout << "totally " << mode_count[i] << " points" << endl;
            }

            sumPoints+=mode_count[i];
            selected_mode.push_back(i);
            count_valid_mode++;
        }
    }
    if(is_debugging){
        cout << count_valid_mode << endl;
    }
    
    int count_second_max{0};
    //** if only one mode is more than 10000 points, will select one more mode if this new mode is more than 5000
    //** otherwise, will select two modes if they are less than 5000
    // if(count_valid_mode<2){
    //     cout << "=====================================================================================" << endl;
    //     cout << "mode smaller than 2, will select one (>5000 points) or more (<5000 points) mode from plane with points less than 10000" << endl;
    //     selected_mode.push_back(0);
    //     for(int i=0; i<sizeofMode; i++){
    //         if(i == selected_mode[0]){
    //             continue;
    //         }
    //         if(mode_count[i]>=3000 && mode_count[i]<=10000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_second_max){
    //             count_second_max = mode_count[i];
    //             selected_mode[1] = i;
    //         }
    //     }
    //     cout << "**************************mode" << selected_mode[1] << ": " << modes[selected_mode[1]] << endl;
    //     cout << "totally " << mode_coor[selected_mode[1]].size() << " points" << endl;
    //     if(count_second_max != 0){
    //         count_valid_mode++;
    //     }
        
    //     if(count_second_max<5000){
    //         selected_mode.push_back(0);
    //         int count_third_max{0};
    //         for(int i=0; i<sizeofMode; i++){
    //             if(i == selected_mode[0] || i == selected_mode[1]){
    //                 continue;
    //             }
    //             if(mode_count[i]>=1000 && mode_count[i]<=10000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_third_max &&
    //             (modes[i].t() * modes[selected_mode[1]]).val[0]<0.5){
    //                 count_third_max = mode_count[i];
    //                 selected_mode[2] = i;
    //             }
    //         }
    //         if(count_third_max != 0){
    //             count_valid_mode++;
    //             cout << "**************************mode" << selected_mode[2] << ": " << modes[selected_mode[2]] << endl;
    //             cout << "totally " << mode_coor[selected_mode[2]].size() << " points" << endl;
    //         }

    //     }
    //     if(count_valid_mode == 1){
    //         cout << "cannot work, because only one plane exists" << endl;
    //     }
    // }

    if(count_valid_mode<2){
        if(is_debugging){
            cout << "=====================================================================================" << endl;
            cout << "mode smaller than 2, will select one (>12000 points) or more (<12000 points) mode from plane with points less than 12000" << endl;
        }
        selected_mode.push_back(0);
        for(int i=0; i<sizeofMode; i++){
            if(i == selected_mode[0]){
                continue;
            }
            if(mode_count[i]>=1000 && mode_count[i]<=12000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_second_max){
                count_second_max = mode_count[i];
                selected_mode[1] = i;
                // selected_mode.push_back(i);
            }
        }

        if(count_second_max != 0){
            count_valid_mode++;
            if(is_debugging){
                cout << "**************************mode" << selected_mode[1] << ": " << modes[selected_mode[1]] << endl;
                cout << "totally (>1000) " << mode_coor[selected_mode[1]].size() << " points" << endl;
            }
        }
        else{
            selected_mode.pop_back();
        }
        
        if(count_valid_mode==2 && count_second_max<12000){
            selected_mode.push_back(0);
            int count_third_max{0};
            for(int i=0; i<sizeofMode; i++){
                if(i == selected_mode[0] || i == selected_mode[1]){
                    continue;
                }
                if(mode_count[i]>=1000 && mode_count[i]<=12000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_third_max &&
                (modes[i].t() * modes[selected_mode[1]]).val[0]<0.5){
                    count_third_max = mode_count[i];
                    selected_mode[2] = i;
                }
            }
            if(count_third_max != 0){
                count_valid_mode++;
                if(is_debugging){
                    cout << "**************************mode" << selected_mode[2] << ": " << modes[selected_mode[2]] << endl;
                    cout << "totally (>1000) " << mode_coor[selected_mode[2]].size() << " points" << endl;
                }
            }
            else{
                selected_mode.pop_back();
            }

        }
        else if(count_valid_mode==1){
            selected_mode.push_back(0);
            for(int i=0; i<sizeofMode; i++){
                if(i == selected_mode[0]){
                    continue;
                }
                if(mode_count[i]>=500 && mode_count[i]<=12000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_second_max){
                    count_second_max = mode_count[i];
                    selected_mode[1] = i;
                }
            }
            if(count_second_max != 0){
                count_valid_mode++;
                if(is_debugging){
                    cout << "**************************mode" << selected_mode[1] << ": " << modes[selected_mode[1]] << endl;
                    cout << "totally (>500) " << mode_coor[selected_mode[1]].size() << " points" << endl;
                }
            }
            else{
                selected_mode.pop_back();
                std::cerr << "only one plane detected!" << endl;
            }

        }
    }

    //** if more than 2 valid modes, we have to calculate rotation matrix from each pair and evaluate which is the best
    //** paired_modes used to store number of a pair of modes
    //** first for loop used to generate all combinations of modes
    //** traverse each paired_mode to generate a rotation matrix, and use the rest of modes to evaluate the rotation matrix
    //** note that 'paired_modes' save the No. of modes, 'selected_mode' also save the No. of modes
    std::unique_ptr<Plane> plane;
    // Plane plane;
    if(count_valid_mode>2){
        if(is_debugging){
            cout << "count valid mode: " << count_valid_mode << endl;
            cout << "---------------------------------------------------------" << endl;
            cout << "mode more than 2, will compare which paired modes is best" << endl;
        }

        vector<vector<int>> paired_modes;
        for(int i=0; i<count_valid_mode; i++)
            for(int j=i+1; j<count_valid_mode; j++){
                paired_modes.push_back({selected_mode[i],selected_mode[j]});
        }

        Eigen::Vector3d v1;
        Eigen::Vector3d v11;
        Eigen::Vector3d v2;
        Eigen::Vector3d v22;
        Eigen::Matrix3d R_;
        int best_paired_modes{0};
        double error{999999};
        for(size_t i=0; i<(paired_modes.size()); i++){
            if(is_debugging){
                cout << "**current paired modes is: " << paired_modes[i][0] << " and " << paired_modes[i][1] << "**" << endl;
            }
            // v1 = oneVectorFromMultiVectors(normalsCV_last, mode_coor[paired_modes[i][0]]);
            // v11 = oneVectorFromMultiVectors(normalsCV, mode_coor[paired_modes[i][0]]);
            // v2 = oneVectorFromMultiVectors(normalsCV_last, mode_coor[paired_modes[i][1]]);
            // v22 = oneVectorFromMultiVectors(normalsCV, mode_coor[paired_modes[i][1]]);

            // v1 = oneVectorFromMultiVectors(mode_count[paired_modes[i][0]], saved_selected_mode_normals_last[paired_modes[i][0]]);
            // v11 = oneVectorFromMultiVectors(mode_count[paired_modes[i][0]], saved_selected_mode_normals[paired_modes[i][0]]);
            // v2 = oneVectorFromMultiVectors(mode_count[paired_modes[i][1]], saved_selected_mode_normals_last[paired_modes[i][1]]);
            // v22 = oneVectorFromMultiVectors(mode_count[paired_modes[i][1]], saved_selected_mode_normals[paired_modes[i][1]]);

            v1 = {mode_normals_x_last[paired_modes[i][0]].findMedian(), 
                    mode_normals_y_last[paired_modes[i][0]].findMedian(), 
                        mode_normals_z_last[paired_modes[i][0]].findMedian()};
            v11 = {mode_normals_x[paired_modes[i][0]].findMedian(), 
                    mode_normals_y[paired_modes[i][0]].findMedian(), 
                        mode_normals_z[paired_modes[i][0]].findMedian()};
            v2 = {mode_normals_x_last[paired_modes[i][1]].findMedian(), 
                    mode_normals_y_last[paired_modes[i][1]].findMedian(), 
                        mode_normals_z_last[paired_modes[i][1]].findMedian()};
            v22 = {mode_normals_x[paired_modes[i][1]].findMedian(), 
                    mode_normals_y[paired_modes[i][1]].findMedian(), 
                        mode_normals_z[paired_modes[i][1]].findMedian()};
            v22 = v22.normalized();
            v2 = v2.normalized();
            v11 = v11.normalized();
            v1 = v1.normalized();            
            cout << "**v1 is : " << v1[0] << ", " << v1[1] << ", " << v1[2] << " **" << endl;
            cout << "**v11 is : " << v11[0] << ", " << v11[1] << ", " << v11[2] << " **" << endl;
            cout << "**v2 is : " << v2[0] << ", " << v2[1] << ", " << v2[2] << " **" << endl;
            cout << "**v22 is : " << v22[0] << ", " << v22[1] << ", " << v22[2] << " **" << endl;

            plane = extractMaxPlane(mode_planepoints_last[paired_modes[i][0]], v1);
            // if (!plane) {
            //     std::cerr << "Could not fit plane for " << paired_modes[i][0] << " last" << std::endl;
            // }
            // v1 = {plane->A, plane->B, plane->C};
            // Plane old_plane = {v1[0], v1[1], v1[2], plane->D};
            // double old_error = calculateError(mode_planepoints_last[paired_modes[i][0]], old_plane);
            // double error = calculateError(mode_planepoints_last[paired_modes[i][0]], *plane);
            // cout << "**old v1's error: " << old_error << endl;
            // cout << "**new v1's error: " << error << endl;
            // cout << "size of mode_normals: " << mode_normals_x_last[paired_modes[i][0]].size() << endl;
            // cout << "size of mode_planepoints: " << mode_planepoints_last[paired_modes[i][0]].size() << endl;

            // cout << "points1: !!!!!!!!!!!" << mode_planepoints_last[paired_modes[i][1]][1].x << mode_planepoints_last[paired_modes[i][1]][1].y << mode_planepoints_last[paired_modes[i][1]][1].z << endl;
            // cout << "points2: !!!!!!!!!!!" << mode_planepoints_last[paired_modes[i][1]][2].x << mode_planepoints_last[paired_modes[i][1]][2].y << mode_planepoints_last[paired_modes[i][1]][2].z << endl;
            // cout << "points3: !!!!!!!!!!!" << mode_planepoints_last[paired_modes[i][1]][3].x << mode_planepoints_last[paired_modes[i][1]][3].y << mode_planepoints_last[paired_modes[i][1]][3].z << endl;
            // cout << "points4: !!!!!!!!!!!" << mode_planepoints_last[paired_modes[i][1]][4].x << mode_planepoints_last[paired_modes[i][1]][4].y << mode_planepoints_last[paired_modes[i][1]][4].z << endl;
            // cout << "points5: !!!!!!!!!!!" << mode_planepoints_last[paired_modes[i][1]][5].x << mode_planepoints_last[paired_modes[i][1]][5].y << mode_planepoints_last[paired_modes[i][1]][5].z << endl;
            // cout << "points6: !!!!!!!!!!!" << mode_planepoints_last[paired_modes[i][1]][6].x << mode_planepoints_last[paired_modes[i][1]][6].y << mode_planepoints_last[paired_modes[i][1]][6].z << endl;
            // cout << "plane: !!!!!!!!!!!!" << plane << endl;
            // savePointCloudAndPlaneAsPCD(mode_planepoints_last[paired_modes[i][0]], *plane, "viz.pcd");
            // savePlaneAsTXT(*plane, "plane.txt");
            // savePointCloudAsTXT(mode_planepoints_last[paired_modes[i][0]], "points.txt");
            // sleep(1000);  

            // plane = extractMaxPlane(mode_planepoints[paired_modes[i][0]], v11);
            // if (!plane) {
            //     std::cerr << "Could not fit plane for " << paired_modes[i][0] << " current" << std::endl;
            // }
            // v11 = {plane->A, plane->B, plane->C};
            // old_plane = {v11[0], v11[1], v11[2], plane->D};
            // old_error = calculateError(mode_planepoints_last[paired_modes[i][0]], old_plane);
            // error = calculateError(mode_planepoints[paired_modes[i][0]], *plane);
            // cout << "**old v11's error: " << old_error << endl;
            // cout << "**new v11's error: " << error << endl;
            // v2 = {0,1,0};
            // plane = extractMaxPlane(mode_planepoints_last[paired_modes[i][1]], v2);
            // if (!plane) {
            //     std::cerr << "Could not fit plane for " << paired_modes[i][1] << " last" << std::endl;
            // }
            // v2 = {plane->A, plane->B, plane->C};
            // old_plane = {v2[0], v2[1], v2[2], plane->D};
            // old_error = calculateError(mode_planepoints_last[paired_modes[i][1]], old_plane);
            // error = calculateError(mode_planepoints_last[paired_modes[i][1]], *plane);
            // cout << "**old v2's error: " << old_error << endl;
            // cout << "**new v2's error: " << error << endl;

            // plane = extractMaxPlane(mode_planepoints[paired_modes[i][1]], v22);
            // if (!plane) {
            //     std::cerr << "Could not fit plane for " << paired_modes[i][1] << " current" << std::endl;
            // }
            // v22 = {plane->A, plane->B, plane->C};

            // old_plane = {v22[0], v22[1], v22[2], plane->D};
            // old_error = calculateError(mode_planepoints_last[paired_modes[i][1]], old_plane);
            // error = calculateError(mode_planepoints[paired_modes[i][1]], *plane);
            // cout << "**old v22's error: " << old_error << endl;
            // cout << "**new v22's error: " << error << endl;

            v22 = v22.normalized();
            v2 = v2.normalized();
            v11 = v11.normalized();
            v1 = v1.normalized(); 
            cout << "**new v1 is : " << v1[0] << ", " << v1[1] << ", " << v1[2] << " **" << mode_planepoints_last[paired_modes[i][0]].size() << endl;
            cout << "**new v11 is : " << v11[0] << ", " << v11[1] << ", " << v11[2] << " **" << endl;
            cout << "**new v2 is : " << v2[0] << ", " << v2[1] << ", " << v2[2] << " **" << mode_planepoints_last[paired_modes[i][1]].size() << endl;
            cout << "**new v22 is : " << v22[0] << ", " << v22[1] << ", " << v22[2] << " **" << endl;


            rotationFromTwoPlanes(v1, v11, v2, v22, R_);
            double count{0.0};
            double error_cur{0};
            for(int j=0; j<count_valid_mode; j++){
                if(selected_mode[j]==paired_modes[i][1] || selected_mode[j]==paired_modes[i][0]){
                    continue;
                }
                count += 1.0;
                // float weight = float(mode_count[selected_mode[j]]) / float(sumPoints);
                // Eigen::Vector3d vTest = oneVectorFromMultiVectors(normalsCV_last, mode_coor[selected_mode[j]]);
                // Eigen::Vector3d vTTest = oneVectorFromMultiVectors(normalsCV, mode_coor[selected_mode[j]]);
                
                // Eigen::Vector3d vTest = oneVectorFromMultiVectors(mode_count[selected_mode[j]], saved_selected_mode_normals_last[selected_mode[j]]);
                // Eigen::Vector3d vTTest = oneVectorFromMultiVectors(mode_count[selected_mode[j]], saved_selected_mode_normals[selected_mode[j]]);

                Eigen::Vector3d vTest = {mode_normals_x_last[selected_mode[j]].findMedian(), 
                                            mode_normals_y_last[selected_mode[j]].findMedian(), 
                                                mode_normals_z_last[selected_mode[j]].findMedian()};
                Eigen::Vector3d vTTest = {mode_normals_x[selected_mode[j]].findMedian(), 
                                            mode_normals_y[selected_mode[j]].findMedian(), 
                                                mode_normals_z[selected_mode[j]].findMedian()};  
                vTest = vTest.normalized();
                vTTest = vTTest.normalized();

                // plane = extractMaxPlane(mode_planepoints_last[selected_mode[j]], vTest);
                // if (!plane) {
                //     std::cerr << "Could not fit plane for " << selected_mode[j] << " last" << std::endl;
                //     }
                // vTest = {plane->A, plane->B, plane->C};

                // plane = extractMaxPlane(mode_planepoints[selected_mode[j]], vTTest);
                // if (!plane) {
                //     std::cerr << "Could not fit plane for " << selected_mode[j] << " current" << std::endl;
                //     }
                // vTTest = {plane->A, plane->B, plane->C};
                
                Eigen::Vector3d vTTest_rotated = R_ * vTTest;
                // cout << "vTest: " << vTest << endl;
                // cout << "vTTest_rotated: " << vTest.transpose()*vTTest_rotated << endl;
                // error_cur += (weight * vTest.cross(vTTest_rotated).norm());
                error_cur += (vTest.cross(vTTest_rotated).norm());
            }
            if(is_debugging){
                cout << "**current paired modes' (weighted) error is: " << error_cur/count << " **\n" << endl;
            }
            if(error_cur/count < error){
                error = error_cur/count;
                _R = R_;
                best_paired_modes = i;
            }
        }
        if(is_debugging){
            cout << "---------------------------------------------------------" << endl;
            cout << "------------best paired mode IS: " << paired_modes[best_paired_modes][0] << " and " << paired_modes[best_paired_modes][1] << " ------------------" << endl;
        }
        valid_points = min(mode_count[paired_modes[best_paired_modes][0]], mode_count[paired_modes[best_paired_modes][1]]);
    }
    else if(count_valid_mode==2){
        Eigen::Vector3d v1{0,0,0};
        Eigen::Vector3d v11{0,0,0};
        Eigen::Vector3d v2{0,0,0};
        Eigen::Vector3d v22{0,0,0};

        if(is_debugging){
            cout << "=====================================================================================" << endl;
            cout << "mode equal to 2" << endl;
        }
        v1 = {mode_normals_x_last[selected_mode[0]].findMedian(), 
                mode_normals_y_last[selected_mode[0]].findMedian(), 
                    mode_normals_z_last[selected_mode[0]].findMedian()};
        v11 = {mode_normals_x[selected_mode[0]].findMedian(), 
                mode_normals_y[selected_mode[0]].findMedian(), 
                    mode_normals_z[selected_mode[0]].findMedian()};
        v2 = {mode_normals_x_last[selected_mode[1]].findMedian(), 
                mode_normals_y_last[selected_mode[1]].findMedian(), 
                    mode_normals_z_last[selected_mode[1]].findMedian()};
        v22 = {mode_normals_x[selected_mode[1]].findMedian(), 
                mode_normals_y[selected_mode[1]].findMedian(), 
                    mode_normals_z[selected_mode[1]].findMedian()};
        v22 = v22.normalized();
        v2 = v2.normalized();
        v11 = v11.normalized();
        v1 = v1.normalized();

        // plane = extractMaxPlane(mode_planepoints_last[selected_mode[0]], v1);
        // if (!plane) {
        //     std::cerr << "Could not fit plane for " << selected_mode[0] << " last" << std::endl;
        // }
        // v1 = {plane->A, plane->B, plane->C};

        // plane = extractMaxPlane(mode_planepoints[selected_mode[0]], v11);
        // if (!plane) {
        //     std::cerr << "Could not fit plane for " << selected_mode[0] << " current" << std::endl;
        // }
        // v11 = {plane->A, plane->B, plane->C};

        // plane = extractMaxPlane(mode_planepoints_last[selected_mode[1]], v2);
        // if (!plane) {
        //     std::cerr << "Could not fit plane for " << selected_mode[1] << " last" << std::endl;
        // }
        // v2 = {plane->A, plane->B, plane->C};

        // plane = extractMaxPlane(mode_planepoints[selected_mode[1]], v22);
        // if (!plane) {
        //     std::cerr << "Could not fit plane for " << selected_mode[1] << " current" << std::endl;
        // }
        // v22 = {plane->A, plane->B, plane->C};

        v22 = v22.normalized();
        v2 = v2.normalized();
        v11 = v11.normalized();
        v1 = v1.normalized(); 

        if(is_debugging){
            cout << "=====================================================================================" << endl;
        }
        valid_points = min(mode_count[selected_mode[0]], mode_count[selected_mode[1]]);
        rotationFromTwoPlanes(v1, v11, v2, v22, _R);
    }
    else{
        cerr << "only one valid plane!" << endl;
    }
     
}

inline void NI_SLAM::HashEfficientNormal2RotationMat(cv::Mat &_normalsCV_last, cv::Mat &_normalsCV, Eigen::Matrix3d &_R, cv::Mat &_AnglesMap){
    //** cannot work, because hash can only search totally equal key 
    ModeMap modeMap;
    cv::Vec3d zeros{0, 0, 0};
    cv::Vec3d N{1,0,0};
    for(int v=0; v<_normalsCV.rows;v++)
        for(int u=0; u<_normalsCV.cols;u++){
            //** exclude invalid points (noise from depth sensor)
            if((_normalsCV.at<cv::Vec3d>(v, u) == zeros) || (_normalsCV_last.at<cv::Vec3d>(v, u) == zeros) || (v <= 10) || (u <= 10) || (abs(v-_normalsCV.rows) <= 10) || (abs(u-_normalsCV.cols) <= 10))
            {
                continue;
            }
            //** exclude non-overlapped points
            if((_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u)).val[0] < 0.9){ //0.7){
                continue;
            }
            //** aim to cancel the points on edge
            if((_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v-1, u)).val[0] < 0.999 ||
            (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v+1, u)).val[0] < 0.999 ||
            (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v, u-1)).val[0] < 0.999 ||
            (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v, u+1)).val[0] < 0.999 ||
            (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v-1, u)).val[0] < 0.999 ||
            (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v+1, u)).val[0] < 0.999 ||
            (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u-1)).val[0] < 0.999 ||
            (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u+1)).val[0] < 0.999){
                continue;
            }
            //** perform mode selection mechanism
            auto normal = _normalsCV.at<cv::Vec3d>(v, u);
            auto &modeData = modeMap[normal];

            Eigen::Vector2i x{v, u};
            modeData.coordinates.push_back(x);
            modeData.normal_x.addNum(normal[0]);
            modeData.normal_x_last.addNum(_normalsCV_last.at<cv::Vec3d>(v, u)[0]);
            modeData.normal_y.addNum(normal[1]);
            modeData.normal_y_last.addNum(_normalsCV_last.at<cv::Vec3d>(v, u)[1]);
            modeData.normal_z.addNum(normal[2]);
            modeData.normal_z_last.addNum(_normalsCV_last.at<cv::Vec3d>(v, u)[2]);
            modeData.count++;
        }

    //** After finishing selecting modes
    // int sizeofMode = static_cast<int>(modes.size()); // sizeofmode saves the amount of modes which indicates how many planes there are
    // Eigen::VectorXi mode_count_eigen = Eigen::Map<Eigen::VectorXi, Eigen::Unaligned>(mode_count.data(), mode_count.size());
    // Eigen::Array<bool, Eigen::Dynamic, 1> mask = (mode_count_eigen.array()>12000);
    // int count_valid_mode = mask.count(); // define a counter to count the modes with points more than 12000
    int count_valid_mode{0};
    vector<cv::Vec3d> selected_mode; // define a vector to save the number of selected modes
    // cout << "how many: " << modes.size() << endl;
    // cout << count_valid_mode << endl;
    // int sumPoints{0};

    if(is_debugging){
        cout << "modeMap's size: " << modeMap.size() << endl;
    }

    for (const auto& pair : modeMap) {
        const cv::Vec3d& key = pair.first;
        const ModeData& value = pair.second;
        if(value.count > 12000){
            if(is_debugging){
                cout << "**************************mode" << " : " << key << endl;
                cout << "totally " << value.count << " points" << endl;
            }
            selected_mode.push_back(key);
            count_valid_mode++;
        }

    }

    if(is_debugging){
        cout << "> 12000 valid modes is: " << count_valid_mode << endl;
    }
    
    int count_second_max{0};
    //** if only one mode is more than 10000 points, will select one more mode if this new mode is more than 5000
    //** otherwise, will select two modes if they are less than 5000
    if(count_valid_mode<2){
        if(is_debugging){
            cout << "=====================================================================================" << endl;
            cout << "mode smaller than 2, will select one (>5000 points) or more (<5000 points) mode from plane with points less than 10000" << endl;
        }

        cv::Vec3d max_mode_key;
        bool found{false};

        for(const auto& entry : modeMap){
            const cv::Vec3d& key = entry.first;
            const ModeData& mode = entry.second;

            // 检查 mode 是否满足条件
            if(mode.count >= 1000 && mode.count <= 12000 && (key.t() * selected_mode[0]).val[0]<0.5) {
                // 如果我们还没有找到满足条件的模式，或者找到的新模式大于当前的最大模式，
                // 我们就更新最大模式的键。
                if(!found || mode.count > modeMap[max_mode_key].count) {
                    max_mode_key = key;
                    count_second_max = mode.count;
                    found = true;
                }
            }
        }

        if(found) {
            // 在这里，你可以处理找到的最大模式。
            // 例如，打印键：
            // const cv::Vec3d& key = *max_mode_key;
            selected_mode.push_back(max_mode_key);
            if(is_debugging){
                cout << "**************************mode" <<" : " << max_mode_key << endl;
                cout << "totally " << modeMap[max_mode_key].count << " points" << endl;
            }
            count_valid_mode++;
            if(count_second_max>0){
                int count_third_max{0};
                cv::Vec3d second_max_mode_key;
                bool found_again{false};

                for(const auto& entry : modeMap) {
                    const cv::Vec3d& key = entry.first;
                    const ModeData& mode = entry.second;

                    // 检查 mode 是否满足条件
                    if(mode.count >= 1000 && mode.count <= 12000 && (key.t() * selected_mode[0]).val[0]<0.5 && (key.t() * selected_mode[1]).val[0]<0.5) {
                        // 如果我们还没有找到满足条件的模式，或者找到的新模式大于当前的最大模式，
                        // 我们就更新最大模式的键。
                        if(!found_again || mode.count > modeMap[second_max_mode_key].count) {
                            second_max_mode_key = key;
                            count_third_max = mode.count;
                            found_again = true;
                        }
                    }
                }

                if(found_again) {
                    // 在这里，你可以处理找到的最大模式。
                    // 例如，打印键：
                    // const cv::Vec3d& key = *second_max_mode_key;
                    selected_mode.push_back(second_max_mode_key);
                    if(is_debugging){
                        cout << "**************************mode" <<" : " << second_max_mode_key << endl;
                        cout << "totally " << modeMap[second_max_mode_key].count << " points" << endl;
                    }
                    count_valid_mode++;
                }
            }
        }
        else{
            int count_third_max{0};
            cv::Vec3d second_max_mode_key;
            bool found_again{false};
            for(const auto& entry : modeMap) {
                const cv::Vec3d& key = entry.first;
                const ModeData& mode = entry.second;

                // 检查 mode 是否满足条件
                if(mode.count >= 500 && mode.count <= 12000 && (key.t() * selected_mode[0]).val[0]<0.5) {
                    // 如果我们还没有找到满足条件的模式，或者找到的新模式大于当前的最大模式，
                    // 我们就更新最大模式的键。
                    if(!found_again || mode.count > modeMap[second_max_mode_key].count) {
                        second_max_mode_key = key;
                        count_third_max = mode.count;
                        found_again = true;
                    }
                }
            }

            if(found_again) {
                // 在这里，你可以处理找到的最大模式。
                // 例如，打印键：
                // const cv::Vec3d& key = *second_max_mode_key;
                selected_mode.push_back(second_max_mode_key);
                if(is_debugging){
                    cout << "**************************mode" <<" : " << second_max_mode_key << endl;
                    cout << "totally " << modeMap[second_max_mode_key].count << " points" << endl;
                }
                count_valid_mode++;
            }
        }

        if(count_valid_mode == 1){
            cerr << "cannot work, because only one plane exists" << endl;
        }
    }

    //** if more than 2 valid modes, we have to calculate rotation matrix from each pair and evaluate which is the best
    //** paired_modes used to store number of a pair of modes
    //** first for loop used to generate all combinations of modes
    //** traverse each paired_mode to generate a rotation matrix, and use the rest of modes to evaluate the rotation matrix
    //** note that 'paired_modes' save the No. of modes, 'selected_mode' also save the No. of modes
    if(count_valid_mode>2){
        if(is_debugging){
            cout << "count valid mode: " << count_valid_mode << endl;
            cout << "---------------------------------------------------------" << endl;
            cout << "mode more than 2, will compare which paired modes is best" << endl;
        }

        vector<vector<cv::Vec3d>> paired_modes;
        for(int i=0; i<count_valid_mode; i++)
            for(int j=i+1; j<count_valid_mode; j++){
                paired_modes.push_back({selected_mode[i],selected_mode[j]});
        }

        Eigen::Vector3d v1;
        Eigen::Vector3d v11;
        Eigen::Vector3d v2;
        Eigen::Vector3d v22;
        Eigen::Matrix3d R_;
        int best_paired_modes{0};
        double error{999999};
        for(size_t i=0; i<(paired_modes.size()); i++){
            if(is_debugging){
                cout << "**current paired modes is: " << paired_modes[i][0] << " and " << paired_modes[i][1] << "**" << endl;
            }
            v1 = {modeMap[paired_modes[i][0]].normal_x_last.findMedian(), 
                    modeMap[paired_modes[i][0]].normal_y_last.findMedian(), 
                        modeMap[paired_modes[i][0]].normal_z_last.findMedian()};
            v11 = {modeMap[paired_modes[i][0]].normal_x.findMedian(), 
                    modeMap[paired_modes[i][0]].normal_y.findMedian(), 
                        modeMap[paired_modes[i][0]].normal_y.findMedian()};
            v2 = {modeMap[paired_modes[i][1]].normal_x_last.findMedian(), 
                    modeMap[paired_modes[i][1]].normal_y_last.findMedian(), 
                        modeMap[paired_modes[i][1]].normal_z_last.findMedian()};
            v22 = {modeMap[paired_modes[i][1]].normal_x.findMedian(), 
                    modeMap[paired_modes[i][1]].normal_y.findMedian(), 
                        modeMap[paired_modes[i][1]].normal_y.findMedian()};
            v22 = v22.normalized();
            v2 = v2.normalized();
            v11 = v11.normalized();
            v1 = v1.normalized();            

            // cout << "**v1 is : " << v1[0] << ", " << v1[1] << ", " << v1[2] << " **" << endl;
            // cout << "new v1 is : " << mode_normals_x_last[paired_modes[i][0]].findMedian() << "," <<
            //         mode_normals_y_last[paired_modes[i][0]].findMedian() << "," << 
            //             mode_normals_z_last[paired_modes[i][0]].findMedian() << endl;
            // cout << "**v11 is : " << v11[0] << ", " << v11[1] << ", " << v11[2] << " **" << endl;
            // cout << "new v11 is : " << mode_normals_x[paired_modes[i][0]].findMedian() << "," <<
            //         mode_normals_y[paired_modes[i][0]].findMedian() << "," << 
            //             mode_normals_z[paired_modes[i][0]].findMedian() << endl;
            // cout << "**v2 is : " << v2[0] << ", " << v2[1] << ", " << v2[2] << " **" << endl;
            // cout << "new v2 is : " << mode_normals_x_last[paired_modes[i][1]].findMedian() << "," <<
            //         mode_normals_y_last[paired_modes[i][1]].findMedian() << "," << 
            //             mode_normals_z_last[paired_modes[i][1]].findMedian() << endl;
            // cout << "**v22 is : " << v22[0] << ", " << v22[1] << ", " << v22[2] << " **" << endl;
            // cout << "new v22 is : " << mode_normals_x[paired_modes[i][1]].findMedian() << "," <<
            //         mode_normals_y[paired_modes[i][1]].findMedian() << "," << 
            //             mode_normals_z[paired_modes[i][1]].findMedian() << endl;
            rotationFromTwoPlanes(v1, v11, v2, v22, R_);
            double count{0.0};
            double error_cur{0};
            for(int j=0; j<count_valid_mode; j++){
                if(selected_mode[j]==paired_modes[i][1] || selected_mode[j]==paired_modes[i][0]){ // TODO: revise?
                    continue;
                }
                count += 1.0;
                // float weight = float(mode_count[selected_mode[j]]) / float(sumPoints);
                // Eigen::Vector3d vTest = oneVectorFromMultiVectors(normalsCV_last, mode_coor[selected_mode[j]]);
                // Eigen::Vector3d vTTest = oneVectorFromMultiVectors(normalsCV, mode_coor[selected_mode[j]]);
                
                // Eigen::Vector3d vTest = oneVectorFromMultiVectors(mode_count[selected_mode[j]], saved_selected_mode_normals_last[selected_mode[j]]);
                // Eigen::Vector3d vTTest = oneVectorFromMultiVectors(mode_count[selected_mode[j]], saved_selected_mode_normals[selected_mode[j]]);

                Eigen::Vector3d vTest = {modeMap[selected_mode[j]].normal_x_last.findMedian(), 
                                            modeMap[selected_mode[j]].normal_y_last.findMedian(), 
                                                modeMap[selected_mode[j]].normal_z_last.findMedian()};
                Eigen::Vector3d vTTest = {modeMap[selected_mode[j]].normal_x.findMedian(), 
                                            modeMap[selected_mode[j]].normal_y.findMedian(), 
                                                modeMap[selected_mode[j]].normal_z.findMedian()};
                vTest = vTest.normalized();
                vTTest = vTTest.normalized();
                
                Eigen::Vector3d vTTest_rotated = R_ * vTTest;
                // cout << "vTest: " << vTest << endl;
                // cout << "vTTest_rotated: " << vTest.transpose()*vTTest_rotated << endl;
                // error_cur += (weight * vTest.cross(vTTest_rotated).norm());
                error_cur += (vTest.cross(vTTest_rotated).norm());
            }
            if(is_debugging){
                cout << "**current paired modes' (weighted) error is: " << error_cur/count << " **\n" << endl;
            }
            if(error_cur/count < error){
                error = error_cur/count;
                _R = R_;
                best_paired_modes = i;
            }
        }
        if(is_debugging){
            cout << "---------------------------------------------------------" << endl;
            cout << "------------best paired mode IS: " << paired_modes[best_paired_modes][0] << " and " << paired_modes[best_paired_modes][1] << " ------------------" << endl;
        }
        valid_points = min(modeMap[paired_modes[best_paired_modes][0]].count, modeMap[paired_modes[best_paired_modes][1]].count);
    }
    else if(count_valid_mode==2){
        Eigen::Vector3d v1{0,0,0};
        Eigen::Vector3d v11{0,0,0};
        Eigen::Vector3d v2{0,0,0};
        Eigen::Vector3d v22{0,0,0};

        if(is_debugging){
            cout << "=====================================================================================" << endl;
            cout << "mode equal to 2" << endl;
        }
        v1 = {modeMap[selected_mode[0]].normal_x_last.findMedian(), 
                modeMap[selected_mode[0]].normal_y_last.findMedian(), 
                    modeMap[selected_mode[0]].normal_z_last.findMedian()};
        v11 = {modeMap[selected_mode[0]].normal_x.findMedian(), 
                modeMap[selected_mode[0]].normal_y.findMedian(), 
                    modeMap[selected_mode[0]].normal_z.findMedian()};
        v2 = {modeMap[selected_mode[1]].normal_x_last.findMedian(), 
                modeMap[selected_mode[1]].normal_y_last.findMedian(), 
                    modeMap[selected_mode[1]].normal_z_last.findMedian()};
        v22 = {modeMap[selected_mode[1]].normal_x.findMedian(), 
                modeMap[selected_mode[1]].normal_y.findMedian(), 
                    modeMap[selected_mode[1]].normal_z.findMedian()};
        v22 = v22.normalized();
        v2 = v2.normalized();
        v11 = v11.normalized();
        v1 = v1.normalized();

        if(is_debugging){
            cout << "=====================================================================================" << endl;
        }
        valid_points = min(modeMap[selected_mode[0]].count, modeMap[selected_mode[1]].count);
        rotationFromTwoPlanes(v1, v11, v2, v22, _R);
    }
    else{
        cerr << "only one valid plane!" << endl;
    }
}

void NI_SLAM::set_publish_refined_keyframe()
{
    flag_publish_refined_keyframe = true;

    map_info_pub = nh.advertise<ni_slam::MapInfo>("refined/map_info", 1);
    image_pub_refined_color = image_transport.advertise("refined/image_color", 1);
    image_pub_refined_depth = image_transport.advertise("refined/image_depth", 1);

    cv_refined_color.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    cv_refined_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
}

inline void NI_SLAM::publish_keyframe()
{
    cv::Mat key_frame_color(height, width, CV_32F, color_key.data());
    cv::Mat key_frame_depth(height, width, CV_32F, depth_key.data());

    cv_refined_color.header = ros_header;
    cv_refined_depth.header = ros_header;
    cv_refined_color.image = key_frame_color;
    cv_refined_depth.image = key_frame_depth;

    map_info.header = ros_header;
    map_info.resolution = resolution;
    map_info.width = width;
    map_info.height = height;
    tf::poseTFToMsg(pose_key, map_info.origin);

    image_pub_refined_color.publish(cv_refined_color.toImageMsg());
    image_pub_refined_depth.publish(cv_refined_depth.toImageMsg());
    map_info_pub.publish(map_info);
}

void NI_SLAM::set_publish_incremental_keypose()
{
    pose_cov_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("incremental_pose_cov", 1000);

    flag_publish_incremental_keypose_cov = true;

    pose_inc_cov.pose.covariance[3*7] = 4.592449e-06 * 2;

    pose_inc_cov.pose.covariance[4*7] = 4.592449e-06 * 2;

    pose_inc_cov.pose.covariance[5*7] = 4.592449e-06 * 2;
}

void NI_SLAM::set_publish_twist()
{
    twist_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("twiststamped", 1000);

    flag_publish_twist = true;
}

void NI_SLAM::rotationFromTwoPlanes(Eigen::Vector3d &_v1, Eigen::Vector3d &_v11, Eigen::Vector3d &_v2, Eigen::Vector3d &_v22, Eigen::Matrix3d &_R)
{
    //** This function only return the R, such that,
    //** v1 = R * v11, v2 = R * v22
    //** for example, if v1 and v2 are in the reference frame, v11 and v22 are in the current frame
    //** so R represents the rotation matrix from reference frame to current 

    Eigen::Vector3d N;
    double alpha;
    if((_v1.transpose() * _v11)>0.999999 && (_v2.transpose() * _v2)>0.999999){
        if(is_debugging){
            cout << "**in case 1 (no rotation)**" << endl;
        }

        _R.setIdentity();
        return;
    } 
    else if((_v1.transpose() * _v11)>0.999999){
        if(is_debugging){
            cout << "**in case 2 (axis is _v1)**" << endl;
        }
        N = _v1;
        double num1 = _v2.transpose()*_v22;
        double num2 = _v22.transpose()*N;
        double den1 = _v2.transpose()*(N.cross(_v22));
        double den2 = (N.cross(_v22)).transpose()*(N.cross(_v22));
        alpha = acos((num1-num2*num2)/(1-num2*num2));
        double sin_alpha = den1/den2;
        if(sin_alpha < 0){
            alpha = -alpha;
        }

    }
    else if((_v2.transpose() * _v22)>0.999999){
        if(is_debugging){
            cout << "**in case 2 (axis is _v2)**" << endl;
        }
        N = _v2;
        double num1 = _v1.transpose()*_v11;
        double num2 = _v11.transpose()*N;
        double den1 = _v1.transpose()*(N.cross(_v11));
        double den2 = (N.cross(_v11)).transpose()*(N.cross(_v11));
        alpha = acos((num1-num2*num2)/(1-num2*num2));
        double sin_alpha = den1/den2;
        if(sin_alpha < 0){
            alpha = -alpha;
        }
    }
    else{
        if(is_debugging){
            cout << "**in case 3**" << endl;
        }
        N = ((_v11-_v1).cross(_v22-_v2))/((_v11-_v1).cross(_v22-_v2).norm());
        if(is_debugging){
            cout << "N: " << N << endl;
        }
        double num1 = _v2.transpose()*_v22;
        double num2 = _v22.transpose()*N;
        double den1 = _v2.transpose()*(N.cross(_v22));
        double den2 = (N.cross(_v22)).transpose()*(N.cross(_v22));
        alpha = acos((num1-num2*num2)/(1-num2*num2));
        double sin_alpha = den1/den2;
        if(sin_alpha < 0){
            alpha = -alpha;
        }
    }
    
    Eigen::AngleAxisd angleaxis(Eigen::AngleAxisd(alpha, N));
    _R = angleaxis.toRotationMatrix();
    if(is_debugging){
        cout << "axis-angle: \n" << angleaxis.angle() * angleaxis.axis() << endl;
    }

}

inline ArrayXXcf NI_SLAM::fft(const ArrayXXf& x)
{

    ArrayXXcf xf = ArrayXXcf(width/2+1, height);
    double start = clock();
    fft_plan=fftwf_plan_dft_r2c_2d(height, width, (float(*))(x.data()), 
        (float(*)[2])(xf.data()), FFTW_ESTIMATE); // reverse order for column major


    fftwf_execute(fft_plan);
    // fftwf_destroy_plan(fft_plan);
    // fftw_cleanup();
    double endd = clock();
    double thisTime = (double)(endd - start) / CLOCKS_PER_SEC;
    cout << "testMKL: " << thisTime << endl;

    return xf;
}

inline ArrayXXf NI_SLAM::ifft(const ArrayXXcf& xf)
{
    ArrayXXf x = ArrayXXf(width, height);
    ArrayXXcf cxf = xf;
    fft_plan=fftwf_plan_dft_c2r_2d(height, width, (float(*)[2])(cxf.data()),
        (float(*))(x.data()), FFTW_ESTIMATE);
    
    fftwf_execute(fft_plan);
    // fftwf_destroy_plan(fft_plan);
    // fftw_cleanup();
    return x/x.size();
}

inline ArrayXXf NI_SLAM::shift(const ArrayXXf& x, int a, int b)
{
    ArrayXXf y = ArrayXXf::Zero(width, height);
    y.block(max(0,  a), max(0,  b), width - abs(a), height - abs(b))=
    x.block(max(0, -a), max(0, -b), width - abs(a), height - abs(b));
    return y;
}

void NI_SLAM::set_visualization()
{
    flag_visualization = true;
    image_pub_key = image_transport.advertise("keyframe/image", 1);
    image_pub_new = image_transport.advertise("newframe/image", 1);
    image_pub_response = image_transport.advertise("response/image", 1);
    cv_keyframe.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    cv_newframe.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    cv_response.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
}

void NI_SLAM::set_rotation_imu(double yaw, double pitch, double roll)
{
    rotation_imu.setEuler(yaw, pitch, roll);
}

inline void NI_SLAM::show()
{
    cv::Mat key_frame(height, width, CV_32F, color_key.data());
    
    ArrayXXf color_clear = color*weights_dep;
    cv::Mat newframe(height, width, CV_32F, color_clear.data());

    ArrayXXf response_max = response/max_response;
    cv::Mat response_cv(height, width, CV_32F, response_max.data());

    cv_keyframe.header = ros_header;
    cv_newframe.header = ros_header;
    cv_response.header = ros_header;

    cv_keyframe.image = key_frame;
    cv_newframe.image = newframe;
    cv_response.image = response_cv;

    image_pub_key.publish(cv_keyframe.toImageMsg());
    image_pub_new.publish(cv_newframe.toImageMsg());
    image_pub_response.publish(cv_response.toImageMsg());
}

void NI_SLAM::set_file(string name_prefix)
{
    flag_save_file = true;
    char s[30];
    struct tm tim;
    time_t now;
    now = time(NULL);
    tim = *(localtime(&now));
    strftime(s,30,"_%Y_%b_%d_%H_%M_%S.txt",&tim);
    filename = name_prefix + string(s);
    file.open(filename.c_str(), ios::trunc|ios::out);
    file<<"# "<<"width:"<<width<<" "
              <<"height:"<<height<<" "
              <<"psr bound:"<<psrbound<<" "
              <<"mu:"<<adaptive_mu<<"\n";
    file.close();
}

void NI_SLAM::set_gt_poses(string file_path)
{
    flag_show_gt=true;
    ifstream fGroundtruth;
    fGroundtruth.open(file_path.c_str());
    while(!fGroundtruth.eof())
    {
        string s;
        getline(fGroundtruth, s);
        vector<double> gt_pose;
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            double x;
            double y;
            double z;
            double q0;
            double q1;
            double q2;
            double q3;
            string sRGB, sD;
            ss >> t;
            // vTimestamps.push_back(t);
            // ss >> sD;
            // ss >> t;
            // ss >> sRGB;
            // ss >> t;
            ss >> x;
            gt_pose.push_back(x);
            ss >> y;
            gt_pose.push_back(y);
            ss >> z;
            gt_pose.push_back(z);
            ss >> q0;
            gt_pose.push_back(q0);
            ss >> q1;
            gt_pose.push_back(q1);
            ss >> q2;
            gt_pose.push_back(q2);
            ss >> q3;
            gt_pose.push_back(q3);       
        }
        gt_poses.push_back(gt_pose); 
    }
}

void NI_SLAM::save_file()
{
    file.open(filename.c_str(), ios::app);

    // tf::Transform t = (pose_key.inverse()*pose).inverse() * (pose_real_key.inverse()*pose_real);

    // file<<boost::format("%.9f") % (ros_header.stamp.sec+
    //      ros_header.stamp.nsec/1000000000.0)<<" "


    // t2 = std::chrono::steady_clock::now();
    // double total_time = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    file<<nth_frame+1<<" "
        <<pose.getOrigin()[0]<<" "
        <<pose.getOrigin()[1]<<" "
        <<pose.getOrigin()[2]<<" "
        <<pose.getRotation().x()<<" "
        <<pose.getRotation().y()<<" "
        <<pose.getRotation().z()<<" "
        <<pose.getRotation().w()<< "\n"; // " " << total_time / float(nth_frame+1) << "\n"; // " " <<max_response<<" "<<psr<<" "<<time_use<<"\n";
        // <<pose_real.getOrigin()[0]<<" "
        // <<pose_real.getOrigin()[1]<<" "
        // <<pose_real.getOrigin()[2]<<" "
        // <<pose_real.getRotation().x()<<" "
        // <<pose_real.getRotation().y()<<" "
        // <<pose_real.getRotation().z()<<" "
        // <<pose_real.getRotation().w()<<" "
        
        // <<t.getOrigin().x()<<" "
        // <<t.getOrigin().y()<<" "
        // <<t.getOrigin().z()<<"\n";
    
    file.close();
}

void NI_SLAM::save_file(FramePtr frame)
{
    file.open(filename.c_str(), ios::app);

    // tf::Transform t = (pose_key.inverse()*pose).inverse() * (pose_real_key.inverse()*pose_real);

    // file<<boost::format("%.9f") % (ros_header.stamp.sec+
    //      ros_header.stamp.nsec/1000000000.0)<<" "
    int frame_id = frame->GetFrameId();
    tf::Transform pose_to_save = frame->GetPose();

    t2 = std::chrono::steady_clock::now();
    double total_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();


    // file<<frame_id+1<<" "
    file<<frame_id<<" " // for ethl
        <<pose_to_save.getOrigin()[0]<<" "
        <<pose_to_save.getOrigin()[1]<<" "
        <<pose_to_save.getOrigin()[2]<<" "
        <<pose_to_save.getRotation().x()<<" "
        <<pose_to_save.getRotation().y()<<" "
        <<pose_to_save.getRotation().z()<<" "
        <<pose_to_save.getRotation().w()<<" " << total_time/(nth_frame+1) << "\n";
    
    file.close();
}

void NI_SLAM::ShutDown(){
    _shutdown = true;
    _normalMap_thread.join();
    // _rotation_thread.join();
    _translation_thread.join();
    
    _rotation_part1_thread.join();
    _rotation_part2_thread.join();
    // t2 = std::chrono::steady_clock::now();
    // double total_time = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    // cout << "average time(ms) for each frame: " << total_time / float(nth_frame) << endl;

}

NI_SLAM::~NI_SLAM()
{
    fftwf_destroy_plan(fft_plan);
    // _shutdown = true;
    // _normalMap_thread.join();
    // _rotation_thread.join();
    // _translation_thread.join();
}

void DynamicMedian::addNum(double _num)
{
    _sum += _num;
    _count++;
    if(big_heap.empty())
    {
        big_heap.push(_num);
        return;
    }
    if(big_heap.size() > small_heap.size())
    {
        if(_num > big_heap.top())
        {
            small_heap.push(_num);
        }
        else{
            small_heap.push(big_heap.top());
            big_heap.pop();
            big_heap.push(_num);
        }
    }
    else if(big_heap.size() == small_heap.size()){
        if(_num<big_heap.top())
        {
            big_heap.push(_num);
        }
        else{
            small_heap.push(_num);
        }
    }
    else if(big_heap.size() < small_heap.size())
    {
        if(_num<small_heap.top())
        {
            big_heap.push(_num);
        }
        else{
            big_heap.push(small_heap.top());
            small_heap.pop();
            small_heap.push(_num);
        }
    }
}

double DynamicMedian::findMedian(){
    if(big_heap.size()==0 && small_heap.size()==0)
    {
        cerr << "there is no values!!!!!!" << endl;
    }
    if(big_heap.size() == small_heap.size())   
        return (big_heap.top()+small_heap.top())/2;
    else if(big_heap.size() > small_heap.size())
        return big_heap.top();
    else
        return small_heap.top();
}

double DynamicMedian::findMean(){
    return (_sum/_count);
}

Frame::Frame(){
}

Frame::Frame(int frame_id_, cv::Mat normal_map_, CloudType::ConstPtr pcl_cloud_):
        _frame_id(frame_id_), _normal_map(normal_map_), _pcl_cloud(pcl_cloud_){

        }

Frame& Frame::operator=(const Frame& other){
    _frame_id = other._frame_id;
    _normal_map = other._normal_map;
    _pcl_cloud = other._pcl_cloud;
    _pose = other._pose;
    _rotation = other._rotation;
    ref_rot_selected = other.ref_rot_selected;
    ref_trans_selected = other.ref_trans_selected;
    return *this;
}

void Frame::SetFrameId(int frame_id_){
    _frame_id = frame_id_;
}

int Frame::GetFrameId(){
    return _frame_id;
}

void Frame::SetRotation(tf::Quaternion rotation_){
    _rotation = rotation_;
}

tf::Quaternion Frame::GetRotation(){
    return _rotation;
}

void Frame::SetPose(tf::Transform pose_){
    _pose = pose_;
}

tf::Transform Frame::GetPose(){
    return _pose;
}

void Frame::SetKeyRot(){
    ref_rot_selected = true;
}

bool Frame::IsKeyRot(){
    return ref_rot_selected;
}

CloudType::ConstPtr Frame::GetPclCloud(){
    return _pcl_cloud;
}

std::unique_ptr<Plane> fitPlane(const std::vector<PlanePoint>& points) {
    int numPoints = points.size();
    cout << "numPoints: " << numPoints << endl;
    if (numPoints < 3) {
        return nullptr;  // Not enough points to define a plane
    }
    if (numPoints > 1000) {
        numPoints = 1000;
    }
    // 构建系数矩阵A和常数向量b
    MatrixXd A(numPoints-1, 3);
    VectorXd b(numPoints-1);
    for (int i = 0; i < numPoints-1; ++i) {
        A(i, 0) = points[i+1].x;
        A(i, 1) = points[i+1].y;
        A(i, 2) = 1.0;
        b(i) = -points[i+1].z;
    }

    // 使用最小二乘法求解
    VectorXd x = A.colPivHouseholderQr().solve(b);

    // 使用 std::make_unique 来创建一个 std::unique_ptr<Plane>
    return std::make_unique<Plane>(Plane{x(0), x(1), 1.0, x(2)});
}

// std::unique_ptr<Plane> fitPlane(const std::vector<PlanePoint>& points) {
//     int numPoints = points.size();
//     cout << "numPoints: " << numPoints << endl;
//     if (numPoints < 3) {
//         return nullptr;  // Not enough points to define a plane
//     }

//     // 构建系数矩阵A和常数向量b
//     MatrixXd A(numPoints-1, 3);
//     VectorXd b(numPoints-1);
//     for (int i = 0; i < numPoints; ++i) {
//         A(i, 0) = points[i+1].x - points[1].x;
//         A(i, 1) = points[i+1].y - points[1].y;
//         A(i, 2) = points[i+1].z - points[1].z;
//         b(i) = 0;
//     }

//     // 使用最小二乘法求解
//     VectorXd x = A.colPivHouseholderQr().solve(b);

//     // 使用 std::make_unique 来创建一个 std::unique_ptr<Plane>
//     return std::make_unique<Plane>(Plane{x(0), x(1), x(2), 0.0});
// }

std::unique_ptr<Plane> fitNormal(const std::vector<PlanePoint>& points) {
    int numPoints = points.size();
    cout << "numPoints: " << numPoints << endl;
    if (numPoints < 3) {
        return nullptr;  // Not enough points to define a plane
    }

    // Calculate the centroid of the points
    PlanePoint centroid = std::accumulate(points.begin(), points.end(), PlanePoint{0, 0, 0}, [](const PlanePoint& a, const PlanePoint& b) {
        return PlanePoint{a.x + b.x, a.y + b.y, a.z + b.z};
    });
    centroid.x /= numPoints;
    centroid.y /= numPoints;
    centroid.z /= numPoints;

    // Construct the covariance matrix
    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
    for (const auto& point : points) {
        Eigen::Vector3d d = Eigen::Vector3d(point.x - centroid.x, point.y - centroid.y, point.z - centroid.z);
        C += d * d.transpose();
    }

    // Solve the eigenvalue problem
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(C);
    if (eigensolver.info() != Success) {
        // Eigenvalue computation failed
        return nullptr;
    }

    // The normal to the plane is the eigenvector corresponding to the smallest eigenvalue
    Eigen::Vector3d normal = eigensolver.eigenvectors().col(0);

    // Create a unique_ptr<Eigen::Vector3d>
    return std::make_unique<Plane>(Plane{normal[0], normal[1], normal[2], 0.0});
}

double calculateError(const std::vector<PlanePoint>& points, const Plane& plane) {
    auto squareError = [](double a, double b) {
        auto e = a - b;
        return e * e;
    };
    double sum = 0.0;
    for (const auto& point : points) {
        double predictedZ = -(plane.A * point.x + plane.B * point.y + plane.D);
        sum += squareError(predictedZ, point.z);
    }
    return std::sqrt(sum / points.size());
}

Plane normalizePlane(const Plane& plane) {
    double length = std::sqrt(plane.A * plane.A + plane.B * plane.B + plane.C * plane.C);
    return Plane{plane.A / length, plane.B / length, plane.C / length, plane.D / length};
}

void visualizePlane(const std::vector<PlanePoint>& points, const Plane& plane) {
    // Create a point cloud
    PointCloud::Ptr cloud(new PointCloud);
    for (const auto& point : points) {
        cloud->points.push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    // Create a PCLVisualizer object
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    // // Add the plane to the viewer
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // coefficients->values.resize(4); // We need 4 values for the plane model Ax + By + Cz + D = 0
    // coefficients->values[0] = plane.A;
    // coefficients->values[1] = plane.B;
    // coefficients->values[2] = plane.C;
    // coefficients->values[3] = plane.D;
    // viewer->addPlane(*coefficients, "sample plane");

    // Spin the viewer
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void savePointCloudAndPlaneAsPCD(const std::vector<PlanePoint>& points, const Plane& plane, const std::string& filename) {
    // // Create a point cloud
    PointCloud::Ptr cloud(new PointCloud);
    for (const auto& point : points) {
        cloud->points.push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }

    // Add points from the plane
    for (double x = -10; x <= 10; x += 1) {
        for (double y = -10; y <= 10; y += 1) {
            double z = (-plane.D - plane.A * x - plane.B * y) / plane.C;  // Calculate z from the plane equation
            cloud->points.push_back(pcl::PointXYZ(x, y, z));
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;

    // Save the point cloud to a PCD file
    cout << "is saving" << endl;

    int result = pcl::io::savePCDFileASCII(filename, *cloud);
    if (result != 0) {
        std::cerr << "Error: savePCDFileASCII returned " << result << '\n';
    }
}

void savePointCloudAsTXT(const std::vector<PlanePoint>& points, const std::string& filename) {
    std::ofstream file(filename);

    if (!file) {
        std::cerr << "Could not open file for writing: " << filename << '\n';
        return;
    }

    for (const auto& point : points) {
        file << point.x << ' ' << point.y << ' ' << point.z << '\n';
    }
}

void savePlaneAsTXT(const Plane& plane, const std::string& filename) {
    std::ofstream file(filename);

    if (!file) {
        std::cerr << "Could not open file for writing: " << filename << '\n';
        return;
    }

    file << plane.A << ' ' << plane.B << ' ' << plane.C << ' ' << plane.D << '\n';
}

std::unique_ptr<Plane> fitPlaneRansac(const std::vector<PlanePoint>& points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : points) {
        cloud->points.push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }

    if (cloud->points.size() < 3) {
        return nullptr;  // Not enough points to define a plane
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        return nullptr;  // Unable to find a plane in the point cloud
    }

    // Return the plane coefficients in Hessian Normal Form
    return std::make_unique<Plane>(Plane{coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]});
}

pcl::PointCloud<pcl::PointXYZ>::Ptr vectorToPointcloud(std::vector<PlanePoint> points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < points.size(); ++i)
    {
        pcl::PointXYZ point;
        point.x = points[i].x;
        point.y = points[i].y;
        point.z = points[i].z;
        cloud->points.push_back(point);
    }
    cloud->width = points.size();
    cloud->height = 1;
    return cloud;
}

// 提取最大平面
std::unique_ptr<Plane> extractMaxPlane(std::vector<PlanePoint> points, Eigen::Vector3d reference_normal) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = vectorToPointcloud(points);
    
    // 使用当前的时间戳生成一个唯一的文件名
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    std::string filename = "/home/zheng/projects/ni_slam_ws/build_old/test_ply_" + std::to_string(timestamp) + ".pcd";

    // 保存点云为PLY文件
    pcl::io::savePCDFileASCII(filename, *cloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // 创建法向量估计对象
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // 创建一个空的kd树对象，并将其传递给法向量估计对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // 输出数据集
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // 使用半径在3cm范围内的近邻点
    ne.setRadiusSearch(0.03);

    // 计算特征值
    ne.compute(*cloud_normals);

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01);
    seg.setRadiusLimits(0, 0.1);

    // 设置参考法向量
    Eigen::Vector3f axis(reference_normal.cast<float>());
    seg.setAxis(axis);

    // 设置最大偏离角为5度
    seg.setEpsAngle(pcl::deg2rad(95.0f));

    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers, *coefficients);

    std::unique_ptr<Plane> plane(new Plane);
    plane->A = coefficients->values[0];
    plane->B = coefficients->values[1];
    plane->C = coefficients->values[2];
    plane->D = coefficients->values[3];

    return plane;
}
