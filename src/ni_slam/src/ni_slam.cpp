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
    cellSize = 10;
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

}


// void NI_SLAM::callback(const CloudType::ConstPtr& pcl_cloud, const ImuConstPtr& imu, const ImageConstPtr& depth_img)
void NI_SLAM::callback(const CloudType::ConstPtr& pcl_cloud, const ImageConstPtr& depth_img)
{
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

    InputDataPtr data = std::shared_ptr<InputData>(new InputData());
    data->depth_image = imD.clone();
    data->pcl_cloud = pcl_cloud.clone();
    data->time = nth_frame;

    while(_data_buffer.size()>=3 && !_shutdown){
        usleep(2000);
    }

    _dataBuffer_mutex.lock();
    _data_buffer.push(data);
    _dataBuffer_mutex.unlock();

    
    // time_use = jtimer.end();
    // rotation = tf::Quaternion(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w); // current rotation relative to inital frame
    // rotation *= rotation_imu; // for ICL_NUIM setting, rotation_imu should be (0,0,0)    
    // For debugging, real values are stored in acceleration, can be deleted safely.
    // tf::Vector3 position(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    // pose_real.setOrigin(position);
    // pose_real.setRotation(rotation);
    
    if (initialized == false)
    {
        EfficientDepth2NormalMap(imD, normalsCV, cellSize, vertexMapx, vertexMapy);
        rotation_rel = tf::Quaternion(0,0,0,1);
        rotation_key = rotation_rel;

        rotation = (rotation_key * rotation_rel).normalized(); // current rotation relative to inital frame

        init_poses();
        
        adapt_field_of_view(pcl_cloud); // need rotation, so should get rotation before this
        
        reproject(pcl_cloud);
        
        train();
        initialized = true;
        if(flag_save_file)
            save_file();

        if(flag_show_gt)
        {
            gt_pose_last = gt_poses[nth_frame];
        }
        nth_frame++;
        return;
    }

    EfficientDepth2NormalMap(imD, normalsCV, cellSize, vertexMapx, vertexMapy);
    cv::Mat AnglesMap = cv::Mat::ones(cv::Size(depth_height/maxPyramidLevel, depth_width/maxPyramidLevel), CV_64FC3)*720;
    Eigen::Matrix3d rotation_matrix;
    EfficientNormal2RotationMat(normalsCV_last, normalsCV, rotation_matrix, AnglesMap);
    Eigen::Quaterniond q_rel(rotation_matrix);
    rotation_rel = tf::Quaternion(q_rel.coeffs()[0], q_rel.coeffs()[1], q_rel.coeffs()[2], q_rel.coeffs()[3]);
    rotation = (rotation_key * rotation_rel).normalized(); // current rotation relative to inital frame

    reproject(pcl_cloud, tf::Transform(rotation_key.inverse()*rotation)); // reproject current frame using rotation to key frame coordinates
    // sleep(1000); 

    get_response();

    get_psr();

    refine_keyclouds_poses();

    publish_poses();

    if (flag_publish_twist)
        publish_twists();

    if (flag_publish_incremental_keypose_cov)
        publish_incremental_keypose_cov();

    // if (!(psr > psrbound))
    if (!(psr > psrbound) || valid_points<3000 || tf::Transform(rotation_key.inverse()*rotation).getRotation().getAngle() >= 0.15)
    {
        // cout << "!!!!!!!!!!rotation: " << tf::Transform(rotation_key.inverse()*rotation).getRotation().getAngle() << endl;
        // cout << "valid points: " << valid_points << endl;
        if(flag_publish_refined_keyframe)
            publish_keyframe();

        adapt_field_of_view(pcl_cloud);

        reproject(pcl_cloud); // reproject original current frame for training
        
        train();

        if(flag_show_gt)
        {
            Eigen::Matrix3d R_gt_last;
            Eigen::Quaterniond q_gt_last(gt_pose_last[6], gt_pose_last[3], gt_pose_last[4], gt_pose_last[5]);
            R_gt_last = q_gt_last.toRotationMatrix();
            Eigen::Matrix3d R_gt_cur;
            Eigen::Quaterniond q_gt_cur(gt_poses[nth_frame][6], gt_poses[nth_frame][3], gt_poses[nth_frame][4], gt_poses[nth_frame][5]);
            R_gt_cur = q_gt_cur.toRotationMatrix();
            Eigen::Matrix3d R_gt_rlt;
            R_gt_rlt = R_gt_last.transpose() * R_gt_cur;
            Eigen::AngleAxisd gt_angleaxis;
            gt_angleaxis.fromRotationMatrix(R_gt_rlt);
            cout << "gt axis-angle: \n" << gt_angleaxis.angle() * gt_angleaxis.axis() << endl;
            cout << "gt_pose_last: " << gt_pose_last[3] << ", " << gt_pose_last[4] << ", " << gt_pose_last[5] << ", " << gt_pose_last[6] << ", " << endl;
            cout << "gt_pose: " << gt_poses[nth_frame][3] << ", " << gt_poses[nth_frame][4] << ", " << gt_poses[nth_frame][5] << ", " << gt_poses[nth_frame][6] << ", " << endl;
            gt_pose_last = gt_poses[nth_frame]; // shoule make sure the nth_frameth gt pose corresponding to the nth_frame.png
        }

        ROS_WARN("Trained. %d times with PSR: %.1f............", train_num++, psr);
        if(flag_save_file)
            save_file();
    }

    time_use = jtimer.end();

    ROS_INFO("(%td,%td); Res: %5fm  Timing: %.4fs = %.2fHz; Dt: %.4fs PSR: %04.1f;", 
        max_index[0], max_index[1], resolution, time_use, 1/time_use, time_diff, psr);

    // if(flag_save_file)
    //     save_file();

    if(flag_visualization)
        show();
    
    nth_frame++;
}

void NI_SLAM::EstimateNormalMapThread(){
    while(!_shutdown){
        if(_data_buffer.empty()){
            usleep();
            continue;
        }
    
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
        normalMap_data->time = frame_id;
        normalMap_data->normal_map = normalsCV.clone();
        normalMap_data->pcl_cloud = input_data->pcl_cloud.clone();


        while(_normalMap_buffer.size()>=2){
            usleep(2000);
        }

        _normalMap_mutex.lock();
        _normalMap_buffer.push(normalsMap_data);
        _normalMap_mutex.unlock();
    }
}

void NI_SLAM::EstimateRotationThread(){
    while(!_shutdown){
        if(_normalMap_buffer.empty()){
            usleep(2000);
            continue;
        }
        NormalMapPtr normal_map;
        _normalMap_mutex.lock();
        normal_map = _normalMap_buffer.front();
        _normalMap_buffer.pop();
        _normalMap_mutex.unlock();

        int frame_id = normal_map->time;
        

        FramePtr frame = std::shared_ptr<Frame>(new Frame(frame_id, normal_map->normal_map, normal_map->pcl_cloud));

        if(!initialized){
            rotation_rel = tf::Quaternion(0,0,0,1);
            rotation_key = rotation_rel;

            rotation = (rotation_key * rotation_rel).normalized(); // current rotation relative to inital frame

            init_poses();
            
            adapt_field_of_view(pcl_cloud); // need rotation, so should get rotation before this
            
            reproject(pcl_cloud);
            
            train();
            initialized = true;
            if(flag_save_file)
                save_file(); // TODO: add input parameters to decide which frame to be recorded
            
            frame->SetRotation(rotation);
            frame->SetPose(pose);
            ref_frame = // TODO: define a ref frame to save the normal_map
            // no need to save color_key etc.(maybe????)
            
            
        }
        else{


        }

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

    cout << "translation is: " << transform.getOrigin().getX() << ", " << transform.getOrigin().getY() << ", " << transform.getOrigin().getZ() << ", " << endl;
    cout << "rotation is: " << transform.getRotation().getAngle() << endl;

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

    rotation_key = rotation;

    pose_key = pose;

    // pose_real_key = pose_real;

    color_key = color;

    depth_key = depth;

    weights = weights_dep;
   
    weights_key = weights_dep;
    
    model_fft = fft(color_key);
    
    kernel = gaussian_kernel();
    
    alpha = labels_fft/(kernel + lambda);

    normalsCV_last = normalsCV.clone();


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
    rotation_key = rotation;

    tf::Vector3 position(0, 0, 0);

    pose_key.setOrigin(position);

    // pose_key.setOrigin(pose_real.getOrigin());
        
    pose_key.setRotation(rotation);

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

    tf::Quaternion rotation_inc = ((rotation_key.inverse()) * rotation).normalized();

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
    normalsEig_x_mat = (normalsEig_x / (1e-6 + sqrt(normalsEig_x * normalsEig_x + normalsEig_y * normalsEig_y + normalsEig_z * normalsEig_z))).matrix();
    normalsEig_y_mat = (normalsEig_y / (1e-6 + sqrt(normalsEig_x * normalsEig_x + normalsEig_y * normalsEig_y + normalsEig_z * normalsEig_z))).matrix();
    normalsEig_z_mat = (normalsEig_z / (1e-6 + sqrt(normalsEig_x * normalsEig_x + normalsEig_y * normalsEig_y + normalsEig_z * normalsEig_z))).matrix();
    cv::eigen2cv(normalsEig_x_mat, channels_all[0]);
    cv::eigen2cv(normalsEig_y_mat, channels_all[1]);
    cv::eigen2cv(normalsEig_z_mat, channels_all[2]);

    cv::merge(channels_all, _normalsCV);
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
            if((_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u)).val[0] < 0.85){ //0.7){
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
                    if((modes[i].t() * _normalsCV.at<cv::Vec3d>(v, u)).val[0] >= 0.999){
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
                        break;
                    }
                }
                if(found_mode == false){
                    // if((_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v-1, u)).val[0] < 0.999 ||
                    // (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v+1, u)).val[0] < 0.999 ||
                    // (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v, u-1)).val[0] < 0.999 ||
                    // (_normalsCV.at<cv::Vec3d>(v, u).t() * _normalsCV.at<cv::Vec3d>(v, u+1)).val[0] < 0.999 ||
                    // (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v-1, u)).val[0] < 0.999 ||
                    // (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v+1, u)).val[0] < 0.999 ||
                    // (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u-1)).val[0] < 0.999 ||
                    // (_normalsCV_last.at<cv::Vec3d>(v, u).t() * _normalsCV_last.at<cv::Vec3d>(v, u+1)).val[0] < 0.999){
                    //     continue;
                    // }
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
            cout << "**************************mode" << i << ": " << modes[i] << endl;
            cout << "totally " << mode_count[i] << " points" << endl;
            sumPoints+=mode_count[i];
            selected_mode.push_back(i);
            count_valid_mode++;
        }
    }
    cout << count_valid_mode << endl;

    
    int count_second_max{0};
    //** if only one mode is more than 12000 points, will select one more mode if this new mode is more than 5000
    //** otherwise, will select two modes if they are less than 5000
    // if(count_valid_mode<2){
    //     cout << "=====================================================================================" << endl;
    //     cout << "mode smaller than 2, will select one (>5000 points) or more (<5000 points) mode from plane with points less than 12000" << endl;
    //     selected_mode.push_back(0);
    //     for(int i=0; i<sizeofMode; i++){
    //         if(i == selected_mode[0]){
    //             continue;
    //         }
    //         if(mode_count[i]>=3000 && mode_count[i]<=12000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_second_max){
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
    //             if(mode_count[i]>=3000 && mode_count[i]<=12000 && (modes[i].t() * modes[selected_mode[0]]).val[0]<0.5 && mode_count[i]>count_third_max &&
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
        cout << "=====================================================================================" << endl;
        cout << "mode smaller than 2, will select one (>12000 points) or more (<12000 points) mode from plane with points less than 12000" << endl;
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
            cout << "**************************mode" << selected_mode[1] << ": " << modes[selected_mode[1]] << endl;
            cout << "totally (>1000) " << mode_coor[selected_mode[1]].size() << " points" << endl;
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
                cout << "**************************mode" << selected_mode[2] << ": " << modes[selected_mode[2]] << endl;
                cout << "totally (>1000) " << mode_coor[selected_mode[2]].size() << " points" << endl;
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
                cout << "**************************mode" << selected_mode[1] << ": " << modes[selected_mode[1]] << endl;
                cout << "totally (>500) " << mode_coor[selected_mode[1]].size() << " points" << endl;
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
        cout << "count valid mode: " << count_valid_mode << endl;
        cout << "---------------------------------------------------------" << endl;
        cout << "mode more than 2, will compare which paired modes is best" << endl;
        
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
        for(int i=0; i<(paired_modes.size()); i++){
            cout << "**current paired modes is: " << paired_modes[i][0] << " and " << paired_modes[i][1] << "**" << endl;
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
                float weight = float(mode_count[selected_mode[j]]) / float(sumPoints);
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
                
                Eigen::Vector3d vTTest_rotated = R_ * vTTest;
                // cout << "vTest: " << vTest << endl;
                // cout << "vTTest_rotated: " << vTest.transpose()*vTTest_rotated << endl;
                // error_cur += (weight * vTest.cross(vTTest_rotated).norm());
                error_cur += (vTest.cross(vTTest_rotated).norm());
            }
            cout << "**current paired modes' (weighted) error is: " << error_cur/count << " **\n" << endl;
            if(error_cur/count < error){
                error = error_cur/count;
                _R = R_;
                best_paired_modes = i;
            }
        }
        cout << "---------------------------------------------------------" << endl;
        cout << "------------best paired mode IS: " << paired_modes[best_paired_modes][0] << " and " << paired_modes[best_paired_modes][1] << " ------------------" << endl;
        valid_points = min(mode_count[paired_modes[best_paired_modes][0]], mode_count[paired_modes[best_paired_modes][1]]);
    }
    else if(count_valid_mode==2){
        Eigen::Vector3d v1{0,0,0};
        Eigen::Vector3d v11{0,0,0};
        Eigen::Vector3d v2{0,0,0};
        Eigen::Vector3d v22{0,0,0};

        cout << "=====================================================================================" << endl;
        cout << "mode equal to 2" << endl;
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

        cout << "=====================================================================================" << endl;
        valid_points = min(mode_count[selected_mode[0]], mode_count[selected_mode[1]]);
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
        cout << "**in case 1 (no rotation)**" << endl;

        _R.setIdentity();
        return;
    } 
    else if((_v1.transpose() * _v11)>0.999999){
        cout << "**in case 2 (axis is _v1)**" << endl;
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
        cout << "**in case 2 (axis is _v2)**" << endl;
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
        cout << "**in case 3**" << endl;
        N = ((_v11-_v1).cross(_v22-_v2))/((_v11-_v1).cross(_v22-_v2).norm());
        cout << "N: " << N << endl;
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
    cout << "axis-angle: \n" << angleaxis.angle() * angleaxis.axis() << endl;

}


inline ArrayXXcf NI_SLAM::fft(const ArrayXXf& x)
{
    ArrayXXcf xf = ArrayXXcf(width/2+1, height);
    fft_plan=fftwf_plan_dft_r2c_2d(height, width, (float(*))(x.data()), 
        (float(*)[2])(xf.data()), FFTW_ESTIMATE); // reverse order for column major
    
    fftwf_execute(fft_plan);
    fftwf_destroy_plan(fft_plan);
    fftw_cleanup();
    return xf;
}


inline ArrayXXf NI_SLAM::ifft(const ArrayXXcf& xf)
{
    ArrayXXf x = ArrayXXf(width, height);
    ArrayXXcf cxf = xf;
    fft_plan=fftwf_plan_dft_c2r_2d(height, width, (float(*)[2])(cxf.data()),
        (float(*))(x.data()), FFTW_ESTIMATE);
    
    fftwf_execute(fft_plan);
    fftwf_destroy_plan(fft_plan);
    fftw_cleanup();
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
            ss >> sD;
            ss >> t;
            ss >> sRGB;
            ss >> t;
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
    file<<nth_frame+1<<" "
        <<pose.getOrigin()[0]<<" "
        <<pose.getOrigin()[1]<<" "
        <<pose.getOrigin()[2]<<" "
        <<pose.getRotation().x()<<" "
        <<pose.getRotation().y()<<" "
        <<pose.getRotation().z()<<" "
        <<pose.getRotation().w()<<"\n"; // " " <<max_response<<" "<<psr<<" "<<time_use<<"\n";
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

void NI_SLAM::ShutDown(){
    _shutdown = true;
    _normalMap_thread.join();
    _rotation_thread.join();
    _translation_thread.join();
}


NI_SLAM::~NI_SLAM()
{
    fftwf_destroy_plan(fft_plan);
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
        cout << "there is no values!!!!!!" << endl;
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