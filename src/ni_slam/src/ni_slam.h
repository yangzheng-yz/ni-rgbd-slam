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

#ifndef NI_SLAM_H
#define NI_SLAM_H
// #define EIGEN_USE_MKL_ALL
#include <iostream>
#include <sstream>
#include <string.h>
#include <fstream>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/eigen.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ni_slam/MapInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "lib.h"

using namespace std;
using namespace sensor_msgs;

typedef pcl::PointCloud<pcl::PointXYZRGB> CloudType;

typedef Array<bool,Dynamic,Dynamic> ArrayXXb;

class NI_SLAM
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NI_SLAM(ros::NodeHandle, int height, int width, int depth_height, int depth_width, float psr);

    // NI_SLAM(ros::NodeHandle, int height, int width, float psr);

    ~NI_SLAM();

    // void callback(const CloudType::ConstPtr&, const ImuConstPtr&, const ImageConstPtr& depth_img);
    void callback(const CloudType::ConstPtr&, const ImageConstPtr& depth_img);

    void set_rotation_imu(double yaw, double pitch, double roll);

    void set_publish_refined_keyframe();

    void set_publish_incremental_keypose();

    void set_publish_twist();

    void rotationFromTwoPlanes(Eigen::Vector3d &_v1, Eigen::Vector3d &_v11, Eigen::Vector3d &_v2, Eigen::Vector3d &_v22, Eigen::Matrix3d &_R);

    void ShutDown();

    void EstimateNormalMapThread();
    // vector<double> DirectRotationEst(bool &initialized);

private:

    inline bool check_synchronization(std_msgs::Header, std_msgs::Header, double);

    inline void adapt_field_of_view(const CloudType::ConstPtr&);

    inline void reproject(const CloudType::ConstPtr&, const tf::Transform& = tf::Transform(tf::Quaternion(0,0,0,1)));

    inline void train();

    inline ArrayXXcf gaussian_kernel();

    inline ArrayXXcf gaussian_kernel(const ArrayXXcf&);

    inline ArrayXXcf get_labels_fft();

    inline void init_poses();

    inline void get_response();

    inline void refine_keyclouds_poses();

    inline ArrayXXf shift(const ArrayXXf&, int, int);

    inline float get_psr();

    inline ArrayXXcf fft(const ArrayXXf&);

    inline ArrayXXf ifft(const ArrayXXcf&);

    inline void publish_poses();

    inline void publish_twists();

    inline void show();

    inline void publish_keyframe();

    inline void publish_incremental_keypose_cov();

    inline void EfficientDepth2NormalMap(cv::Mat &_imD, cv::Mat &_normalsCV, int &_cellSize,
                                Eigen::MatrixXd &vertexMapxx, Eigen::MatrixXd &vertexMapyy);

    inline void EfficientNormal2RotationMat(cv::Mat &_normalsCV_last, cv::Mat &_normalsCV, Eigen::Matrix3d &_R, cv::Mat &_AnglesMap);

    

private:

    bool _shutdown;
    // normal thread
    std::mutex _dataBuffer_mutex;
    std::queue<InputDataPtr> _data_buffer;
    std::thread _normalMap_thread;

    ros::NodeHandle nh;          // ROS node handle

    // for adaptive field of view
    float max_distance;          // max sensing distance for 3D cameras
    int point_cloud_sample;      // jump points
    int len_hist;                // histogram length of clouds
    ArrayXi histogram;           // histogram of clouds, for adapting field of view
    ArrayXi histogram_sequence;  // arithemetic sequence for histogram

    // for localization
    bool initialized;
    float resolution;            // resolution of axonometric image
    float min_valid_depth;       // minimum valid depth of obtained clouds
    int height, width;           // size of axonometric image
    int depth_height, depth_width;  
    int train_num;               // key frame number
    ArrayXXf color;              // current color cloud
    ArrayXXf depth;              // current depth cloud
    ArrayXXf color_key;          // key color cloud
    ArrayXXf depth_key;          // key depth cloud
    ArrayXXf color_rect;         // current color cloud rectified by translation
    ArrayXXf depth_rect;         // current depth cloud rectified by translation
    ArrayXXf color_empty;        // empty color cloud for initilization
    ArrayXXf depth_empty;        // empty depth cloud for initilization
    tf::Quaternion rotation_imu; // rotation from imu to camera, decided by hardware
    tf::Quaternion rotation;     // current imu rotation
    tf::Quaternion rotation_key; // key frame rotation
    tf::Transform  pose;         // current estimated pose relative to initial pose
    tf::Transform  pose_key;     // pose of key point cloud
    tf::Transform  transform_inc;// incremental pose transformation relative to key pose
    tf::Transform  transform_dif;// difference of pose transformation
    geometry_msgs::PoseStamped posestamped;
    geometry_msgs::TwistWithCovarianceStamped twiststamped;
    geometry_msgs::PoseWithCovarianceStamped pose_inc_cov;
    ros::Publisher pose_pub;
    ros::Publisher twist_pub;
    ros::Publisher pose_cov_pub;
    ni_slam::MapInfo map_info;
    ros::Publisher map_info_pub;

public:
    string parent_frame;         // parent frame name
    string child_frame;          // child frame name
    float adaptive_mu;             // F(x) = hist/his_seq + mu * his_seq

private:

    // for rotation estimation
    vector<double> absolute_rotation_q[4]; // current frame's quaternion
    Eigen::Matrix3d Ab_R; // current frame's rotation matrix relative to initial frame
    Eigen::Matrix3d Ab_R_key; // key frame's rotation matrix relative to initial frame
    int gap{1}; // jump $gap frames for pose estimation
    Eigen::Matrix3d K;
    int cellSize;
    double maxPyramidLevel;
    Eigen::VectorXd v; // used to generate linspaced pixel coordinates-y
    Eigen::VectorXd u; // pixel coordinates-x
    Eigen::MatrixXd vMap;
    Eigen::MatrixXd uMap;
    Eigen::MatrixXd uMap_t;
    Eigen::MatrixXd vertexMapx;
    Eigen::MatrixXd vertexMapy;
    cv::Mat normalsCV_last;
    cv::Mat normalsCV;
    tf::Quaternion rotation_rel; // rotation between frames
    int nth_frame{0};
    int valid_points;
    vector<vector<double>> gt_poses;
    vector<double> gt_pose_last;

    // for translation estimation
    fftwf_plan fft_plan;
    float sigma;                 // the gaussian_kernel sigma
    float lambda;                // the regulerazaion tersmall
    float small;                 // prevent divison by zero
    float max_response;          // maximum response value in response matrix
    float model_square;          // mean of elements square
    float translation_z;         // translation in depth direction
    Vector2i trans;              // translation of image
    ArrayXXf::Index max_index[2];// index of max value
    ArrayXXcf color_fft;         // fft of point cloud
    ArrayXXcf labels_fft;        // fft of labels
    ArrayXXcf model_fft;         // fft of key cloud
    ArrayXXcf model_fft_conj;    // conjugate of fft of key cloud
    ArrayXXcf kernel;            // kernel matrix of key cloud
    ArrayXXcf alpha;             // coefficient
    ArrayXXf response;           // response matrix
    ArrayXXf valid_diff;         // valid differences for matching
    ArrayXXb inliers;            // inliers of matching flag

    // for key cloud refinement
    float psr;                   // peak sidelobe ratio
    float psrbound;              // psr bound
    ArrayXXf weights;            // key cloud weights
    ArrayXXf weights_rect;       // rectified cloud weights
    ArrayXXf weights_dep;        // init weights for depth image
    ArrayXXf weights_key;        // remember original key weights, 1 or 0
    ArrayXXf weights_sum;        // sumation of weights, for refinement of clouds
    ArrayXXf depth_trans;        // depth translation
    ArrayXXf weighted;           // combined weight array
    double weighted_sum;         // sum of combined weight array
    double response_mean;        // mean of sidelobe of response array
    double sigma_x2, sigma_y2, sigma_z2; // square of sigma translation estimation
    bool flag_publish_refined_keyframe;
    bool flag_publish_incremental_keypose_cov;
    bool flag_publish_twist;
    cv_bridge::CvImage cv_refined_color;
    cv_bridge::CvImage cv_refined_depth;
    image_transport::Publisher image_pub_refined_color;
    image_transport::Publisher image_pub_refined_depth;
    double time_diff;            // header stamped difference between two clouds

    // intermediate variable
    ArrayXXcf xyf;
    ArrayXXf xy;
    ArrayXXf xxyy;


// the following is for debug
public:

    void set_file(string);
    void set_visualization();
    void set_gt_poses(string);

private:

    void save_file();

    Jeffsan::CPPTimer timer;
    double time_use;
    bool flag_save_file;
    bool flag_visualization;
    bool flag_show_gt;
    ofstream file;               // estimated poses to save
    string filename;
    tf::TransformBroadcaster br;
    tf::Transform  pose_real;    // real pose
    tf::Transform  pose_real_key;// real key pose
    std_msgs::Header ros_header;
    image_transport::ImageTransport image_transport;
    cv_bridge::CvImage cv_keyframe;
    cv_bridge::CvImage cv_newframe;
    cv_bridge::CvImage cv_response;
    image_transport::Publisher image_pub_key;
    image_transport::Publisher image_pub_new;
    image_transport::Publisher image_pub_response;
};

class DynamicMedian{
private:
    priority_queue<double> big_heap;
    priority_queue<double, vector<double>, greater<double>> small_heap;
    double _sum{0};
    int _count{0};
public:
    void addNum(double _num);
    double findMedian();
    double findMean();
};

struct InputData{
    cv::Mat depth_image;
    int time;
    CloudType::ConstPtr pcl_cloud;
    InputData() {};
    InputData& operator =(InputData& other){
        depth_image = other.depth_image.clone();
        time = other.time;
        pcl_cloud = other.pcl_cloud.clone();
        return *this;
    }
}
typedef std::shared_ptr<InputData> InputDataPtr;

struct TrackingData{
  FramePtr frame;
  FramePtr ref_keyframe;
  std::vector<cv::DMatch> matches;
  InputDataPtr input_data;

  TrackingData() {}
  TrackingData& operator =(TrackingData& other){
		frame = other.frame;
		ref_keyframe = other.ref_keyframe;
		matches = other.matches;
		input_data = other.input_data;
		return *this;
	}
};

#endif
