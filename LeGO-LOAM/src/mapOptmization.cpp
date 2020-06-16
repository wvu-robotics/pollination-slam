// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
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
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.
#include "utility.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/nonlinear/ISAM2.h>

using namespace gtsam;

class mapOptimization{

private:

    NonlinearFactorGraph _gtSAMgraph;
    Values _initialEstimate;
    Values _optimizedEstimate;
    ISAM2 *_isam;
    Values _isamCurrentEstimate;

    noiseModel::Diagonal::shared_ptr _priorNoise;
    noiseModel::Diagonal::shared_ptr _odometryNoise;
    noiseModel::Diagonal::shared_ptr _constraintNoise;

    ros::NodeHandle nh;

    ros::Publisher _pubLaserCloudSurround;
    ros::Publisher _pubOdomAftMapped;
    ros::Publisher _pubKeyPoses;

    ros::Publisher _pubHistoryKeyFrames;
    ros::Publisher _pubIcpKeyFrames;
    ros::Publisher _pubRecentKeyFrames;

    ros::Subscriber _subLaserCloudCornerLast;
    ros::Subscriber _subLaserCloudSurfLast;
    ros::Subscriber _subOutlierCloudLast;
    ros::Subscriber _subLaserOdometry;
    ros::Subscriber _subImu;

    nav_msgs::Odometry _odomAftMapped;
    tf::StampedTransform _aftMappedTrans;
    // tf::TransformBroadcaster _tfBroadcaster;

    vector<pcl::PointCloud<PointType>::Ptr> _cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> _surfCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> _outlierCloudKeyFrames;

    deque<pcl::PointCloud<PointType>::Ptr> _recentCornerCloudKeyFrames;
    deque<pcl::PointCloud<PointType>::Ptr> _recentSurfCloudKeyFrames;
    deque<pcl::PointCloud<PointType>::Ptr> _recentOutlierCloudKeyFrames;
    int _latestFrameID;

    vector<int> _surroundingExistingKeyPosesID;
    deque<pcl::PointCloud<PointType>::Ptr> _surroundingCornerCloudKeyFrames;
    deque<pcl::PointCloud<PointType>::Ptr> _surroundingSurfCloudKeyFrames;
    deque<pcl::PointCloud<PointType>::Ptr> _surroundingOutlierCloudKeyFrames;

    PointType _previousRobotPosPoint;
    PointType _currentRobotPosPoint;

    pcl::PointCloud<PointType>::Ptr _cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr _cloudKeyPoses6D;



    pcl::PointCloud<PointType>::Ptr _surroundingKeyPoses;
    pcl::PointCloud<PointType>::Ptr _surroundingKeyPosesDS;

    pcl::PointCloud<PointType>::Ptr _laserCloudCornerLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr _laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr _laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr _laserCloudSurfLastDS; // downsampled surf featuer set from odoOptimization

    pcl::PointCloud<PointType>::Ptr _laserCloudOutlierLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr _laserCloudOutlierLastDS; // corner feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr _laserCloudSurfTotalLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr _laserCloudSurfTotalLastDS; // downsampled corner featuer set from odoOptimization

    pcl::PointCloud<PointType>::Ptr _laserCloudOri;
    pcl::PointCloud<PointType>::Ptr _coeffSel;

    pcl::PointCloud<PointType>::Ptr _laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr _laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr _laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr _laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr _kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr _kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr _kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr _kdtreeHistoryKeyPoses;


    pcl::PointCloud<PointType>::Ptr _nearHistoryCornerKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr _nearHistoryCornerKeyFrameCloudDS;
    pcl::PointCloud<PointType>::Ptr _nearHistorySurfKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr _nearHistorySurfKeyFrameCloudDS;

    pcl::PointCloud<PointType>::Ptr _latestCornerKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr _latestSurfKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr _latestSurfKeyFrameCloudDS;

    pcl::KdTreeFLANN<PointType>::Ptr _kdtreeGlobalMap;
    pcl::PointCloud<PointType>::Ptr _globalMapKeyPoses;
    pcl::PointCloud<PointType>::Ptr _globalMapKeyPosesDS;
    pcl::PointCloud<PointType>::Ptr _globalMapKeyFrames;
    pcl::PointCloud<PointType>::Ptr _globalMapKeyFramesDS;

    std::vector<int> _pointSearchInd;
    std::vector<float> _pointSearchSqDis;

    pcl::VoxelGrid<PointType> _downSizeFilterCorner;
    pcl::VoxelGrid<PointType> _downSizeFilterSurf;
    pcl::VoxelGrid<PointType> _downSizeFilterOutlier;
    pcl::VoxelGrid<PointType> _downSizeFilterHistoryKeyFrames; // for histor key frames of loop closure
    pcl::VoxelGrid<PointType> _downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    pcl::VoxelGrid<PointType> _downSizeFilterGlobalMapKeyPoses; // for global map visualization
    pcl::VoxelGrid<PointType> _downSizeFilterGlobalMapKeyFrames; // for global map visualization

    double _timeLaserCloudCornerLast;
    double _timeLaserCloudSurfLast;
    double _timeLaserOdometry;
    double _timeLaserCloudOutlierLast;
    double _timeLastGloalMapPublish;

    bool _newLaserCloudCornerLast;
    bool _newLaserCloudSurfLast;
    bool _newLaserOdometry;
    bool _newLaserCloudOutlierLast;


    float _transformLast[6];
    float _transformSum[6];
    float _transformIncre[6];
    float _transformTobeMapped[6];
    float _transformBefMapped[6];
    float _transformAftMapped[6];


    int _imuPointerFront;
    int _imuPointerLast;

    std::vector<double> _imuTime;
    std::vector<float> _imuRoll;
    std::vector<float> _imuPitch;
    // double _imuTime[_imuQueLength];
    // float _imuRoll[_imuQueLength];
    // float _imuPitch[_imuQueLength];

    std::mutex _mtx;

    double _timeLastProcessing;

    PointType _pointOri, _pointSel, _pointProj, _coeff;

    cv::Mat _matA0;
    cv::Mat _matB0;
    cv::Mat _matX0;

    cv::Mat _matA1;
    cv::Mat _matD1;
    cv::Mat _matV1;

    bool _isDegenerate;
    cv::Mat _matP;

    int _laserCloudCornerFromMapDSNum;
    int _laserCloudSurfFromMapDSNum;
    int _laserCloudCornerLastDSNum;
    int _laserCloudSurfLastDSNum;
    int _laserCloudOutlierLastDSNum;
    int _laserCloudSurfTotalLastDSNum;

    bool _potentialLoopFlag;
    double _timeSaveFirstCurrentScanForLoopClosure;
    int _closestHistoryFrameID;
    int _latestFrameIDLoopCloure;

    bool _aLoopIsClosed;

    float _cRoll, _sRoll, _cPitch, _sPitch, _cYaw, _sYaw, _tX, _tY, _tZ;
    float _ctRoll, _stRoll, _ctPitch, _stPitch, _ctYaw, _stYaw, _tInX, _tInY, _tInZ;

    int _imuQueLength;
    string _imuTopic;
    float _scanPeriod;
    string _fileDirectory;
    bool _loopClosureEnableFlag;
    double _mappingProcessInterval;
    float _surroundingKeyframeSearchRadius;
    int _surroundingKeyframeSearchNum;
    float _historyKeyframeSearchRadius;
    int _historyKeyframeSearchNum;
    float _historyKeyframeFitnessScore;
    float _globalMapVisualizationSearchRadius;

    string _mapFrameId;
public:



    mapOptimization():
        nh("~")
    {
        if(!nh.getParam("/lego_loam/imu/imuQueLength", _imuQueLength)) ROS_ERROR("Failed to get param '/lego_loam/imu/imuQueLength'");
        if(!nh.getParam("/lego_loam/imuTopic", _imuTopic)) ROS_ERROR("Failed to get param '/lego_loam/imuTopic'");
        if(!nh.getParam("/lego_loam/fileDirectory", _fileDirectory)) ROS_ERROR("Failed to get param '/lego_loam/fileDirectory'");
        if(!nh.getParam("/lego_loam/laser/scanPeriod", _scanPeriod)) ROS_ERROR("Failed to get param '/lego_loam/laser/scanPeriod'");
        if(!nh.getParam("/lego_loam/mapping/loopClosureEnableFlag", _loopClosureEnableFlag)) ROS_ERROR("Failed to get param '/lego_loam/mapping/loopClosureEnableFlag'");
        if(!nh.getParam("/lego_loam/mapping/mappingProcessInterval", _mappingProcessInterval)) ROS_ERROR("Failed to get param '/lego_loam/mapping/mappingProcessInterval'");
        if(!nh.getParam("/lego_loam/mapping/surroundingKeyframeSearchRadius", _surroundingKeyframeSearchRadius)) ROS_ERROR("Failed to get param '/lego_loam/mapping/surroundingKeyframeSearchRadius'");
        if(!nh.getParam("/lego_loam/mapping/surroundingKeyframeSearchNum", _surroundingKeyframeSearchNum)) ROS_ERROR("Failed to get param '/lego_loam/mapping/surroundingKeyframeSearchRadius'");
        if(!nh.getParam("/lego_loam/mapping/historyKeyframeSearchRadius", _historyKeyframeSearchRadius)) ROS_ERROR("Failed to get param '/lego_loam//mapping/historyKeyframeSearchRadius'");
        if(!nh.getParam("/lego_loam/mapping/historyKeyframeSearchNum", _historyKeyframeSearchNum)) ROS_ERROR("Failed to get param '/lego_loam/mapping/historyKeyframeSearchNum'");
        if(!nh.getParam("/lego_loam/mapping/historyKeyframeFitnessScore", _historyKeyframeFitnessScore)) ROS_ERROR("Failed to get param '/lego_loam/mapping/historyKeyframeFitnessScore'");
        if(!nh.getParam("/lego_loam/mapping/globalMapVisualizationSearchRadius", _globalMapVisualizationSearchRadius)) ROS_ERROR("Failed to get param '/lego_loam/mapping/globalMapVisualizationSearchRadius'");
        if(!nh.getParam("/lego_loam/mapFrameId", _mapFrameId)) ROS_ERROR("Failed to get param '/lego_loam/mapFrameId'");

    	  ISAM2Params parameters;
		    parameters.relinearizeThreshold = 0.01;
		    parameters.relinearizeSkip = 1;
    	  _isam = new ISAM2(parameters);

        _pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("/key_pose_origin", 2);
        // _pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2);
        _pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/slam_map", 2);
        _pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 5);

        _subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2, &mapOptimization::laserCloudCornerLastHandler, this);
        _subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2, &mapOptimization::laserCloudSurfLastHandler, this);
        _subOutlierCloudLast = nh.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2, &mapOptimization::laserCloudOutlierLastHandler, this);
        _subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 5, &mapOptimization::laserOdometryHandler, this);
        _subImu = nh.subscribe<sensor_msgs::Imu> (_imuTopic, 50, &mapOptimization::imuHandler, this);

        _pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/history_cloud", 2);
        _pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/corrected_cloud", 2);
        _pubRecentKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/recent_cloud", 2);

        _downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
        _downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
        _downSizeFilterOutlier.setLeafSize(0.4, 0.4, 0.4);

        _downSizeFilterHistoryKeyFrames.setLeafSize(0.4, 0.4, 0.4); // for histor key frames of loop closure
        _downSizeFilterSurroundingKeyPoses.setLeafSize(1.0, 1.0, 1.0); // for surrounding key poses of scan-to-map optimization

        _downSizeFilterGlobalMapKeyPoses.setLeafSize(1.0, 1.0, 1.0); // for global map visualization
        _downSizeFilterGlobalMapKeyFrames.setLeafSize(0.4, 0.4, 0.4); // for global map visualization

        // _odomAftMapped.header.frame_id = "/camera_init";
        // _odomAftMapped.child_frame_id = "/aft_mapped";
        //
        // _aftMappedTrans.frame_id_ = "/camera_init";
        // _aftMappedTrans.child_frame_id_ = "/aft_mapped";

        _odomAftMapped.header.frame_id = _mapFrameId;
        _odomAftMapped.child_frame_id = "/velodyne_aft_mapped";

        _aftMappedTrans.frame_id_ = _mapFrameId;
        _aftMappedTrans.child_frame_id_ = "/velodyne_aft_mapped";

        allocateMemory();
    }

    void allocateMemory(){

        _imuTime.resize(_imuQueLength);
        _imuRoll.resize(_imuQueLength);
        _imuPitch.resize(_imuQueLength);

        _cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        _cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        _kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        _kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        _surroundingKeyPoses.reset(new pcl::PointCloud<PointType>());
        _surroundingKeyPosesDS.reset(new pcl::PointCloud<PointType>());

        _laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        _laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        _laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        _laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization
        _laserCloudOutlierLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        _laserCloudOutlierLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner feature set from odoOptimization
        _laserCloudSurfTotalLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        _laserCloudSurfTotalLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        _laserCloudOri.reset(new pcl::PointCloud<PointType>());
        _coeffSel.reset(new pcl::PointCloud<PointType>());

        _laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        _laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        _laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        _laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        _kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        _kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());


        _nearHistoryCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        _nearHistoryCornerKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());
        _nearHistorySurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        _nearHistorySurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

        _latestCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        _latestSurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        _latestSurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

        _kdtreeGlobalMap.reset(new pcl::KdTreeFLANN<PointType>());
        _globalMapKeyPoses.reset(new pcl::PointCloud<PointType>());
        _globalMapKeyPosesDS.reset(new pcl::PointCloud<PointType>());
        _globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
        _globalMapKeyFramesDS.reset(new pcl::PointCloud<PointType>());

        _timeLaserCloudCornerLast = 0;
        _timeLaserCloudSurfLast = 0;
        _timeLaserOdometry = 0;
        _timeLaserCloudOutlierLast = 0;
        _timeLastGloalMapPublish = 0;

        _timeLastProcessing = -1;

        _newLaserCloudCornerLast = false;
        _newLaserCloudSurfLast = false;

        _newLaserOdometry = false;
        _newLaserCloudOutlierLast = false;

        for (int i = 0; i < 6; ++i){
            _transformLast[i] = 0;
            _transformSum[i] = 0;
            _transformIncre[i] = 0;
            _transformTobeMapped[i] = 0;
            _transformBefMapped[i] = 0;
            _transformAftMapped[i] = 0;
        }

        _imuPointerFront = 0;
        _imuPointerLast = -1;

        for (int i = 0; i < _imuQueLength; ++i){
            _imuTime[i] = 0;
            _imuRoll[i] = 0;
            _imuPitch[i] = 0;
        }

        gtsam::Vector Vector6(6);
        Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
        _priorNoise = noiseModel::Diagonal::Variances(Vector6);
        _odometryNoise = noiseModel::Diagonal::Variances(Vector6);

        _matA0 = cv::Mat (5, 3, CV_32F, cv::Scalar::all(0));
        _matB0 = cv::Mat (5, 1, CV_32F, cv::Scalar::all(-1));
        _matX0 = cv::Mat (3, 1, CV_32F, cv::Scalar::all(0));

        _matA1 = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));
        _matD1 = cv::Mat (1, 3, CV_32F, cv::Scalar::all(0));
        _matV1 = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));

        _isDegenerate = false;
        _matP = cv::Mat (6, 6, CV_32F, cv::Scalar::all(0));

        _laserCloudCornerFromMapDSNum = 0;
        _laserCloudSurfFromMapDSNum = 0;
        _laserCloudCornerLastDSNum = 0;
        _laserCloudSurfLastDSNum = 0;
        _laserCloudOutlierLastDSNum = 0;
        _laserCloudSurfTotalLastDSNum = 0;

        _potentialLoopFlag = false;
        _aLoopIsClosed = false;

        _latestFrameID = 0;
    }

    void transformAssociateToMap()
    {
        // rotate with z axis.
        float y1 = cos(_transformSum[1]) * (_transformBefMapped[4] - _transformSum[4])
                 - sin(_transformSum[1]) * (_transformBefMapped[3] - _transformSum[3]);
        float z1 = _transformBefMapped[5] - _transformSum[5];
        float x1 = sin(_transformSum[1]) * (_transformBefMapped[4] - _transformSum[4])
                 + cos(_transformSum[1]) * (_transformBefMapped[3] - _transformSum[3]);

        // float x1 = cos(_transformSum[1]) * (_transformBefMapped[3] - _transformSum[3])
        //          - sin(_transformSum[1]) * (_transformBefMapped[5] - _transformSum[5]);
        // float y1 = _transformBefMapped[4] - _transformSum[4];
        // float z1 = sin(_transformSum[1]) * (_transformBefMapped[3] - _transformSum[3])
        //          + cos(_transformSum[1]) * (_transformBefMapped[5] - _transformSum[5]);

        // rotate with y axis.
        float y2 = y1;
        float z2 = cos(_transformSum[0]) * z1 + sin(_transformSum[0]) * x1;
        float x2 = -sin(_transformSum[0]) * z1 + cos(_transformSum[0]) * x1;

        // float x2 = x1;
        // float y2 = cos(_transformSum[0]) * y1 + sin(_transformSum[0]) * z1;
        // float z2 = -sin(_transformSum[0]) * y1 + cos(_transformSum[0]) * z1;

        // rotate with x axis.
        _transformIncre[4] = cos(_transformSum[2]) * y2 + sin(_transformSum[2]) * z2;
        _transformIncre[5] = -sin(_transformSum[2]) * y2 + cos(_transformSum[2]) * z2;
        _transformIncre[3] = x2;

        // float sbcy = sin(_transformSum[0]);
        // float cbcy = cos(_transformSum[0]);
        // float sbcz = sin(_transformSum[1]);
        // float cbcz = cos(_transformSum[1]);
        // float sbcx = sin(_transformSum[2]);
        // float cbcx = cos(_transformSum[2]);

        float sbcx = sin(_transformSum[0]);
        float cbcx = cos(_transformSum[0]);
        float sbcy = sin(_transformSum[1]);
        float cbcy = cos(_transformSum[1]);
        float sbcz = sin(_transformSum[2]);
        float cbcz = cos(_transformSum[2]);

        // float sbly = sin(_transformBefMapped[0]);
        // float cbly = cos(_transformBefMapped[0]);
        // float sblz = sin(_transformBefMapped[1]);
        // float cblz = cos(_transformBefMapped[1]);
        // float sblx = sin(_transformBefMapped[2]);
        // float cblx = cos(_transformBefMapped[2]);

        float sblx = sin(_transformBefMapped[0]);
        float cblx = cos(_transformBefMapped[0]);
        float sbly = sin(_transformBefMapped[1]);
        float cbly = cos(_transformBefMapped[1]);
        float sblz = sin(_transformBefMapped[2]);
        float cblz = cos(_transformBefMapped[2]);

        // float saly = sin(_transformAftMapped[0]);
        // float caly = cos(_transformAftMapped[0]);
        // float salz = sin(_transformAftMapped[1]);
        // float calz = cos(_transformAftMapped[1]);
        // float salx = sin(_transformAftMapped[2]);
        // float calx = cos(_transformAftMapped[2]);

        float salx = sin(_transformAftMapped[0]);
        float calx = cos(_transformAftMapped[0]);
        float saly = sin(_transformAftMapped[1]);
        float caly = cos(_transformAftMapped[1]);
        float salz = sin(_transformAftMapped[2]);
        float calz = cos(_transformAftMapped[2]);

        float sry = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
                  - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                  - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                  - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                  - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
        _transformTobeMapped[0] = -asin(sry);

        // float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
        //           - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
        //           - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
        //           - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
        //           - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
        // _transformTobeMapped[0] = -asin(srx);

        float srzcry = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                     - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
                     - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                     + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
                     + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                     + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);

        // float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
        //              - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
        //              - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
        //              + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
        //              + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
        //              + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);

        float crzcry = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                     - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
                     + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                     + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
                     - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                     + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);

        // float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
        //              - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
        //              + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
        //              + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
        //              - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
        //              + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
        _transformTobeMapped[1] = atan2(srzcry / cos(_transformTobeMapped[0]),
                                       crzcry / cos(_transformTobeMapped[0]));

        // _transformTobeMapped[1] = atan2(srycrx / cos(_transformTobeMapped[0]),
        //                                crycrx / cos(_transformTobeMapped[0]));

        float srxcry = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                     - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                     - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                     - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                     + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);

        // float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
        //              - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
        //              - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
        //              - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
        //              + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);

        float crxcry = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                     - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                     - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                     - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                     + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);

        // float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
        //              - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
        //              - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
        //              - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
        //              + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);

        _transformTobeMapped[2] = atan2(srxcry / cos(_transformTobeMapped[0]),
                                       crxcry / cos(_transformTobeMapped[0]));

        // _transformTobeMapped[2] = atan2(srzcrx / cos(_transformTobeMapped[0]),
        //                                crzcrx / cos(_transformTobeMapped[0]));

        // rotate with x axis.
        y1 = cos(_transformTobeMapped[2]) * _transformIncre[4] - sin(_transformTobeMapped[2]) * _transformIncre[5];
        z1 = sin(_transformTobeMapped[2]) * _transformIncre[4] + cos(_transformTobeMapped[2]) * _transformIncre[5];
        x1 = _transformIncre[3];

        // x1 = cos(_transformTobeMapped[2]) * _transformIncre[3] - sin(_transformTobeMapped[2]) * _transformIncre[4];
        // y1 = sin(_transformTobeMapped[2]) * _transformIncre[3] + cos(_transformTobeMapped[2]) * _transformIncre[4];
        // z1 = _transformIncre[5];

        // rotate with y axis.
        y2 = y1;
        z2 = cos(_transformTobeMapped[0]) * z1 - sin(_transformTobeMapped[0]) * x1;
        x2 = sin(_transformTobeMapped[0]) * z1 + cos(_transformTobeMapped[0]) * x1;

        // x2 = x1;
        // y2 = cos(_transformTobeMapped[0]) * y1 - sin(_transformTobeMapped[0]) * z1;
        // z2 = sin(_transformTobeMapped[0]) * y1 + cos(_transformTobeMapped[0]) * z1;

        // rotate with z axis.
        _transformTobeMapped[4] = _transformAftMapped[4]
                               - (cos(_transformTobeMapped[1]) * y2 + sin(_transformTobeMapped[1]) * x2);
        _transformTobeMapped[5] = _transformAftMapped[5] - z2;
        _transformTobeMapped[3] = _transformAftMapped[3]
                               - (-sin(_transformTobeMapped[1]) * y2 + cos(_transformTobeMapped[1]) * x2);

        // _transformTobeMapped[3] = _transformAftMapped[3]
        //                        - (cos(_transformTobeMapped[1]) * x2 + sin(_transformTobeMapped[1]) * z2);
        // _transformTobeMapped[4] = _transformAftMapped[4] - y2;
        // _transformTobeMapped[5] = _transformAftMapped[5]
        //                        - (-sin(_transformTobeMapped[1]) * x2 + cos(_transformTobeMapped[1]) * z2);
    }

    void transformUpdate()
    {
		if (_imuPointerLast >= 0) {
		    float imuRollLast = 0, imuPitchLast = 0;
		    while (_imuPointerFront != _imuPointerLast) {
		        if (_timeLaserOdometry + _scanPeriod < _imuTime[_imuPointerFront]) {
		            break;
		        }
		        _imuPointerFront = (_imuPointerFront + 1) % _imuQueLength;
		    }

		    if (_timeLaserOdometry + _scanPeriod > _imuTime[_imuPointerFront]) {
		        imuRollLast = _imuRoll[_imuPointerFront];
		        imuPitchLast = _imuPitch[_imuPointerFront];
		    } else {
		        int imuPointerBack = (_imuPointerFront + _imuQueLength - 1) % _imuQueLength;
		        float ratioFront = (_timeLaserOdometry + _scanPeriod - _imuTime[imuPointerBack])
		                         / (_imuTime[_imuPointerFront] - _imuTime[imuPointerBack]);
		        float ratioBack = (_imuTime[_imuPointerFront] - _timeLaserOdometry - _scanPeriod)
		                        / (_imuTime[_imuPointerFront] - _imuTime[imuPointerBack]);

		        imuRollLast = _imuRoll[_imuPointerFront] * ratioFront + _imuRoll[imuPointerBack] * ratioBack;
		        imuPitchLast = _imuPitch[_imuPointerFront] * ratioFront + _imuPitch[imuPointerBack] * ratioBack;
		    }

		    _transformTobeMapped[0] = 0.998 * _transformTobeMapped[0] + 0.002 * imuPitchLast;
		    _transformTobeMapped[2] = 0.998 * _transformTobeMapped[2] + 0.002 * imuRollLast;
		  }

		for (int i = 0; i < 6; i++) {
		    _transformBefMapped[i] = _transformSum[i];
		    _transformAftMapped[i] = _transformTobeMapped[i];
		}
    }

    void updatePointAssociateToMapSinCos(){
        _cPitch = cos(_transformTobeMapped[0]);
        _sPitch = sin(_transformTobeMapped[0]);

        _cYaw = cos(_transformTobeMapped[1]);
        _sYaw = sin(_transformTobeMapped[1]);

        _cRoll = cos(_transformTobeMapped[2]);
        _sRoll = sin(_transformTobeMapped[2]);

        // _cRoll = cos(_transformTobeMapped[0]);
        // _sRoll = sin(_transformTobeMapped[0]);
        //
        // _cPitch = cos(_transformTobeMapped[1]);
        // _sPitch = sin(_transformTobeMapped[1]);
        //
        // _cYaw = cos(_transformTobeMapped[2]);
        // _sYaw = sin(_transformTobeMapped[2]);

        _tX = _transformTobeMapped[3];
        _tY = _transformTobeMapped[4];
        _tZ = _transformTobeMapped[5];
    }

    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        // rotate with x axis.
        float y1 = _cRoll * pi->y - _sRoll * pi->z;
        float z1 = _sRoll * pi->y + _cRoll * pi->z;
        float x1 = pi->x;

        // float x1 = _cYaw * pi->x - _sYaw * pi->y;
        // float y1 = _sYaw * pi->x + _cYaw * pi->y;
        // float z1 = pi->z;

        // rotate with y axis.
        float y2 = y1;
        float z2 = _cPitch * z1 - _sPitch * x1;
        float x2 = _sPitch * z1 + _cPitch * x1;

        // float x2 = x1;
        // float y2 = _cRoll * y1 - _sRoll * z1;
        // float z2 = _sRoll * y1 + _cRoll * z1;

        // rotate with z axis.
        po->y = _cYaw * y2 + _sYaw * x2 + _tY;
        po->z = z2 + _tZ;
        po->x = -_sYaw * y2 + _cYaw * x2 + _tX;

        // po->x = _cPitch * x2 + _sPitch * z2 + _tX;
        // po->y = y2 + _tY;
        // po->z = -_sPitch * x2 + _cPitch * z2 + _tZ;
        po->intensity = pi->intensity;
    }

    void updateTransformPointCloudSinCos(PointTypePose *tIn){

        _ctRoll = cos(tIn->roll);
        _stRoll = sin(tIn->roll);

        _ctPitch = cos(tIn->pitch);
        _stPitch = sin(tIn->pitch);

        _ctYaw = cos(tIn->yaw);
        _stYaw = sin(tIn->yaw);

        _tInX = tIn->x;
        _tInY = tIn->y;
        _tInZ = tIn->z;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn){
	// !!! DO NOT use pcl for point cloud transformation, results are not accurate
        // Reason: unkown
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;
        PointType pointTo;

        int cloudSize = cloudIn->points.size();
        cloudOut->resize(cloudSize);

        for (int i = 0; i < cloudSize; ++i){

            pointFrom = &cloudIn->points[i];
            // rotate with x axis.
            float y1 = _ctRoll * pointFrom->y - _stRoll * pointFrom->z;
            float z1 = _stRoll * pointFrom->y + _ctRoll * pointFrom->z;
            float x1 = pointFrom->x;
            // rotate with y axis.
            float y2 = y1;
            float z2 = _ctPitch * z1 - _stPitch * x1;
            float x2 = _stPitch * z1 + _ctPitch * x1;
            // rotate with x axis.
            pointTo.y = _ctYaw * y2 + _stYaw * x2 + _tInY;
            pointTo.z = z2 + _tInZ;
            pointTo.x = -_stYaw * y2 + _ctYaw * x2 + _tInX;

            // float x1 = _ctYaw * pointFrom->x - _stYaw * pointFrom->y;
            // float y1 = _stYaw * pointFrom->x + _ctYaw* pointFrom->y;
            // float z1 = pointFrom->z;
            //
            // float x2 = x1;
            // float y2 = _ctRoll * y1 - _stRoll * z1;
            // float z2 = _stRoll * y1 + _ctRoll* z1;
            //
            // pointTo.x = _ctPitch * x2 + _stPitch * z2 + _tInX;
            // pointTo.y = y2 + _tInY;
            // pointTo.z = -_stPitch * x2 + _ctPitch * z2 + _tInZ;

            pointTo.intensity = pointFrom->intensity;

            cloudOut->points[i] = pointTo;
        }
        return cloudOut;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn){

        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;
        PointType pointTo;

        int cloudSize = cloudIn->points.size();
        cloudOut->resize(cloudSize);

        for (int i = 0; i < cloudSize; ++i){

            pointFrom = &cloudIn->points[i];
            // rotate with x axis.
            float y1 = cos(transformIn->roll) * pointFrom->y - sin(transformIn->roll) * pointFrom->z;
            float z1 = sin(transformIn->roll) * pointFrom->y + cos(transformIn->roll)* pointFrom->z;
            float x1 = pointFrom->x;
            // rotate with y axis.
            float y2 = y1;
            float z2 = cos(transformIn->pitch) * z1 - sin(transformIn->pitch) * x1;
            float x2 = sin(transformIn->pitch) * z1 + cos(transformIn->pitch) * x1;
            // rotate with z axis.
            pointTo.y = cos(transformIn->yaw) * y2 + sin(transformIn->yaw) * x2 + transformIn->y;
            pointTo.z = z2 + transformIn->z;
            pointTo.x = -sin(transformIn->yaw) * y2 + cos(transformIn->yaw) * x2 + transformIn->x;

            // float x1 = cos(transformIn->yaw) * pointFrom->x - sin(transformIn->yaw) * pointFrom->y;
            // float y1 = sin(transformIn->yaw) * pointFrom->x + cos(transformIn->yaw)* pointFrom->y;
            // float z1 = pointFrom->z;
            //
            // float x2 = x1;
            // float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
            // float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll)* z1;
            //
            // pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 + transformIn->x;
            // pointTo.y = y2 + transformIn->y;
            // pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 + transformIn->z;
            pointTo.intensity = pointFrom->intensity;

            cloudOut->points[i] = pointTo;
        }
        return cloudOut;
    }

    void laserCloudOutlierLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        _timeLaserCloudOutlierLast = msg->header.stamp.toSec();
        _laserCloudOutlierLast->clear();
        pcl::fromROSMsg(*msg, *_laserCloudOutlierLast);
        _newLaserCloudOutlierLast = true;
    }

    void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        _timeLaserCloudCornerLast = msg->header.stamp.toSec();
        _laserCloudCornerLast->clear();
        pcl::fromROSMsg(*msg, *_laserCloudCornerLast);
        _newLaserCloudCornerLast = true;
    }

    void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        _timeLaserCloudSurfLast = msg->header.stamp.toSec();
        _laserCloudSurfLast->clear();
        pcl::fromROSMsg(*msg, *_laserCloudSurfLast);
        _newLaserCloudSurfLast = true;
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry){
        _timeLaserOdometry = laserOdometry->header.stamp.toSec();
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
        // tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

        tf::Matrix3x3(tf::Quaternion(geoQuat.x, -geoQuat.y, -geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
        _transformSum[0] = -pitch;
        _transformSum[1] = -yaw;
        _transformSum[2] = roll;
        _transformSum[3] = laserOdometry->pose.pose.position.x;
        _transformSum[4] = laserOdometry->pose.pose.position.y;
        _transformSum[5] = laserOdometry->pose.pose.position.z;
        _newLaserOdometry = true;
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn){
        double roll, pitch, yaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imuIn->orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        _imuPointerLast = (_imuPointerLast + 1) % _imuQueLength;
        _imuTime[_imuPointerLast] = imuIn->header.stamp.toSec();
        _imuRoll[_imuPointerLast] = roll;
        _imuPitch[_imuPointerLast] = pitch;
    }

    void publishTF(){

        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                                  (_transformAftMapped[2], -_transformAftMapped[0], -_transformAftMapped[1]);

        _odomAftMapped.header.stamp = ros::Time().fromSec(_timeLaserOdometry);
        // _odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
        // _odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
        // _odomAftMapped.pose.pose.orientation.z = geoQuat.x;

        _odomAftMapped.pose.pose.orientation.x = geoQuat.x;
        _odomAftMapped.pose.pose.orientation.y = -geoQuat.y;
        _odomAftMapped.pose.pose.orientation.z = -geoQuat.z;
        _odomAftMapped.pose.pose.orientation.w = geoQuat.w;
        _odomAftMapped.pose.pose.position.x = _transformAftMapped[3];
        _odomAftMapped.pose.pose.position.y = _transformAftMapped[4];
        _odomAftMapped.pose.pose.position.z = _transformAftMapped[5];
        // _odomAftMapped.twist.twist.angular.x = _transformBefMapped[0];
        // _odomAftMapped.twist.twist.angular.y = _transformBefMapped[1];
        // _odomAftMapped.twist.twist.angular.z = _transformBefMapped[2];

        _odomAftMapped.twist.twist.angular.x = _transformBefMapped[2];
        _odomAftMapped.twist.twist.angular.y = _transformBefMapped[0];
        _odomAftMapped.twist.twist.angular.z = _transformBefMapped[1];
        _odomAftMapped.twist.twist.linear.x = _transformBefMapped[3];
        _odomAftMapped.twist.twist.linear.y = _transformBefMapped[4];
        _odomAftMapped.twist.twist.linear.z = _transformBefMapped[5];
        _pubOdomAftMapped.publish(_odomAftMapped);

        // _aftMappedTrans.stamp_ = ros::Time().fromSec(_timeLaserOdometry);
        // // _aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        //
        // _aftMappedTrans.setRotation(tf::Quaternion(geoQuat.x, -geoQuat.y, -geoQuat.z, geoQuat.w));
        // _aftMappedTrans.setOrigin(tf::Vector3(_transformAftMapped[3], _transformAftMapped[4], _transformAftMapped[5]));
        // _tfBroadcaster.sendTransform(_aftMappedTrans);
    }

    void publishKeyPosesAndFrames(){

        if (_pubKeyPoses.getNumSubscribers() != 0){
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*_cloudKeyPoses3D, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(_timeLaserOdometry);
            // cloudMsgTemp.header.frame_id = "/camera_init";
            cloudMsgTemp.header.frame_id = _mapFrameId;
            _pubKeyPoses.publish(cloudMsgTemp);
        }

        if (_pubRecentKeyFrames.getNumSubscribers() != 0){
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*_laserCloudSurfFromMapDS, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(_timeLaserOdometry);
            // cloudMsgTemp.header.frame_id = "/camera_init";
            cloudMsgTemp.header.frame_id = _mapFrameId;
            _pubRecentKeyFrames.publish(cloudMsgTemp);
        }
    }

    void visualizeGlobalMapThread(){
        ros::Rate rate(0.2);
        while (ros::ok()){
            rate.sleep();
            publishGlobalMap();
        }
        // save final point cloud
        pcl::io::savePCDFileASCII(_fileDirectory+"finalCloud.pcd", *_globalMapKeyFramesDS);

        string cornerMapString = "/tmp/cornerMap.pcd";
        string surfaceMapString = "/tmp/surfaceMap.pcd";
        string trajectoryString = "/tmp/trajectory.pcd";

        pcl::PointCloud<PointType>::Ptr cornerMapCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr cornerMapCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceMapCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceMapCloudDS(new pcl::PointCloud<PointType>());

        for(int i = 0; i < _cornerCloudKeyFrames.size(); i++) {
            *cornerMapCloud  += *transformPointCloud(_cornerCloudKeyFrames[i],   &_cloudKeyPoses6D->points[i]);
    	    *surfaceMapCloud += *transformPointCloud(_surfCloudKeyFrames[i],     &_cloudKeyPoses6D->points[i]);
    	    *surfaceMapCloud += *transformPointCloud(_outlierCloudKeyFrames[i],  &_cloudKeyPoses6D->points[i]);
        }

        _downSizeFilterCorner.setInputCloud(cornerMapCloud);
        _downSizeFilterCorner.filter(*cornerMapCloudDS);
        _downSizeFilterSurf.setInputCloud(surfaceMapCloud);
        _downSizeFilterSurf.filter(*surfaceMapCloudDS);

        pcl::io::savePCDFileASCII(_fileDirectory+"cornerMap.pcd", *cornerMapCloudDS);
        pcl::io::savePCDFileASCII(_fileDirectory+"surfaceMap.pcd", *surfaceMapCloudDS);
        pcl::io::savePCDFileASCII(_fileDirectory+"trajectory.pcd", *_cloudKeyPoses3D);
    }

    void publishGlobalMap(){

        if (_pubLaserCloudSurround.getNumSubscribers() == 0)
            return;

        if (_cloudKeyPoses3D->points.empty() == true)
            return;
	    // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
	    // search near key frames to visualize
        _mtx.lock();
        _kdtreeGlobalMap->setInputCloud(_cloudKeyPoses3D);
        _kdtreeGlobalMap->radiusSearch(_currentRobotPosPoint, _globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        _mtx.unlock();

        for (int i = 0; i < pointSearchIndGlobalMap.size(); ++i)
          _globalMapKeyPoses->points.push_back(_cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
	    // downsample near selected key frames
        _downSizeFilterGlobalMapKeyPoses.setInputCloud(_globalMapKeyPoses);
        _downSizeFilterGlobalMapKeyPoses.filter(*_globalMapKeyPosesDS);
	    // extract visualized and downsampled key frames
        for (int i = 0; i < _globalMapKeyPosesDS->points.size(); ++i){
			int thisKeyInd = (int)_globalMapKeyPosesDS->points[i].intensity;
			*_globalMapKeyFrames += *transformPointCloud(_cornerCloudKeyFrames[thisKeyInd],   &_cloudKeyPoses6D->points[thisKeyInd]);
			*_globalMapKeyFrames += *transformPointCloud(_surfCloudKeyFrames[thisKeyInd],    &_cloudKeyPoses6D->points[thisKeyInd]);
			*_globalMapKeyFrames += *transformPointCloud(_outlierCloudKeyFrames[thisKeyInd], &_cloudKeyPoses6D->points[thisKeyInd]);
        }
	    // downsample visualized points
        _downSizeFilterGlobalMapKeyFrames.setInputCloud(_globalMapKeyFrames);
        _downSizeFilterGlobalMapKeyFrames.filter(*_globalMapKeyFramesDS);

        sensor_msgs::PointCloud2 cloudMsgTemp;
        // pcl::toROSMsg(*_globalMapKeyFramesDS, cloudMsgTemp);
        pcl::toROSMsg(*_globalMapKeyFrames, cloudMsgTemp); // publish full size map.
        cloudMsgTemp.header.stamp = ros::Time().fromSec(_timeLaserOdometry);
        // cloudMsgTemp.header.frame_id = "/camera_init";
        cloudMsgTemp.header.frame_id = _mapFrameId;
        _pubLaserCloudSurround.publish(cloudMsgTemp);

        _globalMapKeyPoses->clear();
        _globalMapKeyPosesDS->clear();
        _globalMapKeyFrames->clear();
        // _globalMapKeyFramesDS->clear();
    }

    void loopClosureThread(){

        if (_loopClosureEnableFlag == false)
            return;

        ros::Rate rate(1);
        while (ros::ok()){
            rate.sleep();
            performLoopClosure();
        }
    }

    bool detectLoopClosure(){

        _latestSurfKeyFrameCloud->clear();
        _nearHistorySurfKeyFrameCloud->clear();
        _nearHistorySurfKeyFrameCloudDS->clear();

        std::lock_guard<std::mutex> lock(_mtx);
        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        _kdtreeHistoryKeyPoses->setInputCloud(_cloudKeyPoses3D);
        _kdtreeHistoryKeyPoses->radiusSearch(_currentRobotPosPoint, _historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

        _closestHistoryFrameID = -1;
        for (int i = 0; i < pointSearchIndLoop.size(); ++i){
            int id = pointSearchIndLoop[i];
            if (abs(_cloudKeyPoses6D->points[id].time - _timeLaserOdometry) > 30.0){
                _closestHistoryFrameID = id;
                break;
            }
        }
        if (_closestHistoryFrameID == -1){
            return false;
        }
        // save latest key frames
        _latestFrameIDLoopCloure = _cloudKeyPoses3D->points.size() - 1;
        *_latestSurfKeyFrameCloud += *transformPointCloud(_cornerCloudKeyFrames[_latestFrameIDLoopCloure], &_cloudKeyPoses6D->points[_latestFrameIDLoopCloure]);
        *_latestSurfKeyFrameCloud += *transformPointCloud(_surfCloudKeyFrames[_latestFrameIDLoopCloure],   &_cloudKeyPoses6D->points[_latestFrameIDLoopCloure]);

        pcl::PointCloud<PointType>::Ptr hahaCloud(new pcl::PointCloud<PointType>());
        int cloudSize = _latestSurfKeyFrameCloud->points.size();
        for (int i = 0; i < cloudSize; ++i){
            if ((int)_latestSurfKeyFrameCloud->points[i].intensity >= 0){
                hahaCloud->push_back(_latestSurfKeyFrameCloud->points[i]);
            }
        }
        _latestSurfKeyFrameCloud->clear();
        *_latestSurfKeyFrameCloud = *hahaCloud;
	   // save history near key frames
        for (int j = -_historyKeyframeSearchNum; j <= _historyKeyframeSearchNum; ++j){
            if (_closestHistoryFrameID + j < 0 || _closestHistoryFrameID + j > _latestFrameIDLoopCloure)
                continue;
            *_nearHistorySurfKeyFrameCloud += *transformPointCloud(_cornerCloudKeyFrames[_closestHistoryFrameID+j], &_cloudKeyPoses6D->points[_closestHistoryFrameID+j]);
            *_nearHistorySurfKeyFrameCloud += *transformPointCloud(_surfCloudKeyFrames[_closestHistoryFrameID+j],   &_cloudKeyPoses6D->points[_closestHistoryFrameID+j]);
        }

        _downSizeFilterHistoryKeyFrames.setInputCloud(_nearHistorySurfKeyFrameCloud);
        _downSizeFilterHistoryKeyFrames.filter(*_nearHistorySurfKeyFrameCloudDS);
        // publish history near key frames
        if (_pubHistoryKeyFrames.getNumSubscribers() != 0){
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*_nearHistorySurfKeyFrameCloudDS, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(_timeLaserOdometry);
            // cloudMsgTemp.header.frame_id = "/camera_init";
            cloudMsgTemp.header.frame_id = _mapFrameId;
            _pubHistoryKeyFrames.publish(cloudMsgTemp);
        }

        return true;
    }


    void performLoopClosure(){

        if (_cloudKeyPoses3D->points.empty() == true)
            return;
        // try to find close key frame if there are any
        if (_potentialLoopFlag == false){

            if (detectLoopClosure() == true){
                _potentialLoopFlag = true; // find some key frames that is old enough or close enough for loop closure
                _timeSaveFirstCurrentScanForLoopClosure = _timeLaserOdometry;
            }
            if (_potentialLoopFlag == false)
                return;
        }
        // reset the flag first no matter icp successes or not
        _potentialLoopFlag = false;
        // GICP Settings.
        pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
        gicp.setMaxCorrespondenceDistance(100);
        gicp.setMaximumIterations(100);
        gicp.setTransformationEpsilon(1e-6);
        gicp.setEuclideanFitnessEpsilon(1e-6);
        gicp.setRANSACIterations(0);
        // Align clouds
        gicp.setInputSource(_latestSurfKeyFrameCloud);
        gicp.setInputTarget(_nearHistorySurfKeyFrameCloudDS);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        gicp.align(*unused_result);
        // // ICP Settings
        // pcl::IterativeClosestPoint<PointType, PointType> icp;
        // icp.setMaxCorrespondenceDistance(100);
        // icp.setMaximumIterations(100);
        // icp.setTransformationEpsilon(1e-6);
        // icp.setEuclideanFitnessEpsilon(1e-6);
        // icp.setRANSACIterations(0);
        // // Align clouds
        // icp.setInputSource(_latestSurfKeyFrameCloud);
        // icp.setInputTarget(_nearHistorySurfKeyFrameCloudDS);
        // pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        // icp.align(*unused_result);

        // if (icp.hasConverged() == false || icp.getFitnessScore() > _historyKeyframeFitnessScore)
        if (gicp.hasConverged() == false || gicp.getFitnessScore() > _historyKeyframeFitnessScore)
            return;
        // publish corrected cloud
        if (_pubIcpKeyFrames.getNumSubscribers() != 0){
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            // pcl::transformPointCloud (*_latestSurfKeyFrameCloud, *closed_cloud, icp.getFinalTransformation());
            pcl::transformPointCloud (*_latestSurfKeyFrameCloud, *closed_cloud, gicp.getFinalTransformation());
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*closed_cloud, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(_timeLaserOdometry);
            cloudMsgTemp.header.frame_id = _mapFrameId;
            // cloudMsgTemp.header.frame_id = "/camera_init";
            _pubIcpKeyFrames.publish(cloudMsgTemp);
        }
        /*
        	get pose constraint
        	*/
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionCameraFrame;
        // correctionCameraFrame = icp.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)
        correctionCameraFrame = gicp.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)
        // pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll, pitch, yaw);

        // Eigen::Affine3f correctionLidarFrame = pcl::getTransformation(z, x, y, yaw, roll, pitch);

        Eigen::Affine3f correctionLidarFrame = correctionCameraFrame;
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3fCameraToLidar(_cloudKeyPoses6D->points[_latestFrameIDLoopCloure]);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(_cloudKeyPoses6D->points[_closestHistoryFrameID]);
        gtsam::Vector Vector6(6);
        // float noiseScore = icp.getFitnessScore();
        float noiseScore = gicp.getFitnessScore();
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        _constraintNoise = noiseModel::Diagonal::Variances(Vector6);
        /*
        	add constraints
        	*/
        std::lock_guard<std::mutex> lock(_mtx);
        _gtSAMgraph.add(BetweenFactor<Pose3>(_latestFrameIDLoopCloure, _closestHistoryFrameID, poseFrom.between(poseTo), _constraintNoise));
        _isam->update(_gtSAMgraph);
        _isam->update();
        _gtSAMgraph.resize(0);

        _aLoopIsClosed = true;
    }

    Pose3 pclPointTogtsamPose3(PointTypePose thisPoint){ // camera frame to lidar frame
    	// return Pose3(Rot3::RzRyRx(double(thisPoint.yaw), double(thisPoint.roll), double(thisPoint.pitch)),
                           // Point3(double(thisPoint.z),   double(thisPoint.x),    double(thisPoint.y)));

      return Pose3(Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                           Point3(double(thisPoint.x),   double(thisPoint.y),    double(thisPoint.z)));
    }

    Eigen::Affine3f pclPointToAffine3fCameraToLidar(PointTypePose thisPoint){ // camera frame to lidar frame
    	// return pcl::getTransformation(thisPoint.z, thisPoint.x, thisPoint.y, thisPoint.yaw, thisPoint.roll, thisPoint.pitch);

      return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    void extractSurroundingKeyFrames(){

        if (_cloudKeyPoses3D->points.empty() == true)
            return;

    	if (_loopClosureEnableFlag == true){
    	    // only use recent key poses for graph building
                if (_recentCornerCloudKeyFrames.size() < _surroundingKeyframeSearchNum){ // queue is not full (the beginning of mapping or a loop is just closed)
                    // clear recent key frames queue
                    _recentCornerCloudKeyFrames. clear();
                    _recentSurfCloudKeyFrames.   clear();
                    _recentOutlierCloudKeyFrames.clear();
                    int numPoses = _cloudKeyPoses3D->points.size();
                    for (int i = numPoses-1; i >= 0; --i){
                        int thisKeyInd = (int)_cloudKeyPoses3D->points[i].intensity;
                        PointTypePose thisTransformation = _cloudKeyPoses6D->points[thisKeyInd];
                        updateTransformPointCloudSinCos(&thisTransformation);
                        // extract surrounding map
                        _recentCornerCloudKeyFrames. push_front(transformPointCloud(_cornerCloudKeyFrames[thisKeyInd]));
                        _recentSurfCloudKeyFrames.   push_front(transformPointCloud(_surfCloudKeyFrames[thisKeyInd]));
                        _recentOutlierCloudKeyFrames.push_front(transformPointCloud(_outlierCloudKeyFrames[thisKeyInd]));
                        if (_recentCornerCloudKeyFrames.size() >= _surroundingKeyframeSearchNum)
                            break;
                    }
                }else{  // queue is full, pop the oldest key frame and push the latest key frame
                    if (_latestFrameID != _cloudKeyPoses3D->points.size() - 1){  // if the robot is not moving, no need to update recent frames

                        _recentCornerCloudKeyFrames. pop_front();
                        _recentSurfCloudKeyFrames.   pop_front();
                        _recentOutlierCloudKeyFrames.pop_front();
                        // push latest scan to the end of queue
                        _latestFrameID = _cloudKeyPoses3D->points.size() - 1;
                        PointTypePose thisTransformation = _cloudKeyPoses6D->points[_latestFrameID];
                        updateTransformPointCloudSinCos(&thisTransformation);
                        _recentCornerCloudKeyFrames. push_back(transformPointCloud(_cornerCloudKeyFrames[_latestFrameID]));
                        _recentSurfCloudKeyFrames.   push_back(transformPointCloud(_surfCloudKeyFrames[_latestFrameID]));
                        _recentOutlierCloudKeyFrames.push_back(transformPointCloud(_outlierCloudKeyFrames[_latestFrameID]));
                    }
                }

                for (int i = 0; i < _recentCornerCloudKeyFrames.size(); ++i){
                    *_laserCloudCornerFromMap += *_recentCornerCloudKeyFrames[i];
                    *_laserCloudSurfFromMap   += *_recentSurfCloudKeyFrames[i];
                    *_laserCloudSurfFromMap   += *_recentOutlierCloudKeyFrames[i];
                }
    	}else{
            _surroundingKeyPoses->clear();
            _surroundingKeyPosesDS->clear();
    	    // extract all the nearby key poses and downsample them
    	    _kdtreeSurroundingKeyPoses->setInputCloud(_cloudKeyPoses3D);
    	    _kdtreeSurroundingKeyPoses->radiusSearch(_currentRobotPosPoint, (double)_surroundingKeyframeSearchRadius, _pointSearchInd, _pointSearchSqDis, 0);
    	    for (int i = 0; i < _pointSearchInd.size(); ++i)
                _surroundingKeyPoses->points.push_back(_cloudKeyPoses3D->points[_pointSearchInd[i]]);
    	    _downSizeFilterSurroundingKeyPoses.setInputCloud(_surroundingKeyPoses);
    	    _downSizeFilterSurroundingKeyPoses.filter(*_surroundingKeyPosesDS);
    	    // delete key frames that are not in surrounding region
            int numSurroundingPosesDS = _surroundingKeyPosesDS->points.size();
            for (int i = 0; i < _surroundingExistingKeyPosesID.size(); ++i){
                bool existingFlag = false;
                for (int j = 0; j < numSurroundingPosesDS; ++j){
                    if (_surroundingExistingKeyPosesID[i] == (int)_surroundingKeyPosesDS->points[j].intensity){
                        existingFlag = true;
                        break;
                    }
                }
                if (existingFlag == false){
                    _surroundingExistingKeyPosesID.   erase(_surroundingExistingKeyPosesID.   begin() + i);
                    _surroundingCornerCloudKeyFrames. erase(_surroundingCornerCloudKeyFrames. begin() + i);
                    _surroundingSurfCloudKeyFrames.   erase(_surroundingSurfCloudKeyFrames.   begin() + i);
                    _surroundingOutlierCloudKeyFrames.erase(_surroundingOutlierCloudKeyFrames.begin() + i);
                    --i;
                }
            }
    	    // add new key frames that are not in calculated existing key frames
            for (int i = 0; i < numSurroundingPosesDS; ++i) {
                bool existingFlag = false;
                for (auto iter = _surroundingExistingKeyPosesID.begin(); iter != _surroundingExistingKeyPosesID.end(); ++iter){
                    if ((*iter) == (int)_surroundingKeyPosesDS->points[i].intensity){
                        existingFlag = true;
                        break;
                    }
                }
                if (existingFlag == true){
                    continue;
                }else{
                    int thisKeyInd = (int)_surroundingKeyPosesDS->points[i].intensity;
                    PointTypePose thisTransformation = _cloudKeyPoses6D->points[thisKeyInd];
                    updateTransformPointCloudSinCos(&thisTransformation);
                    _surroundingExistingKeyPosesID.   push_back(thisKeyInd);
                    _surroundingCornerCloudKeyFrames. push_back(transformPointCloud(_cornerCloudKeyFrames[thisKeyInd]));
                    _surroundingSurfCloudKeyFrames.   push_back(transformPointCloud(_surfCloudKeyFrames[thisKeyInd]));
                    _surroundingOutlierCloudKeyFrames.push_back(transformPointCloud(_outlierCloudKeyFrames[thisKeyInd]));
                }
            }

            for (int i = 0; i < _surroundingExistingKeyPosesID.size(); ++i) {
                *_laserCloudCornerFromMap += *_surroundingCornerCloudKeyFrames[i];
                *_laserCloudSurfFromMap   += *_surroundingSurfCloudKeyFrames[i];
                *_laserCloudSurfFromMap   += *_surroundingOutlierCloudKeyFrames[i];
            }
    	}
        // Downsample the surrounding corner key frames (or map)
        _downSizeFilterCorner.setInputCloud(_laserCloudCornerFromMap);
        _downSizeFilterCorner.filter(*_laserCloudCornerFromMapDS);
        _laserCloudCornerFromMapDSNum = _laserCloudCornerFromMapDS->points.size();
        // Downsample the surrounding surf key frames (or map)
        _downSizeFilterSurf.setInputCloud(_laserCloudSurfFromMap);
        _downSizeFilterSurf.filter(*_laserCloudSurfFromMapDS);
        _laserCloudSurfFromMapDSNum = _laserCloudSurfFromMapDS->points.size();
    }

    void downsampleCurrentScan(){

        _laserCloudCornerLastDS->clear();
        _downSizeFilterCorner.setInputCloud(_laserCloudCornerLast);
        _downSizeFilterCorner.filter(*_laserCloudCornerLastDS);
        _laserCloudCornerLastDSNum = _laserCloudCornerLastDS->points.size();

        _laserCloudSurfLastDS->clear();
        _downSizeFilterSurf.setInputCloud(_laserCloudSurfLast);
        _downSizeFilterSurf.filter(*_laserCloudSurfLastDS);
        _laserCloudSurfLastDSNum = _laserCloudSurfLastDS->points.size();

        _laserCloudOutlierLastDS->clear();
        _downSizeFilterOutlier.setInputCloud(_laserCloudOutlierLast);
        _downSizeFilterOutlier.filter(*_laserCloudOutlierLastDS);
        _laserCloudOutlierLastDSNum = _laserCloudOutlierLastDS->points.size();

        _laserCloudSurfTotalLast->clear();
        _laserCloudSurfTotalLastDS->clear();
        *_laserCloudSurfTotalLast += *_laserCloudSurfLastDS;
        *_laserCloudSurfTotalLast += *_laserCloudOutlierLastDS;
        _downSizeFilterSurf.setInputCloud(_laserCloudSurfTotalLast);
        _downSizeFilterSurf.filter(*_laserCloudSurfTotalLastDS);
        _laserCloudSurfTotalLastDSNum = _laserCloudSurfTotalLastDS->points.size();
    }

    void cornerOptimization(int iterCount){

        updatePointAssociateToMapSinCos();
        for (int i = 0; i < _laserCloudCornerLastDSNum; i++) {
            _pointOri = _laserCloudCornerLastDS->points[i];
            pointAssociateToMap(&_pointOri, &_pointSel);
            _kdtreeCornerFromMap->nearestKSearch(_pointSel, 5, _pointSearchInd, _pointSearchSqDis);

            if (_pointSearchSqDis[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += _laserCloudCornerFromMapDS->points[_pointSearchInd[j]].x;
                    cy += _laserCloudCornerFromMapDS->points[_pointSearchInd[j]].y;
                    cz += _laserCloudCornerFromMapDS->points[_pointSearchInd[j]].z;
                }
                cx /= 5; cy /= 5;  cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax = _laserCloudCornerFromMapDS->points[_pointSearchInd[j]].x - cx;
                    float ay = _laserCloudCornerFromMapDS->points[_pointSearchInd[j]].y - cy;
                    float az = _laserCloudCornerFromMapDS->points[_pointSearchInd[j]].z - cz;

                    a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                    a22 += ay * ay; a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

                _matA1.at<float>(0, 0) = a11; _matA1.at<float>(0, 1) = a12; _matA1.at<float>(0, 2) = a13;
                _matA1.at<float>(1, 0) = a12; _matA1.at<float>(1, 1) = a22; _matA1.at<float>(1, 2) = a23;
                _matA1.at<float>(2, 0) = a13; _matA1.at<float>(2, 1) = a23; _matA1.at<float>(2, 2) = a33;

                cv::eigen(_matA1, _matD1, _matV1);

                if (_matD1.at<float>(0, 0) > 3 * _matD1.at<float>(0, 1)) {

                    float x0 = _pointSel.x;
                    float y0 = _pointSel.y;
                    float z0 = _pointSel.z;
                    float x1 = cx + 0.1 * _matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * _matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * _matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * _matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * _matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * _matV1.at<float>(0, 2);

                    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                    * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                    * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                                    * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                              + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                               - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                               + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    _coeff.x = s * la;
                    _coeff.y = s * lb;
                    _coeff.z = s * lc;
                    _coeff.intensity = s * ld2;

                    if (s > 0.1) {
                        _laserCloudOri->push_back(_pointOri);
                        _coeffSel->push_back(_coeff);
                    }
                }
            }
        }
    }

    void surfOptimization(int iterCount){
        updatePointAssociateToMapSinCos();
        for (int i = 0; i < _laserCloudSurfTotalLastDSNum; i++) {
            _pointOri = _laserCloudSurfTotalLastDS->points[i];
            pointAssociateToMap(&_pointOri, &_pointSel);
            _kdtreeSurfFromMap->nearestKSearch(_pointSel, 5, _pointSearchInd, _pointSearchSqDis);

            if (_pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    _matA0.at<float>(j, 0) = _laserCloudSurfFromMapDS->points[_pointSearchInd[j]].x;
                    _matA0.at<float>(j, 1) = _laserCloudSurfFromMapDS->points[_pointSearchInd[j]].y;
                    _matA0.at<float>(j, 2) = _laserCloudSurfFromMapDS->points[_pointSearchInd[j]].z;
                }
                cv::solve(_matA0, _matB0, _matX0, cv::DECOMP_QR);

                float pa = _matX0.at<float>(0, 0);
                float pb = _matX0.at<float>(1, 0);
                float pc = _matX0.at<float>(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * _laserCloudSurfFromMapDS->points[_pointSearchInd[j]].x +
                             pb * _laserCloudSurfFromMapDS->points[_pointSearchInd[j]].y +
                             pc * _laserCloudSurfFromMapDS->points[_pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * _pointSel.x + pb * _pointSel.y + pc * _pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(_pointSel.x * _pointSel.x
                            + _pointSel.y * _pointSel.y + _pointSel.z * _pointSel.z));

                    _coeff.x = s * pa;
                    _coeff.y = s * pb;
                    _coeff.z = s * pc;
                    _coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        _laserCloudOri->push_back(_pointOri);
                        _coeffSel->push_back(_coeff);
                    }
                }
            }
        }
    }

    bool LMOptimization(int iterCount){
        // float sry = sin(_transformTobeMapped[0]);
        // float cry = cos(_transformTobeMapped[0]);
        // float srz = sin(_transformTobeMapped[1]);
        // float crz = cos(_transformTobeMapped[1]);
        // float srx = sin(_transformTobeMapped[2]);
        // float crx = cos(_transformTobeMapped[2]);

        float srx = sin(_transformTobeMapped[0]);
        float crx = cos(_transformTobeMapped[0]);
        float sry = sin(_transformTobeMapped[1]);
        float cry = cos(_transformTobeMapped[1]);
        float srz = sin(_transformTobeMapped[2]);
        float crz = cos(_transformTobeMapped[2]);

        int laserCloudSelNum = _laserCloudOri->points.size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        for (int i = 0; i < laserCloudSelNum; i++) {
            _pointOri = _laserCloudOri->points[i];
            _coeff = _coeffSel->points[i];

            float ary = (crx*sry*srz*_pointOri.y + crx*crz*sry*_pointOri.z - srx*sry*_pointOri.x) * _coeff.y
                      + (-srx*srz*_pointOri.y - crz*srx*_pointOri.z - crx*_pointOri.x) * _coeff.z
                      + (crx*cry*srz*_pointOri.y + crx*cry*crz*_pointOri.z - cry*srx*_pointOri.x) * _coeff.x;

            float arz = ((cry*srx*srz - crz*sry)*_pointOri.y
                      + (sry*srz + cry*crz*srx)*_pointOri.z + crx*cry*_pointOri.x) * _coeff.y
                      + ((-cry*crz - srx*sry*srz)*_pointOri.y
                      + (cry*srz - crz*srx*sry)*_pointOri.z - crx*sry*_pointOri.x) * _coeff.x;

            float arx = ((crz*srx*sry - cry*srz)*_pointOri.y + (-cry*crz-srx*sry*srz)*_pointOri.z)*_coeff.y
                      + (crx*crz*_pointOri.y - crx*srz*_pointOri.z) * _coeff.z
                      + ((sry*srz + cry*crz*srx)*_pointOri.y + (crz*sry-cry*srx*srz)*_pointOri.z)*_coeff.x;

            // float arx = (crx*sry*srz*_pointOri.x + crx*crz*sry*_pointOri.y - srx*sry*_pointOri.z) * _coeff.x
            //           + (-srx*srz*_pointOri.x - crz*srx*_pointOri.y - crx*_pointOri.z) * _coeff.y
            //           + (crx*cry*srz*_pointOri.x + crx*cry*crz*_pointOri.y - cry*srx*_pointOri.z) * _coeff.z;
            //
            // float ary = ((cry*srx*srz - crz*sry)*_pointOri.x
            //           + (sry*srz + cry*crz*srx)*_pointOri.y + crx*cry*_pointOri.z) * _coeff.x
            //           + ((-cry*crz - srx*sry*srz)*_pointOri.x
            //           + (cry*srz - crz*srx*sry)*_pointOri.y - crx*sry*_pointOri.z) * _coeff.z;
            //
            // float arz = ((crz*srx*sry - cry*srz)*_pointOri.x + (-cry*crz-srx*sry*srz)*_pointOri.y)*_coeff.x
            //           + (crx*crz*_pointOri.x - crx*srz*_pointOri.y) * _coeff.y
            //           + ((sry*srz + cry*crz*srx)*_pointOri.x + (crz*sry-cry*srx*srz)*_pointOri.y)*_coeff.z;

            // matA.at<float>(i, 0) = arx;
            // matA.at<float>(i, 1) = ary;
            // matA.at<float>(i, 2) = arz;

            matA.at<float>(i, 0) = ary;
            matA.at<float>(i, 1) = arz;
            matA.at<float>(i, 2) = arx;
            matA.at<float>(i, 3) = _coeff.x;
            matA.at<float>(i, 4) = _coeff.y;
            matA.at<float>(i, 5) = _coeff.z;
            matB.at<float>(i, 0) = -_coeff.intensity;
        }
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            _isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    _isDegenerate = true;
                } else {
                    break;
                }
            }
            _matP = matV.inv() * matV2;
        }

        if (_isDegenerate) {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = _matP * matX2;
        }

        _transformTobeMapped[0] += matX.at<float>(0, 0);
        _transformTobeMapped[1] += matX.at<float>(1, 0);
        _transformTobeMapped[2] += matX.at<float>(2, 0);
        _transformTobeMapped[3] += matX.at<float>(3, 0);
        _transformTobeMapped[4] += matX.at<float>(4, 0);
        _transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true;
        }
        return false;
    }

    void scan2MapOptimization(){

        if (_laserCloudCornerFromMapDSNum > 10 && _laserCloudSurfFromMapDSNum > 100) {

            _kdtreeCornerFromMap->setInputCloud(_laserCloudCornerFromMapDS);
            _kdtreeSurfFromMap->setInputCloud(_laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < 10; iterCount++) {

                _laserCloudOri->clear();
                _coeffSel->clear();

                cornerOptimization(iterCount);
                surfOptimization(iterCount);

                if (LMOptimization(iterCount) == true)
                    break;
            }

            transformUpdate();
        }
    }


    void saveKeyFramesAndFactor(){

        _currentRobotPosPoint.x = _transformAftMapped[3];
        _currentRobotPosPoint.y = _transformAftMapped[4];
        _currentRobotPosPoint.z = _transformAftMapped[5];

        bool saveThisKeyFrame = true;
        if (sqrt((_previousRobotPosPoint.x-_currentRobotPosPoint.x)*(_previousRobotPosPoint.x-_currentRobotPosPoint.x)
                +(_previousRobotPosPoint.y-_currentRobotPosPoint.y)*(_previousRobotPosPoint.y-_currentRobotPosPoint.y)
                +(_previousRobotPosPoint.z-_currentRobotPosPoint.z)*(_previousRobotPosPoint.z-_currentRobotPosPoint.z)) < 0.3){
            saveThisKeyFrame = false;
        }



        if (saveThisKeyFrame == false && !_cloudKeyPoses3D->points.empty())
        	return;

        _previousRobotPosPoint = _currentRobotPosPoint;
        /**
         * update grsam graph
         */
        if (_cloudKeyPoses3D->points.empty()){
            // _gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(_transformTobeMapped[2], _transformTobeMapped[0], _transformTobeMapped[1]),
            //                                            		 Point3(_transformTobeMapped[5], _transformTobeMapped[3], _transformTobeMapped[4])), _priorNoise));
            _gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(_transformTobeMapped[2], _transformTobeMapped[0], _transformTobeMapped[1]),
                                                      		   Point3(_transformTobeMapped[3], _transformTobeMapped[4], _transformTobeMapped[5])), _priorNoise));

            // _initialEstimate.insert(0, Pose3(Rot3::RzRyRx(_transformTobeMapped[2], _transformTobeMapped[0], _transformTobeMapped[1]),
            //                                       Point3(_transformTobeMapped[5], _transformTobeMapped[3], _transformTobeMapped[4])));

            _initialEstimate.insert(0, Pose3(Rot3::RzRyRx(_transformTobeMapped[2], _transformTobeMapped[0], _transformTobeMapped[1]),
                                                  Point3(_transformTobeMapped[3], _transformTobeMapped[4], _transformTobeMapped[5])));
            for (int i = 0; i < 6; ++i)
            	_transformLast[i] = _transformTobeMapped[i];
        }
        else{
            // gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(_transformLast[2], _transformLast[0], _transformLast[1]),
            //                                     Point3(_transformLast[5], _transformLast[3], _transformLast[4]));

            gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(_transformLast[2], _transformLast[0], _transformLast[1]),
                                                Point3(_transformLast[3], _transformLast[4], _transformLast[5]));

            // gtsam::Pose3 poseTo   = Pose3(Rot3::RzRyRx(_transformAftMapped[2], _transformAftMapped[0], _transformAftMapped[1]),
            //                                     Point3(_transformAftMapped[5], _transformAftMapped[3], _transformAftMapped[4]));

            gtsam::Pose3 poseTo   = Pose3(Rot3::RzRyRx(_transformAftMapped[2], _transformAftMapped[0], _transformAftMapped[1]),
                                                Point3(_transformAftMapped[3], _transformAftMapped[4], _transformAftMapped[5]));
            _gtSAMgraph.add(BetweenFactor<Pose3>(_cloudKeyPoses3D->points.size()-1, _cloudKeyPoses3D->points.size(), poseFrom.between(poseTo), _odometryNoise));

            // _initialEstimate.insert(_cloudKeyPoses3D->points.size(), Pose3(Rot3::RzRyRx(_transformAftMapped[2], _transformAftMapped[0], _transformAftMapped[1]),
            //                                                          		   Point3(_transformAftMapped[5], _transformAftMapped[3], _transformAftMapped[4])));

            _initialEstimate.insert(_cloudKeyPoses3D->points.size(), Pose3(Rot3::RzRyRx(_transformAftMapped[2], _transformAftMapped[0], _transformAftMapped[1]),
                                                                     		   Point3(_transformAftMapped[3], _transformAftMapped[4], _transformAftMapped[5])));
        }
        /**
         * update iSAM
         */
        _isam->update(_gtSAMgraph, _initialEstimate);
        _isam->update();

        _gtSAMgraph.resize(0);
        _initialEstimate.clear();

        /**
         * save key poses
         */
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        _isamCurrentEstimate = _isam->calculateEstimate();
        latestEstimate = _isamCurrentEstimate.at<Pose3>(_isamCurrentEstimate.size()-1);

        // thisPose3D.x = latestEstimate.translation().y();
        // thisPose3D.y = latestEstimate.translation().z();
        // thisPose3D.z = latestEstimate.translation().x();

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = _cloudKeyPoses3D->points.size(); // this can be used as index
        _cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity; // this can be used as index

        // thisPose6D.roll  = latestEstimate.rotation().pitch();
        // thisPose6D.pitch = latestEstimate.rotation().yaw();
        // thisPose6D.yaw   = latestEstimate.rotation().roll(); // in camera frame

        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw(); // in camera frame
        thisPose6D.time = _timeLaserOdometry;
        _cloudKeyPoses6D->push_back(thisPose6D);
        /**
         * save updated transform
         */
        if (_cloudKeyPoses3D->points.size() > 1){
            _transformAftMapped[0] = latestEstimate.rotation().pitch();
            _transformAftMapped[1] = latestEstimate.rotation().yaw();
            _transformAftMapped[2] = latestEstimate.rotation().roll();

            // _transformAftMapped[3] = latestEstimate.translation().y();
            // _transformAftMapped[4] = latestEstimate.translation().z();
            // _transformAftMapped[5] = latestEstimate.translation().x();

            _transformAftMapped[3] = latestEstimate.translation().x();
            _transformAftMapped[4] = latestEstimate.translation().y();
            _transformAftMapped[5] = latestEstimate.translation().z();

            for (int i = 0; i < 6; ++i){
            	_transformLast[i] = _transformAftMapped[i];
            	_transformTobeMapped[i] = _transformAftMapped[i];
            }
        }

        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisOutlierKeyFrame(new pcl::PointCloud<PointType>());

        pcl::copyPointCloud(*_laserCloudCornerLastDS,  *thisCornerKeyFrame);
        pcl::copyPointCloud(*_laserCloudSurfLastDS,    *thisSurfKeyFrame);
        pcl::copyPointCloud(*_laserCloudOutlierLastDS, *thisOutlierKeyFrame);

        _cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        _surfCloudKeyFrames.push_back(thisSurfKeyFrame);
        _outlierCloudKeyFrames.push_back(thisOutlierKeyFrame);
    }

    void correctPoses(){
    	if (_aLoopIsClosed == true){
            _recentCornerCloudKeyFrames. clear();
            _recentSurfCloudKeyFrames.   clear();
            _recentOutlierCloudKeyFrames.clear();
            // update key poses
                int numPoses = _isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i){
            // _cloudKeyPoses3D->points[i].x = _isamCurrentEstimate.at<Pose3>(i).translation().y();
            // _cloudKeyPoses3D->points[i].y = _isamCurrentEstimate.at<Pose3>(i).translation().z();
            // _cloudKeyPoses3D->points[i].z = _isamCurrentEstimate.at<Pose3>(i).translation().x();

            _cloudKeyPoses3D->points[i].x = _isamCurrentEstimate.at<Pose3>(i).translation().x();
            _cloudKeyPoses3D->points[i].y = _isamCurrentEstimate.at<Pose3>(i).translation().y();
            _cloudKeyPoses3D->points[i].z = _isamCurrentEstimate.at<Pose3>(i).translation().z();

            _cloudKeyPoses6D->points[i].x = _cloudKeyPoses3D->points[i].x;
            _cloudKeyPoses6D->points[i].y = _cloudKeyPoses3D->points[i].y;
            _cloudKeyPoses6D->points[i].z = _cloudKeyPoses3D->points[i].z;
            // _cloudKeyPoses6D->points[i].roll  = _isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
            // _cloudKeyPoses6D->points[i].pitch = _isamCurrentEstimate.at<Pose3>(i).rotation().yaw();
            // _cloudKeyPoses6D->points[i].yaw   = _isamCurrentEstimate.at<Pose3>(i).rotation().roll();

            _cloudKeyPoses6D->points[i].roll  = _isamCurrentEstimate.at<Pose3>(i).rotation().roll();
            _cloudKeyPoses6D->points[i].pitch = _isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
            _cloudKeyPoses6D->points[i].yaw   = _isamCurrentEstimate.at<Pose3>(i).rotation().yaw();
            }

            _aLoopIsClosed = false;
        }
    }

    void clearCloud(){
        _laserCloudCornerFromMap->clear();
        _laserCloudSurfFromMap->clear();
        _laserCloudCornerFromMapDS->clear();
        _laserCloudSurfFromMapDS->clear();
    }

    void run(){

        if (_newLaserCloudCornerLast  && std::abs(_timeLaserCloudCornerLast  - _timeLaserOdometry) < 0.005 &&
            _newLaserCloudSurfLast    && std::abs(_timeLaserCloudSurfLast    - _timeLaserOdometry) < 0.005 &&
            _newLaserCloudOutlierLast && std::abs(_timeLaserCloudOutlierLast - _timeLaserOdometry) < 0.005 &&
            _newLaserOdometry)
        {

            _newLaserCloudCornerLast = false; _newLaserCloudSurfLast = false; _newLaserCloudOutlierLast = false; _newLaserOdometry = false;

            std::lock_guard<std::mutex> lock(_mtx);

            if (_timeLaserOdometry - _timeLastProcessing >= _mappingProcessInterval) {

                _timeLastProcessing = _timeLaserOdometry;

                transformAssociateToMap();

                extractSurroundingKeyFrames();

                downsampleCurrentScan();

                scan2MapOptimization();

                saveKeyFramesAndFactor();

                correctPoses();

                publishTF();

                publishKeyPosesAndFrames();

                clearCloud();
            }
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");

    ROS_INFO("\033[1;32m---->\033[0m Map Optimization Started.");

    mapOptimization MO;

    std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        MO.run();

        rate.sleep();
    }

    loopthread.join();
    visualizeMapThread.join();

    return 0;
}
