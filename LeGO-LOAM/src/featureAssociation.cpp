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
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.

#include "utility.h"

class FeatureAssociation{

private:

	ros::NodeHandle nh;

    ros::Subscriber _subLaserCloud;
    ros::Subscriber _subLaserCloudInfo;
    ros::Subscriber _subOutlierCloud;
    ros::Subscriber _subImu;

    ros::Publisher _pubCornerPointsSharp;
    ros::Publisher _pubCornerPointsLessSharp;
    ros::Publisher _pubSurfPointsFlat;
    ros::Publisher _pubSurfPointsLessFlat;

    pcl::PointCloud<PointType>::Ptr _segmentedCloud;
    pcl::PointCloud<PointType>::Ptr _outlierCloud;

    pcl::PointCloud<PointType>::Ptr _cornerPointsSharp;
    pcl::PointCloud<PointType>::Ptr _cornerPointsLessSharp;
    pcl::PointCloud<PointType>::Ptr _surfPointsFlat;
    pcl::PointCloud<PointType>::Ptr _surfPointsLessFlat;

    pcl::PointCloud<PointType>::Ptr _surfPointsLessFlatScan;
    pcl::PointCloud<PointType>::Ptr _surfPointsLessFlatScanDS;

    pcl::VoxelGrid<PointType> _downSizeFilter;

    double _timeScanCur;
    double _timeNewSegmentedCloud;
    double _timeNewSegmentedCloudInfo;
    double _timeNewOutlierCloud;

    bool _newSegmentedCloud;
    bool _newSegmentedCloudInfo;
    bool _newOutlierCloud;

    cloud_msgs::cloud_info _segInfo;
    std_msgs::Header _cloudHeader;

    int _systemInitCount;
    bool _systemInited;

    std::vector<smoothness_t> _cloudSmoothness;
		std::vector<float> _cloudCurvature;
		std::vector<int> _cloudNeighborPicked;
		std::vector<int> _cloudLabel;
    // float _cloudCurvature[_N_SCAN*_Horizon_SCAN];
    // int _cloudNeighborPicked[_N_SCAN*_Horizon_SCAN];
    // int _cloudLabel[_N_SCAN*_Horizon_SCAN];

    int _imuPointerFront;
    int _imuPointerLast;
    int _imuPointerLastIteration;

    float _imuRollStart, _imuPitchStart, _imuYawStart;
    float _cosImuRollStart, _cosImuPitchStart, _cosImuYawStart, _sinImuRollStart, _sinImuPitchStart, _sinImuYawStart;
    float _imuRollCur, _imuPitchCur, _imuYawCur;

    float _imuVeloXStart, _imuVeloYStart, _imuVeloZStart;
    float _imuShiftXStart, _imuShiftYStart, _imuShiftZStart;

    float _imuVeloXCur, _imuVeloYCur, _imuVeloZCur;
    float _imuShiftXCur, _imuShiftYCur, _imuShiftZCur;

    float _imuShiftFromStartXCur, _imuShiftFromStartYCur, _imuShiftFromStartZCur;
    float _imuVeloFromStartXCur, _imuVeloFromStartYCur, _imuVeloFromStartZCur;

    float _imuAngularRotationXCur, _imuAngularRotationYCur, _imuAngularRotationZCur;
    float _imuAngularRotationXLast, _imuAngularRotationYLast, _imuAngularRotationZLast;
    float _imuAngularFromStartX, _imuAngularFromStartY, _imuAngularFromStartZ;

		std::vector<double> _imuTime;
		std::vector<float> _imuRoll;
		std::vector<float> _imuPitch;
		std::vector<float> _imuYaw;

		std::vector<float> _imuAccX;
		std::vector<float> _imuAccY;
		std::vector<float> _imuAccZ;

		std::vector<float> _imuVeloX;
		std::vector<float> _imuVeloY;
		std::vector<float> _imuVeloZ;

		std::vector<float> _imuShiftX;
		std::vector<float> _imuShiftY;
		std::vector<float> _imuShiftZ;

		std::vector<float> _imuAngularVeloX;
		std::vector<float> _imuAngularVeloY;
		std::vector<float> _imuAngularVeloZ;

		std::vector<float> _imuAngularRotationX;
		std::vector<float> _imuAngularRotationY;
		std::vector<float> _imuAngularRotationZ;

    // double _imuTime[_imuQueLength];
    // float _imuRoll[_imuQueLength];
    // float _imuPitch[_imuQueLength];
    // float _imuYaw[_imuQueLength];
		//
    // float _imuAccX[_imuQueLength];
    // float _imuAccY[_imuQueLength];
    // float _imuAccZ[_imuQueLength];
		//
    // float _imuVeloX[_imuQueLength];
    // float _imuVeloY[_imuQueLength];
    // float _imuVeloZ[_imuQueLength];
		//
    // float _imuShiftX[_imuQueLength];
    // float _imuShiftY[_imuQueLength];
    // float _imuShiftZ[_imuQueLength];
		//
    // float _imuAngularVeloX[_imuQueLength];
    // float _imuAngularVeloY[_imuQueLength];
    // float _imuAngularVeloZ[_imuQueLength];
		//
    // float _imuAngularRotationX[_imuQueLength];
    // float _imuAngularRotationY[_imuQueLength];
    // float _imuAngularRotationZ[_imuQueLength];



    ros::Publisher _pubLaserCloudCornerLast;
    ros::Publisher _pubLaserCloudSurfLast;
    ros::Publisher _pubLaserOdometry;
    ros::Publisher _pubOutlierCloudLast;

    int _skipFrameNum;
    bool _systemInitedLM;

    int _laserCloudCornerLastNum;
    int _laserCloudSurfLastNum;

		// int pointSelCornerInd[N_SCAN*Horizon_SCAN];
		std::vector<float> _pointSearchCornerInd1;
		std::vector<float> _pointSearchCornerInd2;
    // float _pointSearchCornerInd1[N_SCAN*Horizon_SCAN];
    // float _pointSearchCornerInd2[N_SCAN*Horizon_SCAN];

    // int pointSelSurfInd[N_SCAN*Horizon_SCAN];
		std::vector<float> _pointSearchSurfInd1;
		std::vector<float> _pointSearchSurfInd2;
		std::vector<float> _pointSearchSurfInd3;
    // float _pointSearchSurfInd1[N_SCAN*Horizon_SCAN];
    // float _pointSearchSurfInd2[N_SCAN*Horizon_SCAN];
    // float _pointSearchSurfInd3[N_SCAN*Horizon_SCAN];

    float _transformCur[6];
    float _transformSum[6];

    float _imuRollLast, _imuPitchLast, _imuYawLast;
    float _imuShiftFromStartX, _imuShiftFromStartY, _imuShiftFromStartZ;
    float _imuVeloFromStartX, _imuVeloFromStartY, _imuVeloFromStartZ;

    pcl::PointCloud<PointType>::Ptr _laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr _laserCloudSurfLast;
    pcl::PointCloud<PointType>::Ptr _laserCloudOri;
    pcl::PointCloud<PointType>::Ptr _coeffSel;

    pcl::KdTreeFLANN<PointType>::Ptr _kdtreeCornerLast;
    pcl::KdTreeFLANN<PointType>::Ptr _kdtreeSurfLast;

    std::vector<int> _pointSearchInd;
    std::vector<float> _pointSearchSqDis;

    PointType _pointOri, _pointSel, _tripod1, _tripod2, _tripod3, _pointProj, _coeff;

    nav_msgs::Odometry _laserOdometry;

    // tf::TransformBroadcaster _tfBroadcaster;
    tf::StampedTransform _laserOdometryTrans;

    bool _isDegenerate;
    cv::Mat _matP;

    int _frameCount;

		int _N_SCAN;
		int _Horizon_SCAN;
		int _imuQueLength;
		string _imuTopic;
		float _scanPeriod;
		float _edgeThreshold;
		float _surfThreshold;
		float _nearestFeatureSearchDist;
		float _nearestFeatureSearchSqDist;

		// bool _transformToRobotCenter;
		string _lidarFrameId;
    string _lidarFrameIdTrans;
		string _mapFrameId;
    string _frameId;
public:

    FeatureAssociation():
        nh("~")
        {

			  if(!nh.getParam("/lego_loam/laser/N_SCAN", _N_SCAN)) ROS_ERROR("Failed to get param '/lego_loam/laser/N_SCAN'");
	      if(!nh.getParam("/lego_loam/laser/Horizon_SCAN", _Horizon_SCAN)) ROS_ERROR("Failed to get param '/lego_loam/laser/Horizon_SCAN'");
				if(!nh.getParam("/lego_loam/imuTopic", _imuTopic)) ROS_ERROR("Failed to get param '/lego_loam/imuTopic'");
				if(!nh.getParam("/lego_loam/imu/imuQueLength", _imuQueLength)) ROS_ERROR("Failed to get param '/lego_loam/imu/imuQueLength'");
				if(!nh.getParam("/lego_loam/laser/scanPeriod", _scanPeriod)) ROS_ERROR("Failed to get param '/lego_loam/laser/scanPeriod'");
				if(!nh.getParam("/lego_loam/featureAssociation/edgeThreshold", _edgeThreshold)) ROS_ERROR("Failed to get param '/lego_loam/featureAssociation/edgeThreshold'");
				if(!nh.getParam("/lego_loam/featureAssociation/surfThreshold", _surfThreshold)) ROS_ERROR("Failed to get param '/lego_loam/featureAssociation/surfThreshold'");
				if(!nh.getParam("/lego_loam/featureAssociation/nearestFeatureSearchDist", _nearestFeatureSearchDist)) ROS_ERROR("Failed to get param '/lego_loam/featureAssociation/nearestFeatureSearchDist'");

				// if(!nh.getParam("/lego_loam/pointTransform/transformToRobotCenter", _transformToRobotCenter)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/transformToRobotCenter'");
				if(!nh.getParam("/lego_loam/lidarFrameId", _lidarFrameId)) ROS_ERROR("Failed to get param '/lego_loam/lidarFrameId'");
        // if(!nh.getParam("/lego_loam/lidarFrameIdTrans", _lidarFrameIdTrans)) ROS_ERROR("Failed to get param '/lego_loam/lidarFrameId'");
				if(!nh.getParam("/lego_loam/mapFrameId", _mapFrameId)) ROS_ERROR("Failed to get param '/lego_loam/mapFrameId'");

        _subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud", 1, &FeatureAssociation::laserCloudHandler, this);
        _subLaserCloudInfo = nh.subscribe<cloud_msgs::cloud_info>("/segmented_cloud_info", 1, &FeatureAssociation::laserCloudInfoHandler, this);
        _subOutlierCloud = nh.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud", 1, &FeatureAssociation::outlierCloudHandler, this);
        _subImu = nh.subscribe<sensor_msgs::Imu>(_imuTopic, 50, &FeatureAssociation::imuHandler, this);

        _pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 1);
        _pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 1);
        _pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 1);
        _pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 1);

        _pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
        _pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
        _pubOutlierCloudLast = nh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2);
        _pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);

				_nearestFeatureSearchSqDist = _nearestFeatureSearchDist*_nearestFeatureSearchDist;

				// if(_transformToRobotCenter) {_frameId = _lidarFrameIdTrans;}
        // else {_frameId = _lidarFrameId;}

				_frameId = _lidarFrameId;

        initializationValue();
    }

    void initializationValue()
    {
        _cloudSmoothness.resize(_N_SCAN*_Horizon_SCAN);
				_cloudCurvature.resize(_N_SCAN*_Horizon_SCAN);
		    _cloudNeighborPicked.resize(_N_SCAN*_Horizon_SCAN);
		    _cloudLabel.resize(_N_SCAN*_Horizon_SCAN);

				_pointSearchCornerInd1.resize(_N_SCAN*_Horizon_SCAN);
				_pointSearchCornerInd2.resize(_N_SCAN*_Horizon_SCAN);
				_pointSearchSurfInd1.resize(_N_SCAN*_Horizon_SCAN);
				_pointSearchSurfInd2.resize(_N_SCAN*_Horizon_SCAN);
				_pointSearchSurfInd3.resize(_N_SCAN*_Horizon_SCAN);

				_imuTime.resize(_imuQueLength);;
				_imuRoll.resize(_imuQueLength);
				_imuPitch.resize(_imuQueLength);
				_imuYaw.resize(_imuQueLength);

				_imuAccX.resize(_imuQueLength);
				_imuAccY.resize(_imuQueLength);
				_imuAccZ.resize(_imuQueLength);

				_imuVeloX.resize(_imuQueLength);
				_imuVeloY.resize(_imuQueLength);
				_imuVeloZ.resize(_imuQueLength);

				_imuShiftX.resize(_imuQueLength);
				_imuShiftY.resize(_imuQueLength);
				_imuShiftZ.resize(_imuQueLength);

				_imuAngularVeloX.resize(_imuQueLength);
				_imuAngularVeloY.resize(_imuQueLength);
				_imuAngularVeloZ.resize(_imuQueLength);

				_imuAngularRotationX.resize(_imuQueLength);
				_imuAngularRotationY.resize(_imuQueLength);
				_imuAngularRotationZ.resize(_imuQueLength);

        _downSizeFilter.setLeafSize(0.2, 0.2, 0.2);

        _segmentedCloud.reset(new pcl::PointCloud<PointType>());
        _outlierCloud.reset(new pcl::PointCloud<PointType>());

        _cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
        _cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
        _surfPointsFlat.reset(new pcl::PointCloud<PointType>());
        _surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());

        _surfPointsLessFlatScan.reset(new pcl::PointCloud<PointType>());
        _surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointType>());

        _timeScanCur = 0;
        _timeNewSegmentedCloud = 0;
        _timeNewSegmentedCloudInfo = 0;
        _timeNewOutlierCloud = 0;

        _newSegmentedCloud = false;
        _newSegmentedCloudInfo = false;
        _newOutlierCloud = false;

        _systemInitCount = 0;
        _systemInited = false;

        _imuPointerFront = 0;
        _imuPointerLast = -1;
        _imuPointerLastIteration = 0;

        _imuRollStart = 0; _imuPitchStart = 0; _imuYawStart = 0;
        _cosImuRollStart = 0; _cosImuPitchStart = 0; _cosImuYawStart = 0;
        _sinImuRollStart = 0; _sinImuPitchStart = 0; _sinImuYawStart = 0;
        _imuRollCur = 0; _imuPitchCur = 0; _imuYawCur = 0;

        _imuVeloXStart = 0; _imuVeloYStart = 0; _imuVeloZStart = 0;
        _imuShiftXStart = 0; _imuShiftYStart = 0; _imuShiftZStart = 0;

        _imuVeloXCur = 0; _imuVeloYCur = 0; _imuVeloZCur = 0;
        _imuShiftXCur = 0; _imuShiftYCur = 0; _imuShiftZCur = 0;

        _imuShiftFromStartXCur = 0; _imuShiftFromStartYCur = 0; _imuShiftFromStartZCur = 0;
        _imuVeloFromStartXCur = 0; _imuVeloFromStartYCur = 0; _imuVeloFromStartZCur = 0;

        _imuAngularRotationXCur = 0; _imuAngularRotationYCur = 0; _imuAngularRotationZCur = 0;
        _imuAngularRotationXLast = 0; _imuAngularRotationYLast = 0; _imuAngularRotationZLast = 0;
        _imuAngularFromStartX = 0; _imuAngularFromStartY = 0; _imuAngularFromStartZ = 0;

        for (int i = 0; i < _imuQueLength; ++i)
        {
            _imuTime[i] = 0;
            _imuRoll[i] = 0; _imuPitch[i] = 0; _imuYaw[i] = 0;
            _imuAccX[i] = 0; _imuAccY[i] = 0; _imuAccZ[i] = 0;
            _imuVeloX[i] = 0; _imuVeloY[i] = 0; _imuVeloZ[i] = 0;
            _imuShiftX[i] = 0; _imuShiftY[i] = 0; _imuShiftZ[i] = 0;
            _imuAngularVeloX[i] = 0; _imuAngularVeloY[i] = 0; _imuAngularVeloZ[i] = 0;
            _imuAngularRotationX[i] = 0; _imuAngularRotationY[i] = 0; _imuAngularRotationZ[i] = 0;
        }


        _skipFrameNum = 1;

        for (int i = 0; i < 6; ++i){
            _transformCur[i] = 0;
            _transformSum[i] = 0;
        }

        _systemInitedLM = false;

        _imuRollLast = 0; _imuPitchLast = 0; _imuYawLast = 0;
        _imuShiftFromStartX = 0; _imuShiftFromStartY = 0; _imuShiftFromStartZ = 0;
        _imuVeloFromStartX = 0; _imuVeloFromStartY = 0; _imuVeloFromStartZ = 0;

        _laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
        _laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        _laserCloudOri.reset(new pcl::PointCloud<PointType>());
        _coeffSel.reset(new pcl::PointCloud<PointType>());

        _kdtreeCornerLast.reset(new pcl::KdTreeFLANN<PointType>());
        _kdtreeSurfLast.reset(new pcl::KdTreeFLANN<PointType>());

        // _laserOdometry.header.frame_id = "/camera_init";
        // _laserOdometry.child_frame_id = "/laser_odom";
				//
        // _laserOdometryTrans.frame_id_ = "/camera_init";
        // _laserOdometryTrans.child_frame_id_ = "/laser_odom";

				_laserOdometry.header.frame_id = _mapFrameId;
        _laserOdometry.child_frame_id = "/velodyne_odom";

        _laserOdometryTrans.frame_id_ = _mapFrameId;
        _laserOdometryTrans.child_frame_id_ = "/velodyne_odom";

        _isDegenerate = false;
        _matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

        _frameCount = _skipFrameNum;
    }

    void updateImuRollPitchYawStartSinCos(){
        _cosImuRollStart = cos(_imuRollStart);
        _cosImuPitchStart = cos(_imuPitchStart);
        _cosImuYawStart = cos(_imuYawStart);
        _sinImuRollStart = sin(_imuRollStart);
        _sinImuPitchStart = sin(_imuPitchStart);
        _sinImuYawStart = sin(_imuYawStart);
    }


    void ShiftToStartIMU(float pointTime)
    {
        _imuShiftFromStartXCur = _imuShiftXCur - _imuShiftXStart - _imuVeloXStart * pointTime;
        _imuShiftFromStartYCur = _imuShiftYCur - _imuShiftYStart - _imuVeloYStart * pointTime;
        _imuShiftFromStartZCur = _imuShiftZCur - _imuShiftZStart - _imuVeloZStart * pointTime;

				// rotate with z axis
				float y1 = _cosImuYawStart * _imuShiftFromStartYCur - _sinImuYawStart * _imuShiftFromStartXCur;
        float z1 = _imuShiftFromStartZCur;
        float x1 = _sinImuYawStart * _imuShiftFromStartYCur + _cosImuYawStart * _imuShiftFromStartXCur;

        // float x1 = _cosImuYawStart * _imuShiftFromStartXCur - _sinImuYawStart * _imuShiftFromStartZCur;
        // float y1 = _imuShiftFromStartYCur;
        // float z1 = _sinImuYawStart * _imuShiftFromStartXCur + _cosImuYawStart * _imuShiftFromStartZCur;

				// rotate with y axis
				float y2 = y1;
        float z2 = _cosImuPitchStart * z1 + _sinImuPitchStart * x1;
        float x2 = -_sinImuPitchStart * z1 + _cosImuPitchStart * x1;

        // float x2 = x1;
        // float y2 = _cosImuPitchStart * y1 + _sinImuPitchStart * z1;
        // float z2 = -_sinImuPitchStart * y1 + _cosImuPitchStart * z1;

				// rotate with x axis
        _imuShiftFromStartYCur = _cosImuRollStart * y2 + _sinImuRollStart * z2;
        _imuShiftFromStartZCur = -_sinImuRollStart * y2 + _cosImuRollStart * z2;
        _imuShiftFromStartXCur = x2;

				// _imuShiftFromStartXCur = _cosImuRollStart * x2 + _sinImuRollStart * y2;
        // _imuShiftFromStartYCur = -_sinImuRollStart * x2 + _cosImuRollStart * y2;
        // _imuShiftFromStartZCur = z2;
    }

    void VeloToStartIMU()
    {
        _imuVeloFromStartXCur = _imuVeloXCur - _imuVeloXStart;
        _imuVeloFromStartYCur = _imuVeloYCur - _imuVeloYStart;
        _imuVeloFromStartZCur = _imuVeloZCur - _imuVeloZStart;

				// rotate with z axis
				float y1 = _cosImuYawStart * _imuVeloFromStartYCur - _sinImuYawStart * _imuVeloFromStartXCur;
        float z1 = _imuVeloFromStartZCur;
        float x1 = _sinImuYawStart * _imuVeloFromStartYCur + _cosImuYawStart * _imuVeloFromStartXCur;

				// float x1 = _cosImuYawStart * _imuVeloFromStartXCur - _sinImuYawStart * _imuVeloFromStartZCur;
        // float y1 = _imuVeloFromStartYCur;
        // float z1 = _sinImuYawStart * _imuVeloFromStartXCur + _cosImuYawStart * _imuVeloFromStartZCur;

				// rotate with y axis
        float y2 = y1;
        float z2 = _cosImuPitchStart * z1 + _sinImuPitchStart * x1;
        float x2 = -_sinImuPitchStart * z1 + _cosImuPitchStart * x1;

				// float x2 = x1;
        // float y2 = _cosImuPitchStart * y1 + _sinImuPitchStart * z1;
        // float z2 = -_sinImuPitchStart * y1 + _cosImuPitchStart * z1;

				// rotate with x axis
        _imuVeloFromStartYCur = _cosImuRollStart * y2 + _sinImuRollStart * z2;
        _imuVeloFromStartZCur = -_sinImuRollStart * y2 + _cosImuRollStart * z2;
        _imuVeloFromStartXCur = x2;

				// _imuVeloFromStartXCur = _cosImuRollStart * x2 + _sinImuRollStart * y2;
        // _imuVeloFromStartYCur = -_sinImuRollStart * x2 + _cosImuRollStart * y2;
        // _imuVeloFromStartZCur = z2;
    }

    void TransformToStartIMU(PointType *p)
    {
				// local to global.
				// rotate with x axis
				float y1 = cos(_imuRollCur) * p->y - sin(_imuRollCur) * p->z;
        float z1 = sin(_imuRollCur) * p->y + cos(_imuRollCur) * p->z;
        float x1 = p->x;

				// float x1 = cos(_imuRollCur) * p->x - sin(_imuRollCur) * p->y;
        // float y1 = sin(_imuRollCur) * p->x + cos(_imuRollCur) * p->y;
        // float z1 = p->z;

				// rotate with y axis
        float y2 = y1;
        float z2 = cos(_imuPitchCur) * z1 - sin(_imuPitchCur) * x1;
        float x2 = sin(_imuPitchCur) * z1 + cos(_imuPitchCur) * x1;

				// float x2 = x1;
        // float y2 = cos(_imuPitchCur) * y1 - sin(_imuPitchCur) * z1;
        // float z2 = sin(_imuPitchCur) * y1 + cos(_imuPitchCur) * z1;

				// rotate with z axis
        float y3 = cos(_imuYawCur) * y2 + sin(_imuYawCur) * x2;
        float z3 = z2;
        float x3 = -sin(_imuYawCur) * y2 + cos(_imuYawCur) * x2;

				// float x3 = cos(_imuYawCur) * x2 + sin(_imuYawCur) * z2;
        // float y3 = y2;
        // float z3 = -sin(_imuYawCur) * x2 + cos(_imuYawCur) * z2;

				// global to local.
				// rotate with z axis.
        float y4 = _cosImuYawStart * y3 - _sinImuYawStart * x3;
        float z4 = z3;
        float x4 = _sinImuYawStart * y3 + _cosImuYawStart * x3;

				// float x4 = _cosImuYawStart * x3 - _sinImuYawStart * z3;
        // float y4 = y3;
        // float z4 = _sinImuYawStart * x3 + _cosImuYawStart * z3;

				// rotate with y axis.
        float y5 = y4;
        float z5 = _cosImuPitchStart * z4 + _sinImuPitchStart * x4;
        float x5 = -_sinImuPitchStart * z4 + _cosImuPitchStart * x4;

				// float x5 = x4;
				// float y5 = _cosImuPitchStart * y4 + _sinImuPitchStart * z4;
				// float z5 = -_sinImuPitchStart * y4 + _cosImuPitchStart * z4;

				// rotate with x axis.
        p->y = _cosImuRollStart * y5 + _sinImuRollStart * z5 + _imuShiftFromStartYCur;
        p->z = -_sinImuRollStart * y5 + _cosImuRollStart * z5 + _imuShiftFromStartZCur;
        p->x = x5 + _imuShiftFromStartXCur;
    }

    void AccumulateIMUShiftAndRotation()
    {
        float roll = _imuRoll[_imuPointerLast];
        float pitch = _imuPitch[_imuPointerLast];
        float yaw = _imuYaw[_imuPointerLast];
        float accX = _imuAccX[_imuPointerLast];
        float accY = _imuAccY[_imuPointerLast];
        float accZ = _imuAccZ[_imuPointerLast];
				// rotate with x axis
				float x1 = accX;
        float y1 = cos(roll) * accY - sin(roll) * accZ;
        float z1 = sin(roll) * accY + cos(roll) * accZ;

        // float x1 = cos(roll) * accX - sin(roll) * accY;
        // float y1 = sin(roll) * accX + cos(roll) * accY;
        // float z1 = accZ;

				// rotate with y axis
				float x2 = sin(pitch) * z1 + cos(pitch) * x1;
        float y2 = y1;
        float z2 = cos(pitch) * z1 - sin(pitch) * x1;

        // float x2 = x1;
        // float y2 = cos(pitch) * y1 - sin(pitch) * z1;
        // float z2 = sin(pitch) * y1 + cos(pitch) * z1;

				// rotate with z axis
				accX = -sin(yaw) * y2 + cos(yaw) * x2;
        accY = cos(yaw) * y2 + sin(yaw) * x2;
        accZ = z2;

        // accX = cos(yaw) * x2 + sin(yaw) * z2;
        // accY = y2;
        // accZ = -sin(yaw) * x2 + cos(yaw) * z2;

        int imuPointerBack = (_imuPointerLast + _imuQueLength - 1) % _imuQueLength;
        double timeDiff = _imuTime[_imuPointerLast] - _imuTime[imuPointerBack];
        if (timeDiff < _scanPeriod) {

            _imuShiftX[_imuPointerLast] = _imuShiftX[imuPointerBack] + _imuVeloX[imuPointerBack] * timeDiff + accX * timeDiff * timeDiff / 2;
            _imuShiftY[_imuPointerLast] = _imuShiftY[imuPointerBack] + _imuVeloY[imuPointerBack] * timeDiff + accY * timeDiff * timeDiff / 2;
            _imuShiftZ[_imuPointerLast] = _imuShiftZ[imuPointerBack] + _imuVeloZ[imuPointerBack] * timeDiff + accZ * timeDiff * timeDiff / 2;

            _imuVeloX[_imuPointerLast] = _imuVeloX[imuPointerBack] + accX * timeDiff;
            _imuVeloY[_imuPointerLast] = _imuVeloY[imuPointerBack] + accY * timeDiff;
            _imuVeloZ[_imuPointerLast] = _imuVeloZ[imuPointerBack] + accZ * timeDiff;

            _imuAngularRotationX[_imuPointerLast] = _imuAngularRotationX[imuPointerBack] + _imuAngularVeloX[imuPointerBack] * timeDiff;
            _imuAngularRotationY[_imuPointerLast] = _imuAngularRotationY[imuPointerBack] + _imuAngularVeloY[imuPointerBack] * timeDiff;
            _imuAngularRotationZ[_imuPointerLast] = _imuAngularRotationZ[imuPointerBack] + _imuAngularVeloZ[imuPointerBack] * timeDiff;
        }
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
    {
        double roll, pitch, yaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imuIn->orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

				// std::cout<< "pitch: " << pitch * 180.0 / 3.1415926 << " "<< "roll: "<< roll * 180.0 / 3.1415926 <<std::endl;
        float accY = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
        float accZ = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
        float accX = imuIn->linear_acceleration.x + sin(pitch) * 9.81;

        _imuPointerLast = (_imuPointerLast + 1) % _imuQueLength;

        _imuTime[_imuPointerLast] = imuIn->header.stamp.toSec();

        _imuRoll[_imuPointerLast] = roll;
        _imuPitch[_imuPointerLast] = pitch;
        _imuYaw[_imuPointerLast] = yaw;

        _imuAccX[_imuPointerLast] = accX;
        _imuAccY[_imuPointerLast] = accY;
        _imuAccZ[_imuPointerLast] = accZ;

        _imuAngularVeloX[_imuPointerLast] = imuIn->angular_velocity.x;
        _imuAngularVeloY[_imuPointerLast] = imuIn->angular_velocity.y;
        _imuAngularVeloZ[_imuPointerLast] = imuIn->angular_velocity.z;

        AccumulateIMUShiftAndRotation();
    }

    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        _cloudHeader = laserCloudMsg->header;

        _timeScanCur = _cloudHeader.stamp.toSec();
        _timeNewSegmentedCloud = _timeScanCur;

        _segmentedCloud->clear();
        pcl::fromROSMsg(*laserCloudMsg, *_segmentedCloud);

        _newSegmentedCloud = true;
    }

    void outlierCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msgIn){

        _timeNewOutlierCloud = msgIn->header.stamp.toSec();

        _outlierCloud->clear();
        pcl::fromROSMsg(*msgIn, *_outlierCloud);

        _newOutlierCloud = true;
    }

    void laserCloudInfoHandler(const cloud_msgs::cloud_infoConstPtr& msgIn)
    {
        _timeNewSegmentedCloudInfo = msgIn->header.stamp.toSec();
        _segInfo = *msgIn;
        _newSegmentedCloudInfo = true;
    }

    void adjustDistortion()
    {
        bool halfPassed = false;
        int cloudSize = _segmentedCloud->points.size();

        PointType point;

        for (int i = 0; i < cloudSize; i++) {

            point.x = _segmentedCloud->points[i].x;
            point.y = _segmentedCloud->points[i].y;
            point.z = _segmentedCloud->points[i].z;

            float ori = -atan2(point.y, point.x);
            if (!halfPassed) {
                if (ori < _segInfo.startOrientation - M_PI / 2)
                    ori += 2 * M_PI;
                else if (ori > _segInfo.startOrientation + M_PI * 3 / 2)
                    ori -= 2 * M_PI;

                if (ori - _segInfo.startOrientation > M_PI)
                    halfPassed = true;
            } else {
                ori += 2 * M_PI;

                if (ori < _segInfo.endOrientation - M_PI * 3 / 2)
                    ori += 2 * M_PI;
                else if (ori > _segInfo.endOrientation + M_PI / 2)
                    ori -= 2 * M_PI;
            }

            float relTime = (ori - _segInfo.startOrientation) / _segInfo.orientationDiff;
            point.intensity = int(_segmentedCloud->points[i].intensity) + _scanPeriod * relTime;

            if (_imuPointerLast >= 0) {
                float pointTime = relTime * _scanPeriod;
                _imuPointerFront = _imuPointerLastIteration;
                while (_imuPointerFront != _imuPointerLast) {
                    if (_timeScanCur + pointTime < _imuTime[_imuPointerFront]) {
                        break;
                    }
                    _imuPointerFront = (_imuPointerFront + 1) % _imuQueLength;
                }

                if (_timeScanCur + pointTime > _imuTime[_imuPointerFront]) {
                    _imuRollCur = _imuRoll[_imuPointerFront];
                    _imuPitchCur = _imuPitch[_imuPointerFront];
                    _imuYawCur = _imuYaw[_imuPointerFront];

                    _imuVeloXCur = _imuVeloX[_imuPointerFront];
                    _imuVeloYCur = _imuVeloY[_imuPointerFront];
                    _imuVeloZCur = _imuVeloZ[_imuPointerFront];

                    _imuShiftXCur = _imuShiftX[_imuPointerFront];
                    _imuShiftYCur = _imuShiftY[_imuPointerFront];
                    _imuShiftZCur = _imuShiftZ[_imuPointerFront];
                } else {
                    int imuPointerBack = (_imuPointerFront + _imuQueLength - 1) % _imuQueLength;
                    float ratioFront = (_timeScanCur + pointTime - _imuTime[imuPointerBack])
                                                     / (_imuTime[_imuPointerFront] - _imuTime[imuPointerBack]);
                    float ratioBack = (_imuTime[_imuPointerFront] - _timeScanCur - pointTime)
                                                    / (_imuTime[_imuPointerFront] - _imuTime[imuPointerBack]);

                    _imuRollCur = _imuRoll[_imuPointerFront] * ratioFront + _imuRoll[imuPointerBack] * ratioBack;
                    _imuPitchCur = _imuPitch[_imuPointerFront] * ratioFront + _imuPitch[imuPointerBack] * ratioBack;
                    if (_imuYaw[_imuPointerFront] - _imuYaw[imuPointerBack] > M_PI) {
                        _imuYawCur = _imuYaw[_imuPointerFront] * ratioFront + (_imuYaw[imuPointerBack] + 2 * M_PI) * ratioBack;
                    } else if (_imuYaw[_imuPointerFront] - _imuYaw[imuPointerBack] < -M_PI) {
                        _imuYawCur = _imuYaw[_imuPointerFront] * ratioFront + (_imuYaw[imuPointerBack] - 2 * M_PI) * ratioBack;
                    } else {
                        _imuYawCur = _imuYaw[_imuPointerFront] * ratioFront + _imuYaw[imuPointerBack] * ratioBack;
                    }

                    _imuVeloXCur = _imuVeloX[_imuPointerFront] * ratioFront + _imuVeloX[imuPointerBack] * ratioBack;
                    _imuVeloYCur = _imuVeloY[_imuPointerFront] * ratioFront + _imuVeloY[imuPointerBack] * ratioBack;
                    _imuVeloZCur = _imuVeloZ[_imuPointerFront] * ratioFront + _imuVeloZ[imuPointerBack] * ratioBack;

                    _imuShiftXCur = _imuShiftX[_imuPointerFront] * ratioFront + _imuShiftX[imuPointerBack] * ratioBack;
                    _imuShiftYCur = _imuShiftY[_imuPointerFront] * ratioFront + _imuShiftY[imuPointerBack] * ratioBack;
                    _imuShiftZCur = _imuShiftZ[_imuPointerFront] * ratioFront + _imuShiftZ[imuPointerBack] * ratioBack;
                }

                if (i == 0) {
                    _imuRollStart = _imuRollCur;
                    _imuPitchStart = _imuPitchCur;
                    _imuYawStart = _imuYawCur;

                    _imuVeloXStart = _imuVeloXCur;
                    _imuVeloYStart = _imuVeloYCur;
                    _imuVeloZStart = _imuVeloZCur;

                    _imuShiftXStart = _imuShiftXCur;
                    _imuShiftYStart = _imuShiftYCur;
                    _imuShiftZStart = _imuShiftZCur;

                    if (_timeScanCur + pointTime > _imuTime[_imuPointerFront]) {
                        _imuAngularRotationXCur = _imuAngularRotationX[_imuPointerFront];
                        _imuAngularRotationYCur = _imuAngularRotationY[_imuPointerFront];
                        _imuAngularRotationZCur = _imuAngularRotationZ[_imuPointerFront];
                    }else{
                        int imuPointerBack = (_imuPointerFront + _imuQueLength - 1) % _imuQueLength;
                        float ratioFront = (_timeScanCur + pointTime - _imuTime[imuPointerBack])
                                                         / (_imuTime[_imuPointerFront] - _imuTime[imuPointerBack]);
                        float ratioBack = (_imuTime[_imuPointerFront] - _timeScanCur - pointTime)
                                                        / (_imuTime[_imuPointerFront] - _imuTime[imuPointerBack]);
                        _imuAngularRotationXCur = _imuAngularRotationX[_imuPointerFront] * ratioFront + _imuAngularRotationX[imuPointerBack] * ratioBack;
                        _imuAngularRotationYCur = _imuAngularRotationY[_imuPointerFront] * ratioFront + _imuAngularRotationY[imuPointerBack] * ratioBack;
                        _imuAngularRotationZCur = _imuAngularRotationZ[_imuPointerFront] * ratioFront + _imuAngularRotationZ[imuPointerBack] * ratioBack;
                    }

                    _imuAngularFromStartX = _imuAngularRotationXCur - _imuAngularRotationXLast;
                    _imuAngularFromStartY = _imuAngularRotationYCur - _imuAngularRotationYLast;
                    _imuAngularFromStartZ = _imuAngularRotationZCur - _imuAngularRotationZLast;

                    _imuAngularRotationXLast = _imuAngularRotationXCur;
                    _imuAngularRotationYLast = _imuAngularRotationYCur;
                    _imuAngularRotationZLast = _imuAngularRotationZCur;

                    updateImuRollPitchYawStartSinCos();
                } else {
										ShiftToStartIMU(pointTime);
                    VeloToStartIMU();
                    TransformToStartIMU(&point);
                }
            }
            _segmentedCloud->points[i] = point;
        }

        _imuPointerLastIteration = _imuPointerLast;
    }

    void calculateSmoothness()
    {
        int cloudSize = _segmentedCloud->points.size();
        for (int i = 5; i < cloudSize - 5; i++) {

            float diffRange = _segInfo.segmentedCloudRange[i-5] + _segInfo.segmentedCloudRange[i-4]
                            + _segInfo.segmentedCloudRange[i-3] + _segInfo.segmentedCloudRange[i-2]
                            + _segInfo.segmentedCloudRange[i-1] - _segInfo.segmentedCloudRange[i] * 10
                            + _segInfo.segmentedCloudRange[i+1] + _segInfo.segmentedCloudRange[i+2]
                            + _segInfo.segmentedCloudRange[i+3] + _segInfo.segmentedCloudRange[i+4]
                            + _segInfo.segmentedCloudRange[i+5];

            _cloudCurvature[i] = diffRange*diffRange;

            _cloudNeighborPicked[i] = 0;
            _cloudLabel[i] = 0;

            _cloudSmoothness[i].value = _cloudCurvature[i];
            _cloudSmoothness[i].ind = i;
        }
    }

    void markOccludedPoints()
    {
        int cloudSize = _segmentedCloud->points.size();

        for (int i = 5; i < cloudSize - 6; ++i){

            float depth1 = _segInfo.segmentedCloudRange[i];
            float depth2 = _segInfo.segmentedCloudRange[i+1];
            int columnDiff = std::abs(int(_segInfo.segmentedCloudColInd[i+1] - _segInfo.segmentedCloudColInd[i]));

            if (columnDiff < 10){

                if (depth1 - depth2 > 0.3){
                    _cloudNeighborPicked[i - 5] = 1;
                    _cloudNeighborPicked[i - 4] = 1;
                    _cloudNeighborPicked[i - 3] = 1;
                    _cloudNeighborPicked[i - 2] = 1;
                    _cloudNeighborPicked[i - 1] = 1;
                    _cloudNeighborPicked[i] = 1;
                }else if (depth2 - depth1 > 0.3){
                    _cloudNeighborPicked[i + 1] = 1;
                    _cloudNeighborPicked[i + 2] = 1;
                    _cloudNeighborPicked[i + 3] = 1;
                    _cloudNeighborPicked[i + 4] = 1;
                    _cloudNeighborPicked[i + 5] = 1;
                    _cloudNeighborPicked[i + 6] = 1;
                }
            }

            float diff1 = std::abs(_segInfo.segmentedCloudRange[i-1] - _segInfo.segmentedCloudRange[i]);
            float diff2 = std::abs(_segInfo.segmentedCloudRange[i+1] - _segInfo.segmentedCloudRange[i]);

            if (diff1 > 0.02 * _segInfo.segmentedCloudRange[i] && diff2 > 0.02 * _segInfo.segmentedCloudRange[i])
                _cloudNeighborPicked[i] = 1;
        }
    }

    void extractFeatures()
    {
        _cornerPointsSharp->clear();
        _cornerPointsLessSharp->clear();
        _surfPointsFlat->clear();
        _surfPointsLessFlat->clear();

        for (int i = 0; i < _N_SCAN; i++) {

            _surfPointsLessFlatScan->clear();

            for (int j = 0; j < 6; j++) {

                int sp = (_segInfo.startRingIndex[i] * (6 - j)    + _segInfo.endRingIndex[i] * j) / 6;
                int ep = (_segInfo.startRingIndex[i] * (5 - j)    + _segInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;

                std::sort(_cloudSmoothness.begin()+sp, _cloudSmoothness.begin()+ep, by_value());

                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--) {
                    int ind = _cloudSmoothness[k].ind;
                    if (_cloudNeighborPicked[ind] == 0 &&
                        _cloudCurvature[ind] > _edgeThreshold &&
                        _segInfo.segmentedCloudGroundFlag[ind] == false) {

                        largestPickedNum++;
                        if (largestPickedNum <= 2) {
                            _cloudLabel[ind] = 2;
                            _cornerPointsSharp->push_back(_segmentedCloud->points[ind]);
                            _cornerPointsLessSharp->push_back(_segmentedCloud->points[ind]);
                        } else if (largestPickedNum <= 20) {
                            _cloudLabel[ind] = 1;
                            _cornerPointsLessSharp->push_back(_segmentedCloud->points[ind]);
                        } else {
                            break;
                        }

                        _cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {
                            int columnDiff = std::abs(int(_segInfo.segmentedCloudColInd[ind + l] - _segInfo.segmentedCloudColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            _cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            int columnDiff = std::abs(int(_segInfo.segmentedCloudColInd[ind + l] - _segInfo.segmentedCloudColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            _cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++) {
                    int ind = _cloudSmoothness[k].ind;
                    if (_cloudNeighborPicked[ind] == 0 &&
                        _cloudCurvature[ind] < _surfThreshold &&
                        _segInfo.segmentedCloudGroundFlag[ind] == true) {

                        _cloudLabel[ind] = -1;
                        _surfPointsFlat->push_back(_segmentedCloud->points[ind]);

                        smallestPickedNum++;
                        if (smallestPickedNum >= 4) {
                            break;
                        }

                        _cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {

                            int columnDiff = std::abs(int(_segInfo.segmentedCloudColInd[ind + l] - _segInfo.segmentedCloudColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            _cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {

                            int columnDiff = std::abs(int(_segInfo.segmentedCloudColInd[ind + l] - _segInfo.segmentedCloudColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            _cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++) {
                    if (_cloudLabel[k] <= 0) {
                        _surfPointsLessFlatScan->push_back(_segmentedCloud->points[k]);
                    }
                }
            }

            _surfPointsLessFlatScanDS->clear();
            _downSizeFilter.setInputCloud(_surfPointsLessFlatScan);
            _downSizeFilter.filter(*_surfPointsLessFlatScanDS);

            *_surfPointsLessFlat += *_surfPointsLessFlatScanDS;
        }
    }

    void publishCloud()
    {
        sensor_msgs::PointCloud2 laserCloudOutMsg;

	    if (_pubCornerPointsSharp.getNumSubscribers() != 0){
	        pcl::toROSMsg(*_cornerPointsSharp, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = _cloudHeader.stamp;
	        // laserCloudOutMsg.header.frame_id = "/camera";
					laserCloudOutMsg.header.frame_id = _frameId;
	        _pubCornerPointsSharp.publish(laserCloudOutMsg);
	    }

	    if (_pubCornerPointsLessSharp.getNumSubscribers() != 0){
	        pcl::toROSMsg(*_cornerPointsLessSharp, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = _cloudHeader.stamp;
	        // laserCloudOutMsg.header.frame_id = "/camera";
					laserCloudOutMsg.header.frame_id = _frameId;
	        _pubCornerPointsLessSharp.publish(laserCloudOutMsg);
	    }

	    if (_pubSurfPointsFlat.getNumSubscribers() != 0){
	        pcl::toROSMsg(*_surfPointsFlat, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = _cloudHeader.stamp;
	        // laserCloudOutMsg.header.frame_id = "/camera";
					laserCloudOutMsg.header.frame_id = _frameId;
	        _pubSurfPointsFlat.publish(laserCloudOutMsg);
	    }

	    if (_pubSurfPointsLessFlat.getNumSubscribers() != 0){
	        pcl::toROSMsg(*_surfPointsLessFlat, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = _cloudHeader.stamp;
	        // laserCloudOutMsg.header.frame_id = "/camera";
					laserCloudOutMsg.header.frame_id = _frameId;
	        _pubSurfPointsLessFlat.publish(laserCloudOutMsg);
	    }
    }



























// laser odometry.
    void TransformToStart(PointType const * const pi, PointType * const po)
    {
        float s = 10 * (pi->intensity - int(pi->intensity));

        // float rx = s * _transformCur[0];
        // float ry = s * _transformCur[1];
        // float rz = s * _transformCur[2];
				float ry = s * _transformCur[0];
        float rz = s * _transformCur[1];
        float rx = s * _transformCur[2];
        float tx = s * _transformCur[3];
        float ty = s * _transformCur[4];
        float tz = s * _transformCur[5];

				// rotate with x axis.
				float y1 = cos(rx) * (pi->y - ty) + sin(rx) * (pi->z - tz);
        float z1 = -sin(rx) * (pi->y - ty) + cos(rx) * (pi->z - tz);
        float x1 = (pi->x - tx);

        // float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
        // float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
        // float z1 = (pi->z - tz);

				// rotate with y axis.
        float y2 = y1;
        float z2 = cos(ry) * z1 + sin(ry) * x1;
        float x2 = -sin(ry) * z1 + cos(ry) * x1;

				// float x2 = x1;
        // float y2 = cos(rx) * y1 + sin(rx) * z1;
        // float z2 = -sin(rx) * y1 + cos(rx) * z1;

				// rotate with z axis.
        po->y = cos(rz) * y2 - sin(rz) * x2;
        po->z = z2;
        po->x = sin(rz) * y2 + cos(rz) * x2;

				// po->x = cos(ry) * x2 - sin(ry) * z2;
        // po->y = y2;
        // po->z = sin(ry) * x2 + cos(ry) * z2;

        po->intensity = pi->intensity;
    }

    void TransformToEnd(PointType const * const pi, PointType * const po)
    {
        float s = 10 * (pi->intensity - int(pi->intensity));

        float ry = s * _transformCur[0];
        float rz = s * _transformCur[1];
        float rx = s * _transformCur[2];
        float tx = s * _transformCur[3];
        float ty = s * _transformCur[4];
        float tz = s * _transformCur[5];

				// rotate with x axis.
        float y1 = cos(rx) * (pi->y - ty) + sin(rx) * (pi->z - tz);
        float z1 = -sin(rx) * (pi->y - ty) + cos(rx) * (pi->z - tz);
        float x1 = (pi->x - tx);

				// float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
        // float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
        // float z1 = (pi->z - tz);

				// rotate with y axis.
        float y2 = y1;
        float z2 = cos(ry) * z1 + sin(ry) * x1;
        float x2 = -sin(ry) * z1 + cos(ry) * x1;

				// float x2 = x1;
        // float y2 = cos(rx) * y1 + sin(rx) * z1;
        // float z2 = -sin(rx) * y1 + cos(rx) * z1;

				// rotate with z axis.
        float y3 = cos(rz) * y2 - sin(rz) * x2;
        float z3 = z2;
        float x3 = sin(rz) * y2 + cos(rz) * x2;

				// float x3 = cos(ry) * x2 - sin(ry) * z2;
        // float y3 = y2;
        // float z3 = sin(ry) * x2 + cos(ry) * z2;

        ry = _transformCur[0];
        rz = _transformCur[1];
        rx = _transformCur[2];
        tx = _transformCur[3];
        ty = _transformCur[4];
        tz = _transformCur[5];

				// rotate with z axis.
				float y4 = cos(rz) * y3 + sin(rz) * x3;
        float z4 = z3;
        float x4 = -sin(rz) * y3 + cos(rz) * x3;

        // float x4 = cos(ry) * x3 + sin(ry) * z3;
        // float y4 = y3;
        // float z4 = -sin(ry) * x3 + cos(ry) * z3;

				// rotate with y axis.
        float y5 = y4;
        float z5 = cos(ry) * z4 - sin(ry) * x4;
        float x5 = sin(ry) * z4 + cos(ry) * x4;

				// float x5 = x4;
        // float y5 = cos(rx) * y4 - sin(rx) * z4;
        // float z5 = sin(rx) * y4 + cos(rx) * z4;

				// rotate with x axis.
        float y6 = cos(rx) * y5 - sin(rx) * z5 + ty;
        float z6 = sin(rx) * y5 + cos(rx) * z5 + tz;
        float x6 = x5 + tx;

				// float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
        // float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
        // float z6 = z5 + tz;

				// rotate with x axis.
        float y7 = _cosImuRollStart * (y6 - _imuShiftFromStartY)
                 - _sinImuRollStart * (z6 - _imuShiftFromStartZ);
        float z7 = _sinImuRollStart * (y6 - _imuShiftFromStartY)
                 + _cosImuRollStart * (z6 - _imuShiftFromStartZ);
        float x7 = x6 - _imuShiftFromStartX;

				// float x7 = cos(_imuRollStart) * (x6 - _imuShiftFromStartX)
        //    			 - sin(_imuRollStart) * (y6 - _imuShiftFromStartY);
  			// float y7 = sin(_imuRollStart) * (x6 - _imuShiftFromStartX)
        //    			 + cos(_imuRollStart) * (y6 - _imuShiftFromStartY);
  			// float z7 = z6 - _imuShiftFromStartZ;

				// rotate with y axis.
        float y8 = y7;
        float z8 = _cosImuPitchStart * z7 - _sinImuPitchStart * x7;
        float x8 = _sinImuPitchStart * z7 + _cosImuPitchStart * x7;

				// float x8 = x7;
				// float y8 = cos(_imuPitchStart) * y7 - sin(_imuPitchStart) * z7;
				// float z8 = sin(_imuPitchStart) * y7 + cos(_imuPitchStart) * z7;

				// rotate with z axis.
        float y9 = _cosImuYawStart * y8 + _sinImuYawStart * x8;
        float z9 = z8;
        float x9 = -_sinImuYawStart * y8 + _cosImuYawStart * x8;

				// float x9 = _cosImuYawStart * x8 + _sinImuYawStart * z8;
        // float y9 = y8;
        // float z9 = -_sinImuYawStart * x8 + _cosImuYawStart * z8;

				// rotate with z axis.
        float y10 = cos(_imuYawLast) * y9 - sin(_imuYawLast) * x9;
        float z10 = z9;
        float x10 = sin(_imuYawLast) * y9 + cos(_imuYawLast) * x9;

				// float x10 = cos(_imuYawLast) * x9 - sin(_imuYawLast) * z9;
        // float y10 = y9;
        // float z10 = sin(_imuYawLast) * x9 + cos(_imuYawLast) * z9;

				// rotate with y axis.
        float y11 = y10;
        float z11 = cos(_imuPitchLast) * z10 + sin(_imuPitchLast) * x10;
        float x11 = -sin(_imuPitchLast) * z10 + cos(_imuPitchLast) * x10;

				// float x11 = x10;
        // float y11 = cos(_imuPitchLast) * y10 + sin(_imuPitchLast) * z10;
        // float z11 = -sin(_imuPitchLast) * y10 + cos(_imuPitchLast) * z10;

				// rotate with x axis.
        po->y = cos(_imuRollLast) * y11 + sin(_imuRollLast) * z11;
        po->z = -sin(_imuRollLast) * y11 + cos(_imuRollLast) * z11;
        po->x = x11;
        po->intensity = int(pi->intensity);
    }

    void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly, float blz,
                           float alx, float aly, float alz, float &acx, float &acy, float &acz)
    {
        float sbcx = sin(bcx);
        float cbcx = cos(bcx);
        float sbcy = sin(bcy);
        float cbcy = cos(bcy);
        float sbcz = sin(bcz);
        float cbcz = cos(bcz);

        float sblx = sin(blx);
        float cblx = cos(blx);
        float sbly = sin(bly);
        float cbly = cos(bly);
        float sblz = sin(blz);
        float cblz = cos(blz);

        float salx = sin(alx);
        float calx = cos(alx);
        float saly = sin(aly);
        float caly = cos(aly);
        float salz = sin(alz);
        float calz = cos(alz);

        float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly)
                  - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
                  - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
                  - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
                  - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
        acx = -asin(srx);

        float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
                     - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
                     - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
                     - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz)
                     + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
        float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
                     - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz)
                     - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
                     - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
                     + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
        acy = atan2(srycrx / cos(acx), crycrx / cos(acx));

        float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz)
                     - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx)
                     - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly)
                     + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx)
                     - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz
                     + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz)
                     + calx*cblx*salz*sblz);
        float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly)
                     - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx)
                     + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                     + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly)
                     + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly
                     - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz)
                     - calx*calz*cblx*sblz);
        acz = atan2(srzcrx / cos(acx), crzcrx / cos(acx));
    }

    void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz,
                            float &ox, float &oy, float &oz)
    {
        float srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
        ox = -asin(srx);

        float srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz)
                     + sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
        float crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy)
                     - cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
        oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

        float srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz)
                     + sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
        float crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz)
                     - cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
        oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
    }

    double rad2deg(double radians)
    {
        return radians * 180.0 / M_PI;
    }

    double deg2rad(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    void findCorrespondingCornerFeatures(int iterCount){

        int cornerPointsSharpNum = _cornerPointsSharp->points.size();

        for (int i = 0; i < cornerPointsSharpNum; i++) {

            TransformToStart(&_cornerPointsSharp->points[i], &_pointSel);

            if (iterCount % 5 == 0) {

                _kdtreeCornerLast->nearestKSearch(_pointSel, 1, _pointSearchInd, _pointSearchSqDis);
                int closestPointInd = -1, minPointInd2 = -1;

                if (_pointSearchSqDis[0] < _nearestFeatureSearchSqDist) {
                    closestPointInd = _pointSearchInd[0];
                    int closestPointScan = int(_laserCloudCornerLast->points[closestPointInd].intensity);

                    float pointSqDis, minPointSqDis2 = _nearestFeatureSearchSqDist;
                    for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
                        if (int(_laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5) {
                            break;
                        }

                        pointSqDis = (_laserCloudCornerLast->points[j].x - _pointSel.x) *
                                     (_laserCloudCornerLast->points[j].x - _pointSel.x) +
                                     (_laserCloudCornerLast->points[j].y - _pointSel.y) *
                                     (_laserCloudCornerLast->points[j].y - _pointSel.y) +
                                     (_laserCloudCornerLast->points[j].z - _pointSel.z) *
                                     (_laserCloudCornerLast->points[j].z - _pointSel.z);

                        if (int(_laserCloudCornerLast->points[j].intensity) > closestPointScan) {
                            if (pointSqDis < minPointSqDis2) {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }
                    }
                    for (int j = closestPointInd - 1; j >= 0; j--) {
                        if (int(_laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5) {
                            break;
                        }

                        pointSqDis = (_laserCloudCornerLast->points[j].x - _pointSel.x) *
                                     (_laserCloudCornerLast->points[j].x - _pointSel.x) +
                                     (_laserCloudCornerLast->points[j].y - _pointSel.y) *
                                     (_laserCloudCornerLast->points[j].y - _pointSel.y) +
                                     (_laserCloudCornerLast->points[j].z - _pointSel.z) *
                                     (_laserCloudCornerLast->points[j].z - _pointSel.z);

                        if (int(_laserCloudCornerLast->points[j].intensity) < closestPointScan) {
                            if (pointSqDis < minPointSqDis2) {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }
                    }
                }

                _pointSearchCornerInd1[i] = closestPointInd;
                _pointSearchCornerInd2[i] = minPointInd2;
            }

            if (_pointSearchCornerInd2[i] >= 0) {

                _tripod1 = _laserCloudCornerLast->points[_pointSearchCornerInd1[i]];
                _tripod2 = _laserCloudCornerLast->points[_pointSearchCornerInd2[i]];

                float x0 = _pointSel.x;
                float y0 = _pointSel.y;
                float z0 = _pointSel.z;
                float x1 = _tripod1.x;
                float y1 = _tripod1.y;
                float z1 = _tripod1.z;
                float x2 = _tripod2.x;
                float y2 = _tripod2.y;
                float z2 = _tripod2.z;

                float m11 = ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1));
                float m22 = ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1));
                float m33 = ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1));

                float a012 = sqrt(m11 * m11  + m22 * m22 + m33 * m33);

                float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                float la =  ((y1 - y2)*m11 + (z1 - z2)*m22) / a012 / l12;

                float lb = -((x1 - x2)*m11 - (z1 - z2)*m33) / a012 / l12;

                float lc = -((x1 - x2)*m22 + (y1 - y2)*m33) / a012 / l12;

                float ld2 = a012 / l12;

                float s = 1;
                if (iterCount >= 5) {
                    s = 1 - 1.8 * fabs(ld2);
                }

                if (s > 0.1 && ld2 != 0) {
                    _coeff.x = s * la;
                    _coeff.y = s * lb;
                    _coeff.z = s * lc;
                    _coeff.intensity = s * ld2;

                    _laserCloudOri->push_back(_cornerPointsSharp->points[i]);
                    _coeffSel->push_back(_coeff);
                }
            }
        }
    }

    void findCorrespondingSurfFeatures(int iterCount){

        int surfPointsFlatNum = _surfPointsFlat->points.size();

        for (int i = 0; i < surfPointsFlatNum; i++) {

            TransformToStart(&_surfPointsFlat->points[i], &_pointSel);

            if (iterCount % 5 == 0) {

                _kdtreeSurfLast->nearestKSearch(_pointSel, 1, _pointSearchInd, _pointSearchSqDis);
                int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

                if (_pointSearchSqDis[0] < _nearestFeatureSearchSqDist) {
                    closestPointInd = _pointSearchInd[0];
                    int closestPointScan = int(_laserCloudSurfLast->points[closestPointInd].intensity);

                    float pointSqDis, minPointSqDis2 = _nearestFeatureSearchSqDist, minPointSqDis3 = _nearestFeatureSearchSqDist;
                    for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
                        if (int(_laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5) {
                            break;
                        }

                        pointSqDis = (_laserCloudSurfLast->points[j].x - _pointSel.x) *
                                     (_laserCloudSurfLast->points[j].x - _pointSel.x) +
                                     (_laserCloudSurfLast->points[j].y - _pointSel.y) *
                                     (_laserCloudSurfLast->points[j].y - _pointSel.y) +
                                     (_laserCloudSurfLast->points[j].z - _pointSel.z) *
                                     (_laserCloudSurfLast->points[j].z - _pointSel.z);

                        if (int(_laserCloudSurfLast->points[j].intensity) <= closestPointScan) {
                            if (pointSqDis < minPointSqDis2) {
                              minPointSqDis2 = pointSqDis;
                              minPointInd2 = j;
                            }
                        } else {
                            if (pointSqDis < minPointSqDis3) {
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }
                    }
                    for (int j = closestPointInd - 1; j >= 0; j--) {
                        if (int(_laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5) {
                            break;
                        }

                        pointSqDis = (_laserCloudSurfLast->points[j].x - _pointSel.x) *
                                     (_laserCloudSurfLast->points[j].x - _pointSel.x) +
                                     (_laserCloudSurfLast->points[j].y - _pointSel.y) *
                                     (_laserCloudSurfLast->points[j].y - _pointSel.y) +
                                     (_laserCloudSurfLast->points[j].z - _pointSel.z) *
                                     (_laserCloudSurfLast->points[j].z - _pointSel.z);

                        if (int(_laserCloudSurfLast->points[j].intensity) >= closestPointScan) {
                            if (pointSqDis < minPointSqDis2) {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        } else {
                            if (pointSqDis < minPointSqDis3) {
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }
                    }
                }

                _pointSearchSurfInd1[i] = closestPointInd;
                _pointSearchSurfInd2[i] = minPointInd2;
                _pointSearchSurfInd3[i] = minPointInd3;
            }

            if (_pointSearchSurfInd2[i] >= 0 && _pointSearchSurfInd3[i] >= 0) {

                _tripod1 = _laserCloudSurfLast->points[_pointSearchSurfInd1[i]];
                _tripod2 = _laserCloudSurfLast->points[_pointSearchSurfInd2[i]];
                _tripod3 = _laserCloudSurfLast->points[_pointSearchSurfInd3[i]];

                float pa = (_tripod2.y - _tripod1.y) * (_tripod3.z - _tripod1.z)
                         - (_tripod3.y - _tripod1.y) * (_tripod2.z - _tripod1.z);
                float pb = (_tripod2.z - _tripod1.z) * (_tripod3.x - _tripod1.x)
                         - (_tripod3.z - _tripod1.z) * (_tripod2.x - _tripod1.x);
                float pc = (_tripod2.x - _tripod1.x) * (_tripod3.y - _tripod1.y)
                         - (_tripod3.x - _tripod1.x) * (_tripod2.y - _tripod1.y);
                float pd = -(pa * _tripod1.x + pb * _tripod1.y + pc * _tripod1.z);

                float ps = sqrt(pa * pa + pb * pb + pc * pc);

                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                float pd2 = pa * _pointSel.x + pb * _pointSel.y + pc * _pointSel.z + pd;

                float s = 1;
                if (iterCount >= 5) {
                    s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(_pointSel.x * _pointSel.x
                            + _pointSel.y * _pointSel.y + _pointSel.z * _pointSel.z));
                }

                if (s > 0.1 && pd2 != 0) {
                    _coeff.x = s * pa;
                    _coeff.y = s * pb;
                    _coeff.z = s * pc;
                    _coeff.intensity = s * pd2;

                    _laserCloudOri->push_back(_surfPointsFlat->points[i]);
                    _coeffSel->push_back(_coeff);
                }
            }
        }
    }

    bool calculateTransformationSurf(int iterCount){

        int pointSelNum = _laserCloudOri->points.size();

        cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

        float srx = sin(_transformCur[0]);
        float crx = cos(_transformCur[0]);
        float sry = sin(_transformCur[1]);
        float cry = cos(_transformCur[1]);
        float srz = sin(_transformCur[2]);
        float crz = cos(_transformCur[2]);
				// keep original
				// float sry = sin(_transformCur[0]);
        // float cry = cos(_transformCur[0]);
        // float srz = sin(_transformCur[1]);
        // float crz = cos(_transformCur[1]);
        // float srx = sin(_transformCur[2]);
        // float crx = cos(_transformCur[2]);
        float tx = _transformCur[3];
        float ty = _transformCur[4];
        float tz = _transformCur[5];

				float a1 = crx*sry*srz; float a2 = crx*crz*sry; float a3 = srx*sry; float a4 = ty*a1 - tz*a2 - tx*a3;
        float a5 = srx*srz; float a6 = crz*srx; float a7 = tz*a6 - tx*crx - ty*a5;
        float a8 = crx*cry*srz; float a9 = crx*cry*crz; float a10 = cry*srx; float a11 = tx*a10 + tz*a9 - ty*a8;

        float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz;
        float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry;

        float c1 = -b6; float c2 = b5; float c3 = ty*b6 - tz*b5; float c4 = -crx*crz; float c5 = crx*srz; float c6 = tz*c5 + ty*-c4;
        float c7 = b2; float c8 = -b1; float c9 = ty*-b2 - tz*-b1;

        // float a1 = crx*sry*srz; float a2 = crx*crz*sry; float a3 = srx*sry; float a4 = tx*a1 - ty*a2 - tz*a3;
        // float a5 = srx*srz; float a6 = crz*srx; float a7 = ty*a6 - tz*crx - tx*a5;
        // float a8 = crx*cry*srz; float a9 = crx*cry*crz; float a10 = cry*srx; float a11 = tz*a10 + ty*a9 - tx*a8;
				//
        // float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz;
        // float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry;
				//
        // float c1 = -b6; float c2 = b5; float c3 = tx*b6 - ty*b5; float c4 = -crx*crz; float c5 = crx*srz; float c6 = ty*c5 + tx*-c4;
        // float c7 = b2; float c8 = -b1; float c9 = tx*-b2 - ty*-b1;

        for (int i = 0; i < pointSelNum; i++) {

            _pointOri = _laserCloudOri->points[i];
            _coeff = _coeffSel->points[i];

            // float arx = (-a1*_pointOri.x + a2*_pointOri.y + a3*_pointOri.z + a4) * _coeff.x
            //           + (a5*_pointOri.x - a6*_pointOri.y + crx*_pointOri.z + a7) * _coeff.y
            //           + (a8*_pointOri.x - a9*_pointOri.y - a10*_pointOri.z + a11) * _coeff.z;
						//
            // float arz = (c1*_pointOri.x + c2*_pointOri.y + c3) * _coeff.x
            //           + (c4*_pointOri.x - c5*_pointOri.y + c6) * _coeff.y
            //           + (c7*_pointOri.x + c8*_pointOri.y + c9) * _coeff.z;
						//
            // float aty = -b6 * _coeff.x + c4 * _coeff.y + b2 * _coeff.z;

						float ary = (-a1*_pointOri.y + a2*_pointOri.z + a3*_pointOri.x + a4) * _coeff.y
                      + (a5*_pointOri.y - a6*_pointOri.z + crx*_pointOri.x + a7) * _coeff.z
                      + (a8*_pointOri.y - a9*_pointOri.z - a10*_pointOri.x + a11) * _coeff.x;

            float arx = (c1*_pointOri.y + c2*_pointOri.z + c3) * _coeff.y
                      + (c4*_pointOri.y - c5*_pointOri.z + c6) * _coeff.z
                      + (c7*_pointOri.y + c8*_pointOri.z + c9) * _coeff.x;

            float atz = -b6 * _coeff.y + c4 * _coeff.z + b2 * _coeff.x;

            float d2 = _coeff.intensity;

            // matA.at<float>(i, 0) = arx;
            // matA.at<float>(i, 1) = arz;
            // matA.at<float>(i, 2) = aty;

						// warning!
						matA.at<float>(i, 0) = ary;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = atz;
            matB.at<float>(i, 0) = -0.05 * d2;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            _isDegenerate = false;
            float eignThre[3] = {10, 10, 10};
            for (int i = 2; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 3; j++) {
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
            cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = _matP * matX2;
        }

        _transformCur[0] += matX.at<float>(0, 0);
        _transformCur[2] += matX.at<float>(1, 0);
        // _transformCur[4] += matX.at<float>(2, 0);
				_transformCur[5] += matX.at<float>(2, 0);

        for(int i=0; i<6; i++){
            if(isnan(_transformCur[i]))
                _transformCur[i]=0;
        }

        float deltaR = sqrt(
                            pow(rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(rad2deg(matX.at<float>(1, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(2, 0) * 100, 2));

        if (deltaR < 0.1 && deltaT < 0.1) {
            return false;
        }
        return true;
    }

    bool calculateTransformationCorner(int iterCount){

        int pointSelNum = _laserCloudOri->points.size();

        cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

        float srx = sin(_transformCur[0]);
        float crx = cos(_transformCur[0]);
        float sry = sin(_transformCur[1]);
        float cry = cos(_transformCur[1]);
        float srz = sin(_transformCur[2]);
        float crz = cos(_transformCur[2]);
				// keep original
				// float sry = sin(_transformCur[0]);
        // float cry = cos(_transformCur[0]);
        // float srz = sin(_transformCur[1]);
        // float crz = cos(_transformCur[1]);
        // float srx = sin(_transformCur[2]);
        // float crx = cos(_transformCur[2]);
        float tx = _transformCur[3];
        float ty = _transformCur[4];
        float tz = _transformCur[5];

        // float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz; float b3 = crx*cry; float b4 = tx*-b1 + ty*-b2 + tz*b3;
        // float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry; float b7 = crx*sry; float b8 = tz*b7 - ty*b6 - tx*b5;

				float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz; float b3 = crx*cry; float b4 = ty*-b1 + tz*-b2 + tx*b3;
        float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry; float b7 = crx*sry; float b8 = tx*b7 - tz*b6 - ty*b5;

        float c5 = crx*srz;

        for (int i = 0; i < pointSelNum; i++) {

            _pointOri = _laserCloudOri->points[i];
            _coeff = _coeffSel->points[i];

            // float ary = (b1*_pointOri.x + b2*_pointOri.y - b3*_pointOri.z + b4) * _coeff.x
            //           + (b5*_pointOri.x + b6*_pointOri.y - b7*_pointOri.z + b8) * _coeff.z;
						//
            // float atx = -b5 * _coeff.x + c5 * _coeff.y + b1 * _coeff.z;
						//
            // float atz = b7 * _coeff.x - srx * _coeff.y - b3 * _coeff.z;

						float arz = (b1*_pointOri.y + b2*_pointOri.z - b3*_pointOri.x + b4) * _coeff.y
                      + (b5*_pointOri.y + b6*_pointOri.z - b7*_pointOri.x + b8) * _coeff.x;

            float aty = -b5 * _coeff.y + c5 * _coeff.z + b1 * _coeff.x;

            float atx = b7 * _coeff.y - srx * _coeff.z - b3 * _coeff.x;

            float d2 = _coeff.intensity;

            // matA.at<float>(i, 0) = ary;
            // matA.at<float>(i, 1) = atx;
            // matA.at<float>(i, 2) = atz;

						matA.at<float>(i, 0) = arz;
						matA.at<float>(i, 1) = aty;
						matA.at<float>(i, 2) = atx;

            matB.at<float>(i, 0) = -0.05 * d2;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            _isDegenerate = false;
            float eignThre[3] = {10, 10, 10};
            for (int i = 2; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 3; j++) {
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
            cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = _matP * matX2;
        }

        _transformCur[1] += matX.at<float>(0, 0);
        // _transformCur[3] += matX.at<float>(1, 0);
        // _transformCur[5] += matX.at<float>(2, 0);
				_transformCur[4] += matX.at<float>(1, 0);
        _transformCur[3] += matX.at<float>(2, 0);

        for(int i=0; i<6; i++){
            if(isnan(_transformCur[i]))
                _transformCur[i]=0;
        }

        float deltaR = sqrt(
                            pow(rad2deg(matX.at<float>(0, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(1, 0) * 100, 2) +
                            pow(matX.at<float>(2, 0) * 100, 2));

        if (deltaR < 0.1 && deltaT < 0.1) {
            return false;
        }
        return true;
    }

    bool calculateTransformation(int iterCount){

        int pointSelNum = _laserCloudOri->points.size();

        cv::Mat matA(pointSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, pointSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        float srx = sin(_transformCur[0]);
        float crx = cos(_transformCur[0]);
        float sry = sin(_transformCur[1]);
        float cry = cos(_transformCur[1]);
        float srz = sin(_transformCur[2]);
        float crz = cos(_transformCur[2]);
				// keep original
				// float sry = sin(_transformCur[0]);
        // float cry = cos(_transformCur[0]);
        // float srz = sin(_transformCur[1]);
        // float crz = cos(_transformCur[1]);
        // float srx = sin(_transformCur[2]);
        // float crx = cos(_transformCur[2]);
        float tx = _transformCur[3];
        float ty = _transformCur[4];
        float tz = _transformCur[5];

				// float a1 = crx*sry*srz; float a2 = crx*crz*sry; float a3 = srx*sry; float a4 = tx*a1 - ty*a2 - tz*a3;
        // float a5 = srx*srz; float a6 = crz*srx; float a7 = ty*a6 - tz*crx - tx*a5;
        // float a8 = crx*cry*srz; float a9 = crx*cry*crz; float a10 = cry*srx; float a11 = tz*a10 + ty*a9 - tx*a8;
				//
        // float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz; float b3 = crx*cry; float b4 = tx*-b1 + ty*-b2 + tz*b3;
        // float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry; float b7 = crx*sry; float b8 = tz*b7 - ty*b6 - tx*b5;
				//
        // float c1 = -b6; float c2 = b5; float c3 = tx*b6 - ty*b5; float c4 = -crx*crz; float c5 = crx*srz; float c6 = ty*c5 + tx*-c4;
        // float c7 = b2; float c8 = -b1; float c9 = tx*-b2 - ty*-b1;

        float a1 = crx*sry*srz; float a2 = crx*crz*sry; float a3 = srx*sry; float a4 = ty*a1 - tz*a2 - tx*a3;
        float a5 = srx*srz; float a6 = crz*srx; float a7 = tz*a6 - tx*crx - ty*a5;
        float a8 = crx*cry*srz; float a9 = crx*cry*crz; float a10 = cry*srx; float a11 = tx*a10 + tz*a9 - ty*a8;

        float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz; float b3 = crx*cry; float b4 = ty*-b1 + tz*-b2 + tx*b3;
        float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry; float b7 = crx*sry; float b8 = tx*b7 - tz*b6 - ty*b5;

        float c1 = -b6; float c2 = b5; float c3 = ty*b6 - tz*b5; float c4 = -crx*crz; float c5 = crx*srz; float c6 = tz*c5 + ty*-c4;
        float c7 = b2; float c8 = -b1; float c9 = ty*-b2 - tz*-b1;

        for (int i = 0; i < pointSelNum; i++) {

            _pointOri = _laserCloudOri->points[i];
            _coeff = _coeffSel->points[i];

            // float arx = (-a1*_pointOri.x + a2*_pointOri.y + a3*_pointOri.z + a4) * _coeff.x
            //           + (a5*_pointOri.x - a6*_pointOri.y + crx*_pointOri.z + a7) * _coeff.y
            //           + (a8*_pointOri.x - a9*_pointOri.y - a10*_pointOri.z + a11) * _coeff.z;
						//
            // float ary = (b1*_pointOri.x + b2*_pointOri.y - b3*_pointOri.z + b4) * _coeff.x
            //           + (b5*_pointOri.x + b6*_pointOri.y - b7*_pointOri.z + b8) * _coeff.z;
						//
            // float arz = (c1*_pointOri.x + c2*_pointOri.y + c3) * _coeff.x
            //           + (c4*_pointOri.x - c5*_pointOri.y + c6) * _coeff.y
            //           + (c7*_pointOri.x + c8*_pointOri.y + c9) * _coeff.z;

						float ary = (-a1*_pointOri.y + a2*_pointOri.z + a3*_pointOri.x + a4) * _coeff.y
                      + (a5*_pointOri.y - a6*_pointOri.z + crx*_pointOri.x + a7) * _coeff.z
                      + (a8*_pointOri.y - a9*_pointOri.z - a10*_pointOri.x + a11) * _coeff.x;

            float arz = (b1*_pointOri.y + b2*_pointOri.z - b3*_pointOri.x + b4) * _coeff.y
                      + (b5*_pointOri.y + b6*_pointOri.z - b7*_pointOri.x + b8) * _coeff.x;

            float arx = (c1*_pointOri.y + c2*_pointOri.z + c3) * _coeff.y
                      + (c4*_pointOri.y - c5*_pointOri.z + c6) * _coeff.z
                      + (c7*_pointOri.y + c8*_pointOri.z + c9) * _coeff.x;

						// float atx = -b5 * _coeff.y + c5 * _coeff.z + b1 * _coeff.x;
						//
            // float aty = -b6 * _coeff.y + c4 * _coeff.z + b2 * _coeff.x;
						//
            // float atz = b7 * _coeff.y - srx * _coeff.z - b3 * _coeff.x;

            float aty = -b5 * _coeff.y + c5 * _coeff.z + b1 * _coeff.x;

            float atz = -b6 * _coeff.y + c4 * _coeff.z + b2 * _coeff.x;

            float atx = b7 * _coeff.y - srx * _coeff.z - b3 * _coeff.x;

            float d2 = _coeff.intensity;

            // matA.at<float>(i, 0) = arx;
            // matA.at<float>(i, 1) = ary;
            // matA.at<float>(i, 2) = arz;

						matA.at<float>(i, 0) = ary;
            matA.at<float>(i, 1) = arz;
            matA.at<float>(i, 2) = arx;
            matA.at<float>(i, 3) = aty;
            matA.at<float>(i, 4) = atz;
            matA.at<float>(i, 5) = atx;
            matB.at<float>(i, 0) = -0.05 * d2;
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
            float eignThre[6] = {10, 10, 10, 10, 10, 10};
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

        _transformCur[0] += matX.at<float>(0, 0);
        _transformCur[1] += matX.at<float>(1, 0);
        _transformCur[2] += matX.at<float>(2, 0);
        _transformCur[4] += matX.at<float>(3, 0);
        _transformCur[5] += matX.at<float>(4, 0);
        _transformCur[3] += matX.at<float>(5, 0);

        for(int i=0; i<6; i++){
            if(isnan(_transformCur[i]))
                _transformCur[i]=0;
        }

        float deltaR = sqrt(
                            pow(rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.1 && deltaT < 0.1) {
            return false;
        }
        return true;
    }

    void checkSystemInitialization(){

        pcl::PointCloud<PointType>::Ptr laserCloudTemp = _cornerPointsLessSharp;
        _cornerPointsLessSharp = _laserCloudCornerLast;
        _laserCloudCornerLast = laserCloudTemp;

        laserCloudTemp = _surfPointsLessFlat;
        _surfPointsLessFlat = _laserCloudSurfLast;
        _laserCloudSurfLast = laserCloudTemp;

        _kdtreeCornerLast->setInputCloud(_laserCloudCornerLast);
        _kdtreeSurfLast->setInputCloud(_laserCloudSurfLast);

        _laserCloudCornerLastNum = _laserCloudCornerLast->points.size();
        _laserCloudSurfLastNum = _laserCloudSurfLast->points.size();

        sensor_msgs::PointCloud2 laserCloudCornerLast2;
        pcl::toROSMsg(*_laserCloudCornerLast, laserCloudCornerLast2);
        laserCloudCornerLast2.header.stamp = _cloudHeader.stamp;
        // laserCloudCornerLast2.header.frame_id = "/camera";
				laserCloudCornerLast2.header.frame_id = _frameId;
        _pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

        sensor_msgs::PointCloud2 laserCloudSurfLast2;
        pcl::toROSMsg(*_laserCloudSurfLast, laserCloudSurfLast2);
        laserCloudSurfLast2.header.stamp = _cloudHeader.stamp;
        // laserCloudSurfLast2.header.frame_id = "/camera";
				laserCloudSurfLast2.header.frame_id = _frameId;
        _pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

        _transformSum[0] += _imuPitchStart;
        _transformSum[2] += _imuRollStart;

        _systemInitedLM = true;
    }

    void updateInitialGuess(){

        _imuPitchLast = _imuPitchCur;
        _imuYawLast = _imuYawCur;
        _imuRollLast = _imuRollCur;

        _imuShiftFromStartX = _imuShiftFromStartXCur;
        _imuShiftFromStartY = _imuShiftFromStartYCur;
        _imuShiftFromStartZ = _imuShiftFromStartZCur;

        _imuVeloFromStartX = _imuVeloFromStartXCur;
        _imuVeloFromStartY = _imuVeloFromStartYCur;
        _imuVeloFromStartZ = _imuVeloFromStartZCur;

        if (_imuAngularFromStartX != 0 || _imuAngularFromStartY != 0 || _imuAngularFromStartZ != 0){
            _transformCur[0] = - _imuAngularFromStartY;
            _transformCur[1] = - _imuAngularFromStartZ;
            _transformCur[2] = - _imuAngularFromStartX;
        }

        if (_imuVeloFromStartX != 0 || _imuVeloFromStartY != 0 || _imuVeloFromStartZ != 0){
            _transformCur[3] -= _imuVeloFromStartX * _scanPeriod;
            _transformCur[4] -= _imuVeloFromStartY * _scanPeriod;
            _transformCur[5] -= _imuVeloFromStartZ * _scanPeriod;
        }
    }

    void updateTransformation(){

        if (_laserCloudCornerLastNum < 10 || _laserCloudSurfLastNum < 100)
            return;

        for (int iterCount1 = 0; iterCount1 < 25; iterCount1++) {
            _laserCloudOri->clear();
            _coeffSel->clear();

            findCorrespondingSurfFeatures(iterCount1);

            if (_laserCloudOri->points.size() < 10)
                continue;
            if (calculateTransformationSurf(iterCount1) == false)
                break;
        }

        for (int iterCount2 = 0; iterCount2 < 25; iterCount2++) {

            _laserCloudOri->clear();
            _coeffSel->clear();

            findCorrespondingCornerFeatures(iterCount2);

            if (_laserCloudOri->points.size() < 10)
                continue;
            if (calculateTransformationCorner(iterCount2) == false)
                break;
        }
    }

    void integrateTransformation(){
        float rx, ry, rz, tx, ty, tz;
        // AccumulateRotation(_transformSum[0], _transformSum[1], _transformSum[2],
        //                    -_transformCur[0], -_transformCur[1], -_transformCur[2], rx, ry, rz);
			  AccumulateRotation(_transformSum[0], _transformSum[1], _transformSum[2],
                           -_transformCur[0], -_transformCur[1], -_transformCur[2], ry, rz, rx);

				// rotate with x axis.
        float y1 = cos(rx) * (_transformCur[4] - _imuShiftFromStartY)
                 - sin(rx) * (_transformCur[5] - _imuShiftFromStartZ);
        float z1 = sin(rx) * (_transformCur[4] - _imuShiftFromStartY)
                 + cos(rx) * (_transformCur[5] - _imuShiftFromStartZ);
        float x1 = _transformCur[3] - _imuShiftFromStartX;

				// float x1 = cos(rz) * (_transformCur[3] - _imuShiftFromStartX)
        //          - sin(rz) * (_transformCur[4] - _imuShiftFromStartY);
        // float y1 = sin(rz) * (_transformCur[3] - _imuShiftFromStartX)
        //          + cos(rz) * (_transformCur[4] - _imuShiftFromStartY);
        // float z1 = _transformCur[5] - _imuShiftFromStartZ;

				// rotate with y axis.
        float y2 = y1;
        float z2 = cos(ry) * z1 - sin(ry) * x1;
        float x2 = sin(ry) * z1 + cos(ry) * x1;

				// float x2 = x1;
        // float y2 = cos(rx) * y1 - sin(rx) * z1;
        // float z2 = sin(rx) * y1 + cos(rx) * z1;

				// rotate with z axis.
        ty = _transformSum[4] - (cos(rz) * y2 + sin(rz) * x2);
        tz = _transformSum[5] - z2;
        tx = _transformSum[3] - (-sin(rz) * y2 + cos(rz) * x2);

				// tx = _transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
        // ty = _transformSum[4] - y2;
        // tz = _transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

        // PluginIMURotation(rx, ry, rz, _imuPitchStart, _imuYawStart, _imuRollStart,
        //                   _imuPitchLast, _imuYawLast, _imuRollLast, rx, ry, rz);

				PluginIMURotation(ry, rz, rx, _imuPitchStart, _imuYawStart, _imuRollStart,
                          _imuPitchLast, _imuYawLast, _imuRollLast, ry, rz, rx);

        _transformSum[0] = ry;
        _transformSum[1] = rz;
        _transformSum[2] = rx;
        _transformSum[3] = tx;
        _transformSum[4] = ty;
        _transformSum[5] = tz;

				// _transformSum[0] = rx;
        // _transformSum[1] = ry;
        // _transformSum[2] = rz;
        // _transformSum[3] = tx;
        // _transformSum[4] = ty;
        // _transformSum[5] = tz;
    }

    void publishOdometry(){
        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(_transformSum[2], -_transformSum[0], -_transformSum[1]);

        _laserOdometry.header.stamp = _cloudHeader.stamp;
        // _laserOdometry.pose.pose.orientation.x = -geoQuat.y;
        // _laserOdometry.pose.pose.orientation.y = -geoQuat.z;
        // _laserOdometry.pose.pose.orientation.z = geoQuat.x;

				_laserOdometry.pose.pose.orientation.y = -geoQuat.y;
        _laserOdometry.pose.pose.orientation.z = -geoQuat.z;
        _laserOdometry.pose.pose.orientation.x = geoQuat.x;
        _laserOdometry.pose.pose.orientation.w = geoQuat.w;
        _laserOdometry.pose.pose.position.x = _transformSum[3];
        _laserOdometry.pose.pose.position.y = _transformSum[4];
        _laserOdometry.pose.pose.position.z = _transformSum[5];
        _pubLaserOdometry.publish(_laserOdometry);

        // _laserOdometryTrans.stamp_ = _cloudHeader.stamp;
        // // _laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
				// _laserOdometryTrans.setRotation(tf::Quaternion(geoQuat.x, -geoQuat.y, -geoQuat.z, geoQuat.w));
        // _laserOdometryTrans.setOrigin(tf::Vector3(_transformSum[3], _transformSum[4], _transformSum[5]));
        // _tfBroadcaster.sendTransform(_laserOdometryTrans);
    }

    void adjustOutlierCloud(){
        PointType point;
        int cloudSize = _outlierCloud->points.size();
        for (int i = 0; i < cloudSize; ++i)
        {
            // point.x = _outlierCloud->points[i].y;
            // point.y = _outlierCloud->points[i].z;
            // point.z = _outlierCloud->points[i].x;

						point.x = _outlierCloud->points[i].x;
            point.y = _outlierCloud->points[i].y;
            point.z = _outlierCloud->points[i].z;
            point.intensity = _outlierCloud->points[i].intensity;
            _outlierCloud->points[i] = point;
        }
    }

    void publishCloudsLast(){

        updateImuRollPitchYawStartSinCos();

        int cornerPointsLessSharpNum = _cornerPointsLessSharp->points.size();
        for (int i = 0; i < cornerPointsLessSharpNum; i++) {
            TransformToEnd(&_cornerPointsLessSharp->points[i], &_cornerPointsLessSharp->points[i]);
        }


        int surfPointsLessFlatNum = _surfPointsLessFlat->points.size();
        for (int i = 0; i < surfPointsLessFlatNum; i++) {
            TransformToEnd(&_surfPointsLessFlat->points[i], &_surfPointsLessFlat->points[i]);
        }

        pcl::PointCloud<PointType>::Ptr laserCloudTemp = _cornerPointsLessSharp;
        _cornerPointsLessSharp = _laserCloudCornerLast;
        _laserCloudCornerLast = laserCloudTemp;

        laserCloudTemp = _surfPointsLessFlat;
        _surfPointsLessFlat = _laserCloudSurfLast;
        _laserCloudSurfLast = laserCloudTemp;

        _laserCloudCornerLastNum = _laserCloudCornerLast->points.size();
        _laserCloudSurfLastNum = _laserCloudSurfLast->points.size();

        if (_laserCloudCornerLastNum > 10 && _laserCloudSurfLastNum > 100) {
            _kdtreeCornerLast->setInputCloud(_laserCloudCornerLast);
            _kdtreeSurfLast->setInputCloud(_laserCloudSurfLast);
        }

        _frameCount++;

        if (_frameCount >= _skipFrameNum + 1) {

            _frameCount = 0;

            adjustOutlierCloud();
            sensor_msgs::PointCloud2 outlierCloudLast2;
            pcl::toROSMsg(*_outlierCloud, outlierCloudLast2);
            outlierCloudLast2.header.stamp = _cloudHeader.stamp;
            // outlierCloudLast2.header.frame_id = "/camera";
						outlierCloudLast2.header.frame_id = _frameId;
            _pubOutlierCloudLast.publish(outlierCloudLast2);

            sensor_msgs::PointCloud2 laserCloudCornerLast2;
            pcl::toROSMsg(*_laserCloudCornerLast, laserCloudCornerLast2);
            laserCloudCornerLast2.header.stamp = _cloudHeader.stamp;
            // laserCloudCornerLast2.header.frame_id = "/camera";
						laserCloudCornerLast2.header.frame_id = _frameId;
            _pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

            sensor_msgs::PointCloud2 laserCloudSurfLast2;
            pcl::toROSMsg(*_laserCloudSurfLast, laserCloudSurfLast2);
            laserCloudSurfLast2.header.stamp = _cloudHeader.stamp;
            // laserCloudSurfLast2.header.frame_id = "/camera";
						laserCloudSurfLast2.header.frame_id = _frameId;
            _pubLaserCloudSurfLast.publish(laserCloudSurfLast2);
        }
    }

    void runFeatureAssociation()
    {

        if (_newSegmentedCloud && _newSegmentedCloudInfo && _newOutlierCloud &&
            std::abs(_timeNewSegmentedCloudInfo - _timeNewSegmentedCloud) < 0.05 &&
            std::abs(_timeNewOutlierCloud - _timeNewSegmentedCloud) < 0.05){

            _newSegmentedCloud = false;
            _newSegmentedCloudInfo = false;
            _newOutlierCloud = false;
        }else{
            return;
        }
        /**
        	1. Feature Extraction
        */
        adjustDistortion();

        calculateSmoothness();

        markOccludedPoints();

        extractFeatures();

        publishCloud(); // cloud for visualization

        /**
		2. Feature Association
        */
        if (!_systemInitedLM) {
            checkSystemInitialization();
            return;
        }

        updateInitialGuess();

        updateTransformation();

        integrateTransformation();

        publishOdometry();

        publishCloudsLast(); // cloud to mapOptimization
    }
};




int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");

    ROS_INFO("\033[1;32m---->\033[0m Feature Association Started.");

    FeatureAssociation FA;

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        FA.runFeatureAssociation();

        rate.sleep();
    }

    ros::spin();
    return 0;
}
