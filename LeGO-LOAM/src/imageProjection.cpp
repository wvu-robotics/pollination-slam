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

#include "utility.h"


class ImageProjection{
private:

    ros::NodeHandle nh;

    ros::Subscriber _subLaserCloud;

    ros::Publisher _pubFullCloud;
    ros::Publisher _pubFullInfoCloud;

    ros::Publisher _pubGroundCloud;
    ros::Publisher _pubSegmentedCloud;
    ros::Publisher _pubSegmentedCloudPure;
    ros::Publisher _pubSegmentedCloudInfo;
    ros::Publisher _pubOutlierCloud;
    ros::Publisher _pubCloudFiltered;

    pcl::PointCloud<PointType>::Ptr _laserCloudIn;

    pcl::PointCloud<PointType>::Ptr _fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr _fullInfoCloud; // same as _fullCloud, but with intensity - range

    pcl::PointCloud<PointType>::Ptr _groundCloud;
    pcl::PointCloud<PointType>::Ptr _segmentedCloud;
    pcl::PointCloud<PointType>::Ptr _segmentedCloudPure;
    pcl::PointCloud<PointType>::Ptr _outlierCloud;
    pcl::PointCloud<PointType>::Ptr _pointCloudFiltered;

    PointType _nanPoint; // fill in _fullCloud at each iteration

    cv::Mat _rangeMat; // range matrix for range image
    cv::Mat _labelMat; // label matrix for segmentaiton marking
    cv::Mat _groundMat; // ground matrix for ground cloud marking
    int _labelCount;

    float _startOrientation;
    float _endOrientation;

    cloud_msgs::cloud_info _segMsg; // info of segmented cloud
    std_msgs::Header _cloudHeader;

    std::vector<std::pair<uint8_t, uint8_t> > _neighborIterator; // neighbor iterator for segmentaiton process

    uint16_t *_allPushedIndX; // array for tracking points of a segmented object
    uint16_t *_allPushedIndY;

    uint16_t *_queueIndX; // array for breadth-first search process of segmentation
    uint16_t *_queueIndY;

    string _pointCloudTopic;
    int _N_SCAN;
    int _Horizon_SCAN;
    float _ang_res_x;
    float _ang_res_y;
    float _ang_top;
    float _ang_bottom;
    int _groundScanInd;

    bool _removeRobotPoints;
    float _robotForwardX;
    float _robotBackwardX;
    float _robotRightY;
    float _robotLeftY;
    float _robotUpZ;
    float _robotDownZ;

    float _sensorMountAngle;
    float _segmentTheta;
    int _segmentValidPointNum;
    int _segmentValidLineNum;
    float _segmentAlphaX;
    float _segmentAlphaY;

    // bool _transformToRobotCenter;
    // float _RToLRoll;
    // float _RToLPitch;
    // float _RToLYaw;
    // float _TX;
    // float _TY;
    // float _TZ;

    string _lidarFrameId;
    // string _lidarFrameIdTrans;
    string _frameId;

public:
    ImageProjection():
        nh("~"){
        // get parameters.
        if(!nh.getParam("/lego_loam/pointCloudTopic", _pointCloudTopic)) ROS_ERROR("Failed to get param '/lego_loam/pointCloudTopic'");
        if(!nh.getParam("/lego_loam/laser/N_SCAN", _N_SCAN)) ROS_ERROR("Failed to get param '/lego_loam/laser/N_SCAN'");
        if(!nh.getParam("/lego_loam/laser/Horizon_SCAN", _Horizon_SCAN)) ROS_ERROR("Failed to get param '/lego_loam/laser/Horizon_SCAN'");
        if(!nh.getParam("/lego_loam/laser/ang_top", _ang_top)) ROS_ERROR("Failed to get param '/lego_loam/laser/ang_top'");
        if(!nh.getParam("/lego_loam/laser/ang_bottom", _ang_bottom)) ROS_ERROR("Failed to get param '/lego_loam/laser/ang_bottom'");
        if(!nh.getParam("/lego_loam/laser/groundScanInd", _groundScanInd)) ROS_ERROR("Failed to get param '/lego_loam/laser/groundScanInd'");
        if(!nh.getParam("/lego_loam/laser/sensorMountAngle", _sensorMountAngle)) ROS_ERROR("Failed to get param '/lego_loam/laser/sensorMountAngle'");

        if(!nh.getParam("/lego_loam/pointFilter/removeRobotPoints", _removeRobotPoints)) ROS_ERROR("Failed to get param '/lego_loam/pointFilter/removeRobotPoints'");
        if(!nh.getParam("/lego_loam/pointFilter/robotForwardX", _robotForwardX)) ROS_ERROR("Failed to get param '/lego_loam/pointFilter/robotForwardX'");
        if(!nh.getParam("/lego_loam/pointFilter/robotBackwardX", _robotBackwardX)) ROS_ERROR("Failed to get param '/lego_loam/pointFilter/robotBackwardX'");
        if(!nh.getParam("/lego_loam/pointFilter/robotRightY", _robotRightY)) ROS_ERROR("Failed to get param '/lego_loam/pointFilter/robotRightY'");
        if(!nh.getParam("/lego_loam/pointFilter/robotLeftY", _robotLeftY)) ROS_ERROR("Failed to get param '/lego_loam/pointFilter/robotLeftY'");
        if(!nh.getParam("/lego_loam/pointFilter/robotUpZ", _robotUpZ)) ROS_ERROR("Failed to get param '/lego_loam/pointFilter/robotUpZ'");
        if(!nh.getParam("/lego_loam/pointFilter/robotDownZ", _robotDownZ)) ROS_ERROR("Failed to get param '/lego_loam/pointFilter/robotDownZ'");

        // if(!nh.getParam("/lego_loam/pointTransform/transformToRobotCenter", _transformToRobotCenter)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/transformToRobotCenter'");
        // if(!nh.getParam("/lego_loam/pointTransform/RToLRoll", _RToLRoll)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/RToLRoll'");
        // if(!nh.getParam("/lego_loam/pointTransform/RToLPitch", _RToLPitch)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/RToLPitch'");
        // if(!nh.getParam("/lego_loam/pointTransform/RToLYaw", _RToLYaw)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/RToLYaw'");
        // if(!nh.getParam("/lego_loam/pointTransform/TX", _TX)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/TX'");
        // if(!nh.getParam("/lego_loam/pointTransform/TY", _TY)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/TY'");
        // if(!nh.getParam("/lego_loam/pointTransform/TZ", _TZ)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/TZ'");

        if(!nh.getParam("/lego_loam/imageProjection/segmentTheta", _segmentTheta)) ROS_ERROR("Failed to get param '/lego_loam/imageProjection/segmentTheta'");
        if(!nh.getParam("/lego_loam/imageProjection/segmentValidPointNum", _segmentValidPointNum)) ROS_ERROR("Failed to get param '/lego_loam/imageProjection/segmentValidPointNum'");
        if(!nh.getParam("/lego_loam/imageProjection/segmentValidLineNum", _segmentValidLineNum)) ROS_ERROR("Failed to get param '/lego_loam/imageProjection/segmentValidLineNum'");

        if(!nh.getParam("/lego_loam/lidarFrameId", _lidarFrameId)) ROS_ERROR("Failed to get param '/lego_loam/lidarFrameId'");
        // if(!nh.getParam("/lego_loam/lidarFrameIdTrans", _lidarFrameIdTrans)) ROS_ERROR("Failed to get param '/lego_loam/lidarFrameId'");
        _subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(_pointCloudTopic, 1, &ImageProjection::cloudHandler, this);

        _pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        _pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);

        _pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        _pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        _pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        _pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info> ("/segmented_cloud_info", 1);
        _pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);
        _pubCloudFiltered = nh.advertise<sensor_msgs::PointCloud2> ("/point_cloud_filtered", 1);

        _nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        _nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        _nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        _nanPoint.intensity = -1;

        _ang_res_x = 360.0/float(_Horizon_SCAN);
        _ang_res_y = (_ang_top+_ang_bottom)/float(_N_SCAN-1);

        _segmentAlphaX = _ang_res_x / 180.0 * M_PI;
        _segmentAlphaY = _ang_res_y / 180.0 * M_PI;

        // if(_transformToRobotCenter) {_frameId = _lidarFrameIdTrans;}
        // else {_frameId = _lidarFrameId;}

        _frameId = _lidarFrameId;

        allocateMemory();
        resetParameters();
    }

    void allocateMemory(){

        _laserCloudIn.reset(new pcl::PointCloud<PointType>());

        _fullCloud.reset(new pcl::PointCloud<PointType>());
        _fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        _groundCloud.reset(new pcl::PointCloud<PointType>());
        _segmentedCloud.reset(new pcl::PointCloud<PointType>());
        _segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        _outlierCloud.reset(new pcl::PointCloud<PointType>());
        _pointCloudFiltered.reset(new pcl::PointCloud<PointType>());

        _fullCloud->points.resize(_N_SCAN*_Horizon_SCAN);
        _fullInfoCloud->points.resize(_N_SCAN*_Horizon_SCAN);

        _segMsg.startRingIndex.assign(_N_SCAN, 0);
        _segMsg.endRingIndex.assign(_N_SCAN, 0);

        _segMsg.segmentedCloudGroundFlag.assign(_N_SCAN*_Horizon_SCAN, false);
        _segMsg.segmentedCloudColInd.assign(_N_SCAN*_Horizon_SCAN, 0);
        _segMsg.segmentedCloudRange.assign(_N_SCAN*_Horizon_SCAN, 0);

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; _neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; _neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; _neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; _neighborIterator.push_back(neighbor);

        _allPushedIndX = new uint16_t[_N_SCAN*_Horizon_SCAN];
        _allPushedIndY = new uint16_t[_N_SCAN*_Horizon_SCAN];

        _queueIndX = new uint16_t[_N_SCAN*_Horizon_SCAN];
        _queueIndY = new uint16_t[_N_SCAN*_Horizon_SCAN];
    }

    void resetParameters(){
        _laserCloudIn->clear();
        _groundCloud->clear();
        _segmentedCloud->clear();
        _segmentedCloudPure->clear();
        _outlierCloud->clear();
        _pointCloudFiltered->clear();

        _rangeMat = cv::Mat(_N_SCAN, _Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        _groundMat = cv::Mat(_N_SCAN, _Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        _labelMat = cv::Mat(_N_SCAN, _Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        _labelCount = 1;

        std::fill(_fullCloud->points.begin(), _fullCloud->points.end(), _nanPoint);
        std::fill(_fullInfoCloud->points.begin(), _fullInfoCloud->points.end(), _nanPoint);
    }

    ~ImageProjection(){}

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        _cloudHeader = laserCloudMsg->header;
        // _cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(*laserCloudMsg, *_laserCloudIn);
        // Remove Nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*_laserCloudIn, *_laserCloudIn, indices);
    }

    void removeRobotPoints(){
        if(!_removeRobotPoints) return;

        pcl::CropBox<PointType> boxFilter;
        boxFilter.setNegative(true); // true: remove points inside the box.
        boxFilter.setMin(Eigen::Vector4f(_robotBackwardX, _robotRightY, _robotDownZ, 1.0));
        boxFilter.setMax(Eigen::Vector4f(_robotForwardX, _robotLeftY, _robotUpZ, 1.0));
        boxFilter.setInputCloud(_laserCloudIn);
        boxFilter.filter(*_pointCloudFiltered);
        // _laserCloudIn = _pointCloudFiltered;

    }

    // void transformToRobotCenter(){  //need (Robot)^T_(Lidar)
    //     if(!_transformToRobotCenter) return;
    //
    //     size_t cloudSize;
    //     cloudSize = _laserCloudIn->points.size();
    //
    //     // if only translation needed, only do translate
    //     if(_RToLRoll == 0.0 || _RToLPitch == 0.0 || _RToLYaw == 0.0){
    //       for (size_t i = 0; i < cloudSize; ++i){
    //         _laserCloudIn->points[i].x = _laserCloudIn->points[i].x + _TX;
    //         _laserCloudIn->points[i].y = _laserCloudIn->points[i].y + _TY;
    //         _laserCloudIn->points[i].z = _laserCloudIn->points[i].z + _TZ;
    //       }
    //       return;
    //     }
    //
    //     for (size_t i = 0; i < cloudSize; ++i){
    //       // rotate with x axis(roll)
    //       float x1 = _laserCloudIn->points[i].x;
    //       float y1 = cos(_RToLRoll) * _laserCloudIn->points[i].y - sin(_RToLRoll) * _laserCloudIn->points[i].z;
    //       float z1 = sin(_RToLRoll) * _laserCloudIn->points[i].y + cos(_RToLRoll) * _laserCloudIn->points[i].z;
    //
    //       // rotate with y axis(pitch)
    //       float x2 = sin(_RToLPitch) * z1 + cos(_RToLPitch) * x1;
    //       float y2 = y1;
    //       float z2 = cos(_RToLPitch) * z1 - sin(_RToLPitch) * x1;
    //
    //       // rotate with z axis(yaw) and translate;
    //       _laserCloudIn->points[i].x = -sin(_RToLYaw) * y2 + cos(_RToLYaw) * x2 + _TX;
    //       _laserCloudIn->points[i].y = cos(_RToLYaw) * y2 + sin(_RToLYaw) * x2 + _TY;
    //       _laserCloudIn->points[i].z = z2 + _TZ;
    //     }
    // }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        // 1. Convert ros message to pcl point cloud
        copyPointCloud(laserCloudMsg);
        // 2. Remove points belong to robot itself and transform
        removeRobotPoints();
        // transformToRobotCenter();
        // 3. Start and end angle of a scan
        findStartEndAngle();
        // 4. Range image projection
        projectPointCloud();
        // 5. Mark ground points
        groundRemoval();
        // 6. Point cloud segmentation
        cloudSegmentation();
        // 7. Publish all clouds
        publishCloud();
        // 8. Reset parameters for next iteration
        resetParameters();
    }

    void findStartEndAngle(){
        // start and end orientation of this cloud
        _segMsg.startOrientation = -atan2(_laserCloudIn->points[0].y, _laserCloudIn->points[0].x);
        _segMsg.endOrientation   = -atan2(_laserCloudIn->points[_laserCloudIn->points.size() - 1].y,
                                                     _laserCloudIn->points[_laserCloudIn->points.size() - 2].x) + 2 * M_PI;
        if (_segMsg.endOrientation - _segMsg.startOrientation > 3 * M_PI) {
            _segMsg.endOrientation -= 2 * M_PI;
        } else if (_segMsg.endOrientation - _segMsg.startOrientation < M_PI)
            _segMsg.endOrientation += 2 * M_PI;
        _segMsg.orientationDiff = _segMsg.endOrientation - _segMsg.startOrientation;
    }

    void projectPointCloud(){
        // range image projection
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize;
        PointType thisPoint;

        cloudSize = _laserCloudIn->points.size();

        for (size_t i = 0; i < cloudSize; ++i){

            thisPoint.x = _laserCloudIn->points[i].x;
            thisPoint.y = _laserCloudIn->points[i].y;
            thisPoint.z = _laserCloudIn->points[i].z;
            // find the row and column index in the iamge for this point
            verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            rowIdn = (verticalAngle + _ang_bottom) / _ang_res_y;
            if (rowIdn < 0 || rowIdn >= _N_SCAN)
                continue;

            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            columnIdn = -round((horizonAngle-90.0)/_ang_res_x) + _Horizon_SCAN/2;
            if (columnIdn >= _Horizon_SCAN)
                columnIdn -= _Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= _Horizon_SCAN)
                continue;

            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < 0.1)
                continue;

            _rangeMat.at<float>(rowIdn, columnIdn) = range;

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn  + rowIdn * _Horizon_SCAN;
            _fullCloud->points[index] = thisPoint;
            _fullInfoCloud->points[index] = thisPoint;
            _fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
        }
    }


    void groundRemoval(){
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // _groundMat
        // -1, no valid info to check if ground of not
        //  0, initial value, after validation, means not ground
        //  1, ground
        for (size_t j = 0; j < _Horizon_SCAN; ++j){
            for (size_t i = 0; i < _groundScanInd; ++i){

                lowerInd = j + ( i )*_Horizon_SCAN;
                upperInd = j + (i+1)*_Horizon_SCAN;

                if (_fullCloud->points[lowerInd].intensity == -1 ||
                    _fullCloud->points[upperInd].intensity == -1){
                    // no info to check, invalid points
                    _groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }

                diffX = _fullCloud->points[upperInd].x - _fullCloud->points[lowerInd].x;
                diffY = _fullCloud->points[upperInd].y - _fullCloud->points[lowerInd].y;
                diffZ = _fullCloud->points[upperInd].z - _fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

                if (abs(angle - _sensorMountAngle) <= 10){
                    _groundMat.at<int8_t>(i,j) = 1;
                    _groundMat.at<int8_t>(i+1,j) = 1;
                }
            }
        }
        // extract ground cloud (_groundMat == 1)
        // mark entry that doesn't need to label (ground and invalid point) for segmentation
        // note that ground remove is from 0~_N_SCAN-1, need _rangeMat for mark label matrix for the 16th scan
        for (size_t i = 0; i < _N_SCAN; ++i){
            for (size_t j = 0; j < _Horizon_SCAN; ++j){
                if (_groundMat.at<int8_t>(i,j) == 1 || _rangeMat.at<float>(i,j) == FLT_MAX){
                    _labelMat.at<int>(i,j) = -1;
                }
            }
        }
        if (_pubGroundCloud.getNumSubscribers() != 0){
            for (size_t i = 0; i <= _groundScanInd; ++i){
                for (size_t j = 0; j < _Horizon_SCAN; ++j){
                    if (_groundMat.at<int8_t>(i,j) == 1)
                        _groundCloud->push_back(_fullCloud->points[j + i*_Horizon_SCAN]);
                }
            }
        }
    }

    void cloudSegmentation(){
        // segmentation process
        for (size_t i = 0; i < _N_SCAN; ++i)
            for (size_t j = 0; j < _Horizon_SCAN; ++j)
                if (_labelMat.at<int>(i,j) == 0)
                    labelComponents(i, j);

        int sizeOfSegCloud = 0;
        // extract segmented cloud for lidar odometry
        for (size_t i = 0; i < _N_SCAN; ++i) {

            _segMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5;

            for (size_t j = 0; j < _Horizon_SCAN; ++j) {
                if (_labelMat.at<int>(i,j) > 0 || _groundMat.at<int8_t>(i,j) == 1){
                    // outliers that will not be used for optimization (always continue)
                    if (_labelMat.at<int>(i,j) == 999999){
                        if (i > _groundScanInd && j % 5 == 0){
                            _outlierCloud->push_back(_fullCloud->points[j + i*_Horizon_SCAN]);
                            continue;
                        }else{
                            continue;
                        }
                    }
                    // majority of ground points are skipped
                    if (_groundMat.at<int8_t>(i,j) == 1){
                        if (j%5!=0 && j>5 && j<_Horizon_SCAN-5)
                            continue;
                    }
                    // mark ground points so they will not be considered as edge features later
                    _segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (_groundMat.at<int8_t>(i,j) == 1);
                    // mark the points' column index for marking occlusion later
                    _segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                    // save range info
                    _segMsg.segmentedCloudRange[sizeOfSegCloud]  = _rangeMat.at<float>(i,j);
                    // save seg cloud
                    _segmentedCloud->push_back(_fullCloud->points[j + i*_Horizon_SCAN]);
                    // size of seg cloud
                    ++sizeOfSegCloud;
                }
            }

            _segMsg.endRingIndex[i] = sizeOfSegCloud-1 - 5;
        }

        // extract segmented cloud for visualization
        if (_pubSegmentedCloudPure.getNumSubscribers() != 0){
            for (size_t i = 0; i < _N_SCAN; ++i){
                for (size_t j = 0; j < _Horizon_SCAN; ++j){
                    if (_labelMat.at<int>(i,j) > 0 && _labelMat.at<int>(i,j) != 999999){
                        _segmentedCloudPure->push_back(_fullCloud->points[j + i*_Horizon_SCAN]);
                        _segmentedCloudPure->points.back().intensity = _labelMat.at<int>(i,j);
                    }
                }
            }
        }
    }

    void labelComponents(int row, int col){
        // use std::queue std::vector std::deque will slow the program down greatly
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY;
        bool lineCountFlag[_N_SCAN] = {false};

        _queueIndX[0] = row;
        _queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        _allPushedIndX[0] = row;
        _allPushedIndY[0] = col;
        int allPushedIndSize = 1;

        while(queueSize > 0){
            // Pop point
            fromIndX = _queueIndX[queueStartInd];
            fromIndY = _queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;
            // Mark popped point
            _labelMat.at<int>(fromIndX, fromIndY) = _labelCount;
            // Loop through all the neighboring grids of popped grid
            for (auto iter = _neighborIterator.begin(); iter != _neighborIterator.end(); ++iter){
                // new index
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;
                // index should be within the boundary
                if (thisIndX < 0 || thisIndX >= _N_SCAN)
                    continue;
                // at range image margin (left or right side)
                if (thisIndY < 0)
                    thisIndY = _Horizon_SCAN - 1;
                if (thisIndY >= _Horizon_SCAN)
                    thisIndY = 0;
                // prevent infinite loop (caused by put already examined point back)
                if (_labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                d1 = std::max(_rangeMat.at<float>(fromIndX, fromIndY),
                              _rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(_rangeMat.at<float>(fromIndX, fromIndY),
                              _rangeMat.at<float>(thisIndX, thisIndY));

                if ((*iter).first == 0)
                    alpha = _segmentAlphaX;
                else
                    alpha = _segmentAlphaY;

                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

                if (angle > _segmentTheta/180.0*M_PI){

                    _queueIndX[queueEndInd] = thisIndX;
                    _queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    _labelMat.at<int>(thisIndX, thisIndY) = _labelCount;
                    lineCountFlag[thisIndX] = true;

                    _allPushedIndX[allPushedIndSize] = thisIndX;
                    _allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }

        // check if this segment is valid
        bool feasibleSegment = false;
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
        else if (allPushedIndSize >= _segmentValidPointNum){
            int lineCount = 0;
            for (size_t i = 0; i < _N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= _segmentValidLineNum)
                feasibleSegment = true;
        }
        // segment is valid, mark these points
        if (feasibleSegment == true){
            ++_labelCount;
        }else{ // segment is invalid, mark these points
            for (size_t i = 0; i < allPushedIndSize; ++i){
                _labelMat.at<int>(_allPushedIndX[i], _allPushedIndY[i]) = 999999;
            }
        }
    }


    void publishCloud(){
        // 1. Publish Seg Cloud Info
        _segMsg.header = _cloudHeader;
        _pubSegmentedCloudInfo.publish(_segMsg);
        // 2. Publish clouds
        sensor_msgs::PointCloud2 laserCloudTemp;

        pcl::toROSMsg(*_outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = _cloudHeader.stamp;
        // laserCloudTemp.header.frame_id = "base_link";
        laserCloudTemp.header.frame_id = _frameId;
        _pubOutlierCloud.publish(laserCloudTemp);
        // segmented cloud with ground
        pcl::toROSMsg(*_segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = _cloudHeader.stamp;
        // laserCloudTemp.header.frame_id = "base_link";
        laserCloudTemp.header.frame_id = _frameId;
        _pubSegmentedCloud.publish(laserCloudTemp);
        // projected full cloud
        if (_pubFullCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*_fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = _cloudHeader.stamp;
            // laserCloudTemp.header.frame_id = "base_link";
            laserCloudTemp.header.frame_id = _frameId;
            _pubFullCloud.publish(laserCloudTemp);
        }
        // original dense ground cloud
        if (_pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*_groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = _cloudHeader.stamp;
            // laserCloudTemp.header.frame_id = "base_link";
            laserCloudTemp.header.frame_id = _frameId;
            _pubGroundCloud.publish(laserCloudTemp);
        }
        // segmented cloud without ground
        if (_pubSegmentedCloudPure.getNumSubscribers() != 0){
            pcl::toROSMsg(*_segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = _cloudHeader.stamp;
            // laserCloudTemp.header.frame_id = "base_link";
            laserCloudTemp.header.frame_id = _frameId;
            _pubSegmentedCloudPure.publish(laserCloudTemp);
        }
        // projected full cloud info
        if (_pubFullInfoCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*_fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = _cloudHeader.stamp;
            // laserCloudTemp.header.frame_id = "base_link";
            laserCloudTemp.header.frame_id = _frameId;
            _pubFullInfoCloud.publish(laserCloudTemp);
        }

        // point cloud filtered
        if (_pubCloudFiltered.getNumSubscribers() != 0){
            pcl::toROSMsg(*_pointCloudFiltered, laserCloudTemp);
            laserCloudTemp.header.stamp = _cloudHeader.stamp;
            // laserCloudTemp.header.frame_id = "base_link";
            laserCloudTemp.header.frame_id = _frameId;
            _pubCloudFiltered.publish(laserCloudTemp);
        }
    }
};

int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");

    ImageProjection IP;

    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    ros::spin();
    return 0;
}
