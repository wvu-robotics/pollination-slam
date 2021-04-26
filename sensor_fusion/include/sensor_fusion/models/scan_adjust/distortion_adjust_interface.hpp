#ifndef SENSOR_FUSION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_INTERFACE_HPP_
#define SENSOR_FUSION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_INTERFACE_HPP_

#include <pcl/common/transforms.h>
#include <Eigen/Dense>

#include "sensor_fusion/sensor_data/cloud_data.hpp"
#include "sensor_fusion/sensor_data/imu_data.hpp"
#include "sensor_fusion/sensor_data/pose_data.hpp"

#include <deque>

namespace sensor_fusion {
class DistortionAdjustInterface {
    public:
        virtual bool AdjustCloud(CloudData& input_cloud, CloudData& output_cloud) = 0;
        
        virtual bool SetIMUData(std::deque<IMUData>& imu_data_buff) = 0;
        virtual bool SetPoseData(std::deque<PoseData>& pose_data_buff) = 0;

        virtual bool SetScanPeriod(double scan_period) = 0;

};
}
#endif