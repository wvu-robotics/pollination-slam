#ifndef SENSOR_FUSION_MODELS_SCAN_ADJUST_IMU_DISTORTION_ADJUST_HPP_
#define SENSOR_FUSION_MODELS_SCAN_ADJUST_IMU_DISTORTION_ADJUST_HPP_

#include "sensor_fusion/models/scan_adjust/distortion_adjust_interface.hpp"

#include "sensor_fusion/sensor_data/imu_data.hpp"
#include "sensor_fusion/sensor_data/pose_data.hpp"
#include <vector>

namespace sensor_fusion {
class IMUDistortionAdjust: public DistortionAdjustInterface {

    public:
        bool AdjustCloud(CloudData& input_cloud, CloudData& output_cloud) override;

        bool SetIMUData(std::deque<IMUData>& imu_data_buff) override;
        bool SetPoseData(std::deque<PoseData>& pose_data_buff) override;
        bool SetScanPeriod(double scan_period) override;

    private:
        void ResetParameters();
        bool PruneIMUDeque();
        void ComputeRotation(double point_time, float& cur_rot_x, float& cur_rot_y, float& cur_rot_z);
        void ComputePosition(double point_time, float& cur_pos_x, float& cur_pos_y, float& cur_pos_z);

        void CorrectLaserScan();


    private:
        double scan_period_;
        double current_scan_time_start_;
        double current_scan_time_end_;

        CloudData::CLOUD_PTR origin_cloud_ptr_;
        CloudData::CLOUD_PTR output_cloud_ptr_;

        int current_imu_index_;
        std::vector<double> imu_time_;
        std::vector<double> imu_rot_x_;
        std::vector<double> imu_rot_y_;
        std::vector<double> imu_rot_z_;

        std::deque<IMUData> imu_data_buff_;
        std::deque<PoseData> pose_data_buff_;
};
}

#endif