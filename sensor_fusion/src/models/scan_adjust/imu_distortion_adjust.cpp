#include "sensor_fusion/models/scan_adjust/imu_distortion_adjust.hpp"

#include <math.h>

namespace sensor_fusion {
bool IMUDistortionAdjust::AdjustCloud(CloudData& input_cloud, CloudData& output_cloud) {
    ResetParameters();

    // get information from input cloud
    current_scan_time_start_ = input_cloud.time;
    current_scan_time_end_ = current_scan_time_start_ + scan_period_;

    origin_cloud_ptr_.reset(new CloudData::CLOUD(*(input_cloud.cloud_ptr)));
    output_cloud_ptr_.reset(new CloudData::CLOUD());

    if (!PruneIMUDeque()) return false;

    CorrectLaserScan();

    output_cloud.time = current_scan_time_start_;
    output_cloud.cloud_ptr.reset(new CloudData::CLOUD(*output_cloud_ptr_));

    return true;
}
bool IMUDistortionAdjust::SetScanPeriod(double scan_period) {
    scan_period_ = scan_period;
    return true;
}
bool IMUDistortionAdjust::SetIMUData(std::deque<IMUData>& imu_data_buff) {
    imu_data_buff_ = imu_data_buff;
    return true;
}

bool IMUDistortionAdjust::SetPoseData(std::deque<PoseData>& pose_data_buff) {
    pose_data_buff_ = pose_data_buff;
    return true;
}

void IMUDistortionAdjust::ResetParameters() {
    int queue_length = 2000;

    imu_time_.clear();
    imu_rot_x_.clear();
    imu_rot_y_.clear();
    imu_rot_z_.clear();

    imu_time_ = std::vector<double>(queue_length, 0);
    imu_rot_x_ = std::vector<double>(queue_length, 0);
    imu_rot_y_ = std::vector<double>(queue_length, 0);
    imu_rot_z_ = std::vector<double>(queue_length, 0);

}

bool IMUDistortionAdjust::PruneIMUDeque() {
    if (imu_data_buff_.back().time < current_scan_time_end_) {
        std::cout<<"IMU data is slower than cloud data ... " << std::endl;
        return false;
    }

    current_imu_index_ = 0;

    IMUData tmp_imu_data;
    double current_imu_time;

    for (int i = 0; i < (int)imu_data_buff_.size(); ++i){
        tmp_imu_data = imu_data_buff_[i];
        current_imu_time = tmp_imu_data.time;

        if (current_imu_time < current_scan_time_start_){
            if (current_imu_index_ == 0) {
                imu_rot_x_[0] = 0;
                imu_rot_y_[0] = 0;
                imu_rot_z_[0] = 0;
                imu_time_ [0] = current_imu_time;
                ++current_imu_index_;
            }

            continue;
        }

        if (current_imu_time > current_scan_time_end_)
            break;
        
        // get angular velocity
        double angular_x, angular_y, angular_z;
        angular_x = tmp_imu_data.angular_velocity.x;
        angular_y = tmp_imu_data.angular_velocity.y;
        angular_z = tmp_imu_data.angular_velocity.z;

        double time_diff = current_imu_time - imu_time_[current_imu_index_ - 1];
        imu_rot_x_[current_imu_index_] = imu_rot_x_[current_imu_index_ - 1] + angular_x * time_diff;
        imu_rot_y_[current_imu_index_] = imu_rot_y_[current_imu_index_ - 1] + angular_y * time_diff;
        imu_rot_z_[current_imu_index_] = imu_rot_z_[current_imu_index_ - 1] + angular_z * time_diff;
        imu_time_[current_imu_index_] = current_imu_time;
        ++current_imu_index_;

    }

    // imu point to the last index of the cloud time
    --current_imu_index_;

    return true;

}

void IMUDistortionAdjust::ComputeRotation(double point_time, float& cur_rot_x, float& cur_rot_y, float& cur_rot_z){
    cur_rot_x = 0;
    cur_rot_y = 0;
    cur_rot_z = 0;

    int imu_pointer_front = 0;
    while (imu_pointer_front < current_imu_index_) {
        if (point_time < imu_time_[imu_pointer_front])
            break;
        ++imu_pointer_front;
    }

    if (point_time > imu_time_[imu_pointer_front] || imu_pointer_front == 0){
        cur_rot_x = imu_rot_x_[imu_pointer_front];
        cur_rot_y = imu_rot_y_[imu_pointer_front];
        cur_rot_z = imu_rot_z_[imu_pointer_front];
    }else {
        int imu_pointer_back = imu_pointer_front - 1;

        double ratio_front = (point_time - imu_time_[imu_pointer_back]) / (imu_time_[imu_pointer_front] - imu_time_[imu_pointer_back]);
        double ratio_back = (imu_time_[imu_pointer_front] - point_time) / (imu_time_[imu_pointer_front] - imu_time_[imu_pointer_back]);

        cur_rot_x = imu_rot_x_[imu_pointer_front] * ratio_front + imu_rot_x_[imu_pointer_back] * ratio_back;
        cur_rot_y = imu_rot_y_[imu_pointer_front] * ratio_front + imu_rot_y_[imu_pointer_back] * ratio_back;
        cur_rot_z = imu_rot_z_[imu_pointer_front] * ratio_front + imu_rot_z_[imu_pointer_back] * ratio_back;
    }
}

void IMUDistortionAdjust::ComputePosition(double point_time, float& cur_pos_x, float& cur_pos_y, float& cur_pos_z){
    cur_pos_x = 0;
    cur_pos_y = 0;
    cur_pos_z = 0;
}

void IMUDistortionAdjust::CorrectLaserScan() {
    bool first_point_flag = true;

    Eigen::Affine3f trans_start_inverse, trans_final, trans_bt;

    int cloud_size = origin_cloud_ptr_->points.size();
    float start_ori = -atan2(origin_cloud_ptr_->points[0].y, origin_cloud_ptr_->points[0].x);
    float end_ori = -atan2(origin_cloud_ptr_->points[cloud_size - 1].y, origin_cloud_ptr_->points[cloud_size - 1].x) + 2 * M_PI;

    // new velodyne drive provide each point's relative time, using points[i].time, not work for 64 channels lidar

    if (end_ori - start_ori > 3 * M_PI) {
        end_ori -= 2 * M_PI;
    }else if (end_ori - start_ori < M_PI) {
        end_ori += 2 * M_PI;
    }

    bool half_passed = false;

    for(int i = 0; i < cloud_size; i++) {
        float ori = -atan2(origin_cloud_ptr_->points[i].y, origin_cloud_ptr_->points[i].x);
        if (!half_passed) {
            if (ori < start_ori - M_PI / 2) {
                ori += 2 * M_PI;
            }else if (ori > start_ori + M_PI * 3 / 2) {
                ori -= 2 * M_PI;
            }

            if (ori - start_ori > M_PI) {
                half_passed = true;
            }
        } else {
            ori += 2 * M_PI;
            if (ori < end_ori - M_PI * 3 / 2) {
                ori += 2 * M_PI;
            } else if (ori > end_ori + M_PI / 2) {
                ori -= 2 * M_PI;
            }
        }

        double real_time = (ori - start_ori) / (end_ori - start_ori) * scan_period_;
        double point_time = current_scan_time_start_ + real_time;

        float cur_rot_x = 0, cur_rot_y = 0, cur_rot_z = 0;
        float cur_pos_x = 0, cur_pos_y = 0, cur_pos_z = 0;

        ComputeRotation(point_time, cur_rot_x, cur_rot_y, cur_rot_z);

        // ignore odometry update
        if (first_point_flag == true) {
            trans_start_inverse = (pcl::getTransformation(cur_pos_x, cur_pos_y, cur_pos_z,
                                                          cur_rot_x, cur_rot_y, cur_rot_z)).inverse();
            first_point_flag = false;
        }

        trans_final = pcl::getTransformation(cur_pos_x, cur_pos_y, cur_pos_z,
                                             cur_rot_x, cur_rot_y, cur_rot_z);

        trans_bt = trans_start_inverse * trans_final;

        Eigen::Vector3f origin_point(origin_cloud_ptr_->points[i].x,
                                     origin_cloud_ptr_->points[i].y,
                                     origin_cloud_ptr_->points[i].z);

        CloudData::POINT point;
        point.x = trans_bt(0, 0) * origin_cloud_ptr_->points[i].x 
                + trans_bt(0, 1) * origin_cloud_ptr_->points[i].y 
                + trans_bt(0, 2) * origin_cloud_ptr_->points[i].z
                + trans_bt(0, 3);
        point.y = trans_bt(1, 0) * origin_cloud_ptr_->points[i].x 
                + trans_bt(1, 1) * origin_cloud_ptr_->points[i].y 
                + trans_bt(1, 2) * origin_cloud_ptr_->points[i].z
                + trans_bt(1, 3);
        point.z = trans_bt(2, 0) * origin_cloud_ptr_->points[i].x 
                + trans_bt(2, 1) * origin_cloud_ptr_->points[i].y 
                + trans_bt(2, 2) * origin_cloud_ptr_->points[i].z
                + trans_bt(2, 3);

        output_cloud_ptr_->points.push_back(point);
    }
}




}