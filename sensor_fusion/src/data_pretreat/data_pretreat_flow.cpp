#include "sensor_fusion/data_pretreat/data_pretreat_flow.hpp"

#include "sensor_fusion/models/scan_adjust/imu_distortion_adjust.hpp"

namespace sensor_fusion {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh): nh_(nh) {
    InitParam();
    InitROS();
}

void DataPretreatFlow::InitParam() {
    if(!nh_.getParam("/subscriber/cloud_topic_name", cloud_topic_name_)){
        ROS_ERROR("Failed to get param '/subscriber/cloud_topic_name'");
        cloud_topic_name_ = "/velodyne_points";
    }
    if(!nh_.getParam("/subscriber/imu_topic_name", imu_topic_name_)){
        ROS_ERROR("Failed to get param '/subscriber/imu_topic_name'");
        imu_topic_name_ = "/imu/data";
    }
    if(!nh_.getParam("/subscriber/wo_topic_name", wo_topic_name_)){
        ROS_ERROR("Failed to get param '/subscriber/wo_topic_name'");
        wo_topic_name_ = "/husky_velocity_controller/odom";
    }
    if(!nh_.getParam("/publisher/cloud_adjusted_topic_name", cloud_adjusted_topic_name_)){
        ROS_ERROR("Failed to get param '/publisher/cloud_adjusted_topic_name'");
        cloud_adjusted_topic_name_ = "/velondyne_points_adjusted";
    }
    if(!nh_.getParam("/publisher/cloud_frame_id", cloud_frame_id_)){
        ROS_ERROR("Failed to get param '/publisher/cloud_frame_id'");
        cloud_frame_id_ = "/velodyne";
    }

    if(!nh_.getParam("/scan_adjust/distortion_adjust_method", distortion_adjust_method_)){
        ROS_ERROR("Failed to get param '/scan_adjust/distortion_adjust_method'");
        distortion_adjust_method_ = "IMU";
    }

    if (distortion_adjust_method_ == "IMU") {
        distortion_adjust_ptr_ = std::make_shared<IMUDistortionAdjust>();
    } else {
        ROS_ERROR("Wrong distortion adjust method input! Put IMU method!");
        distortion_adjust_ptr_ = std::make_shared<IMUDistortionAdjust>();
    }
}

void DataPretreatFlow::InitROS() {
    // subscriber
    // a. velodyne measurement:
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh_, cloud_topic_name_, 100);
    // b. IMU:
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh_, imu_topic_name_, 1000);
    // c. wheel odometry:
    wo_sub_ptr_ = std::make_shared<WoSubscriber>(nh_, wo_topic_name_, 100);

    //publisher
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh_, cloud_adjusted_topic_name_, cloud_frame_id_, 100);
}

bool DataPretreatFlow::Run() {
    if (!ReadData())
        return false;
    
    while(HasData()) {
        if (!ValidData())
            continue;
        // TransformData();
        PublishData();
    }

    return true;
}

bool DataPretreatFlow::ReadData() {

    // fetch lidar measurements from buffer:
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    imu_sub_ptr_->ParseData(unsynced_imu_);
    wo_sub_ptr_->ParseData(unsynced_pose_);

    if (cloud_data_buff_.size() == 0)
        return false;
    
    // use timestamp of lidar measurement as reference:
    double cloud_time = cloud_data_buff_.front().time;

    // in this way, the update frequence of the lidar measurement should be the slowest
    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
    bool valid_wo = PoseData::SyncData(unsynced_pose_, pose_data_buff_, cloud_time);

    // only mark lidar as 'inited' when all the three sensors are synced
    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!valid_imu || !valid_wo) {
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    return true;
}

bool DataPretreatFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;
    if (pose_data_buff_.size() == 0)
        return false;

    return true;
}

bool DataPretreatFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_pose_data_ = pose_data_buff_.front();

    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    double diff_pose_time = current_cloud_data_.time - current_pose_data_.time;
    // time difference should less than 0.5 * scan_period 
    if (diff_imu_time < -0.05 || diff_pose_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_imu_time > 0.05) {
        imu_data_buff_.pop_front();
        return false;
    }

    if (diff_pose_time > 0.05) {
        pose_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    pose_data_buff_.pop_front();

    return true;
}

bool DataPretreatFlow::TransformData() {
    // adjust cloud
    distortion_adjust_ptr_->SetIMUData(unsynced_imu_);
    distortion_adjust_ptr_->SetPoseData(unsynced_pose_);
    distortion_adjust_ptr_->SetScanPeriod(0.1);
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_, current_cloud_data_);

    return true;
}

bool DataPretreatFlow::PublishData() {
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);

    return true;
}
}