#include "sensor_fusion/fusion/fusion_flow.hpp"

namespace sensor_fusion{
FusionFlow::FusionFlow(ros::NodeHandle& nh):nh_(nh), initialized_(false){
    initParam();
    initRos();    
}

void FusionFlow::initParam(){
    if(!nh_.getParam("/subscriber/imu_topic_name", imu_topic_name_)){
        ROS_ERROR("Failed to get param '/subscriber/imu_topic_name'");
        imu_topic_name_ = "/imu/data";
    }
    if(!nh_.getParam("/subscriber/lo_topic_name", lo_topic_name_)){
        ROS_ERROR("Failed to get param '/subscriber/lo_topic_name'");
        lo_topic_name_ = "/aft_mapped_to_init_high_frec";
    }
    if(!nh_.getParam("/subscriber/wo_topic_name", wo_topic_name_)){
        ROS_ERROR("Failed to get param '/subscriber/wo_topic_name'");
        wo_topic_name_ = "/husky_velocity_controller/odom";
    }
    if(!nh_.getParam("/publisher/pose_topic_name", pose_topic_name_)){
        ROS_ERROR("Failed to get param '/publisher/pose_topic_name'");
        pose_topic_name_ = "states";
    }
    if(!nh_.getParam("/publisher/base_frame_id", base_frame_id_)){
        ROS_ERROR("Failed to get param '/publisher/base_frame_id'");
        base_frame_id_ = "map";
    }
    if(!nh_.getParam("/publisher/child_frame_id", child_frame_id_)){
        ROS_ERROR("Failed to get param '/publisher/child_frame_id'");
        child_frame_id_ = "base_link";
    }



}

void FusionFlow::initRos(){
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh_, imu_topic_name_, 1000000);
    lo_sub_ptr_ = std::make_shared<LoSubscriber>(nh_, lo_topic_name_, 10000000);
    wo_sub_ptr_ = std::make_shared<WoSubscriber>(nh_, wo_topic_name_, 10000000);

    pose_pub_ptr_ = std::make_shared<OdometryPublisher>(nh_, pose_topic_name_, base_frame_id_, child_frame_id_, 100);
}

bool FusionFlow::run(){
    // read data
    static std::deque<IMUData> unsynced_imu_data_buff_;
    static std::deque<PoseData> unsynced_wo_data_buff_;

    lo_sub_ptr_->ParseData(lo_data_buff_);
    imu_sub_ptr_->ParseData(unsynced_imu_data_buff_);
    wo_sub_ptr_->ParseData(unsynced_wo_data_buff_);
    // initialization
    if(!initialized_ && lo_data_buff_.size() > 0){
        initialized_ = true;
    }else if(!initialized_ && lo_data_buff_.size() == 0){
        return false;
    }

    // new lidar odometry data
    // assumption: imu and wheel odometry udpate rate is higher than lidar odometry
    if(lo_data_buff_.size() > 0){
        start_lo_data_ = lo_data_buff_.front();
        lo_data_buff_.pop_front();

        // sync IMU and wheel odometry measurement using lidar odometry time
        double lo_time = start_lo_data_.time;
        bool valid_imu = IMUData::SyncData(unsynced_imu_data_buff_, imu_data_buff_, lo_time);
        bool valid_wo = PoseData::SyncData(unsynced_wo_data_buff_, wo_data_buff_, lo_time);

        if(valid_imu){
            start_imu_data_ = imu_data_buff_.front();
            imu_data_buff_.pop_front();
            imu_idx_ = 1;   // read data from unsync imu data buff
        }else return false;

        if(valid_wo){
            start_wo_data_ = wo_data_buff_.front();
            wo_data_buff_.pop_front();
            wo_idx_ = 1;    // read data from unsync wo data buff
        }else return false;

    }else{
        if((int)unsynced_imu_data_buff_.size() > imu_idx_){
            current_imu_data_ = unsynced_imu_data_buff_[imu_idx_];
            imu_idx_++;
        }else return false;
        if((int)unsynced_wo_data_buff_.size() > wo_idx_){
            current_wo_data_ = unsynced_wo_data_buff_[wo_idx_];
            wo_idx_++;
        }else return false;
    }

    // get the high freq update pose
    // start_lo_data += imu_data_.delta_orientation, wo_data.delta_position
    Eigen::Quaterniond q_cur_imu(current_imu_data_.orientation.w,
                                current_imu_data_.orientation.x,
                                current_imu_data_.orientation.y,
                                current_imu_data_.orientation.z);
    Eigen::Quaterniond q_start_imu(start_imu_data_.orientation.w,
                                start_imu_data_.orientation.x,
                                start_imu_data_.orientation.y,
                                start_imu_data_.orientation.z);
    Eigen::Quaterniond q_start_lo(start_lo_data_.orientation.w,
                                start_lo_data_.orientation.x,
                                start_lo_data_.orientation.y,
                                start_lo_data_.orientation.z);
    Eigen::Quaterniond q_pose;
    q_pose = q_cur_imu * q_start_imu.inverse() * q_start_lo;
    // q_pose = q_pose.normoalized();

    Eigen::Vector3d t_cur_wo(current_wo_data_.position.x,
                            current_wo_data_.position.y,
                            current_wo_data_.position.z);

    Eigen::Vector3d t_start_wo(start_wo_data_.position.x,
                            start_wo_data_.position.y,
                            start_wo_data_.position.z);

    Eigen::Vector3d t_start_lo(start_lo_data_.position.x,
                            start_lo_data_.position.y,
                            start_lo_data_.position.z);

    Eigen::Vector3d t_pose;

    // in case the wheel odometry has wrong solution
    if((t_cur_wo - t_start_wo).norm() > 0.3){
        return false;
    }
    t_pose = t_start_lo + t_cur_wo - t_start_wo;
    

    // publish
    pose_pub_ptr_->Publish(t_pose, q_pose, current_imu_data_.time);

    return true;
}
}