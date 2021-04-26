
#ifndef SENSOR_FUSION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_
#define SENSOR_FUSION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_

#include <ros/ros.h>
// subscriber
#include "sensor_fusion/subscriber/imu_subscriber.hpp"
#include "sensor_fusion/subscriber/wo_subscriber.hpp"
#include "sensor_fusion/subscriber/cloud_subscriber.hpp"
// publisher
#include "sensor_fusion/publisher/cloud_publisher.hpp"
// models
#include "sensor_fusion/models/scan_adjust/distortion_adjust_interface.hpp"

namespace sensor_fusion {
class DataPretreatFlow {
    public:
        DataPretreatFlow(ros::NodeHandle& nh);

        bool Run();
    
    private:
        void InitParam();
        void InitROS();  
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool TransformData();
        bool PublishData();

    private:
        // subscriber
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
        std::shared_ptr<WoSubscriber> wo_sub_ptr_;

        //publisher
        std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
        // models
        std::shared_ptr<DistortionAdjustInterface> distortion_adjust_ptr_;

        std::deque<CloudData> cloud_data_buff_;
        std::deque<IMUData> imu_data_buff_;
        std::deque<PoseData> pose_data_buff_;

        CloudData current_cloud_data_;
        IMUData current_imu_data_;
        PoseData current_pose_data_;

        std::deque<IMUData> unsynced_imu_;
        std::deque<PoseData> unsynced_pose_;

        // parameters
        std::string imu_topic_name_;
        std::string cloud_topic_name_;
        std::string wo_topic_name_;

        std::string cloud_adjusted_topic_name_;
        std::string cloud_frame_id_;

        std::string distortion_adjust_method_;

        ros::NodeHandle nh_;
};
}

#endif