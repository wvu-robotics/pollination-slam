#ifndef SENSOR_FUSION_FUSION_FUSION_FLOW_HPP_
#define SENSOR_FUSION_FUSION_FUSION_FLOW_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include "sensor_fusion/sensor_data/imu_data.hpp"
#include "sensor_fusion/sensor_data/odometry_data.hpp"
#include "sensor_fusion/sensor_data/pose_data.hpp"
// subscriber
#include "sensor_fusion/subscriber/imu_subscriber.hpp"
#include "sensor_fusion/subscriber/lo_subscriber.hpp"
#include "sensor_fusion/subscriber/wo_subscriber.hpp"
// publisher
#include "sensor_fusion/publisher/odometry_publisher.hpp"


namespace sensor_fusion{
class FusionFlow{
    public:
        FusionFlow(ros::NodeHandle& nh);

        bool run();

    private:
        void initParam();
        void initRos();

    private:
        // subscriber
        std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
        std::shared_ptr<LoSubscriber> lo_sub_ptr_;
        std::shared_ptr<WoSubscriber> wo_sub_ptr_;

        // publisher
        std::shared_ptr<OdometryPublisher> pose_pub_ptr_;

        // parameters
        std::string imu_topic_name_;
        std::string lo_topic_name_;
        std::string wo_topic_name_;

        std::string pose_topic_name_;
        std::string base_frame_id_;
        std::string child_frame_id_;

        ros::NodeHandle nh_;

        bool initialized_;

        std::deque<IMUData> imu_data_buff_;
        std::deque<PoseData> lo_data_buff_;
        std::deque<OdometryData> wo_data_buff_;

        IMUData current_imu_data_;
        PoseData current_lo_data_;
        OdometryData current_wo_data_;

        IMUData start_imu_data_;
        PoseData start_lo_data_;
        OdometryData start_wo_data_;

        int imu_idx_;
        int wo_idx_;
        
};

}

#endif
