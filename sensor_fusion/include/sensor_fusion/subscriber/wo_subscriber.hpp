#ifndef SENSOR_FUSION_SUBSRIBER_WO_SUBSCRIBER_HPP_
#define SENSOR_FUSION_SUBSRIBER_WO_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "sensor_fusion/sensor_data/pose_data.hpp"

namespace sensor_fusion{
class WoSubscriber{
    public:
        WoSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        WoSubscriber() = default;
        void ParseData(std::deque<PoseData>& pose_data_buff);

    private:
        void msg_callback(const nav_msgs::OdometryConstPtr& wo_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<PoseData> new_pose_data_;
};

}

#endif
