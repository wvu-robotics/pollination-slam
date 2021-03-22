#include "sensor_fusion/subscriber/wo_subscriber.hpp"

namespace sensor_fusion{
WoSubscriber::WoSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh){
    subscriber_ = nh_.subscribe(topic_name, buff_size, &WoSubscriber::msg_callback, this);
}

void WoSubscriber::msg_callback(const nav_msgs::OdometryConstPtr& wo_msg_ptr){
    buff_mutex_.lock();

    PoseData pose_data;
    pose_data.time = wo_msg_ptr->header.stamp.toSec();

    pose_data.position.x = wo_msg_ptr->pose.pose.position.x;
    pose_data.position.y = wo_msg_ptr->pose.pose.position.y;
    pose_data.position.z = wo_msg_ptr->pose.pose.position.z;

    pose_data.orientation.x = wo_msg_ptr->pose.pose.orientation.x;
    pose_data.orientation.y = wo_msg_ptr->pose.pose.orientation.y;
    pose_data.orientation.z = wo_msg_ptr->pose.pose.orientation.z;
    pose_data.orientation.w = wo_msg_ptr->pose.pose.orientation.w;

    new_pose_data_.push_back(pose_data);

    buff_mutex_.unlock();
}

void WoSubscriber::ParseData(std::deque<PoseData>& pose_data_buff){
    buff_mutex_.lock();

    if(new_pose_data_.size() > 0){
        pose_data_buff.insert(pose_data_buff.end(), new_pose_data_.begin(), new_pose_data_.end());
        new_pose_data_.clear();
    }

    buff_mutex_.unlock();
}


}