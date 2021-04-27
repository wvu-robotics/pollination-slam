#include "sensor_fusion/subscriber/wo_subscriber.hpp"

namespace sensor_fusion{
WoSubscriber::WoSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh){
    subscriber_ = nh_.subscribe(topic_name, buff_size, &WoSubscriber::msg_callback, this);
}

void WoSubscriber::msg_callback(const nav_msgs::OdometryConstPtr& wo_msg_ptr){
    buff_mutex_.lock();

    OdometryData odometry_data;
    odometry_data.time = wo_msg_ptr->header.stamp.toSec();

    odometry_data.position.x = wo_msg_ptr->pose.pose.position.x;
    odometry_data.position.y = wo_msg_ptr->pose.pose.position.y;
    odometry_data.position.z = wo_msg_ptr->pose.pose.position.z;

    odometry_data.orientation.x = wo_msg_ptr->pose.pose.orientation.x;
    odometry_data.orientation.y = wo_msg_ptr->pose.pose.orientation.y;
    odometry_data.orientation.z = wo_msg_ptr->pose.pose.orientation.z;
    odometry_data.orientation.w = wo_msg_ptr->pose.pose.orientation.w;

    odometry_data.linear.x = wo_msg_ptr->twist.twist.linear.x;
    odometry_data.linear.y = wo_msg_ptr->twist.twist.linear.y;
    odometry_data.linear.z = wo_msg_ptr->twist.twist.linear.z;
    odometry_data.angular.x = wo_msg_ptr->twist.twist.angular.x;
    odometry_data.angular.y = wo_msg_ptr->twist.twist.angular.y;
    odometry_data.angular.z = wo_msg_ptr->twist.twist.angular.z;
    
    new_odometry_data_.push_back(odometry_data);

    buff_mutex_.unlock();
}

void WoSubscriber::ParseData(std::deque<OdometryData>& odometry_data_buff){
    buff_mutex_.lock();

    if(new_odometry_data_.size() > 0){
        odometry_data_buff.insert(odometry_data_buff.end(), new_odometry_data_.begin(), new_odometry_data_.end());
        new_odometry_data_.clear();
    }

    buff_mutex_.unlock();
}


}
