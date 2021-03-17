#include "sensor_fusion/publisher/odometry_publisher.hpp"

namespace sensor_fusion{
OdometryPublisher::OdometryPublisher(ros::NodeHandle& nh,
                                    std::string topic_name,
                                    std::string base_frame_id,
                                    std::string child_frame_id,
                                    int buff_size)
    :nh_(nh){
    
    publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
    odometry_.header.frame_id = base_frame_id;
    odometry_.child_frame_id = child_frame_id;
}

void OdometryPublisher::Publish(const Eigen::Vector3d& t, const Eigen::Quaterniond& q, double time){
    ros::Time ros_time(time);

    odometry_.header.stamp = ros_time;

    odometry_.pose.pose.position.x = t.x();
    odometry_.pose.pose.position.y = t.y();
    odometry_.pose.pose.position.z = t.z();

    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    odometry_.pose.pose.orientation.w = q.w();

    publisher_.publish(odometry_);

}

bool OdometryPublisher::hasSubscribers(){
    return publisher_.getNumSubscribers() != 0;
}

}