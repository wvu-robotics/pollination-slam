#ifndef SENSOR_FUSION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_
#define SENSOR_FUSION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_

#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace sensor_fusion{
class OdometryPublisher{
    public:
        OdometryPublisher(ros::NodeHandle& nh,
                        std::string topic_name,
                        std::string base_frame_id,
                        std::string child_frame_id,
                        int buff_size);
        OdometryPublisher() = default;

        void Publish(const Eigen::Vector3d& t, const Eigen::Quaterniond& q, 
		const Eigen::Vector3d& linear, const Eigen::Vector3d& angular, double time);

        bool hasSubscribers();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        nav_msgs::Odometry odometry_;


};
}


#endif
