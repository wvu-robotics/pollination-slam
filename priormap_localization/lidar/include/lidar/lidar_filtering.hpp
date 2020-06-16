#ifndef LIDAR_FILTERING_H
#define LIDAR_FILTERING_H
#include "ros/ros.h"
#include "ros/console.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <stdlib.h>
#include <pcl/filters/passthrough.h>

#include <messages/LocalMap.h>

class LidarFilter
{
public:

	LidarFilter();

	ros::NodeHandle _nh;
	ros::Subscriber _sub_velodyne;

	void doMathMapping();	//ground remove and compress to 2D
	bool newPointCloudAvailable();
	void packLocalMapMessage(messages::LocalMap &msg);
	void setPreviousCounters();
	void savePointcloud();
	// void levelingPointclouod();	//add later with nav data

	short int _registration_counter;
	short int _registration_counter_prev;
	pcl::PointCloud<pcl::PointXYZI> _object_filtered;

private:

	//registration callback
	pcl::PointCloud<pcl::PointXYZI> _input_cloud;
	bool _registration_new;

	

	void velodyneCallback(pcl::PointCloud<pcl::PointXYZI> const &input_cloud);

};
#endif // LIDAR_FILTERING_H