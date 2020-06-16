// hough transform in here works for localization in greenhouse
// input: processed lidar data: project to xy coordinate
// output: robot's pose


#ifndef HOUGH_H_
#define HOUGH_H_

#include "ros/ros.h"
#include "ros/console.h"

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

#define deg2rad 0.017453293f

class Hough
{
public:

	Hough();
	virtual ~Hough();

	ros::NodeHandle _nh;
	ros::Subscriber _sub_localmap;
	
	void GeneratePointcloud();	// generate fake 2d point cloud for testing 
	void GetPointcloud(pcl::PointCloud<pcl::PointXYZ> const &raw_cloud);	// read pointcloud from pcd file
	// void GetBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void GetBoundary();
	// hough transform to detect lines
	void Transform(pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud); 

	void DrawLines();	//draw lines for visualization

	void DetectUShape();	//detect u shape from line information

	void GetPose();

	void DrawUShape();

	bool newLocalmapAvailable();

	void setPreviousCounters();

	void clearVariables();

	// lidar data from precessed pointcloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;	

	std::vector< std::vector<int> > u_shape;

	std::vector< std::vector< int > > line_info;
	// Store lines for draw
	std::vector< std::pair< std::pair<int, int>, std::pair<int, int> > > lines;

	int threshold;	// intensity of the lines

	float min_x, min_y, max_x, max_y;

	int _accu_w, _accu_h, _img_w, _img_h;
	unsigned int* _accu;

	double global_x, global_y, global_theta;

	short int _registration_counter;
	short int _registration_counter_prev;

private:
	
	// Store lines for draw
	// std::vector< std::pair< std::pair<int, int>, std::pair<int, int> > > lines;
	bool _registration_new;
	

	

	

};

#endif /* HOUGH_H_ */
