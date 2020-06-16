#include <lidar/lidar_filtering.hpp>

LidarFilter::LidarFilter()
{
	//velodyne callback initializations
	_registration_counter = 0;
	_registration_counter_prev = 0;
	_registration_new = false;
	_sub_velodyne = _nh.subscribe("/velodyne_points", 1, &LidarFilter::velodyneCallback, this);
}

void LidarFilter::velodyneCallback(pcl::PointCloud<pcl::PointXYZI> const &input_cloud)
{
	_registration_counter = _registration_counter + 1;
	_input_cloud = input_cloud;
}

bool LidarFilter::newPointCloudAvailable()
{
	if(_registration_counter != _registration_counter_prev)
	{
		_registration_new = true;
		return true;
	}
	else
	{
		_registration_new = false;
		return false;
	}
}

void LidarFilter::setPreviousCounters()
{
	_registration_counter_prev = _registration_counter;
}

void LidarFilter::doMathMapping()
{

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

	*cloud = _input_cloud;
	//remove ground points based on point z value
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-40,40);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-40,40);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0,2);	// for hough transform localization
	pass.filter(*cloud);

	_object_filtered =  *cloud;

	// _object_filtered = _input_cloud;	// points without ground remove
	std::cout<<_object_filtered<<std::endl;

	//project points on the xy plane
	for (int i=0; i<_object_filtered.points.size(); i++)
	{
		_object_filtered.points[i].z = 0;
	}

}

void LidarFilter::packLocalMapMessage(messages::LocalMap &msg)
{
	//clear message
	msg.x.clear();
	msg.y.clear();

	for(int i=0; i<_object_filtered.points.size(); i++)
	{
		msg.x.push_back(_object_filtered.points[i].x);
		msg.y.push_back(_object_filtered.points[i].y);
	}
}

// save 2d pointcloud to pcd for viewing
void LidarFilter::savePointcloud()
{
	pcl::io::savePCDFile("pointcloud1.pcd", _object_filtered);
}

