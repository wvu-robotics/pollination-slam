#include <lidar/lidar_filtering.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lidar_filtering_node");
	ROS_INFO("lidar_filtering_node running...");
	ros::NodeHandle nh;
	ros::Rate loop_rate(20);

	// ros::Publisher pub_local_map = nh.advertise<messages::LocalMap>("/lidar/lidarfilteringnode/localmap", 1);
	// ros::Publisher pub_local_map = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("/lidar/lidarfilteringnode/localmap", 1);
	ros::Publisher pub_pointcloud_filtered = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("/lidar_filtered", 1);

	LidarFilter lidar_filter;
	messages::LocalMap msg_LocalMap;

	while(ros::ok())
	{
		if(lidar_filter.newPointCloudAvailable())
		{
			lidar_filter.doMathMapping();
			// lidar_filter.savePointcloud();
			
			lidar_filter.setPreviousCounters();


			// pub_local_map.publish(lidar_filter._object_filtered);
			pub_pointcloud_filtered.publish(lidar_filter._object_filtered);
		}

		// lidar_filter.packLocalMapMessage(msg_LocalMap);


		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
