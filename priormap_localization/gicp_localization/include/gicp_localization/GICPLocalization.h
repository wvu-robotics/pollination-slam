#ifndef GICP_LOCALIZATION_H
#define GICP_LOCALIZATION_H

#include <ros/ros.h>
#include <geometry_utils/Transform3.h>
#include <tf/transform_broadcaster.h>

#include <pcl_ros/point_cloud.h>

class GICPLocalization {
	public:
		typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

		GICPLocalization();
		~GICPLocalization();

		// Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
		bool Initialize(const ros::NodeHandle& n);

		// Align incoming SLAM map with prior map.
		bool UpdateInitialOffset(const PointCloud& SLAMMap);

		geometry_msgs::TransformStamped GetTf();

	private:
		// Node initialization.
		bool LoadParameters(const ros::NodeHandle& n);
		bool RegisterCallbacks(const ros::NodeHandle& n);

		void PointCloudCallback(const PointCloud& SLAMMap);
		// Use GICP between incoming SLAM map and prior map to estimate initial offset.
		bool UpdateGICP();

		// Publish estimated initial offset.
		void PublishInitialOffset(const geometry_utils::Transform3& pose,
									const ros::Publisher& pub);

		// The node's name.
		std::string name_;

		// Subscriber
		ros::Subscriber slammap_sub_;
		// Publisher.
		ros::Publisher initial_offset_pub_;
		// Most recent point cloud time stamp for publishers.
		ros::Time stamp_;

		// Pose estimates.
		geometry_utils::Transform3 initial_offset_estimate_;

		// Coordinate frames.
  		std::string fixed_frame_id_;
  		std::string slamemap_frame_id_;

  		// For prior map information.
  		std::string prior_map_path_;

  		// The slam map topic name.
		std::string slam_map_topic_;

		

		// For initialization.
		bool initialized_;

		// Point cloud containers.
		PointCloud::Ptr slammap_;
		PointCloud::Ptr priormap_;

		// Parameters for filtering, and ICP.
		struct Parameters{
			// Stop ICP if the transfoamtion from the last iteration was this small.
			double icp_tf_epsilon;

			//During ICP, two points won't be considered a correspondence if they are at least
			// this far from one another.
			double icp_corr_dist;

			// Iterate ICP this many times.
			unsigned int icp_iterations;
		} params_;

		geometry_msgs::TransformStamped odo_trans_;

};

#endif