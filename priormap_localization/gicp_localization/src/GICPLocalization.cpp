#include <gicp_localization/GICPLocalization.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>


#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;
namespace gr = gu::ros;

using pcl::copyPointCloud;
using pcl::GeneralizedIterativeClosestPoint;
using pcl::PointCloud;
using pcl::PointXYZ;

GICPLocalization::GICPLocalization() : initialized_(false){
        slammap_.reset(new PointCloud);
        priormap_.reset(new PointCloud);
}

GICPLocalization::~GICPLocalization(){
}

bool GICPLocalization::Initialize(const ros::NodeHandle& n){
        name_ = ros::names::append(n.getNamespace(), "GICPLocalization");

        if(!LoadParameters(n)) {
                ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
                return false;
        }

        if(!RegisterCallbacks(n)) {
                ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
                return false;
        }

        initial_offset_estimate_.translation = gu::Vec3(1.7, 2.61, 0);
        initial_offset_estimate_.rotation = gu::Rot3(1, 0, 0,
                                                     0, 1, 0,
                                                     0, 0, 1);

        geometry_msgs::PoseStamped ros_pose;
        ros_pose.pose = gr::ToRosPose(initial_offset_estimate_);
        ros_pose.header.frame_id = fixed_frame_id_;
        ros_pose.header.stamp = ros::Time::now();

        // Publish base_link tf
        geometry_msgs::TransformStamped odo_trans;
        odo_trans.header.stamp = ros::Time::now();
        odo_trans.header.frame_id = fixed_frame_id_;
        odo_trans.child_frame_id = slamemap_frame_id_;
        odo_trans.transform.translation.x = ros_pose.pose.position.x;
        odo_trans.transform.translation.y = ros_pose.pose.position.y;
        odo_trans.transform.translation.z = ros_pose.pose.position.z;
        odo_trans.transform.rotation.w = ros_pose.pose.orientation.w;
        odo_trans.transform.rotation.x = ros_pose.pose.orientation.x;
        odo_trans.transform.rotation.y = ros_pose.pose.orientation.y;
        odo_trans.transform.rotation.z = ros_pose.pose.orientation.z;

        odo_trans_ = odo_trans;

        return true;
}

bool GICPLocalization::LoadParameters(const ros::NodeHandle& n){
        // Load frame ids.
        if(!pu::Get("frame_id_fixed", fixed_frame_id_)) return false;
        if(!pu::Get("frame_id_slammap", slamemap_frame_id_)) return false;

        // Load algorithm parameters.
        if(!pu::Get("icp_tf_epsilon", params_.icp_tf_epsilon)) return false;
        if(!pu::Get("icp_corr_dist", params_.icp_corr_dist)) return false;
        if(!pu::Get("icp_iterations", params_.icp_iterations)) return false;

        // Load prior map information.
        if(!pu::Get("prior_map_path", prior_map_path_)) return false;

        // Load slam map topic
        if(!pu::Get("slam_map_topic", slam_map_topic_)) return false;

        return true;
}

bool GICPLocalization::RegisterCallbacks(const ros::NodeHandle& n){
        // Create a local nodehandle to manage callback subscriptions.
        ros::NodeHandle nl(n);

        initial_offset_pub_ = nl.advertise<geometry_msgs::PoseStamped>(
                "initial_offset", 10, false);

        slammap_sub_ = nl.subscribe(slam_map_topic_, 10, &GICPLocalization::PointCloudCallback, this);

        return true;
}

void GICPLocalization::PointCloudCallback(const PointCloud& SLAMMap){

        UpdateInitialOffset(SLAMMap);
}
bool GICPLocalization::UpdateInitialOffset(const PointCloud& SLAMMap){

        // If this is the first point cloud, load prior map and slam map.
        if(!initialized_) {
                // Load prior map
                if(pcl::io::loadPCDFile<PointXYZ> (prior_map_path_, *priormap_) == -1)
                {
                        ROS_ERROR("%s: Failed to load prior map.", name_.c_str());
                        return false;
                }
        }

        // Move incoming slam map into slammap_ point cloud container
        copyPointCloud(SLAMMap, *slammap_);


        // Update initial offset via GICP.
        return UpdateGICP();
}

bool GICPLocalization::UpdateGICP(){
        // Compute the transformation between prior map and slam map.
        GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> gicp;
        gicp.setTransformationEpsilon(params_.icp_tf_epsilon);
        gicp.setMaxCorrespondenceDistance(params_.icp_corr_dist);
        gicp.setMaximumIterations(params_.icp_iterations);
        gicp.setRANSACIterations(0);

        gicp.setInputSource(slammap_);
        gicp.setInputTarget(priormap_);

        PointCloud unused_result;
        // gicp.align(unused_result);

        const Eigen::Matrix4f T = gicp.getFinalTransformation();

        // Update initial offset
        // initial_offset_estimate_.translation = gu::Vec3(T(0,3), T(1,3), T(2,3));
        // initial_offset_estimate_.rotation = gu::Rot3(T(0,0), T(0,1), T(0,2),
                                                     // T(1,0), T(1,1), T(1,2),
                                                     // T(2,0), T(2,1), T(2,2));

	initial_offset_estimate_.translation = gu::Vec3(1.7, 2.61, 0);
        initial_offset_estimate_.rotation = gu::Rot3(1, 0, 0,
                                                     0, 1, 0,
                                                     0, 0, 1);
        
        // Convert intial pose estimate to ROS format and publish.
        PublishInitialOffset(initial_offset_estimate_, initial_offset_pub_);

}

// Publish estimated initial offset.
void GICPLocalization::PublishInitialOffset(const gu::Transform3& pose,
                                            const ros::Publisher& pub){
        // // Check for subscribers before doing any work.
        // if(pub.getNumSubscribers() == 0)
        //         return;

        // Convert from gu::Transform3 to ROS's PoseStamped type and publish.
        geometry_msgs::PoseStamped ros_pose;
        ros_pose.pose = gr::ToRosPose(pose);
        ros_pose.header.frame_id = fixed_frame_id_;
        ros_pose.header.stamp = ros::Time::now();
        pub.publish(ros_pose);

        // Publish base_link tf
        geometry_msgs::TransformStamped odo_trans;
        odo_trans.header.stamp = ros::Time::now();
        odo_trans.header.frame_id = fixed_frame_id_;
        odo_trans.child_frame_id = slamemap_frame_id_;
        odo_trans.transform.translation.x = ros_pose.pose.position.x;
        odo_trans.transform.translation.y = ros_pose.pose.position.y;
        odo_trans.transform.translation.z = ros_pose.pose.position.z;
        odo_trans.transform.rotation.w = ros_pose.pose.orientation.w;
        odo_trans.transform.rotation.x = ros_pose.pose.orientation.x;
        odo_trans.transform.rotation.y = ros_pose.pose.orientation.y;
        odo_trans.transform.rotation.z = ros_pose.pose.orientation.z;

        odo_trans_ = odo_trans;

        

}

geometry_msgs::TransformStamped GICPLocalization::GetTf(){
    // std::cout<<odo_trans_.header.frame_id<<std::endl;
    odo_trans_.header.stamp = ros::Time::now();
    return odo_trans_;
}
