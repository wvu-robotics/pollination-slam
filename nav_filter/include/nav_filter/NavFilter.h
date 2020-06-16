/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: Ryan
 */

#ifndef NAV_FILTER_H
#define NAV_FILTER_H

#include <ros/ros.h>
#include <geometry_utils/Transform3.h>

#include <Eigen/Dense>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>

class NavFilter {
public:

typedef sensor_msgs::Imu ImuData;
typedef nav_msgs::Odometry OdoData;
typedef geometry_msgs::PoseStamped PoseData;

gtsam::Vector6 imu;
gtsam::NavState odo;
gtsam::Pose3 gicp_tran;
gtsam::Pose3 gtsam_pose;
gtsam::NavState odo_prev;
gtsam::NavState odo_diff;
gtsam::NavState prop_state;
gtsam::NavState prev_state;
gtsam::NavState odo_offset;
gtsam::Pose3 slam_pose, odo_pose;
gtsam::Vector3 gtsam_vel, odo_vel;
gtsam::imuBias::ConstantBias prev_bias;
gtsam::PreintegrationType *imu_preintegrated_;
boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(9.81);

NavFilter();
~NavFilter();

// Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
bool Initialize(const ros::NodeHandle& n);

private:
// Node initialization
bool InitGtsam(const ros::NodeHandle& n);
bool LoadParameters(const ros::NodeHandle& n);
bool RegisterCallbacks(const ros::NodeHandle& n);

// Publish estimated  states.
void PublishStates(const gtsam::NavState& pose, const ros::Publisher& pub);

// Publish estimated  states.
void PublishTransformedStates(const gtsam::NavState& pose,
                              const gtsam::Pose3& trans,
                              const ros::Publisher& pub);

// Publish estimated path
void PublishPath(const gtsam::Pose3& pose, const ros::Publisher& pub);

void ImuCallback(const ImuData& imu_data_);
void OdoCallback(const OdoData& odo_data_);
void SlamCallback(const OdoData& slam_data_);
void GicpCallback(const PoseData& gicp_data_);

typedef gtsam::noiseModel::Isotropic isoNoise;
typedef gtsam::noiseModel::Diagonal diagNoise;

void UpdateGraph(const gtsam::Vector6& imu, const gtsam::NavState& odo);
gtsam::Vector6 ImuToGtsam(const ImuData& imu_data_);
gtsam::NavState OdoToGtsam(const OdoData& odo_data_);
gtsam::Pose3 PoseStampedToGtsam(const PoseData& slam_data_);

std::unique_ptr<gtsam::ISAM2> isam_;
gtsam::Values values_;

gtsam::NonlinearFactorGraph graph_;
gtsam::Values new_value_;

// The node's name.
std::string name_;

// Subscriber
ros::Subscriber imu_sub_;
ros::Subscriber odo_sub_;
ros::Subscriber slam_sub_;
ros::Subscriber gicp_sub_;

// Publisher.
ros::Publisher states_pub_;
ros::Publisher updated_path_pub_;
ros::Publisher transformed_states_pub;
tf::TransformBroadcaster states_tf_broad;

OdoData odo_data_;
OdoData odo_data_prev_;
ImuData imu_data_;
PoseData slam_data_;
PoseData gicp_data_;
bool has_odo_ = false;
bool first_odo_ = true;
bool has_slam_ = false;

// Most recent time stamp for publishers.
ros::Time stamp_;

// Coordinate frames.
std::string frame_id_out_;
std::string frame_id_imu_;
std::string frame_id_odo_;
std::string frame_id_fixed_;

// update rate [hz]
unsigned int publish_hz_;
unsigned int sensor_pub_rate_;

// sub. topics
std::string imu_topic_;
std::string odo_topic_;
std::string slam_topic_;
std::string gicp_topic_;

// For initialization.
bool initialized_;

// imu noise params
double accel_sigma_, accel_rw_, gyro_sigma_, gyro_rw_;

// filter noise params
double position_noise_, attitude_noise_, velocity_noise_, bias_noise_;

// initial pose
unsigned int key_ = 0;
double init_x, init_y, init_z, init_vx, init_vy, init_vz;
double init_qx, init_qy, init_qz, init_qw, sigma_x, sigma_y;
double sigma_z, sigma_vx, sigma_vy, prev_stamp_, up_time_;
double sigma_vz, sigma_roll, sigma_pitch, sigma_yaw, cur_stamp_;

};
#endif
