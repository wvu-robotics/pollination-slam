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
 * Authors: Ryan; Chizhao Yang
 */

// #ifndef NAV_FILTER_H
// #define NAV_FILTER_H

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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

// typedef gtsam::noiseModel::Isotropic isoNoise;
// typedef gtsam::noiseModel::Diagonal diagNoise;
// typedef sensor_msgs::Imu ImuData;
// typedef nav_msgs::Odometry OdoData;
// typedef geometry_msgs::PoseStamped PoseData;

class NavFilter {

private:
  ros::NodeHandle& nh;
  gtsam::Vector6 _imu;
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

  nav_msgs::Odometry odo_data_;
  nav_msgs::Odometry odo_data_prev_;
  sensor_msgs::Imu imu_data_;
  geometry_msgs::PoseStamped slam_data_;
  geometry_msgs::PoseStamped gicp_data_;
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

public:

  NavFilter():
    nh("~")
    {
      // Load frame ids.
          if(!nh.getParam("frames/frame_id_out", frame_id_out_)) ROS_ERROR("Failed to get param 'frames/frame_id_out'");
          if(!nh.getParam("frames/frame_id_imu", frame_id_imu_)) ROS_ERROR("Failed to get param 'frames/frame_id_imu'");
          if(!nh.getParam("frames/frame_id_odo", frame_id_odo_)) ROS_ERROR("Failed to get param 'frames/frame_id_odo'");
          // if(!nh.getParam("frames/frame_id_fixed", frame_id_fixed_)) ROS_ERROR("Failed to get param ''");

          // Load topics
          if(!nh.getParam("imu/topic", imu_topic_)) ROS_ERROR("Failed to get param 'imu/topic'");
          if(!nh.getParam("odo/topic", odo_topic_)) ROS_ERROR("Failed to get param 'odo/topic'");
          if(!nh.getParam("slam/topic", slam_topic_)) ROS_ERROR("Failed to get param 'slam/topic'");
          if(!nh.getParam("gicp_tran/topic", gicp_topic_)) ROS_ERROR("Failed to get param 'gicp_tran/topic'");

          // Load update rate
          if(!nh.getParam("imu/publish_hz", publish_hz_)) ROS_ERROR("Failed to get param 'imu/publish_hz'");
          if(!nh.getParam("imu/sensor_pub_rate", sensor_pub_rate_)) ROS_ERROR("Failed to get param 'imu/sensor_pub_rate'");

          // Load imu noise specs
          if(!nh.getParam("imu/noise/accel_noise_sigma", accel_sigma_)) ROS_ERROR("Failed to get param 'imu/noise/accel_noise_sigma'");
          if(!nh.getParam("imu/noise/gyro_noise_sigma", gyro_sigma_)) ROS_ERROR("Failed to get param 'imu/noise/gyro_noise_sigma'");
          if(!nh.getParam("imu/noise/accel_bias_rw_sigma", accel_rw_)) ROS_ERROR("Failed to get param 'imu/noise/accel_bias_rw_sigma'");
          if(!nh.getParam("imu/noise/gyro_bias_rw_sigma", gyro_rw_)) ROS_ERROR("Failed to get param 'imu/noise/gyro_bias_rw_sigma'");
          if(!nh.getParam("filter/position_noise", position_noise_)) ROS_ERROR("Failed to get param 'filter/position_noise'");
          if(!nh.getParam("filter/attitude_noise", attitude_noise_)) ROS_ERROR("Failed to get param 'filter/attitude_noise'");
          if(!nh.getParam("filter/velocity_noise", velocity_noise_)) ROS_ERROR("Failed to get param 'filter/velocity_noise'");
          if(!nh.getParam("filter/bias_noise", bias_noise_)) ROS_ERROR("Failed to get param 'filter/bias_noise'");

          // Load initial position and orientation.
          if (!nh.getParam("init/position/x", init_x)) ROS_ERROR("Failed to get param 'init/position/x'");
          if (!nh.getParam("init/position/y", init_y)) ROS_ERROR("Failed to get param 'init/position/y'");
          if (!nh.getParam("init/position/z", init_z)) ROS_ERROR("Failed to get param 'init/position/z'");
          if (!nh.getParam("init/velocity/vx", init_vx)) ROS_ERROR("Failed to get param 'init/velocity/vx'");
          if (!nh.getParam("init/velocity/vy", init_vy)) ROS_ERROR("Failed to get param 'init/velocity/vy'");
          if (!nh.getParam("init/velocity/vz", init_vz)) ROS_ERROR("Failed to get param 'init/velocity/vz'");
          if (!nh.getParam("init/orientation/qx", init_qx)) ROS_ERROR("Failed to get param 'init/orientation/qx'");
          if (!nh.getParam("init/orientation/qy", init_qy)) ROS_ERROR("Failed to get param 'init/orientation/qy'");
          if (!nh.getParam("init/orientation/qz", init_qz)) ROS_ERROR("Failed to get param 'init/orientation/qz'");
          if (!nh.getParam("init/orientation/qw", init_qw)) ROS_ERROR("Failed to get param 'init/orientation/qw'");

          // Load initial position and orientation noise.
          if (!nh.getParam("init/position_sigma/x", sigma_x)) ROS_ERROR("Failed to get param 'init/position_sigma/x'");
          if (!nh.getParam("init/position_sigma/y", sigma_y)) ROS_ERROR("Failed to get param 'init/position_sigma/y'");
          if (!nh.getParam("init/position_sigma/z", sigma_z)) ROS_ERROR("Failed to get param 'init/position_sigma/z'");
          if (!nh.getParam("init/velocity/vx", sigma_vx)) ROS_ERROR("Failed to get param 'init/velocity/vx'");
          if (!nh.getParam("init/velocity/vy", sigma_vy)) ROS_ERROR("Failed to get param 'init/velocity/vy'");
          if (!nh.getParam("init/velocity/vz", sigma_vz)) ROS_ERROR("Failed to get param 'init/velocity/vz'");
          if (!nh.getParam("init/orientation_sigma/roll", sigma_roll)) ROS_ERROR("Failed to get param 'init/orientation_sigma/roll'");
          if (!nh.getParam("init/orientation_sigma/pitch", sigma_pitch)) ROS_ERROR("Failed to get param 'init/orientation_sigma/pitch'");
          if (!nh.getParam("init/orientation_sigma/yaw", sigma_yaw)) ROS_ERROR("Failed to get param 'init/orientation_sigma/yaw'");

          // Initialize the GTSAM
          initGtsam();
          // Register call backs
          states_pub_ = nh.advertise<nav_msgs::Odometry>( "states", 25, false);

          // transformed_states_pub = nl.advertise<nav_msgs::Odometry>( "transformed_states", 25, false);

          updated_path_pub_ = nh.advertise<nav_msgs::Path>("nav_path", 10, false);

          imu_sub_ = nh.subscribe(imu_topic_,  10, &NavFilter::ImuCallback, this);
          odo_sub_ = nh.subscribe(odo_topic_,  10, &NavFilter::OdoCallback, this);
          slam_sub_ = nh.subscribe(slam_topic_,10, &NavFilter::SlamCallback, this);
          // gicp_sub_ = nh.subscribe(gicp_topic_,10, &NavFilter::GicpCallback, this);
  }
  ~NavFilter()

  // Node initialization
  bool initGtsam(){
    using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
    using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
    //using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
    // gtsam::Pose3 gicp_tran(gtsam::Rot3::Quaternion(1.0,0.0,0.0,0.0),gtsam::Point3(0.0,0.0,0.0));
    gtsam::Matrix33 measured_acc_cov = gtsam::Matrix33::Identity(3,3) * pow(accel_sigma_,2);
    gtsam::Matrix33 measured_omega_cov = gtsam::Matrix33::Identity(3,3) * pow(gyro_sigma_,2);
    gtsam::Matrix33 integration_error_cov = gtsam::Matrix33::Identity(3,3)*1e-6;
    gtsam::Matrix33 bias_acc_cov = gtsam::Matrix33::Identity(3,3) * pow(accel_rw_,2);
    gtsam::Matrix33 bias_omega_cov = gtsam::Matrix33::Identity(3,3) * pow(gyro_rw_,2);
    gtsam::Matrix66 bias_acc_omega_int = gtsam::Matrix::Identity(6,6)*1e-5;

    p->accelerometerCovariance = measured_acc_cov;
    p->integrationCovariance = integration_error_cov;
    // should be using 2nd order integration
    p->gyroscopeCovariance = measured_omega_cov;
    p->biasAccCovariance = bias_acc_cov;
    p->biasOmegaCovariance = bias_omega_cov;
    p->biasAccOmegaInt = bias_acc_omega_int;

    //gtsam::imuBias::ConstantBias imu_bias; // assume zero initial bias
    // imu_preintegrated_ = new gtsam::PreintegratedCombinedMeasurements(p, imu_bias);

    gtsam::noiseModel::Diagonal::shared_ptr pose_noise_model = gtsam::noiseModel::Diagonal::Sigmas(( gtsam::Vector(6) << attitude_noise_, attitude_noise_, attitude_noise_, position_noise_, position_noise_, position_noise_ ).finished());

    gtsam::noiseModel::Diagonal::shared_ptr velocity_noise_model = gtsam::noiseModel::Isotropic::Sigma(3,velocity_noise_);
    //gtsam::noiseModel::Diagonal::shared_ptr bias_noise_model = gtsam::noiseModel::Isotropic::Sigma(6,bias_noise_);

    gtsam::Rot3 init_rot = gtsam::Rot3::Quaternion(init_qw, init_qx, init_qy, init_qz);
    gtsam::Point3 init_tran(init_x, init_y, init_z);
    gtsam::Pose3 init_pose(init_rot, init_tran);

    gtsam::Vector3 init_vel(init_vx, init_vy, init_vz);
    prev_state = gtsam::NavState(init_pose, init_vel);

    new_value_.insert(X(key_), init_pose);
    graph_.add(gtsam::PriorFactor<Pose3>(X(key_), init_pose, pose_noise_model));

    graph_.add(gtsam::PriorFactor<Vector3>(V(key_), init_vel, velocity_noise_model));
    new_value_.insert(V(key_), init_vel);

    //graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(key_), imu_bias, bias_noise_model));
    //new_value_.insert(B(key_), imu_bias);

    // isam_->update(graph_, new_value_);
    // values_ = isam_->calculateEstimate();
    values_ = gtsam::LevenbergMarquardtOptimizer(graph_, new_value_).optimize();
    key_++;
  }

  void ImuCallback(const sensor_msgs::Imu& imu_data_){
    _imu = ImuToGtsam(imu_data_);
  }

  void SlamCallback(const nav_msgs::Odometry& slam_data_){
    // geometry_msgs::PoseStamped slam_data;
    // slam_data.header = slam_data_.header;
    // slam_data.pose = slam_data_.pose.pose;
    slam_pose = OdoToGtsam(slam_data_);
    has_slam_ = true;
    return;
  }

  void OdoCallback(const nav_msgs::Odometry& odo_data_){
    odo = OdoToGtsam(odo_data_);
    if (first_odo_) { odo_prev = odo; first_odo_ = false; }
    cur_stamp_ = (odo_data_.header.stamp).toSec();
    has_odo_ = true;
    UpdateGraph(_imu,odo);
  }

  gtsam::Vector6 ImuToGtsam(const sensor_msgs::Imu& imu_data_){
    gtsam::Vector6 imuVec((gtsam::Vector(6) <<    imu_data_.linear_acceleration.y,
                                 imu_data_.linear_acceleration.x,
                                 -1*imu_data_.linear_acceleration.z,
                                 imu_data_.angular_velocity.y,
                                 imu_data_.angular_velocity.x,
                                 -1*imu_data_.angular_velocity.z).finished());

          return imuVec;
  }

  // gtsam::Pose3 PoseStampedToGtsam(const geometry_msgs::PoseStamped& slam_data_){
  //   gtsam::Point3 slam_point(slam_data_.pose.position.x,
  //                                  slam_data_.pose.position.y,
  //                                  slam_data_.pose.position.z);
  //
  //         gtsam::Rot3 slam_rot = gtsam::Rot3::Quaternion(slam_data_.pose.orientation.w,
  //                                                        slam_data_.pose.orientation.x,
  //                                                        slam_data_.pose.orientation.y,
  //                                                        slam_data_.pose.orientation.z);
  //
  //         gtsam::Pose3 slam(slam_rot, slam_point);
  //         return slam;
  // }

  gtsam::NavState OdoToGtsam(const nav_msgs::Odometry& odo_data_){
    gtsam::Point3 odoXYZ(odo_data_.pose.pose.position.x,
                         odo_data_.pose.pose.position.y,
                         odo_data_.pose.pose.position.z );

    gtsam::Rot3 odoRot = gtsam::Rot3::Quaternion(odo_data_.pose.pose.orientation.w,
                                                 odo_data_.pose.pose.orientation.x,
                                                 odo_data_.pose.pose.orientation.y,
                                                 odo_data_.pose.pose.orientation.z);

    gtsam::Pose3 odoPose(odoRot, odoXYZ);

    gtsam::Vector3 odoVel(odo_data_.twist.twist.linear.x,
                          odo_data_.twist.twist.linear.y,
                          odo_data_.twist.twist.linear.z);

    gtsam::NavState odoStates(odoPose, odoVel);
    return odoStates;
  }

  void UpdateGraph(const gtsam::Vector6& imu, const gtsam::NavState& odo){
    using gtsam::symbol_shorthand::X;
    using gtsam::symbol_shorthand::V;
    //using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

    // Assemble prior noise model and add it the graph.
    diagNoise::shared_ptr pose_noise_model = diagNoise::Sigmas((
        gtsam::Vector(6) << attitude_noise_, attitude_noise_, attitude_noise_, position_noise_, position_noise_, position_noise_).finished());


    diagNoise::shared_ptr pose_noise_model_slam = diagNoise::Sigmas(( gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());

    diagNoise::shared_ptr pose_noise_model_odo = diagNoise::Sigmas(( gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.5, 0.5, 0.5).finished());

    diagNoise::shared_ptr velocity_noise_model = isoNoise::Sigma(3,velocity_noise_);
    //diagNoise::shared_ptr bias_noise_model = isoNoise::Sigma(6,bias_noise_);

    double dt = 1/double(sensor_pub_rate_);
    double up_rate = 1/double(publish_hz_);
    double step = std::abs(cur_stamp_ - prev_stamp_);

    // if ( step<=up_rate && !has_slam_ ) {
    //         //imu_preintegrated_->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
    // }
    // else {
    // gtsam::PreintegratedCombinedMeasurements *preint_imu_combined = dynamic_cast<gtsam::PreintegratedCombinedMeasurements*>(imu_preintegrated_);
    //
    // gtsam::CombinedImuFactor imu_factor(X(key_-1), V(key_-1), X(key_), V(key_),  B(key_-1), B(key_),  *preint_imu_combined);
    //
    // graph_.add(imu_factor);

    //NavState prop_state = imu_preintegrated_->predict(prev_state, prev_bias);
    if (has_slam_) {
            graph_.add(gtsam::PriorFactor<Pose3>(X(key_), slam_pose, pose_noise_model_slam));
            graph_.add(gtsam::PriorFactor<Vector3>(V(key_), odo.v(), velocity_noise_model));

            // odo_vel = prev_state.v() + ( odo.v() - odo_prev.v() );  //Not be used.
            odo_pose = (prev_state.pose())*(odo.pose().between(odo_prev.pose()));

            graph_.add(gtsam::PriorFactor<Pose3>(X(key_), odo_pose, pose_noise_model_odo));

            new_value_.insert(X(key_), slam_pose);
            new_value_.insert(V(key_), prev_state.v());
            values_ = gtsam::LevenbergMarquardtOptimizer(graph_, new_value_).optimize();

            gtsam::Pose3 gtsam_pose = values_.at<gtsam::Pose3>(X(key_));
            gtsam::Vector3 gtsam_vel = values_.at<gtsam::Vector3>(V(key_));
            prev_state = gtsam::NavState( gtsam_pose, gtsam_vel );

            graph_.resize(0);
            new_value_.clear();
            values_.clear();

            graph_.add(gtsam::PriorFactor<Pose3>(X(key_), gtsam_pose, pose_noise_model_slam));
            graph_.add(gtsam::PriorFactor<Vector3>(V(key_), gtsam_vel, velocity_noise_model));
            // add odo priors here
            new_value_.insert(X(key_), prev_state.pose());
            new_value_.insert(V(key_), prev_state.v());
            key_++;
            has_odo_ = false;
            has_slam_ = false;
    }
    else{
            gtsam_vel = prev_state.v() + ( odo.v() - odo_prev.v() );
            gtsam_pose = (prev_state.pose())*(odo.pose().between(odo_prev.pose()));
            prev_state = NavState( gtsam_pose, gtsam_vel );
            odo_prev = odo;
            has_odo_ = false;
    }

    //graph_.add(PriorFactor<gtsam::imuBias::ConstantBias>(B(key_), prev_bias, bias_noise_model));
    //new_value_.insert(B(key_), prev_bias);

    //isam_->update(graph_, new_value_);
    //values_ = isam_->calculateEstimate();
    //values_ = LevenbergMarquardtOptimizer(graph_, new_value_).optimize();

    // Pose3 gtsam_pose = values_.at<Pose3>(X(key_));
    // Vector3 gtsam_vel = values_.at<Vector3>(V(key_));

    //PublishPath(gtsam_pose, updated_path_pub_);
    PublishStates(prev_state, states_pub_);
    // PublishTransformedStates(prev_state, gicp_tran, transformed_states_pub);
    //prev_bias = values_.at<gtsam::imuBias::ConstantBias>(B(key_));

    // Reset the preintegration object.
    //imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);
    // graph_.resize(0);
    // new_value_.clear();
    // values_.clear();
    // insert prev. values_ for new loop
    // graph_.add(PriorFactor<Pose3>(X(key_), gtsam_pose, pose_noise_model));

    // graph_.add(PriorFactor<Vector3>(V(key_), gtsam_vel, velocity_noise_model));

    //graph_.add(PriorFactor<gtsam::imuBias::ConstantBias>(B(key_), prev_bias, bias_noise_model));

    // new_value_.insert(X(key_), prev_state.pose());
    // new_value_.insert(V(key_), prev_state.v());
    //new_value_.insert(B(key_), prev_bias);

    //key_++;
    prev_stamp_ = cur_stamp_;
    // }
    return;
  }

  // Publish estimated  states.
  void PublishStates(const gtsam::NavState& states, const ros::Publisher& pub){
    // // Check for subscribers before doing any work.
         // if(pub.getNumSubscribers() == 0)
         //         return;

         Vector3 position = states.pose().translation();
         Vector4 attitude = states.pose().rotation().quaternion();
         Vector3 velocity = states.v();

         // Convert from NavState to nav_msgs::Odo
         nav_msgs::Odometry odo_msg;
         odo_msg.pose.pose.position.x = position[0];
         odo_msg.pose.pose.position.y = position[1];
         odo_msg.pose.pose.position.z = position[2];
         odo_msg.twist.twist.linear.x = velocity[0];
         odo_msg.twist.twist.linear.y = velocity[1];
         odo_msg.twist.twist.linear.z = velocity[2];
         odo_msg.pose.pose.orientation.w = attitude[0];
         odo_msg.pose.pose.orientation.x = attitude[1];
         odo_msg.pose.pose.orientation.y = attitude[2];
         odo_msg.pose.pose.orientation.z = attitude[3];
         odo_msg.header.frame_id = frame_id_out_;
         odo_msg.child_frame_id = "base_link";
         odo_msg.header.stamp = stamp_;
         pub.publish(odo_msg);

         // Publish base_link tf
         geometry_msgs::TransformStamped odo_trans;
         odo_trans.header.stamp = ros::Time::now();
         odo_trans.header.frame_id = frame_id_out_;
         odo_trans.child_frame_id = "base_link";
         odo_trans.transform.translation.x = position[0];
         odo_trans.transform.translation.y = position[1];
         odo_trans.transform.translation.z = position[2];
         odo_trans.transform.rotation.w = attitude[0];
         odo_trans.transform.rotation.x = attitude[1];
         odo_trans.transform.rotation.y = attitude[2];
         odo_trans.transform.rotation.z = attitude[3];

         states_tf_broad.sendTransform(odo_trans);
  }

  // Publish estimated path
  void PublishPath(const gtsam::Pose3& pose, const ros::Publisher& pub){
    // // Check for subscribers before doing any work.
          if(pub.getNumSubscribers() == 0)
                  return;

          Vector3 position = states.translation();
          Vector4 attitude = states.rotation().quaternion();
          // Convert from NavState to nav_msgs::Odo
          nav_msgs::Path path_msg;
          path_msg.poses[key_].pose.position.x = position[0];
          path_msg.poses[key_].pose.position.y = position[1];
          path_msg.poses[key_].pose.position.z = position[2];
          path_msg.poses[key_].pose.orientation.w = attitude[0];
          path_msg.poses[key_].pose.orientation.x = attitude[1];
          path_msg.poses[key_].pose.orientation.y = attitude[2];
          path_msg.poses[key_].pose.orientation.z = attitude[3];
          path_msg.header.frame_id = frame_id_out_;
          path_msg.header.stamp = stamp_;
          pub.publish(path_msg);
  }
}
int main(int argc, char** argv){
        ros::init(argc, argv, "dead_reckoning_node");
        ros::NodeHandle n("~");

        NavFilter navfilter;
        if(!navfilter.Initialize(n)) {
                ROS_ERROR("%s: Failed to initialize the nav. filter.",
                          ros::this_node::getName().c_str());
                return EXIT_FAILURE;
        }
        ros::spin();

        return EXIT_SUCCESS;
}
