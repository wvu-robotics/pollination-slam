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

#include <nav_filter/NavFilter.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <geometry_msgs/TransformStamped.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;
namespace gr = gu::ros;

using gtsam::Rot3;
using gtsam::ISAM2;
using gtsam::Pose3;
using gtsam::Point3;
using gtsam::Symbol;
using gtsam::Values;
using gtsam::Vector3;
using gtsam::Vector4;
using gtsam::Vector6;
using gtsam::Vector9;
using gtsam::NavState;
using gtsam::PriorFactor;
using gtsam::ISAM2Params;
using gtsam::BetweenFactor;
using gtsam::CombinedImuFactor;
using gtsam::NonlinearFactorGraph;
using gtsam::LevenbergMarquardtOptimizer;

NavFilter::NavFilter() : key_(0), initialized_(false){
}

NavFilter::~NavFilter(){
}

bool NavFilter::Initialize(const ros::NodeHandle& n){
        name_ = ros::names::append(n.getNamespace(), "NavFilter");

        if(!LoadParameters(n)) {
                ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
                return false;
        }

        if(!InitGtsam(n)) {
                ROS_ERROR("%s: Failed to initialize GTSAM.", name_.c_str());
                return false;
        }

        if(!RegisterCallbacks(n)) {
                ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
                return false;
        }

        return true;
}

bool NavFilter::InitGtsam(const ros::NodeHandle& n){

        using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
        using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
        //using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
        gtsam::Pose3 gicp_tran(gtsam::Rot3::Quaternion(1.0,0.0,0.0,0.0),gtsam::Point3(0.0,0.0,0.0));
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

        diagNoise::shared_ptr pose_noise_model = diagNoise::Sigmas(( gtsam::Vector(6) << attitude_noise_, attitude_noise_, attitude_noise_, position_noise_, position_noise_, position_noise_ ).finished());

        diagNoise::shared_ptr velocity_noise_model = isoNoise::Sigma(3,velocity_noise_);
        //diagNoise::shared_ptr bias_noise_model = isoNoise::Sigma(6,bias_noise_);

        Rot3 init_rot = Rot3::Quaternion(init_qw, init_qx, init_qy, init_qz);
        Point3 init_tran(init_x, init_y, init_z);
        Pose3 init_pose(init_rot, init_tran);

        gtsam::Vector3 init_vel(init_vx, init_vy, init_vz);
        prev_state = NavState(init_pose, init_vel);

        new_value_.insert(X(key_), init_pose);
        graph_.add(PriorFactor<Pose3>(X(key_), init_pose, pose_noise_model));

        graph_.add(PriorFactor<Vector3>(V(key_), init_vel, velocity_noise_model));
        new_value_.insert(V(key_), init_vel);

        //graph_.add(PriorFactor<gtsam::imuBias::ConstantBias>(B(key_), imu_bias, bias_noise_model));
        //new_value_.insert(B(key_), imu_bias);

        // isam_->update(graph_, new_value_);
        // values_ = isam_->calculateEstimate();
        values_ = LevenbergMarquardtOptimizer(graph_, new_value_).optimize();
        key_++;

        return true;
}

bool NavFilter::LoadParameters(const ros::NodeHandle& n){
        // Load frame ids.
        if(!pu::Get("frames/frame_id_out", frame_id_out_)) return false;
        if(!pu::Get("frames/frame_id_imu", frame_id_imu_)) return false;
        if(!pu::Get("frames/frame_id_odo", frame_id_odo_)) return false;
        // if(!pu::Get("frames/frame_id_fixed", frame_id_fixed_)) return false;

        // Load topics
        if(!pu::Get("imu/topic", imu_topic_)) return false;
        if(!pu::Get("odo/topic", odo_topic_)) return false;
        if(!pu::Get("slam/topic", slam_topic_)) return false;
        if(!pu::Get("gicp_tran/topic", gicp_topic_)) return false;

        // Load update rate
        if(!pu::Get("imu/publish_hz", publish_hz_)) return false;
        if(!pu::Get("imu/sensor_pub_rate", sensor_pub_rate_)) return false;

        // Load imu noise specs
        if(!pu::Get("imu/noise/accel_noise_sigma", accel_sigma_)) return false;
        if(!pu::Get("imu/noise/gyro_noise_sigma", gyro_sigma_)) return false;
        if(!pu::Get("imu/noise/accel_bias_rw_sigma", accel_rw_)) return false;
        if(!pu::Get("imu/noise/gyro_bias_rw_sigma", gyro_rw_)) return false;
        if(!pu::Get("filter/position_noise", position_noise_)) return false;
        if(!pu::Get("filter/attitude_noise", attitude_noise_)) return false;
        if(!pu::Get("filter/velocity_noise", velocity_noise_)) return false;
        if(!pu::Get("filter/bias_noise", bias_noise_)) return false;

        // Load initial position and orientation.
        if (!pu::Get("init/position/x", init_x)) return false;
        if (!pu::Get("init/position/y", init_y)) return false;
        if (!pu::Get("init/position/z", init_z)) return false;
        if (!pu::Get("init/velocity/vx", init_vx)) return false;
        if (!pu::Get("init/velocity/vy", init_vy)) return false;
        if (!pu::Get("init/velocity/vz", init_vz)) return false;
        if (!pu::Get("init/orientation/qx", init_qx)) return false;
        if (!pu::Get("init/orientation/qy", init_qy)) return false;
        if (!pu::Get("init/orientation/qz", init_qz)) return false;
        if (!pu::Get("init/orientation/qw", init_qw)) return false;

        // Load initial position and orientation noise.
        if (!pu::Get("init/position_sigma/x", sigma_x)) return false;
        if (!pu::Get("init/position_sigma/y", sigma_y)) return false;
        if (!pu::Get("init/position_sigma/z", sigma_z)) return false;
        if (!pu::Get("init/velocity/vx", sigma_vx)) return false;
        if (!pu::Get("init/velocity/vy", sigma_vy)) return false;
        if (!pu::Get("init/velocity/vz", sigma_vz)) return false;
        if (!pu::Get("init/orientation_sigma/roll", sigma_roll)) return false;
        if (!pu::Get("init/orientation_sigma/pitch", sigma_pitch)) return false;
        if (!pu::Get("init/orientation_sigma/yaw", sigma_yaw)) return false;

        return true;
}

bool NavFilter::RegisterCallbacks(const ros::NodeHandle& n){
        // Create a local nodehandle to manage callback subscriptions.
        ros::NodeHandle nl(n);

        states_pub_ = nl.advertise<nav_msgs::Odometry>( "states", 25, false);

        // transformed_states_pub = nl.advertise<nav_msgs::Odometry>( "transformed_states", 25, false);

        updated_path_pub_ = nl.advertise<nav_msgs::Path>("nav_path", 10, false);

        imu_sub_ = nl.subscribe(imu_topic_,  10, &NavFilter::ImuCallback, this);
        odo_sub_ = nl.subscribe(odo_topic_,  10, &NavFilter::OdoCallback, this);
        slam_sub_ = nl.subscribe(slam_topic_,10, &NavFilter::SlamCallback, this);
        // gicp_sub_ = nl.subscribe(gicp_topic_,10, &NavFilter::GicpCallback, this);

        return true;
}

void NavFilter::OdoCallback(const OdoData& odo_data_){
        odo = OdoToGtsam(odo_data_);
        if (first_odo_) { odo_prev = odo; first_odo_ = false; }
        cur_stamp_ = (odo_data_.header.stamp).toSec();
        has_odo_ = true;
        UpdateGraph(imu,odo);
        return;
}

void NavFilter::SlamCallback(const OdoData& slam_data_){
        PoseData slam_data;
        slam_data.header = slam_data_.header;
        slam_data.pose = slam_data_.pose.pose;
        slam_pose = PoseStampedToGtsam(slam_data);
        has_slam_ = true;
        return;
}

// void NavFilter::GicpCallback(const PoseData& gicp_data_){
//         gicp_tran = PoseStampedToGtsam(gicp_data_);
//         return;
// }

void NavFilter::ImuCallback(const ImuData& imu_data_){
        imu = ImuToGtsam(imu_data_);
        return;
}

void NavFilter::UpdateGraph(const gtsam::Vector6& imu, const gtsam::NavState& odo){

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
                graph_.add(PriorFactor<Pose3>(X(key_), slam_pose, pose_noise_model_slam));
                graph_.add(PriorFactor<Vector3>(V(key_), odo.v(), velocity_noise_model));

                odo_vel = prev_state.v() + ( odo.v() - odo_prev.v() );  //Not be used.
                odo_pose = (prev_state.pose())*(odo.pose().between(odo_prev.pose()));

                graph_.add(PriorFactor<Pose3>(X(key_), odo_pose, pose_noise_model_odo));

                new_value_.insert(X(key_), slam_pose);
                new_value_.insert(V(key_), prev_state.v());
                values_ = LevenbergMarquardtOptimizer(graph_, new_value_).optimize();

                Pose3 gtsam_pose = values_.at<Pose3>(X(key_));
                Vector3 gtsam_vel = values_.at<Vector3>(V(key_));
                prev_state = NavState( gtsam_pose, gtsam_vel );

                graph_.resize(0);
                new_value_.clear();
                values_.clear();

                graph_.add(PriorFactor<Pose3>(X(key_), gtsam_pose, pose_noise_model_slam));
                graph_.add(PriorFactor<Vector3>(V(key_), gtsam_vel, velocity_noise_model));
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

gtsam::Vector6 NavFilter::ImuToGtsam(const ImuData& imu_data_)
{
        gtsam::Vector6 imuVec((gtsam::Vector(6) <<    imu_data_.linear_acceleration.y,
                               imu_data_.linear_acceleration.x,
                               -1*imu_data_.linear_acceleration.z,
                               imu_data_.angular_velocity.y,
                               imu_data_.angular_velocity.x,
                               -1*imu_data_.angular_velocity.z).finished());

        return imuVec;
}

gtsam::Pose3 NavFilter::PoseStampedToGtsam(const PoseData& slam_data_)
{

        gtsam::Point3 slam_point(slam_data_.pose.position.x,
                                 slam_data_.pose.position.y,
                                 slam_data_.pose.position.z);

        gtsam::Rot3 slam_rot = gtsam::Rot3::Quaternion(slam_data_.pose.orientation.w,
                                                       slam_data_.pose.orientation.x,
                                                       slam_data_.pose.orientation.y,
                                                       slam_data_.pose.orientation.z);

        gtsam::Pose3 slam(slam_rot, slam_point);
        return slam;

}

gtsam::NavState NavFilter::OdoToGtsam(const OdoData& odo_data_)
{
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

// Publish estimated states in global frame
void NavFilter::PublishStates(const gtsam::NavState& states,
                              const ros::Publisher& pub){
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

// // Publish states after transfromation into proir-map frame.
// void NavFilter::PublishTransformedStates(const gtsam::NavState& states, const::Pose3& trans, const ros::Publisher& pub){
//         // // Check for subscribers before doing any work.
//         //if(pub.getNumSubscribers() == 0)
//         //        return;

//         //* is a function for: (R_ * T.R_, t_ + R_ * T.t_)
//         gtsam::Pose3 rot_pose = (trans)*(states.pose());
//         Vector3 position = rot_pose.translation();
//         Vector4 attitude = rot_pose.rotation().quaternion();
//         Vector3 velocity = states.v();
//         // Convert from NavState to nav_msgs::Odo
//         nav_msgs::Odometry odo_msg;
//         odo_msg.pose.pose.position.x = position[0];
//         odo_msg.pose.pose.position.y = position[1];
//         odo_msg.pose.pose.position.z = position[2];
//         odo_msg.twist.twist.linear.x = velocity[0];
//         odo_msg.twist.twist.linear.y = velocity[1];
//         odo_msg.twist.twist.linear.z = velocity[2];
//         odo_msg.pose.pose.orientation.w = attitude[0];
//         odo_msg.pose.pose.orientation.x = attitude[1];
//         odo_msg.pose.pose.orientation.y = attitude[2];
//         odo_msg.pose.pose.orientation.z = attitude[3];
//         odo_msg.header.frame_id = frame_id_fixed_;
//         odo_msg.header.stamp = ros::Time::now();
//         odo_msg.child_frame_id = "base_link";
//         // Publish odometry message
//         pub.publish(odo_msg);

//         // Publish base_link tf
//         geometry_msgs::TransformStamped odo_trans;
//         odo_trans.header.stamp = ros::Time::now();
//         odo_trans.header.frame_id = frame_id_fixed_;
//         odo_trans.child_frame_id = "base_link";
//         odo_trans.transform.translation.x = position[0];
//         odo_trans.transform.translation.y = position[1];
//         odo_trans.transform.translation.z = position[2];
//         odo_trans.transform.rotation.w = attitude[0];
//         odo_trans.transform.rotation.x = attitude[1];
//         odo_trans.transform.rotation.y = attitude[2];
//         odo_trans.transform.rotation.z = attitude[3];
//         transformed_states_tf_broad.sendTransform(odo_trans);
// }

// Publish estimated path
void NavFilter::PublishPath(const gtsam::Pose3& states,
                            const ros::Publisher& pub){
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
