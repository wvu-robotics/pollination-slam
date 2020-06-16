// This is for fusing wheel odometry and laser odometry estimates;
// Publish pose estimate w.r.t. map frame and tf from map to base_link;

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// #include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>

class TransformOdomFusion{
private:

  ros::NodeHandle nh;

  ros::Publisher _pubState;
  ros::Subscriber _subLaserOdom;
  ros::Subscriber _subOdometry;

  nav_msgs::Odometry _state;
  tf::StampedTransform _stateTrans;
  tf::TransformBroadcaster _tfBroadcaster;

  std_msgs::Header _currentHeader;

  std::string _frameId;
  std::string _childFrameId;
  std::string _frameIdOut;
  std::string _laserOdomTopic;  // pose estimate from laser
  std::string _odometryTopic; // pose estimate from wheel odometry

  float _transformSum[6];
  float _transformIncre[6];
  float _transformOdomToLaser[6]; // transform from current odometry estimate to current laser odometry frame;
  float _transformOdomCur[6];  // odometry at time step as same as laser odometry;
  float _transformLaserCur[6];  // laser odometry at current update time step;

  bool _transformToRobotCenter;
  float _RToLRoll;
  float _RToLPitch;
  float _RToLYaw;
  float _TRToLX;
  float _TRToLY;
  float _TRToLZ;

  float _LiToMRoll;
  float _LiToMPitch;
  float _LiToMYaw;
  float _TLiToMX;
  float _TLiToMY;
  float _TLiToMZ;

public:

  TransformOdomFusion(){
    if(!nh.getParam("fusion/frames/frameId", _frameId)) ROS_ERROR("Failed to get param 'fusion/frames/frameId'");
    if(!nh.getParam("fusion/frames/childFrameId", _childFrameId)) ROS_ERROR("Failed to get param 'fusion/frames/childFrameId'");
    if(!nh.getParam("fusion/topic/laserOdom", _laserOdomTopic)) ROS_ERROR("Failed to get param 'fusion/topic/laserOdom'");
    if(!nh.getParam("fusion/topic/odometry", _odometryTopic)) ROS_ERROR("Failed to get param 'fusion/topic/odometry'");

    if(!nh.getParam("/lego_loam/pointTransform/transformToRobotCenter", _transformToRobotCenter)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/transformToRobotCenter'");
    if(!nh.getParam("/lego_loam/pointTransform/RToLRoll", _RToLRoll)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/RToLRoll'");
    if(!nh.getParam("/lego_loam/pointTransform/RToLPitch", _RToLPitch)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/RToLPitch'");
    if(!nh.getParam("/lego_loam/pointTransform/RToLYaw", _RToLYaw)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/RToLYaw'");
    if(!nh.getParam("/lego_loam/pointTransform/TRToLX", _TRToLX)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/TRToLX'");
    if(!nh.getParam("/lego_loam/pointTransform/TRToLY", _TRToLY)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/TRToLY'");
    if(!nh.getParam("/lego_loam/pointTransform/TRToLZ", _TRToLZ)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/TRToLZ'");
    if(!nh.getParam("/lego_loam/pointTransform/LiToMRoll", _LiToMRoll)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/LiToMRoll'");
    if(!nh.getParam("/lego_loam/pointTransform/LiToMPitch", _LiToMPitch)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/LiToMPitch'");
    if(!nh.getParam("/lego_loam/pointTransform/LiToMYaw", _LiToMYaw)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/LiToMYaw'");
    if(!nh.getParam("/lego_loam/pointTransform/TLiToMX", _TLiToMX)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/TLiToMX'");
    if(!nh.getParam("/lego_loam/pointTransform/TLiToMY", _TLiToMY)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/TLiToMY'");
    if(!nh.getParam("/lego_loam/pointTransform/TLiToMZ", _TLiToMZ)) ROS_ERROR("Failed to get param '/lego_loam/pointTransform/TLiToMZ'");

    _pubState = nh.advertise<nav_msgs::Odometry> ("/state", 25);

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> SyncPolicy;
    message_filters::Subscriber<nav_msgs::Odometry>* subLaserOdom;
    message_filters::Subscriber<nav_msgs::Odometry>* subOdometry;
    message_filters::Synchronizer<SyncPolicy>* sync;

    subLaserOdom = new message_filters::Subscriber<nav_msgs::Odometry>(nh, _laserOdomTopic, 10);
    subOdometry = new message_filters::Subscriber<nav_msgs::Odometry>(nh, _odometryTopic, 10);
    sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *subLaserOdom, *subOdometry);
    sync->registerCallback(boost::bind(&TransformOdomFusion::laserOdometryHandler, this, _1, _2));

    // _subLaserOdom = nh.subscribe(_laserOdomTopic, 10, &TransformOdomFusion::laserOdometryHandler, this);
    _subOdometry = nh.subscribe(_odometryTopic, 10, &TransformOdomFusion::odometryHandler, this);

    _state.header.frame_id = _frameId;
    _state.child_frame_id = _childFrameId;

    _stateTrans.frame_id_ = _frameId;
    _stateTrans.child_frame_id_ = _childFrameId;

    for (int i=0; i<6; i++){
      _transformSum[i] = 0;
      _transformIncre[i] = 0;
      _transformOdomToLaser[i] = 0;
      _transformOdomCur[i] = 0;
      _transformLaserCur[i] = 0;
    }
  }

  void transformAssociateToLaser(){
    // rotate with z axis.
    float y1 = cos(_transformSum[1]) * (_transformOdomCur[4] - _transformSum[4])
                 - sin(_transformSum[1]) * (_transformOdomCur[3] - _transformSum[3]);
        float z1 = _transformOdomCur[5] - _transformSum[5];
        float x1 = sin(_transformSum[1]) * (_transformOdomCur[4] - _transformSum[4])
                 + cos(_transformSum[1]) * (_transformOdomCur[3] - _transformSum[3]);

        // rotate with y axis.
        float y2 = y1;
        float z2 = cos(_transformSum[0]) * z1 + sin(_transformSum[0]) * x1;
        float x2 = -sin(_transformSum[0]) * z1 + cos(_transformSum[0]) * x1;

        // rotate with x axis.
        _transformIncre[4] = cos(_transformSum[2]) * y2 + sin(_transformSum[2]) * z2;
        _transformIncre[5] = -sin(_transformSum[2]) * y2 + cos(_transformSum[2]) * z2;
        _transformIncre[3] = x2;

        float sbcx = sin(_transformSum[0]);
        float cbcx = cos(_transformSum[0]);
        float sbcy = sin(_transformSum[1]);
        float cbcy = cos(_transformSum[1]);
        float sbcz = sin(_transformSum[2]);
        float cbcz = cos(_transformSum[2]);

        float sblx = sin(_transformOdomCur[0]);
        float cblx = cos(_transformOdomCur[0]);
        float sbly = sin(_transformOdomCur[1]);
        float cbly = cos(_transformOdomCur[1]);
        float sblz = sin(_transformOdomCur[2]);
        float cblz = cos(_transformOdomCur[2]);

        float salx = sin(_transformLaserCur[0]);
        float calx = cos(_transformLaserCur[0]);
        float saly = sin(_transformLaserCur[1]);
        float caly = cos(_transformLaserCur[1]);
        float salz = sin(_transformLaserCur[2]);
        float calz = cos(_transformLaserCur[2]);

        float sry = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
                  - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                  - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                  - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                  - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
        _transformOdomToLaser[0] = -asin(sry);

        float srzcry = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                     - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
                     - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                     + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
                     + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                     + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);
        float crzcry = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                     - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
                     + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                     + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
                     - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                     + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
        _transformOdomToLaser[1] = atan2(srzcry / cos(_transformOdomToLaser[0]),
                                         crzcry / cos(_transformOdomToLaser[0]));

       float srxcry = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                    - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                    - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                    - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                    + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
       float crxcry = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                    - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                    - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                    - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                    + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
       _transformOdomToLaser[2] = atan2(srxcry / cos(_transformOdomToLaser[0]),
                                        crxcry / cos(_transformOdomToLaser[0]));

        // rotate with x axis.
        y1 = cos(_transformOdomToLaser[2]) * _transformIncre[4] - sin(_transformOdomToLaser[2]) * _transformIncre[5];
        z1 = sin(_transformOdomToLaser[2]) * _transformIncre[4] + cos(_transformOdomToLaser[2]) * _transformIncre[5];
        x1 = _transformIncre[3];

        // rotate with y axis.
        y2 = y1;
        z2 = cos(_transformOdomToLaser[0]) * z1 - sin(_transformOdomToLaser[0]) * x1;
        x2 = sin(_transformOdomToLaser[0]) * z1 + cos(_transformOdomToLaser[0]) * x1;

        // rotate with z axis.
        _transformOdomToLaser[4] = _transformLaserCur[4]
                           - (cos(_transformOdomToLaser[1]) * y2 + sin(_transformOdomToLaser[1]) * x2);
        _transformOdomToLaser[5] = _transformLaserCur[5] - z2;
        _transformOdomToLaser[3] = _transformLaserCur[3]
                           - (-sin(_transformOdomToLaser[1]) * y2 + cos(_transformOdomToLaser[1]) * x2);
  }

  void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometry){
    _currentHeader = odometry->header;

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odometry->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, -geoQuat.y, -geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    _transformSum[0] = -pitch;
    _transformSum[1] = -yaw;
    _transformSum[2] = roll;

    _transformSum[3] = odometry->pose.pose.position.x;
    _transformSum[4] = odometry->pose.pose.position.y;
    _transformSum[5] = odometry->pose.pose.position.z;

    transformAssociateToLaser();

    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(_transformOdomToLaser[2], -_transformOdomToLaser[0], -_transformOdomToLaser[1]);

    _state.header.stamp = odometry->header.stamp;
    _state.pose.pose.orientation.x = geoQuat.x;
    _state.pose.pose.orientation.y = -geoQuat.y;
    _state.pose.pose.orientation.z = -geoQuat.z;
    _state.pose.pose.orientation.w = geoQuat.w;
    _state.pose.pose.position.x = _transformOdomToLaser[3];
    _state.pose.pose.position.y = _transformOdomToLaser[4];
    _state.pose.pose.position.z = _transformOdomToLaser[5];
    _state.twist= odometry->twist;
    _pubState.publish(_state);

    _stateTrans.stamp_ = odometry->header.stamp;
    _stateTrans.setRotation(tf::Quaternion(geoQuat.x, -geoQuat.y, -geoQuat.z, geoQuat.w));
    _stateTrans.setOrigin(tf::Vector3(_transformOdomToLaser[3], _transformOdomToLaser[4], _transformOdomToLaser[5]));
    _tfBroadcaster.sendTransform(_stateTrans);

    // // publish path for visualization
    // // Check for subscribers before doing any work.
    // if(pub.getNumSubscribers() == 0)
    //         return;
    //
    // Vector3 position = states.translation();
    // Vector4 attitude = states.rotation().quaternion();
    // // Convert from NavState to nav_msgs::Odo
    // nav_msgs::Path path_msg;
    // path_msg.poses[key_].pose.position.x = position[0];
    // path_msg.poses[key_].pose.position.y = position[1];
    // path_msg.poses[key_].pose.position.z = position[2];
    // path_msg.poses[key_].pose.orientation.w = attitude[0];
    // path_msg.poses[key_].pose.orientation.x = attitude[1];
    // path_msg.poses[key_].pose.orientation.y = attitude[2];
    // path_msg.poses[key_].pose.orientation.z = attitude[3];
    // path_msg.header.frame_id = frame_id_out_;
    // path_msg.header.stamp = stamp_;
    // pub.publish(path_msg);
  }

  void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry, const nav_msgs::Odometry::ConstPtr& Odometry){

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
    // tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

    tf::Matrix3x3(tf::Quaternion(geoQuat.x, -geoQuat.y, -geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    _transformLaserCur[0] = -pitch;
    _transformLaserCur[1] = -yaw;
    _transformLaserCur[2] = roll;

    _transformLaserCur[3] = laserOdometry->pose.pose.position.x;
    _transformLaserCur[4] = laserOdometry->pose.pose.position.y;
    _transformLaserCur[5] = laserOdometry->pose.pose.position.z;

    // transform from lidar pose to robot pose: M^T_R = M^T_L * L^T_R
    // M^T_L is the lidar pose w.r.t. map frame
    // L^T_R is the robot pose w.r.t. lidar frame
    // Note that, if doing active transform to figure out the euler angles,
    // the value need to be multiplied with -1 for cuculated rotation matrix.
    // rotate seq: YZX (yaw pitch raw)
    if(_transformToRobotCenter){
      // 1:z; 2:y; 3:x
      float sa1 = sin(_RToLYaw); // sin(yaw)
      float ca1 = cos(_RToLYaw); // cos(yaw)
      float sa2 = sin(_RToLPitch); // sin(pitch)
      float ca2 = cos(_RToLPitch); // cos(pitch)
      float sa3 = sin(_RToLRoll); // sin(roll)
      float ca3 = cos(_RToLRoll); // cos(roll)

      float sb1 = sin(_transformLaserCur[1]); // sin(yaw)
      float cb1 = cos(_transformLaserCur[1]); // cos(yaw)
      float sb2 = sin(_transformLaserCur[0]); // sin(pitch)
      float cb2 = cos(_transformLaserCur[0]); // cos(pitch)
      float sb3 = sin(_transformLaserCur[2]); // sin(roll)
      float cb3 = cos(_transformLaserCur[2]); // cos(roll)

      float sd1 = sin(_LiToMYaw); // sin(yaw)
      float cd1 = cos(_LiToMYaw); // cos(yaw)
      float sd2 = sin(_LiToMPitch); // sin(pitch)
      float cd2 = cos(_LiToMPitch); // cos(pitch)
      float sd3 = sin(_LiToMRoll); // sin(roll)
      float cd3 = cos(_LiToMRoll); // cos(roll)

      float tax = _TRToLX;
      float tay = _TRToLY;
      float taz = _TRToLZ;

      float tbx = _transformLaserCur[3];
      float tby = _transformLaserCur[4];
      float tbz = _transformLaserCur[5];

      float tdx = _TLiToMX;
      float tdy = _TLiToMY;
      float tdz = _TLiToMZ;

      float a11 = ca1*ca2;
      float a12 = ca1*sa2*sa3-ca3*sa1;
      float a13 = sa1*sa3+ca1*ca3*sa2;
      float a21 = ca2*sa1;
      float a22 = ca1*ca3+sa1*sa2*sa3;
      float a23 = ca3*sa1*sa2-ca1*sa3;
      float a31 = -sa2;
      float a32 = ca2*sa3;
      float a33 = ca2*ca3;

      float b11 = cb1*cb2;
      float b12 = cb1*sb2*sb3-cb3*sb1;
      float b13 = sb1*sb3+cb1*cb3*sb2;
      float b21 = cb2*sb1;
      float b22 = cb1*cb3+sb1*sb2*sb3;
      float b23 = cb3*sb1*sb2-cb1*sb3;
      float b31 = -sb2;
      float b32 = cb2*sb3;
      float b33 = cb2*cb3;

      float d11 = cd1*cd2;
      float d12 = cd1*sd2*sd3-cd3*sd1;
      float d13 = sd1*sd3+cd1*cd3*sd2;
      float d21 = cd2*sd1;
      float d22 = cd1*cd3+sd1*sd2*sd3;
      float d23 = cd3*sd1*sd2-cd1*sd3;
      float d31 = -sd2;
      float d32 = cd2*sd3;
      float d33 = cd2*cd3;

      float f11 = b11*a11+b12*a21+b13*a13;
      float f12 = b11*a12+b12*a22+b13*a32;
      float f13 = b11*a13+b12*a23+b13*a33;
      float f21 = b21*a11+b22*a21+b23*a31;
      float f22 = b21*a12+b22*a22+b23*a32;
      float f23 = b21*a13+b22*a23+b23*a33;
      float f31 = b31*a11+b32*a21+b33*a31;
      float f32 = b31*a12+b32*a22+b33*a32;
      float f33 = b31*a13+b32*a23+b33*a33;

      float cr1cr2 = d11*f11+d12*f21+d13*f31;
      float sr1cr2 = d21*f11+d22*f21+d23*f31;
      float sr2 = -1*(d31*f11+d32*f21+d33*f31);
      float cr2sr3 = d31*f12+d32*f22+d33*f32;
      float cr2cr3 = d31*f13+d32*f23+d33*f33;

      _transformLaserCur[0] = asin(sr2);
      _transformLaserCur[1] = atan2(sr1cr2/cos(_transformLaserCur[0]),
                                    cr1cr2/cos(_transformLaserCur[0]));
      _transformLaserCur[2] = atan2(cr2sr3/cos(_transformLaserCur[0]),
                                    cr2cr3/cos(_transformLaserCur[0]));
      float e1 = b11*tax+b12*tay+b13*taz;
      float e2 = b21*tax+b22*tay+b23*taz;
      float e3 = b31*tax+b32*tay+b33*taz;
      _transformLaserCur[3] = d11*e1+d12*e2+d13*e3+d11*tbx+d12*tby+d13*tbz+tdx;
      _transformLaserCur[4] = d21*e1+d22*e2+d23*e3+d21*tbx+d22*tby+d23*tbz+tdy;
      _transformLaserCur[5] = d31*e1+d32*e2+d33*e3+d31*tbx+d32*tby+d33*tbz+tdz;
    }
    geoQuat = Odometry->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, -geoQuat.y, -geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    _transformOdomCur[0] = -pitch;
    _transformOdomCur[1] = -yaw;
    _transformOdomCur[2] = roll;

    _transformOdomCur[3] = Odometry->pose.pose.position.x;
    _transformOdomCur[4] = Odometry->pose.pose.position.y;
    _transformOdomCur[5] = Odometry->pose.pose.position.z;
  }
};

int main(int argc, char** argv){

  ros::init(argc,argv, "lego_loam");
  TransformOdomFusion TFOdomFusion;

  ROS_INFO("\033[1;32m---->\033[0m Transform Odometry Fusion Started.");

  ros::spin();
  return 0;
}
