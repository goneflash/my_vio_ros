#include "my_vio/kalman_filter.hpp"
#include "my_vio/tilt_tracking.hpp"

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"

std::string p_base_stabilized_frame_;
std::string p_base_frame_;
tf::TransformBroadcaster *tfB_;
tf::TransformBroadcaster *tfB_filtered_;
tf::TransformBroadcaster *tfB_tilt_tracker_;
tf::StampedTransform transform_;
tf::StampedTransform transform_filtered_;
tf::StampedTransform transform_tilt_tracker_;
tf::Quaternion tmp_;
IMUKalmanFilter imu_filter;
vio::TiltTracker<double> tilt_tracker;

bool imu_filter_init;

#ifndef TF_MATRIX3x3_H
typedef btScalar tfScalar;
namespace tf {
typedef btMatrix3x3 Matrix3x3;
}
#endif

void imuMsgCallback(const sensor_msgs::Imu &imu_msg);

int main(int argc, char **argv) {
  ros::init(argc, argv, "trans_transit");

  ros::NodeHandle n;

  n.param("base_stabilized_frame", p_base_stabilized_frame_,
          std::string("base_stabilized_frame"));
  n.param("base_frame", p_base_frame_, std::string("base_frame"));

  tfB_ = new tf::TransformBroadcaster();
  tfB_filtered_ = new tf::TransformBroadcaster();
  tfB_tilt_tracker_ = new tf::TransformBroadcaster();
  transform_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  transform_filtered_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  transform_tilt_tracker_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

  //	transform_.getOrigin().setX(0.0);
  //	transform_.getOrigin().setY(0.0);
  //	transform_.getOrigin().setZ(0.0);
  transform_.frame_id_ = "/imu_frame";  // p_base_stabilized_frame_;//;
  transform_.child_frame_id_ = "/imu_filtered_frame";  // p_base_frame_;//

  transform_filtered_.frame_id_ =
      "/imu_filtered_frame";                     // p_base_stabilized_frame_;//;
  transform_filtered_.child_frame_id_ = "/map";  // p_base_frame_;//

  transform_tilt_tracker_.frame_id_ =
      "/imu_tilt_tracker_frame";  // p_base_stabilized_frame_;//;
  transform_tilt_tracker_.child_frame_id_ = "/map";  // p_base_frame_;//

  ros::Subscriber imu_subscriber =
      n.subscribe("/imu_raw_data", 5, imuMsgCallback);

  imu_filter_init = false;
  ros::spin();

  delete tfB_;
  delete tfB_filtered_;
  delete tfB_tilt_tracker_;

  return 0;
}

void imuMsgCallback(const sensor_msgs::Imu &imu_msg) {
  /*
          tfScalar halfYaw = tfScalar(imu_msg.orientation.x / 180 * 3.14159) *
     tfScalar(0.5);
          tfScalar halfPitch = tfScalar(imu_msg.orientation.z / 180 * 3.14159) *
     tfScalar(0.5);
          tfScalar halfRoll = tfScalar(imu_msg.orientation.y / 180 * 3.14159) *
     tfScalar(0.5);
          tfScalar cosYaw = tfCos(halfYaw);
          tfScalar sinYaw = tfSin(halfYaw);
          tfScalar cosPitch = tfCos(halfPitch);
          tfScalar sinPitch = tfSin(halfPitch);
          tfScalar cosRoll = tfCos(halfRoll);
          tfScalar sinRoll = tfSin(halfRoll);

          sensor_msgs::Imu new_imu_msg;
          new_imu_msg.orientation.x = cosRoll * sinPitch * cosYaw + sinRoll *
     cosPitch * sinYaw;
          new_imu_msg.orientation.y = cosRoll * cosPitch * sinYaw - sinRoll *
     sinPitch * cosYaw;
          new_imu_msg.orientation.z = sinRoll * cosPitch * cosYaw - cosRoll *
     sinPitch * sinYaw;
          new_imu_msg.orientation.w = cosRoll * cosPitch * cosYaw + sinRoll *
     sinPitch * sinYaw;
  */
  tf::quaternionMsgToTF(imu_msg.orientation, tmp_);

  tfScalar yaw, pitch, roll;
  tf::Matrix3x3(tmp_).getRPY(roll, pitch, yaw);

  tmp_.setRPY(roll, pitch, yaw);
  transform_.setRotation(tmp_);
  transform_.stamp_ = ros::Time::now();

  // ROS_INFO("Published          roll: %lf, pitch: %lf.", roll, pitch);

  Matrix<double, 6, 1> imu_state = Matrix<double, 6, 1>::Zero();
  imu_state[0] = roll;
  imu_state[1] = pitch;
  imu_state[2] = yaw;
  imu_state[3] = imu_msg.angular_velocity.x;
  imu_state[4] = imu_msg.angular_velocity.y;
  imu_state[5] = imu_msg.angular_velocity.z;
  if (!imu_filter_init) {
    imu_filter.setInitialState(imu_state);
    imu_filter_init = true;
  } else {
    imu_filter.updateWithMeasurement(imu_state, imu_msg.header.stamp.sec);
    imu_filter.getState(imu_state);
    roll = imu_state[0];
    pitch = imu_state[1];
    yaw = imu_state[2];
  }

  //	roll = -(180 - imu_msg.orientation.x) / 180.0 * 3.1415926;
  //	pitch = (180 - imu_msg.orientation.y) / 180.0 * 3.1415926;
  tmp_.setRPY(roll, pitch, yaw);

  transform_filtered_.setRotation(tmp_);
  transform_filtered_.stamp_ = ros::Time::now();  // imu_msg.header.stamp;

  double new_roll, new_pitch;
  tilt_tracker.GetRollPitch(imu_msg.linear_acceleration.x,
                            imu_msg.linear_acceleration.y,
                            imu_msg.linear_acceleration.z, new_roll, new_pitch);

  tmp_.setRPY(new_roll, new_pitch, 0);
  transform_tilt_tracker_.setRotation(tmp_);
  transform_tilt_tracker_.stamp_ = ros::Time::now();  // imu_msg.header.stamp;

  // ROS_INFO("Published filtered roll: %lf, pitch: %lf.", roll, pitch);

  tfB_->sendTransform(transform_);
  tfB_filtered_->sendTransform(transform_filtered_);
  tfB_tilt_tracker_->sendTransform(transform_tilt_tracker_);
}
