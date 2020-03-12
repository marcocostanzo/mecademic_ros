/*
    Mecademic ROS Client
    Copyright 2020 Università della Campania Luigi Vanvitelli
    Author: Marco Costanzo <marco.costanzo@unicampania.it>
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MECADEMIC_ROS_CLIENT_H
#define MECADEMIC_ROS_CLIENT_H

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TwistStamped.h"
#include "mecademic_msgs/Joints.h"
#include "mecademic_msgs/SetJoints.h"
#include "mecademic_msgs/SetPose.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace mecademic
{
class timeout_exception : public std::runtime_error
{
public:
  timeout_exception(std::string const& msg) : std::runtime_error(msg)
  {
  }
};

class MecademicROSClient
{
protected:
  ros::NodeHandle nh_;

  ros::ServiceClient srv_client_move_lin_, srv_client_move_joints_, srv_client_set_trf_, srv_client_set_wrf_;

  ros::Publisher pub_vel_trf_;

  bool joints_on_fb_are_deg;
  bool position_on_fb_is_mm;
  bool orientation_on_fb_is_xyz;
  bool xyz_on_fb_is_deg;

public:
  const std::string brf_frame_id_, wrf_frame_id_, frf_frame_id_, trf_frame_id_;

public:
  // The node handle should point to the robot namespace
  MecademicROSClient(const ros::NodeHandle& nh, const std::string& tf_prefix = "");

  /* Move the robot following a linear path, pose must be w.r.t. the robot's world frame
  WARNING! position is in meters [m]  (NOT mm)
  */
  void move_lin(const geometry_msgs::PoseStamped& desired_pose);

  /* Move the robot joints to desired position
  WARNING! angle is in [rad]  (NOT deg)
  */
  void move_joints(const mecademic_msgs::Joints& desired_joints);

  /*
  Publish a twist command in TRF
  */
  void pub_twist_command_trf(const geometry_msgs::Twist& desired_twist);

  /*
  Set the robot TRF
  TRF pose MUST be w.r.t. mecademic's FRF
  */
  void set_trf(const geometry_msgs::Pose& trf_pose);

  /*
  Set the robot WRF
  WRF pose MUST be w.r.t. mecademic's BRF
  */
  void set_wrf(const geometry_msgs::Pose& wrf_pose);

  /*
    Get the current tool pose
  */
  geometry_msgs::PoseStamped getToolPose(const ros::Duration& timeout = ros::Duration(-1));

  /*
   Wait for robot stop to desired pose
  */
  void wait_pose(const geometry_msgs::Pose& desired_pose, const ros::Duration& timeout = ros::Duration(-1),
                 double epsilon_pose = 0.0005, double epsilon_rotation = 0.0005,
                 const boost::function<void()>& on_wait_cb = 0);

  /*
   Wait for robot stop to desired joint position
  */
  void wait_joint_position(const mecademic_msgs::Joints& desired_joints,
                           const ros::Duration& timeout = ros::Duration(-1), double epsilon_joints = 0.01,
                           const boost::function<void()>& on_wait_cb = 0);

};  // endclass

}  // namespace mecademic
#endif