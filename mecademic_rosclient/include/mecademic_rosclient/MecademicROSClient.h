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
#include "mecademic_msgs/Joints.h"
#include "mecademic_msgs/SetJoints.h"
#include "mecademic_msgs/SetPose.h"
#include "ros/ros.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace mecademic
{
class MecademicROSClient
{
protected:
  ros::NodeHandle nh_;

  ros::ServiceClient srv_client_move_lin_, srv_client_move_joints_;

public:
  // The node handle should point to the robot namespace
  MecademicROSClient(const ros::NodeHandle& nh);

  /* Move the robot following a linear path, pose must be w.r.t. the robot's world frame
  WARNING! position is in meters [m]  (NOT mm)
  */
  void move_lin(const geometry_msgs::Pose& desired_pose);

  /* Move the robot joints to desired position
  WARNING! angle is in [rad]  (NOT deg)
  */
  void move_joints(const mecademic_msgs::Joints& desired_joints);

  /*
   Wait for robot stop to desired pose
  */
  void wait_pose(const geometry_msgs::Pose& desired_pose, ros::Duration max_wait = ros::Duration(-1));

  /*
   Wait for robot stop to desired joint position
  */
  void wait_joint_position(const mecademic_msgs::Joints& desired_joints, ros::Duration max_wait = ros::Duration(-1));

  void quaternion2RPY(const geometry_msgs::Quaternion& quaternion, double& roll, double& pitch, double& yaw) const;

};  // endclass

}  // namespace mecademic
#endif