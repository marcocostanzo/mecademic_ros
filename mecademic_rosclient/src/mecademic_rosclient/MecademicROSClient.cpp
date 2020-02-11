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

#include "mecademic_rosclient/MecademicROSClient.h"

namespace mecademic
{
// The node handle should point to the robot namespace
MecademicROSClient::MecademicROSClient(const ros::NodeHandle& nh) : nh_(nh)
{
  srv_client_move_lin_ = nh_.serviceClient<mecademic_msgs::SetPose>("move_lin");
  srv_client_move_joints_ = nh_.serviceClient<mecademic_msgs::SetPose>("move_joints");

  srv_client_move_lin_.waitForExistence();
  srv_client_move_joints_.waitForExistence();
}

/* Move the robot following a linear path, pose must be w.r.t. the robot's world frame
WARNING! position is in meters [m]  (NOT mm)
*/
void MecademicROSClient::move_lin(const geometry_msgs::Pose& desired_pose)
{
  mecademic_msgs::SetPose srv_msg;
  //[mm]
  srv_msg.request.position.x = desired_pose.position.x * 1000.0;
  srv_msg.request.position.y = desired_pose.position.y * 1000.0;
  srv_msg.request.position.z = desired_pose.position.z * 1000.0;

  double roll, pitch, yaw;
  quaternion2RPY(desired_pose.orientation, roll, pitch, yaw);

  // Convert quaternion to xyz
  //[deg]
  srv_msg.request.orientation.x = roll * 180.0 / M_PI;
  srv_msg.request.orientation.y = pitch * 180.0 / M_PI;
  srv_msg.request.orientation.z = yaw * 180.0 / M_PI;

  // DBG
  std::cout << "srv_move = \n" << srv_msg.request << std::endl;
  std::cout << "Continue? ";
  char ans;
  std::cin >> ans;
  if (!(ans == 'y' || ans == 'Y'))
  {
    throw "Error";
  }

  if (!srv_client_move_lin_.call(srv_msg))
  {
    throw std::runtime_error("MecademicROSClient::move_lin server error");
  }
  if (!srv_msg.response.success)
  {
    throw std::runtime_error("MecademicROSClient::move_lin success is false");
  }
}

/* Move the robot joints to desired position
 WARNING! angle is in [rad]  (NOT deg)
 */
void MecademicROSClient::move_joints(const mecademic_msgs::Joints& desired_joints)
{
  mecademic_msgs::SetJoints srv_msg;
  srv_msg.request.joints.resize(6);
  //[deg]
  srv_msg.request.joints[0] = desired_joints.joints[0] * 180.0 / M_PI;
  srv_msg.request.joints[1] = desired_joints.joints[1] * 180.0 / M_PI;
  srv_msg.request.joints[2] = desired_joints.joints[2] * 180.0 / M_PI;
  srv_msg.request.joints[3] = desired_joints.joints[3] * 180.0 / M_PI;
  srv_msg.request.joints[4] = desired_joints.joints[4] * 180.0 / M_PI;
  srv_msg.request.joints[5] = desired_joints.joints[5] * 180.0 / M_PI;

  // DBG
  std::cout << "srv_move_joints = \n" << srv_msg.request << std::endl;
  std::cout << "Continue? ";
  char ans;
  std::cin >> ans;
  if (!(ans == 'y' || ans == 'Y'))
  {
    throw "Error";
  }

  if (!srv_client_move_joints_.call(srv_msg))
  {
    throw std::runtime_error("MecademicROSClient::move_joints server error");
  }
  if (!srv_msg.response.success)
  {
    throw std::runtime_error("MecademicROSClient::move_joints success is false");
  }
}

/*
   Wait for robot stop to desired pose
  */
void MecademicROSClient::wait_pose(const geometry_msgs::Pose& desired_pose, ros::Duration max_wait)
{
  throw std::runtime_error("MecademicROSClient::wait_pose not implemented");
}

/*
 Wait for robot stop to desired joint position
*/
void MecademicROSClient::wait_joint_position(const mecademic_msgs::Joints& desired_joints, ros::Duration max_wait)
{
  throw std::runtime_error("MecademicROSClient::wait_joint_position not implemented");
}

void MecademicROSClient::quaternion2RPY(const geometry_msgs::Quaternion& quaternion, double& roll, double& pitch,
                                        double& yaw) const
{
  tf2::Quaternion tf_quat;
  tf2::convert(quaternion, tf_quat);
  tf2::Matrix3x3 m(tf_quat);
  m.getRPY(roll, pitch, yaw);
}

}  // namespace mecademic
