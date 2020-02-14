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

// Helper functions
void quaternion2RPY(const geometry_msgs::Quaternion& quaternion, double& roll, double& pitch, double& yaw);
void joint_position_cb(const sensor_msgs::JointState::ConstPtr& msg, bool deg2rad, sensor_msgs::JointState& out_msg,
                       bool& msg_arrived);
double std_vect_distance(const std::vector<double>& v1, const std::vector<double>& v2);
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg, bool mm2m, bool xyz2quat, bool deg2rad,
             geometry_msgs::PoseStamped& out_msg, bool& msg_arrived);
void pose_distance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, double& position_distance,
                   double& rotation_distance);

namespace mecademic
{
// The node handle should point to the robot namespace
MecademicROSClient::MecademicROSClient(const ros::NodeHandle& nh) : nh_(nh)
{
  joints_on_fb_are_deg = false;
  position_on_fb_is_mm = true;
  orientation_on_fb_is_xyz = true;
  xyz_on_fb_is_deg = true;
  
  srv_client_move_lin_ = nh_.serviceClient<mecademic_msgs::SetPose>("move_lin");
  srv_client_move_joints_ = nh_.serviceClient<mecademic_msgs::SetJoints>("move_joints");

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

geometry_msgs::PoseStamped MecademicROSClient::getToolPose(const ros::Duration& timeout)
{
  ros::Time start_time = ros::Time::now();
  geometry_msgs::PoseStamped pose;
  bool msg_arrived;
  boost::function<void(const geometry_msgs::PoseStamped::ConstPtr& msg)> sub_cb =
      boost::bind(pose_cb, _1, position_on_fb_is_mm, orientation_on_fb_is_xyz, xyz_on_fb_is_deg, boost::ref(pose), boost::ref(msg_arrived));
  ros::Subscriber sub_pose = nh_.subscribe("state/pose", 1, sub_cb);
  
  msg_arrived = false;
  while (ros::ok())
  {
    if (msg_arrived)
    {
      return pose;
    }
    if (timeout >= ros::Duration(0))
    {
      ros::Time current_time = ros::Time::now();
      if ((current_time - start_time) >= timeout)
      {
        throw timeout_exception("MecademicROSClient::getToolPose timeout");
      }
    }
    ros::spinOnce();
  }
}


/*
   Wait for robot stop to desired pose
  */
void MecademicROSClient::wait_pose(const geometry_msgs::Pose& desired_pose, const ros::Duration& timeout,
                                   double epsilon_pose, double epsilon_rotation)
{
  ros::Time start_time = ros::Time::now();

  geometry_msgs::PoseStamped actual_pose;
  bool msg_arrived;
  boost::function<void(const geometry_msgs::PoseStamped::ConstPtr& msg)> sub_cb =
      boost::bind(pose_cb, _1, position_on_fb_is_mm, orientation_on_fb_is_xyz, xyz_on_fb_is_deg, boost::ref(actual_pose), boost::ref(msg_arrived));
  ros::Subscriber sub_pose = nh_.subscribe("state/pose", 1, sub_cb);

  msg_arrived = false;
  while (ros::ok())
  {
    if (msg_arrived)
    {
      double position_distance, rotation_distance;
      pose_distance(desired_pose, actual_pose.pose, position_distance, rotation_distance);
      if (position_distance < epsilon_pose && rotation_distance < epsilon_rotation)
      {
        return;
      }
      msg_arrived = false;
    }
    if (timeout >= ros::Duration(0))
    {
      ros::Time current_time = ros::Time::now();
      if ((current_time - start_time) >= timeout)
      {
        throw timeout_exception("MecademicROSClient::wait_pose timeout");
      }
    }

    ros::spinOnce();
  }
}

/*
 Wait for robot stop to desired joint position
*/
void MecademicROSClient::wait_joint_position(const mecademic_msgs::Joints& desired_joints, const ros::Duration& timeout,
                                             double epsilon_joints)
{
  ros::Time start_time = ros::Time::now();

  sensor_msgs::JointState joint_state;
  bool msg_arrived;
  boost::function<void(const sensor_msgs::JointState::ConstPtr& msg)> sub_cb =
      boost::bind(joint_position_cb, _1, joints_on_fb_are_deg, boost::ref(joint_state), boost::ref(msg_arrived));
  ros::Subscriber sub_joints = nh_.subscribe("state/joint_position", 1, sub_cb);

  msg_arrived = false;
  while (ros::ok())
  {
    if (msg_arrived)
    {
      double distance = std_vect_distance(desired_joints.joints, joint_state.position);
      if (distance < epsilon_joints)
      {
        return;
      }
      msg_arrived = false;
    }
    if (timeout >= ros::Duration(0))
    {
      ros::Time current_time = ros::Time::now();
      if ((current_time - start_time) >= timeout)
      {
        throw timeout_exception("MecademicROSClient::wait_joint_position timeout");
      }
    }

    ros::spinOnce();
  }
}

}  // namespace mecademic

void joint_position_cb(const sensor_msgs::JointState::ConstPtr& msg, bool deg2rad, sensor_msgs::JointState& out_msg,
                       bool& msg_arrived)
{
  out_msg = *msg;
  if (deg2rad)
  {
    for (int i = 0; i < out_msg.position.size(); i++)
    {
      out_msg.position[i] = out_msg.position[i] * M_PI / 180.0;
    }
    for (int i = 0; i < out_msg.velocity.size(); i++)
    {
      out_msg.velocity[i] = out_msg.velocity[i] * M_PI / 180.0;
    }
    for (int i = 0; i < out_msg.effort.size(); i++)
    {
      out_msg.effort[i] = out_msg.effort[i] * M_PI / 180.0;
    }
  }
  msg_arrived = true;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg, bool mm2m, bool xyz2quat, bool deg2rad,
             geometry_msgs::PoseStamped& out_msg, bool& msg_arrived)
{
  out_msg = *msg;
  if (mm2m)
  {
    out_msg.pose.position.x /= 1000.0;
    out_msg.pose.position.y /= 1000.0;
    out_msg.pose.position.z /= 1000.0;
  }
  if (xyz2quat)
  {
    if (deg2rad)
    {
      out_msg.pose.orientation.x *= M_PI / 180.0;
      out_msg.pose.orientation.y *= M_PI / 180.0;
      out_msg.pose.orientation.z *= M_PI / 180.0;
    }
    // transform from xyz to quaternion
    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(out_msg.pose.orientation.x, out_msg.pose.orientation.y, out_msg.pose.orientation.z);
    tf2::convert(tf2_quat, out_msg.pose.orientation);
  }
  msg_arrived = true;
}

double std_vect_distance(const std::vector<double>& v1, const std::vector<double>& v2)
{
  double distance = 0.0;
  for (int i = 0; i < v1.size(); i++)
  {
    distance = pow(v1[i] - v2[i], 2);
  }
  distance = sqrt(distance);
  return distance;
}

void pose_distance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, double& position_distance,
                   double& rotation_distance)
{
  position_distance = sqrt(pow(p1.position.x - p2.position.x, 2) + pow(p1.position.y - p2.position.y, 2) +
                           pow(p1.position.z - p2.position.z, 2));
  tf2::Quaternion tf2_quat1;
  tf2::Quaternion tf2_quat2;
  tf2::convert(p1.orientation, tf2_quat1);
  tf2::convert(p2.orientation, tf2_quat2);
  rotation_distance = fabs(tf2_quat1.angleShortestPath(tf2_quat2));
}

void quaternion2RPY(const geometry_msgs::Quaternion& quaternion, double& roll, double& pitch, double& yaw)
{
  tf2::Quaternion tf_quat;
  tf2::convert(quaternion, tf_quat);
  tf2::Matrix3x3 m(tf_quat);
  m.getRPY(roll, pitch, yaw);
}