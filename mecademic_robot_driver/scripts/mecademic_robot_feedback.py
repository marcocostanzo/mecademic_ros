#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from mecademic_msgs.msg import StatusRobot

from mecademic_pydriver import RobotFeedback

from math import pi as PI


class MecademicRobotROS_Feedback():
    """
    ROS Mecademic Robot Feedback Node Class to make a Feedback Node for the Mecademic Robot
    """

    def __init__(self, ip_address='192.168.0.100'):
        """
        Constructor for the ROS MecademicRobotROS Feedback
        ip_address: str
            Ip address of the robot
        rosnode_name: str
            name to use for the ros node
        """

        self.feedback = RobotFeedback(ip_address)

        self.joints_name = ["meca_axis_1_joint", "meca_axis_2_joint", "meca_axis_3_joint",
                            "meca_axis_4_joint", "meca_axis_5_joint", "meca_axis_6_joint"]
        self.wrf_frame_id = "meca_wrf"

        # Connect to the robot
        rospy.loginfo("Conncting to the Robot Feedback Interface...")
        self.feedback.connect()
        rospy.loginfo("Robot Feedback Interface!")

        self.joint_publisher = rospy.Publisher(
            "state/joint_position", JointState, queue_size=1)
        self.pose_publisher = rospy.Publisher(
            "state/pose", PoseStamped, queue_size=1)
        self.robot_status_publisher = rospy.Publisher(
            "state/status_robot", StatusRobot, queue_size=1, latch=True)

    def loop(self):
        """
        Continuously publish feedbk
        """
        rospy.loginfo("Feedback Loop Started")
        while not rospy.is_shutdown():
            # TODO use stamped messages
            # TODO Use [m] instead of [mm],
            #         [rad] instead of [deg], -> DONE
            #         quaternion instead of euler
            #           q = quaternion_from_euler(0.0, 0.0, 0.0, 'sxyz')
            #           pose_msg.orientation = Quaternion(*q)
            #         programmatic setup of meca_wrf

            joints_deg, pose, robot_status = self.feedback.get_data(
                timeout=1.0)

            time_now = rospy.Time.now()

            if joints_deg:
                # TO RAD
                joints_rad = tuple(joint * PI / 180.0 for joint in joints_deg)

                joints_msg = JointState()
                joints_msg.position = joints_rad
                joints_msg.name = self.joints_name
                joints_msg.header.stamp = time_now
                self.joint_publisher.publish(joints_msg)

            if pose:
                pose_msg = PoseStamped()
                pose_msg.pose.position.x = pose[0]
                pose_msg.pose.position.y = pose[1]
                pose_msg.pose.position.z = pose[2]
                pose_msg.pose.orientation.x = pose[3]
                pose_msg.pose.orientation.y = pose[4]
                pose_msg.pose.orientation.z = pose[5]
                pose_msg.header.stamp = time_now
                pose_msg.header.frame_id = self.wrf_frame_id
                self.pose_publisher.publish(pose_msg)

            if robot_status:
                status_robot_msg = StatusRobot()
                status_robot_msg.activation_state = robot_status['as']
                status_robot_msg.homing_state = robot_status['hs']
                status_robot_msg.simulation = robot_status['sm']
                status_robot_msg.error_state = robot_status['es']
                status_robot_msg.pause_motion = robot_status['pm']
                status_robot_msg.eob = robot_status['eob']
                status_robot_msg.eom = robot_status['eom']
                status_robot_msg.header.stamp = time_now
                self.robot_status_publisher.publish(status_robot_msg)

    def release_resources(self):
        """
        Closes socket connection with the robot
        """
        self.feedback.disconnect()

    def __del__(self):
        """
        Deconstructor for the Mecademic Robot ROS driver
        """
        self.release_resources()


if __name__ == "__main__":

    rospy.init_node("mecademic_robot_feedback")

    ip_address = rospy.get_param('~ip_address', '192.168.0.100')

    mecademic_ros_fb = MecademicRobotROS_Feedback(ip_address=ip_address)

    try:
        mecademic_ros_fb.loop()
    except Exception as e:
        rospy.logerr("MecademicFeedback: {}".format(e))
    finally:
        mecademic_ros_fb.release_resources()
