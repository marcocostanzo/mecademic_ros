#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from mecademic_pydriver import RobotFeedback

class MecademicRobotROS_Feedback():

    def __init__(self, address='192.168.0.100', rosnode_name="mecademic_robot_feedback"):
        """
        Constructor for the ROS MecademicRobotROS Feedback
        """

        rospy.init_node(rosnode_name)

        self.feedback = RobotFeedback(address)

        self.joints_name = ["A1","A2","A3","A4","A5","A6"]

        #Connect to the robot
        rospy.loginfo("Conncting to the Robot Feedback Interface...")
        self.feedback.connect()
        rospy.loginfo("Robot Feedback Interface!")

        self.joint_publisher   = rospy.Publisher("mecademic/state/joint_position", JointState, queue_size=1) 
        self.pose_publisher    = rospy.Publisher("mecademic/state/pose", PoseStamped, queue_size=1)
    
    def loop(self):
        """
        Continuously publish feedbk
        """
        rospy.loginfo("Feedback Loop Started")
        while not rospy.is_shutdown():
            #TODO use stamped messages
            #TODO Use [m] instead of [mm],
            #         [rad] instead of [deg],
            #         quaternion instead of euler
            #           q = quaternion_from_euler(0.0, 0.0, 0.0, 'sxyz')
            #           pose_msg.orientation = Quaternion(*q)
            
            joints, pose = self.feedback.get_data(timeout=1.0)

            time_now = rospy.Time.now()

            joints_msg = JointState()
            joints_msg.position = joints
            joints_msg.name = self.joints_name
            joints_msg.header.stamp = time_now

            pose_msg = PoseStamped()
            pose_msg.pose.position.x = pose[0]  
            pose_msg.pose.position.y = pose[1] 
            pose_msg.pose.position.z = pose[2]
            pose_msg.pose.orientation.x = pose[3] 
            pose_msg.pose.orientation.y = pose[4] 
            pose_msg.pose.orientation.z = pose[5] 
            pose_msg.header.stamp = time_now

            self.joint_publisher.publish(joints_msg)
            self.pose_publisher.publish(pose_msg)

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

    mecademic_ros_fb = MecademicRobotROS_Feedback()
    
    try:
        mecademic_ros_fb.loop()
    finally:
        mecademic_ros_fb.release_resources()