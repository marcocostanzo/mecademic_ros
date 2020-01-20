#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from MecademicRobot import RobotFeedback

class MecademicRobotROS_Feedback():

    def __init__(self, address='192.168.0.100'):
        """
        Constructor for the ROS MecademicRobotROS Feedback
        """

        self.feedback = RobotFeedback(address)
        
        if not self.feedback.Connect():
            raise Exception('MecademicRobotROS_Feedback', 'Connection Fail')

        rospy.init_node("mecademic_robot_feedback")
        self.joint_publisher   = rospy.Publisher("mecademic/state/joint_position", JointState, queue_size=1) 
        self.pose_publisher    = rospy.Publisher("mecademic/state/pose", Pose, queue_size=1)

    def loop(self):
        """
        Continuously publish feedbk
        """
        while not rospy.is_shutdown():
            try:
                # TODO MOD HERE use stamped version
                self.feedback.getData()
                joints_fb = JointState()
                joints_fb.position = self.feedback.joints
                pose_fb = Pose()
                pose_fb.position.x = self.feedback.cartesian[0]  
                pose_fb.position.y = self.feedback.cartesian[1] 
                pose_fb.position.z = self.feedback.cartesian[2] 
                # TODO mod here xyz 2 quat
                #q = quaternion_from_euler(0.0, 0.0, 0.0, 'sxyz')
                #pose_fb.orientation = Quaternion(*q)
                pose_fb.orientation.x = self.feedback.cartesian[3] 
                pose_fb.orientation.y = self.feedback.cartesian[4] 
                pose_fb.orientation.z = self.feedback.cartesian[5] 
                self.joint_publisher.publish(joints_fb)
                self.pose_publisher.publish(pose_fb)
            except Exception as error:
                rospy.logerr(str(error))

    def __del__(self):
        """Deconstructor for the Mecademic Robot ROS driver
        Deactivates the robot and closes socket connection with the robot
        """
        self.feedback.Disconnect()

if __name__ == "__main__":

    mecademic_ros_fb = MecademicRobotROS_Feedback()
    
    mecademic_ros_fb.loop()
