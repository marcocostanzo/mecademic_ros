#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, UInt8MultiArray
from MecademicRobot import RobotController

class MecademicRobotROS_Driver():
    """ROS Mecademic Robot Node Class to make a Node for the Mecademic Robot

    Attributes:
        subscriber: ROS subscriber to send command to the Mecademic Robot through a topic
        publisher: ROS publisher to place replies from the Mecademic Robot in a topic 
        MecademicRobot : driver to control the MecademicRobot Robot
    """
    def __init__(self, address='192.168.0.100', activate_and_home=True):
        """Constructor for the ROS MecademicRobot Driver
        """

        print('Connecting...')
        self.robot = RobotController(address)
        if not self.robot.Connect():
            print('Connection Fail')
            raise Exception('MecademicRobotROS_Driver', 'Connection Fail')
        print('Robot Connected')

        #TODO use Semaphore
        self.socket_available = True

        if activate_and_home:
            self.activate()
            self.home()

        rospy.init_node("mecademic_robot_driver")
        self.joint_subscriber  = rospy.Subscriber("MecademicRobot_joint", JointState, self.joint_callback)
        self.pose_subscriber   = rospy.Subscriber("MecademicRobot_pose", Pose, self.pose_callback)
        self.command_subcriber = rospy.Subscriber("MecademicRobot_command", String, self.command_callback)
        self.gripper_subcriber = rospy.Subscriber("MecademicRobot_gripper", Bool, self.gripper_callback)
        self.reply_publisher   = rospy.Publisher("MecademicRobot_reply", String, queue_size=1)
        self.status_publisher  = rospy.Publisher("MecademicRobot_status", UInt8MultiArray, queue_size=1)

    def command_callback(self, command):
        """Forwards a ascii command to the Mecademic Robot

        :param command: ascii command to forward to the Robot
        """
        while(not self.socket_available):       #wait for socket to be available
            pass
        self.socket_available = False              #block socket from being used in other processes
        
        if(command.data != 'ResetError' and self.robot.isInError()):
            print('Mecademic is in error, cannot send the command')
            return
        
        reply = self.robot.exchangeMsg(command.data, decode=False)
        self.socket_available = True               #Release socket so other processes can use it
        if(reply is not None):
            self.reply_publisher.publish(reply)
        
    def joint_callback(self, joints):
        """
        Callback when the MecademicRobot_emit topic receives a message
        Forwards message to driver that translate into real command
        to the Mecademic Robot

        :param joints: message received from topic containing position and velocity information
        """
        while(not self.socket_available):               #wait for the socket to be available
            pass
        self.socket_available = False                      #Block other processes from using the socket
        if(self.robot.isInError()):
            print('Mecademic is in error, cannot send the command')
            return

        #TODO should parse the joint_name string
        if(len(joints.position)==6):
            #TODO check joint limits
            reply = self.robot.MoveJoints(joints.position[0],joints.position[1],joints.position[2],joints.position[3],joints.position[4],joints.position[5])
        else:
            print('Invalid joints len')
            reply = None

        self.socket_available = True                       #Release the socket so other processes can use it
        if(reply is not None):
            self.reply_publisher.publish(reply)
        
    def pose_callback(self, pose):
        """Callback when the MecademicRobot_emit topic receives a message
        Forwards message to driver that translate into real command
        to the Mecademic Robot

        :param pose: message received from topic containing position and orientation information
        """
        while(not self.socket_available):           #wait for socket to become available
            pass
        self.socket_available = False                  #Block other processes from using the socket while in use

        if(self.robot.isInError()):
            print('Mecademic is in error, cannot send the command')
            return

        #TODO quat2xyz
        reply = self.robot.MovePose(
                            pose.position.x,
                            pose.position.y,
                            pose.position.z,
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z)

        self.socket_available = True                   #Release socket so other processes can continue
        if(reply is not None):
            self.reply_publisher.publish(reply)   

    def gripper_callback(self, state):
        """Controls whether to open or close the gripper.
        True for open, False for close

        :param state: ROS Bool message
        """
        while(not self.socket_available):       #wait for socket to be available
            pass
        self.socket_available = False              #Block other processes from using the socket
        
        if(self.robot.isInError()):
            print('Mecademic is in error, cannot send the command')
            return
        
        if(state.data):
            reply = self.robot.GripperOpen()
        else:
            reply = self.robot.GripperClose()
        
        self.socket_available = True               #Release socket so other processes can use it
        if(reply is not None):
            self.reply_publisher.publish(reply)

    def publishStatus(self, wait_for_socket=True):
        """
        Get status from robot and publish it

        wait_for_socket: bool
            If wait for socket_available
                if wait_for_socket=False and socket_available=false -> raise an exception 
        """
        if wait_for_socket:
            while(not self.socket_available):
                pass
        else:
            if not self.socket_available:
                raise Exception('MecademicRobotROS_Driver.publishStatus', 'socket not available')
        self.socket_available = False

        robot_status = self.robot.GetStatusRobot()
        gripper_status = self.robot.GetStatusGripper()       

        self.socket_available = True

        status = UInt8MultiArray()
        status.data = [
            robot_status["Activated"],
            robot_status["Homing"],
            robot_status["Simulation"],
            robot_status["Error"],
            robot_status["Paused"],
            robot_status["EOB"],
            robot_status["EOM"],
            gripper_status["Gripper enabled"],
            gripper_status["Homing state"],
            gripper_status["Limit reached"],
            gripper_status["Error state"],
            gripper_status["force overload"]
            ]
        self.status_publisher.publish(status)

    def publishStatusLoop(self, rate=10.0):
        """
        Continuously publish the status
        rate = loop_rate
        """
        loop_rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            try:
                self.publishStatus()
                loop_rate.sleep()
            except Exception as error:
                print(str(error))

    def activate(self):
        print('Sending Activate...')
        res = self.robot.Activate()
        if res is not None:
            print(res)
        print('Activate Done')

    def deactivate(self):
        print('Sending Deactivate...')
        res = self.robot.Deactivate()
        if res is not None:
            print(res)
        print('Deactivate Done')

    def home(self):
        print('Sending Home...')
        res = self.robot.Home()
        if res is not None:
            print(res)
        print('Home Done')
    
    def __del__(self):
        """Deconstructor for the Mecademic Robot ROS driver
        Deactivates the robot and closes socket connection with the robot
        """

        self.deactivate()

        print('Disconnecting...')
        self.robot.Disconnect()
        print('Robot Disonnected')
        

if __name__ == "__main__":

    mecademic_ros_driver = MecademicRobotROS_Driver()

    mecademic_ros_driver.publishStatusLoop()

