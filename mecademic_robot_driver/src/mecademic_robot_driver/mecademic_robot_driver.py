#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, UInt8MultiArray
from mecademic_pydriver import RobotController
from std_srvs import srv

import threading

class MecademicRobotROS_Driver():
    """ROS Mecademic Robot Node Class to make a Node for the Mecademic Robot

    Attributes:
        subscriber: ROS subscriber to send command to the Mecademic Robot through a topic
        publisher: ROS publisher to place replies from the Mecademic Robot in a topic 
        MecademicRobot : driver to control the MecademicRobot Robot
    """
    def __init__(
        self, 
        address='192.168.0.100', 
        rosnode_name="mecademic_robot_driver",
        activate=True, 
        home=True
        ):
        """Constructor for the ROS MecademicRobot Driver
        """

        #Lock
        self._robot_lock = threading.Lock() 

        rospy.init_node(rosnode_name)

        self.pub_log = rospy.Publisher("mecademic_log",String,queue_size=50)

        self.robot = RobotController( 
            address, 
            #socket_timeout=0.1,
            #motion_commands_response_timeout=0.001,
            #log_size=100, 
            on_new_messages_received=self.on_new_messages_received
        )

        #Connect to the control interface
        rospy.loginfo("Conncting to the Robot Control Interface...")
        self.robot.connect()
        rospy.loginfo("Connected to the Robot Control Interface!")
        if activate:
            self.activate()
            if home:
                self.home()

        self.srv_activate = rospy.Service('activate', srv.Trigger, self.activate_srv_cb)
        self.srv_clear_motion = rospy.Service('clear_motion', srv.Trigger, self.clear_motion_srv_cb)
        self.srv_deactivate = rospy.Service('deactivate', srv.Trigger, self.deactivate_srv_cb)
        #self.srv_get_conf = rospy.Service('get_conf', TODO, self.get_conf_srv_cb)
        #self.srv_get_status_robot = rospy.Service('get_status_robot', TODO, self.get_status_robot_srv_cb)
        self.srv_home = rospy.Service('home', srv.Trigger, self.home_srv_cb)
        self.srv_reset_error = rospy.Service('reset_error', srv.Trigger, self.reset_error_srv_cb)
        self.srv_resume_motion = rospy.Service('resume_motion', srv.Trigger, self.resume_motion_srv_cb)
        self.srv_set_eob = rospy.Service('set_eob', srv.SetBool, self.set_eob_srv_cb)
        self.srv_set_eom = rospy.Service('set_eom', srv.SetBool, self.set_eom_srv_cb)

    def on_new_messages_received(self, messages):
        """
        Callbk called when the log receives new messages from the socket
        Usefull to intercept the messages without remove them from the log
        """
        # TODO publish on a topic
        for message in messages:
            msg = String()
            msg.data = "[{}][{}]".format(message[0],message[1])
            self.pub_log.publish(msg)

    # def acquire_lock(self,wait=True):
    #     """
    #     Acquire Lock
    #     wait: bool
    #         If wait for unlock
    #     Return True if lock acquired
    #     """
    #     if(not wait and self.locked):
    #         return False
    #     while(self.locked):
    #         pass
    #     self.locked = True
    #     return True

    # def release_lock(self):
    #     """
    #     Release an acquired lock
    #     raise an error if the lock was not True
    #     """
    #     if( not self.locked ):
    #         raise RuntimeError("MecademicRobotROS_Driver::release_lock - lock not acquired")
    #     self.locked = False

    # def mutex_op(self,operation,log_init=None,log_final=None):
    #     """
    #     Performs critical operations using the lock
    #     """
    #     self.acquire_lock()
    #     try:
    #         if log_init:
    #             rospy.loginfo(log_init)
    #         output = operation()
    #         if log_final:
    #             rospy.loginfo(log_final)
    #     finally:
    #         self.release_lock()
    #     return output


    def activate(self):
        """
        Activate the robot
        """
        with self._robot_lock:
            rospy.loginfo("Sending ActivateRobot...")
            self.robot.ActivateRobot()
            rospy.loginfo("ActivateRobot Sent!")

    def deactivate(self):
        """
        Deactivate the robot
        """
        with self._robot_lock:
            rospy.loginfo("Sending DeactivateRobot...")
            self.robot.DeactivateRobot()
            rospy.loginfo("DeactivateRobot Sent!")

    def get_status_robot(self):
        """
        Get Status of the robot
        """
        with self._robot_lock:
            return self.robot.GetStatusRobot()
    
    def home(self):
        """
        Home the robot
        """
        with self._robot_lock:
            rospy.loginfo("Sending Home...")
            self.robot.Home()
            rospy.loginfo("Home Sent!")

    ################################################
    ###     SERVICES CB REQUEST COMMANDS        ####
    ################################################

    def activate_srv_cb(self, req):
        """
        activate cb
        """
        self.activate()
        res = srv.TriggerResponse()
        res.message = "Robot Active"
        res.success = True
        return res

    def clear_motion_srv_cb(self, req):
        """
        ClearMotion cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending ClearMotion...")
            self.robot.ClearMotion()
            rospy.loginfo("ClearMotion Sent!")
        res = srv.TriggerResponse()
        res.message = "Motion Cleared"
        res.success = True
        return res

    def deactivate_srv_cb(self, req):
        """
        deactivate cb
        """
        self.deactivate()
        res = srv.TriggerResponse()
        res.message = "Robot Deactivated"
        res.success = True
        return res

    def get_conf_srv_cb(self, req):
        """
        GetConf cb
        """
        raise NotImplementedError("get_conf_srv_cb")
        # with self._robot_lock:
        #     rospy.loginfo("Sending GetConf...")
        #     conf = self.robot.GetConf()
        #     rospy.loginfo("GetConf Sent!")
        # #TODO getConf service
        # res = srv.TriggerResponse()
        # res.message = "Motion Cleared"
        # res.success = True
        # return res

    def get_status_robot_srv_cb(self, req):
        """
        GetStatusRobot cb
        """
        raise NotImplementedError("get_status_robot_srv_cb")
        # rospy.loginfo("Sending GetStatusRobot...")
        # status = self.get_status_robot()
        # rospy.loginfo("GetStatusRobot Sent!")
        # #TODO GetStatusRobot service
        # res = srv.TriggerResponse()
        # res.message = "Motion Cleared"
        # res.success = True
        # return res

    def home_srv_cb(self, req):
        """
        home cb
        """
        self.home()
        res = srv.TriggerResponse()
        res.message = "Home OK"
        res.success = True
        return res

    def reset_error_srv_cb(self, req):
        """
        ResetError cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending ResetError...")
            self.robot.ResetError()
            rospy.loginfo("ResetError Sent!")
        res = srv.TriggerResponse()
        res.message = "Error Resets"
        res.success = True
        return res

    def resume_motion_srv_cb(self, req):
        """
        ResumeMotion cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending ResumeMotion...")
            self.robot.ResumeMotion()
            rospy.loginfo("ResumeMotion Sent!")
        res = srv.TriggerResponse()
        res.message = "Motion Resumed"
        res.success = True
        return res

    def set_eob_srv_cb(self, req):
        """
        SetEOB cb
        """
        if req.data:
            e = 1
        else:
            e = 0
        with self._robot_lock:
            rospy.loginfo("Sending SetEOB({})...".format(e))
            self.robot.SetEOB(e)
            rospy.loginfo("SetEOB({}) Sent!".format(e))
        res = srv.SetBoolResponse()
        res.message = "SetEOB({}) Sent".format(e)
        res.success = True
        return res

    def set_eom_srv_cb(self, req):
        """
        SetEOM cb
        """
        if req.data:
            e = 1
        else:
            e = 0
        with self._robot_lock:
            rospy.loginfo("Sending SetEOM({})...".format(e))
            self.robot.SetEOM(e)
            rospy.loginfo("SetEOM({}) Sent!".format(e))
        res = srv.SetBoolResponse()
        res.message = "SetEOM({}) Sent".format(e)
        res.success = True
        return res

    ################################################
    ###     SERVICES CB MOTION COMMANDS         ####
    ################################################
    
    # DEL
    def __del__(self):
        """
        Deconstructor for the Mecademic Robot ROS driver
        Deactivates the robot and closes socket connection with the robot
        """

        self.robot.DeactivateRobot()

        #rospy.loginfo('Disconnecting...')
        self.robot.disconnect()
        #rospy.loginfo('Robot Disonnected')
        

if __name__ == "__main__":

    mecademic_ros_driver = MecademicRobotROS_Driver()

    rospy.spin()

