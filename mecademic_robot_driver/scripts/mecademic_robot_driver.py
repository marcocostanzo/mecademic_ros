#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, UInt8MultiArray
from mecademic_pydriver import RobotController
import std_srvs.srv
import mecademic_msgs.srv

import threading

class MecademicRobotROS_Driver():
    """
    ROS Mecademic Robot Driver Node Class to make a Control Node for the Mecademic Robot
    """
    def __init__(
        self, 
        ip_address='192.168.0.100'
        ):
        """
        Constructor for the ROS MecademicRobot Driver
        ip_address: str
            Ip address of the robot
        rosnode_name: str
            name to use for the ros node
        """

        #Lock
        self._robot_lock = threading.Lock() 

        self.pub_log = rospy.Publisher("log",String,queue_size=50)

        self.robot = RobotController( 
            ip_address, 
            #socket_timeout=0.1,
            #motion_commands_response_timeout=0.001,
            #log_size=100, 
            on_new_messages_received=self.on_new_messages_received
        )

    def robot_setup(self,activate=True,home=True, high_performance=False):
        """
        Setup the robot
        activate: bool
            If activate the robot during setup of the object
        home: bool
            If home the robot during setup of the object (only if activate=True)
        high_performance: bool
            Set all performance to max (only if home=True)
        """
        #Connect to the control interface
        rospy.loginfo("Conncting to the Robot Control Interface...")
        self.robot.connect()
        rospy.loginfo("Connected to the Robot Control Interface!")
        if activate:
            rospy.loginfo("Activating...")
            self.robot.ActivateRobot()
            rospy.loginfo("Activarted!")
            if home:
                rospy.loginfo("Homing...")
                self.robot.Home()
                rospy.loginfo("Home Done!")
                if high_performance:
                    self.high_performances()

    def ros_setup(self):
        """
        Setup the ROS interface
        """
        self.srv_activate = rospy.Service('activate_robot', std_srvs.srv.Trigger, self.activate_srv_cb)
        self.srv_clear_motion = rospy.Service('clear_motion', std_srvs.srv.Trigger, self.clear_motion_srv_cb)
        self.srv_deactivate = rospy.Service('deactivate_robot', std_srvs.srv.Trigger, self.deactivate_srv_cb)
        self.srv_get_conf = rospy.Service('get_conf', mecademic_msgs.srv.GetConf, self.get_conf_srv_cb)
        self.srv_get_status_robot = rospy.Service('get_status_robot', mecademic_msgs.srv.GetStatusRobot, self.get_status_robot_srv_cb)
        self.srv_home = rospy.Service('home', std_srvs.srv.Trigger, self.home_srv_cb)
        self.srv_reset_error = rospy.Service('reset_error', std_srvs.srv.Trigger, self.reset_error_srv_cb)
        self.srv_resume_motion = rospy.Service('resume_motion', std_srvs.srv.Trigger, self.resume_motion_srv_cb)
        self.srv_set_eob = rospy.Service('set_eob', std_srvs.srv.SetBool, self.set_eob_srv_cb)
        self.srv_set_eom = rospy.Service('set_eom', std_srvs.srv.SetBool, self.set_eom_srv_cb)
        self.srv_set_monitoring_interval = rospy.Service('set_monitoring_interval', mecademic_msgs.srv.SetValue, self.set_monitoring_interval_srv_cb)
 
        self.srv_move_joints = rospy.Service('move_joints', mecademic_msgs.srv.SetJoints, self.move_joints_srv_cb)
        self.sub_move_joints_vel = rospy.Subscriber('command/joints_vel', mecademic_msgs.msg.Joints, callback=self.move_joints_vel_sub_cb, queue_size=1)
        self.srv_move_lin = rospy.Service('move_lin', mecademic_msgs.srv.SetPose, self.move_lin_srv_cb)
        self.srv_move_lin_rel_trf = rospy.Service('move_lin_rel_trf', mecademic_msgs.srv.SetPose, self.move_lin_rel_trf_srv_cb)
        self.srv_move_lin_rel_wrf = rospy.Service('move_lin_rel_wrf', mecademic_msgs.srv.SetPose, self.move_lin_rel_wrf_srv_cb)
        self.sub_move_lin_vel_trf = rospy.Subscriber('command/vel_trf', TwistStamped, callback=self.move_lin_vel_trf_sub_cb, queue_size=1)
        self.sub_move_lin_vel_wrf = rospy.Subscriber('command/vel_wrf', TwistStamped, callback=self.move_lin_vel_wrf_sub_cb, queue_size=1)
        self.srv_move_pose = rospy.Service('move_pose', mecademic_msgs.srv.SetPose, self.move_pose_srv_cb)
        self.srv_set_auto_conf = rospy.Service('set_auto_conf', std_srvs.srv.SetBool, self.set_auto_conf_srv_cb)
        self.srv_set_blending = rospy.Service('set_blending', mecademic_msgs.srv.SetValue, self.set_blending_srv_cb)
        self.srv_set_cart_acc = rospy.Service('set_cart_acc', mecademic_msgs.srv.SetValue, self.set_cart_acc_srv_cb)
        self.srv_set_cart_ang_vel = rospy.Service('set_cart_ang_vel', mecademic_msgs.srv.SetValue, self.set_cart_ang_vel_srv_cb)
        self.srv_set_cart_lin_vel = rospy.Service('set_cart_lin_vel', mecademic_msgs.srv.SetValue, self.set_cart_lin_vel_srv_cb)
        self.srv_set_conf = rospy.Service('set_conf', mecademic_msgs.srv.SetConf, self.set_conf_srv_cb)
        self.srv_set_joint_acc = rospy.Service('set_joint_acc', mecademic_msgs.srv.SetValue, self.set_joint_acc_srv_cb)
        self.srv_set_joint_vel = rospy.Service('set_joint_vel', mecademic_msgs.srv.SetValue, self.set_joint_vel_srv_cb)
        self.srv_set_trf = rospy.Service('set_trf', mecademic_msgs.srv.SetPose, self.set_trf_srv_cb)
        self.srv_set_wrf = rospy.Service('set_wrf', mecademic_msgs.srv.SetPose, self.set_wrf_srv_cb)
        self.srv_set_vel_timeout = rospy.Service('set_vel_timeout', mecademic_msgs.srv.SetValue, self.set_vel_timeout_srv_cb)

        # for check_vel_timestamp
        self.last_vel_timestamp = rospy.Time.now()

    def high_performances(self):
        """
        Set all to high performances
        """
        rospy.loginfo("MecademicDriver: High Perf")
        self.robot.SetMonitoringInterval(0.015) # Do not put less
        self.robot.SetJointAcc(100.0)
        self.robot.SetJointVel(100.0)
        self.robot.SetVelTimeout(0.200) #Patched
        self.robot.SetBlending(100.0)
        self.robot.SetCartAcc(100.0)
        self.robot.SetCartAngVel(180.0)
        self.robot.SetCartLinVel(500.0)

    def on_new_messages_received(self, messages):
        """
        Callbk called when the log receives new messages from the socket
        Usefull to intercept the messages without remove them from the log
        """
        for message in messages:
            msg = String()
            msg.data = "[{}][{}]".format(message[0],message[1])
            self.pub_log.publish(msg)

    def loop_on_log(self, rate=10):
        """
        Loop forever to update the log
        """
        rospy.loginfo("Log Loop Enabled")
        loop_rate = rospy.Rate(rate) 
        while not rospy.is_shutdown():
            with self._robot_lock:
                self.robot.mecademic_log.update_log(wait_for_new_messages=False)
            loop_rate.sleep()

    def get_status_robot(self):
        """
        Get Status of the robot
        """
        with self._robot_lock:
            return self.robot.GetStatusRobot()
    
    ################################################
    ###     SERVICES CB REQUEST COMMANDS        ####
    ################################################

    def activate_srv_cb(self, req):
        """
        activate cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending ActivateRobot...")
            self.robot.ActivateRobot()
            rospy.loginfo("ActivateRobot Sent!")
        res = std_srvs.srv.TriggerResponse()
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
        res = std_srvs.srv.TriggerResponse()
        res.message = "Motion Cleared"
        res.success = True
        return res

    def deactivate_srv_cb(self, req):
        """
        deactivate cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending DeactivateRobot...")
            self.robot.DeactivateRobot()
            rospy.loginfo("DeactivateRobot Sent!")
        res = std_srvs.srv.TriggerResponse()
        res.message = "Robot Deactivated"
        res.success = True
        return res

    def get_conf_srv_cb(self, req):
        """
        GetConf cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending GetConf...")
            conf = self.robot.GetConf()
            rospy.loginfo("GetConf Sent!")
        res = mecademic_msgs.srv.GetConfResponse()
        res.c1 = conf['c1']
        res.c3 = conf['c3']
        res.c5 = conf['c5']
        return res

    def get_status_robot_srv_cb(self, req):
        """
        GetStatusRobot cb
        """
        rospy.loginfo("Sending GetStatusRobot...")
        status = self.get_status_robot()
        rospy.loginfo("GetStatusRobot Sent!")
        res = mecademic_msgs.srv.GetStatusRobotResponse()
        res.status.activation_state = status['as']
        res.status.homing_state = status['hs']
        res.status.simulation = status['sm']
        res.status.error_state = status['es']
        res.status.pause_motion = status['pm']
        res.status.eob = status['eob']
        res.status.eom = status['eom']
        res.status.header.stamp = rospy.Time.now()
        return res

    def home_srv_cb(self, req):
        """
        home cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending Home...")
            self.robot.Home()
            rospy.loginfo("Home Sent!")
        res = std_srvs.srv.TriggerResponse()
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
        res = std_srvs.srv.TriggerResponse()
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
        res = std_srvs.srv.TriggerResponse()
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
        res = std_srvs.srv.SetBoolResponse()
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
        res = std_srvs.srv.SetBoolResponse()
        res.message = "SetEOM({}) Sent".format(e)
        res.success = True
        return res

    def set_monitoring_interval_srv_cb(self, req):
        """
        SetEOM cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending SetMonitoringInterval({})...".format(req.value))
            self.robot.SetMonitoringInterval(req.value)
            rospy.loginfo("Sending SetMonitoringInterval({}) Sent!".format(req.value))
        res = mecademic_msgs.srv.SetValueResponse()
        res.message = "SetMonitoringInterval({}) Sent".format(req.value)
        res.success = True
        return res

    ################################################
    ###     SERVICES CB MOTION COMMANDS         ####
    ################################################

    def check_vel_timestamp(self,stamp):
        """
        Check and update last vel timestamp
        TODO: This is for security reason
        """
        if stamp > self.last_vel_timestamp:
            self.last_vel_timestamp = stamp
            return True
        else:
            rospy.logwarn("MecademicRobot::check_vel_timestamp: old timestamp")
            return False

    def move_joints_srv_cb(self,req):
        """
        Add a move joints command to the robot queue
        """
        with self._robot_lock:
            self.robot.MoveJoints(req.joints)
        res = mecademic_msgs.srv.SetJointsResponse()
        res.message = "MoveJoints Sent"
        res.success = True
        return res

    def move_joints_vel_sub_cb(self,msg):
        """
        Send a move joint vel command
        """
        with self._robot_lock:
            if not self.check_vel_timestamp(msg.header.stamp):
                return
            self.robot.MoveJointsVel(msg.joints)

    def move_lin_srv_cb(self,req):
        """
        Add a move lin command to the robot queue
        """
        with self._robot_lock:
            self.robot.MoveLin(
                [req.position.x,req.position.y,req.position.z],
                [req.orientation.x,req.orientation.y,req.orientation.z]
                )
        res = mecademic_msgs.srv.SetPoseResponse()
        res.message = "MoveLin Sent"
        res.success = True
        return res

    def move_lin_rel_trf_srv_cb(self,req):
        """
        Add a move lin rel trf command to the robot queue
        """
        with self._robot_lock:
            self.robot.MoveLinRelTRF(
                [req.position.x,req.position.y,req.position.z],
                [req.orientation.x,req.orientation.y,req.orientation.z]
                )
        res = mecademic_msgs.srv.SetPoseResponse()
        res.message = "MoveLinRelTRF Sent"
        res.success = True
        return res

    def move_lin_rel_wrf_srv_cb(self,req):
        """
        Add a move lin rel wrf command to the robot queue
        """
        with self._robot_lock:
            self.robot.MoveLinRelWRF(
                [req.position.x,req.position.y,req.position.z],
                [req.orientation.x,req.orientation.y,req.orientation.z]
                )
        res = mecademic_msgs.srv.SetPoseResponse()
        res.message = "MoveLinRelWRF Sent"
        res.success = True
        return res

    def move_lin_vel_trf_sub_cb(self,msg):
        """
        Send a move lin vel trf command
        """
        with self._robot_lock:
            if not self.check_vel_timestamp(msg.header.stamp):
                return
            self.robot.MoveLinVelTRF(
                [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
                [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
                )

    def move_lin_vel_wrf_sub_cb(self,msg):
        """
        Send a move lin vel wrf command
        """
        with self._robot_lock:
            if not self.check_vel_timestamp(msg.header.stamp):
                return
            self.robot.MoveLinVelWRF(
                [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
                [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
                )

    def move_pose_srv_cb(self,req):
        """
        Add a move pose command to the robot queue
        """
        with self._robot_lock:
            self.robot.MovePose(
                [req.position.x,req.position.y,req.position.z],
                [req.orientation.x,req.orientation.y,req.orientation.z]
                )
        res = mecademic_msgs.srv.SetPoseResponse()
        res.message = "MovePose Sent"
        res.success = True
        return res
    
    def set_auto_conf_srv_cb(self, req):
        """
        SetAutoConf cb
        """
        if req.data:
            e = 1
        else:
            e = 0
        with self._robot_lock:
            rospy.loginfo("Sending SetAutoConf({})...".format(e))
            self.robot.SetAutoConf(e)
            rospy.loginfo("SetAutoConf({}) Sent!".format(e))
        res = std_srvs.srv.SetBoolResponse()
        res.message = "SetAutoConf({}) Sent".format(e)
        res.success = True
        return res

    def set_blending_srv_cb(self, req):
        """
        SetBlending cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending SetBlending({})...".format(req.value))
            self.robot.SetBlending(req.value)
            rospy.loginfo("Sending SetBlending({}) Sent!".format(req.value))
        res = mecademic_msgs.srv.SetValueResponse()
        res.message = "SetBlending({}) Sent".format(req.value)
        res.success = True
        return res

    def set_cart_acc_srv_cb(self, req):
        """
        SetCartAcc cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending SetCartAcc({})...".format(req.value))
            self.robot.SetCartAcc(req.value)
            rospy.loginfo("Sending SetCartAcc({}) Sent!".format(req.value))
        res = mecademic_msgs.srv.SetValueResponse()
        res.message = "SetCartAcc({}) Sent".format(req.value)
        res.success = True
        return res

    def set_cart_ang_vel_srv_cb(self, req):
        """
        SetCartAngVel cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending SetCartAngVel({})...".format(req.value))
            self.robot.SetCartAngVel(req.value)
            rospy.loginfo("Sending SetCartAngVel({}) Sent!".format(req.value))
        res = mecademic_msgs.srv.SetValueResponse()
        res.message = "SetCartAngVel({}) Sent".format(req.value)
        res.success = True
        return res

    def set_cart_lin_vel_srv_cb(self, req):
        """
        SetCartLinVel cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending SetCartLinVel({})...".format(req.value))
            self.robot.SetCartLinVel(req.value)
            rospy.loginfo("Sending SetCartLinVel({}) Sent!".format(req.value))
        res = mecademic_msgs.srv.SetValueResponse()
        res.message = "SetCartLinVel({}) Sent".format(req.value)
        res.success = True
        return res

    def set_conf_srv_cb(self, req):
        """
        SetConf cb
        """
        #TODO
        with self._robot_lock:
            rospy.loginfo("Sending SetConf({},{},{})...".format(req.c1,req.c3,req.c5))
            self.robot.SetConf(c1=req.c1,c3=req.c3,c5=req.c5)
            rospy.loginfo("Sending SetConf({},{},{}) Sent!".format(req.c1,req.c3,req.c5))
        res = mecademic_msgs.srv.SetConfResponse()
        res.message = "SetConf({},{},{}) Sent".format(req.c1,req.c3,req.c5)
        res.success = True
        return res

    def set_joint_acc_srv_cb(self, req):
        """
        SetJointAcc cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending SetJointAcc({})...".format(req.value))
            self.robot.SetJointAcc(req.value)
            rospy.loginfo("Sending SetJointAcc({}) Sent!".format(req.value))
        res = mecademic_msgs.srv.SetValueResponse()
        res.message = "SetJointAcc({}) Sent".format(req.value)
        res.success = True
        return res

    def set_joint_vel_srv_cb(self, req):
        """
        SetJointVel cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending SetJointVel({})...".format(req.value))
            self.robot.SetJointVel(req.value)
            rospy.loginfo("Sending SetJointVel({}) Sent!".format(req.value))
        res = mecademic_msgs.srv.SetValueResponse()
        res.message = "SetJointVel({}) Sent".format(req.value)
        res.success = True
        return res

    def set_trf_srv_cb(self,req):
        """
        Add a set trf command to the robot queue
        """
        with self._robot_lock:
            rospy.loginfo("Sending SetTRF...")
            self.robot.SetTRF(
                [req.position.x,req.position.y,req.position.z],
                [req.orientation.x,req.orientation.y,req.orientation.z]
                )
            rospy.loginfo("SetTRF Sent!")
        res = mecademic_msgs.srv.SetPoseResponse()
        res.message = "SetTRF Sent"
        res.success = True
        return res

    def set_wrf_srv_cb(self,req):
        """
        Add a set trf command to the robot queue
        """
        with self._robot_lock:
            rospy.loginfo("Sending SetWRF...")
            self.robot.SetWRF(
                [req.position.x,req.position.y,req.position.z],
                [req.orientation.x,req.orientation.y,req.orientation.z]
                )
            rospy.loginfo("SetWRF Sent!")
        res = mecademic_msgs.srv.SetPoseResponse()
        res.message = "SetWRF Sent"
        res.success = True
        return res

    def set_vel_timeout_srv_cb(self, req):
        """
        SetVelTimeout cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending SetVelTimeout({})...".format(req.value))
            self.robot.SetVelTimeout(req.value)
            rospy.loginfo("Sending SetVelTimeout({}) Sent!".format(req.value))
        res = mecademic_msgs.srv.SetValueResponse()
        res.message = "SetVelTimeout({}) Sent".format(req.value)
        res.success = True
        return res

    def release_resources(self):
        """
        Deactivates the robot and closes socket connection with the robot
        """
        if self.robot:
            try:
                self.robot.DeactivateRobot()
            except Exception as e:
                rospy.logerr("MecademicDriver: {}".format(e))
            finally:
                self.robot.disconnect()

    # DEL
    def __del__(self):
        """
        Deconstructor for the Mecademic Robot ROS driver
        Deactivates the robot and closes socket connection with the robot
        """
        self.release_resources()
        

if __name__ == "__main__":

    rospy.init_node("mecademic_robot_driver")

    ip_address = rospy.get_param('~ip_address', '192.168.0.100')
    activate = rospy.get_param('~activate', True)
    home = rospy.get_param('~home', True)
    high_performance = rospy.get_param('~high_performance', False)
    try:
        mecademic_ros_driver = MecademicRobotROS_Driver(ip_address=ip_address)
        mecademic_ros_driver.robot_setup(
            activate=activate,
            home=home,
            high_performance=high_performance
            )
        mecademic_ros_driver.ros_setup()
        mecademic_ros_driver.loop_on_log()
    except Exception as e:
        rospy.logerr("MecademicDriver: {}".format(e))
    finally:
        mecademic_ros_driver.release_resources()
