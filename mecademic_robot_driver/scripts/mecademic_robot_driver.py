#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, TwistStamped, TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, UInt8MultiArray
from mecademic_pydriver import RobotController
import std_srvs.srv
import mecademic_msgs.srv

import tf2_ros
import tf

from math import pi as PI

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

        # Lock
        self._robot_lock = threading.Lock()

        self.pub_log = rospy.Publisher("log", String, queue_size=50)

        self.robot = RobotController(
            ip_address,
            # socket_timeout=0.1,
            # motion_commands_response_timeout=0.001,
            # log_size=100,
            on_new_messages_received=self.on_new_messages_received
        )

    def robot_setup(self, activate=True, home=True, high_performance=False):
        """
        Setup the robot
        activate: bool
            If activate the robot during setup of the object
        home: bool
            If home the robot during setup of the object (only if activate=True)
        high_performance: bool
            Set all performance to max (only if home=True)
        """
        # Connect to the control interface
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

    def ros_setup(self, tf_prefix=""):
        """
        Setup the ROS interface
        """

        # for check_vel_timestamp
        self.last_vel_timestamp = rospy.Time.now()

        # TF
        self.brf_frame_id = tf_prefix + "meca_brf"
        self.wrf_frame_id = tf_prefix + "meca_wrf"
        self.frf_frame_id = tf_prefix + "meca_frf"
        self.trf_frame_id = tf_prefix + "meca_trf"
        #TF2
        self._tf_lock = threading.Lock()
        self._tf_transforms_list = [TransformStamped(), TransformStamped()]
        self.update_tf_wrf()
        self.update_tf_trf()
        self.start_tf_publisher_timer()

        self.srv_activate = rospy.Service(
            'activate_robot', std_srvs.srv.Trigger, self.activate_srv_cb)
        self.srv_clear_motion = rospy.Service(
            'clear_motion', std_srvs.srv.Trigger, self.clear_motion_srv_cb)
        self.srv_deactivate = rospy.Service(
            'deactivate_robot', std_srvs.srv.Trigger, self.deactivate_srv_cb)
        self.srv_get_conf = rospy.Service(
            'get_conf', mecademic_msgs.srv.GetConf, self.get_conf_srv_cb)
        self.srv_get_status_robot = rospy.Service(
            'get_status_robot', mecademic_msgs.srv.GetStatusRobot, self.get_status_robot_srv_cb)
        self.srv_home = rospy.Service(
            'home', std_srvs.srv.Trigger, self.home_srv_cb)
        self.srv_reset_error = rospy.Service(
            'reset_error', std_srvs.srv.Trigger, self.reset_error_srv_cb)
        self.srv_resume_motion = rospy.Service(
            'resume_motion', std_srvs.srv.Trigger, self.resume_motion_srv_cb)
        self.srv_set_eob = rospy.Service(
            'set_eob', std_srvs.srv.SetBool, self.set_eob_srv_cb)
        self.srv_set_eom = rospy.Service(
            'set_eom', std_srvs.srv.SetBool, self.set_eom_srv_cb)
        self.srv_set_monitoring_interval = rospy.Service(
            'set_monitoring_interval', mecademic_msgs.srv.SetValue, self.set_monitoring_interval_srv_cb)

        self.srv_move_joints_meca_conv = rospy.Service(
            'meca_convention/move_joints', mecademic_msgs.srv.SetJoints, self.move_joints_meca_conv_srv_cb)
        self.srv_move_joints = rospy.Service(
            'move_joints', mecademic_msgs.srv.SetJoints, self.move_joints_srv_cb)
        self.sub_move_joints_vel_meca_conv = rospy.Subscriber(
            'meca_convention/command/joints_vel', mecademic_msgs.msg.Joints, callback=self.move_joints_vel_meca_conv_sub_cb, queue_size=1)
        self.sub_move_joints_vel = rospy.Subscriber(
            'command/joints_vel', mecademic_msgs.msg.Joints, callback=self.move_joints_vel_sub_cb, queue_size=1)
        self.srv_move_lin_meca_conv = rospy.Service(
            'meca_convention/move_lin', mecademic_msgs.srv.SetMecaPose, self.move_lin_meca_conv_srv_cb)
        self.srv_move_lin = rospy.Service(
            'move_lin', mecademic_msgs.srv.SetPose, self.move_lin_srv_cb)
        self.srv_move_lin_rel_trf_meca_conv = rospy.Service(
            'meca_convention/move_lin_rel_trf', mecademic_msgs.srv.SetMecaPose, self.move_lin_rel_trf_meca_conv_srv_cb)
        self.srv_move_lin_rel_wrf_meca_conv = rospy.Service(
            'meca_convention/move_lin_rel_wrf', mecademic_msgs.srv.SetMecaPose, self.move_lin_rel_wrf_meca_conv_srv_cb)
        self.sub_move_vel_trf_meca_conv = rospy.Subscriber(
            'command/meca_convention/vel_trf', TwistStamped, callback=self.move_lin_vel_trf_meca_conv_sub_cb, queue_size=1)
        self.sub_move_vel_trf = rospy.Subscriber(
            'command/vel_trf', TwistStamped, callback=self.move_lin_vel_trf_sub_cb, queue_size=1)
        self.sub_move_vel_wrf_meca_conv = rospy.Subscriber(
            'command/meca_convention/vel_wrf', TwistStamped, callback=self.move_lin_vel_wrf_meca_conv_sub_cb, queue_size=1)
        self.sub_move_vel_wrf = rospy.Subscriber(
            'command/vel_wrf', TwistStamped, callback=self.move_lin_vel_wrf_sub_cb, queue_size=1)
        self.srv_move_pose_meca_conv = rospy.Service(
            'meca_convention/move_pose', mecademic_msgs.srv.SetMecaPose, self.move_pose_meca_conv_srv_cb)
        self.srv_move_pose = rospy.Service(
            'move_pose', mecademic_msgs.srv.SetPose, self.move_pose_srv_cb)
        self.srv_set_auto_conf = rospy.Service(
            'set_auto_conf', std_srvs.srv.SetBool, self.set_auto_conf_srv_cb)
        self.srv_set_blending = rospy.Service(
            'set_blending', mecademic_msgs.srv.SetPerc, self.set_blending_srv_cb)
        self.srv_set_cart_acc = rospy.Service(
            'set_cart_acc', mecademic_msgs.srv.SetPerc, self.set_cart_acc_srv_cb)
        self.srv_set_cart_ang_vel_meca_conv = rospy.Service(
            'meca_convention/set_cart_ang_vel', mecademic_msgs.srv.SetValue, self.set_cart_ang_vel_meca_conv_srv_cb)
        self.srv_set_cart_lin_vel_meca_conv = rospy.Service(
            'meca_convention/set_cart_lin_vel', mecademic_msgs.srv.SetValue, self.set_cart_lin_vel_meca_conv_srv_cb)
        self.srv_set_conf = rospy.Service(
            'set_conf', mecademic_msgs.srv.SetConf, self.set_conf_srv_cb)
        self.srv_set_joint_acc = rospy.Service(
            'set_joint_acc', mecademic_msgs.srv.SetPerc, self.set_joint_acc_srv_cb)
        self.srv_set_joint_vel = rospy.Service(
            'set_joint_vel', mecademic_msgs.srv.SetPerc, self.set_joint_vel_srv_cb)
        self.srv_set_trf_meca_conv = rospy.Service(
            'meca_convention/set_trf', mecademic_msgs.srv.SetMecaPose, self.set_trf_meca_conv_srv_cb)
        self.srv_set_trf = rospy.Service(
            'set_trf', mecademic_msgs.srv.SetPose, self.set_trf_srv_cb)
        self.srv_set_wrf_meca_conv = rospy.Service(
            'meca_convention/set_wrf', mecademic_msgs.srv.SetMecaPose, self.set_wrf_meca_conv_srv_cb)
        self.srv_set_wrf = rospy.Service(
            'set_wrf', mecademic_msgs.srv.SetMecaPose, self.set_wrf_srv_cb)
        self.srv_set_vel_timeout = rospy.Service(
            'set_vel_timeout', mecademic_msgs.srv.SetValue, self.set_vel_timeout_srv_cb)

    def high_performances(self):
        """
        Set all to high performances
        """
        rospy.loginfo("MecademicDriver: High Perf")
        self.robot.SetMonitoringInterval(0.015)  # Do not put less
        self.robot.SetJointAcc(100.0)
        self.robot.SetJointVel(100.0)
        self.robot.SetVelTimeout(0.200)  # Patched
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
            msg.data = "[{}][{}]".format(message[0], message[1])
            self.pub_log.publish(msg)

    def start_tf_publisher_timer(self, period=0.1):
        """
        Start the tf publisher timer
        """
        self._transform_broadcaster = tf2_ros.TransformBroadcaster()
        self._tf_publisher_timer = rospy.Timer(rospy.Duration(period),self.tf_publisher_timer_cb)
        rospy.loginfo("TF Publisher Enabled")

    def tf_publisher_timer_cb(self,event):
        """
        Publish on \tf in a thread safe way
        """
        with self._tf_lock:
            self._transform_broadcaster.sendTransform(self._tf_transforms_list)

    def start_update_log_timer(self, period=0.1):
        """
        Start the update log timer
        """
        self.update_log_timer = rospy.Timer(rospy.Duration(period),self.update_log_timer_cb)
        rospy.loginfo("Log Auto-Update Enabled")

    def update_log_timer_cb(self,event):
        """
        Update the log in a thread-safe way
        """
        with self._robot_lock:
            self.robot.mecademic_log.update_log(
                wait_for_new_messages=False)

    def update_tf_wrf(self, tanslation=[0, 0, 0], orientation=[0, 0, 0]):
        """
        Update the brf->wrf transform
        translation and orientation are in mecademic convention
        """
        with self._tf_lock:
            self._tf_transforms_list[0] = self.compute_tf(tanslation, orientation,
                                                 self.brf_frame_id, self.wrf_frame_id)

    def update_tf_trf(self, tanslation=[0, 0, 0], orientation=[0, 0, 0]):
        """
        Update the brf->wrf transform
        translation and orientation are in mecademic convention
        """
        with self._tf_lock:
            self._tf_transforms_list[1] = self.compute_tf(tanslation, orientation,
                                                 self.frf_frame_id, self.trf_frame_id)

    def compute_tf(self, tanslation, orientation, source_frame_id, taget_frame_id):
        """
        compute the transform
        translation and orientation are in mecademic convention
        """

        quaternion = tf.transformations.quaternion_from_euler(
            orientation[0] * PI/180.0, orientation[1] * PI/180.0, orientation[2] * PI/180.0, 'rxyz')  # rxyz should be mobile xyz

        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = source_frame_id
        static_transformStamped.child_frame_id = taget_frame_id
        static_transformStamped.transform.translation.x = tanslation[0] / 1000.0
        static_transformStamped.transform.translation.y = tanslation[1] / 1000.0
        static_transformStamped.transform.translation.z = tanslation[2] / 1000.0
        static_transformStamped.transform.rotation.x = quaternion[0]
        static_transformStamped.transform.rotation.y = quaternion[1]
        static_transformStamped.transform.rotation.z = quaternion[2]
        static_transformStamped.transform.rotation.w = quaternion[3]

        return static_transformStamped

    def get_status_robot(self):
        """
        Get Status of the robot
        """
        with self._robot_lock:
            return self.robot.GetStatusRobot()

    def set_pose_req_2_set_meca_pose_req(self, set_pose_req, target_frame_id):
        """
        Convert a set pose req in mecademic convention
        """
        if not set_pose_req.pose.header.frame_id == target_frame_id:
            raise Exception(
                "The required pose must be in the frame " + target_frame_id)

        meca_req = mecademic_msgs.srv.SetMecaPoseRequest()
        meca_req.position.x = set_pose_req.pose.pose.position.x * 1000.0
        meca_req.position.y = set_pose_req.pose.pose.position.y * 1000.0
        meca_req.position.z = set_pose_req.pose.pose.position.z * 1000.0

        eul_xyz = tf.transformations.euler_from_quaternion(
            [set_pose_req.pose.pose.orientation.x, set_pose_req.pose.pose.orientation.y, set_pose_req.pose.pose.orientation.z, set_pose_req.pose.pose.orientation.w], 'rxyz')

        meca_req.orientation.x = eul_xyz[0] * 180.0 / PI
        meca_req.orientation.y = eul_xyz[1] * 180.0 / PI
        meca_req.orientation.z = eul_xyz[2] * 180.0 / PI

        print("meca_req\n")
        print(meca_req)

        return meca_req

    def set_meca_pose_res_2_set_pose_res(self, set_meca_pose_res):
        """
        Convert a set meca pose res in set pose res
        """
        res = mecademic_msgs.srv.SetPoseResponse()
        res.success = set_meca_pose_res.success
        res.message = set_meca_pose_res.message
        return res

    def twist_stamped_2_mecademic_twist(self, msg, target_frame_id):
        """
        Convert SI twist in mecademic convention
        Warning, this updates the input
        """
        if not msg.header.frame_id == target_frame_id:
            raise Exception(
                "The required twist must be in the frame " + target_frame_id)

        msg.twist.linear.x *= 1000.0
        msg.twist.linear.y *= 1000.0
        msg.twist.linear.z *= 1000.0
        msg.twist.angular.x *= 180.0/PI
        msg.twist.angular.y *= 180.0/PI
        msg.twist.angular.z *= 180.0/PI

        return msg

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
            rospy.loginfo(
                "Sending SetMonitoringInterval({})...".format(req.value))
            self.robot.SetMonitoringInterval(req.value)
            rospy.loginfo(
                "Sending SetMonitoringInterval({}) Sent!".format(req.value))
        res = mecademic_msgs.srv.SetValueResponse()
        res.message = "SetMonitoringInterval({}) Sent".format(req.value)
        res.success = True
        return res

    ################################################
    ###     SERVICES CB MOTION COMMANDS         ####
    ################################################

    def check_vel_timestamp(self, stamp):
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

    def move_joints_meca_conv_srv_cb(self, req):
        """
        Add a move joints command to the robot queue
        """
        with self._robot_lock:
            self.robot.MoveJoints(req.joints)
        res = mecademic_msgs.srv.SetJointsResponse()
        res.message = "MoveJoints Sent"
        res.success = True
        return res

    def move_joints_srv_cb(self, req):
        """
        Add a move joints command [rad] to the robot queue
        """
        req.joints = tuple(joint * 180.0 / PI for joint in req.joints)
        return self.move_joints_meca_conv_srv_cb(req)

    def move_joints_vel_meca_conv_sub_cb(self, msg):
        """
        Send a move joint vel command
        """
        with self._robot_lock:
            if not self.check_vel_timestamp(msg.header.stamp):
                return
            self.robot.MoveJointsVel(msg.joints)

    def move_joints_vel_sub_cb(self, msg):
        """
        Send a move joint vel command [rad/s]
        """
        msg.joints = tuple(joint * 180.0 / PI for joint in msg.joints)
        return self.move_joints_vel_meca_conv_sub_cb(msg)

    def move_lin_meca_conv_srv_cb(self, req):
        """
        Add a move lin command to the robot queue
        """
        with self._robot_lock:
            self.robot.MoveLin(
                [req.position.x, req.position.y, req.position.z],
                [req.orientation.x, req.orientation.y, req.orientation.z]
            )
        res = mecademic_msgs.srv.SetMecaPoseResponse()
        res.message = "MoveLin Sent"
        res.success = True
        return res

    def move_lin_srv_cb(self, req):
        """
        Add a move lin command to the robot queue (ros pose stamped)
        """
        meca_req = self.set_pose_req_2_set_meca_pose_req(
            req, self.wrf_frame_id)
        return self.set_meca_pose_res_2_set_pose_res(self.move_lin_meca_conv_srv_cb(meca_req))

    def move_lin_rel_trf_meca_conv_srv_cb(self, req):
        """
        Add a move lin rel trf command to the robot queue
        """
        with self._robot_lock:
            self.robot.MoveLinRelTRF(
                [req.position.x, req.position.y, req.position.z],
                [req.orientation.x, req.orientation.y, req.orientation.z]
            )
        res = mecademic_msgs.srv.SetMecaPoseResponse()
        res.message = "MoveLinRelTRF Sent"
        res.success = True
        return res

    def move_lin_rel_wrf_meca_conv_srv_cb(self, req):
        """
        Add a move lin rel wrf command to the robot queue
        """
        with self._robot_lock:
            self.robot.MoveLinRelWRF(
                [req.position.x, req.position.y, req.position.z],
                [req.orientation.x, req.orientation.y, req.orientation.z]
            )
        res = mecademic_msgs.srv.SetMecaPoseResponse()
        res.message = "MoveLinRelWRF Sent"
        res.success = True
        return res

    def move_lin_vel_trf_meca_conv_sub_cb(self, msg):
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

    def move_lin_vel_trf_sub_cb(self, msg):
        """
        Send a move lin vel trf command in [m/s] and [rad/s]
        """
        msg = self.twist_stamped_2_mecademic_twist(msg, self.trf_frame_id)
        self.move_lin_vel_trf_meca_conv_sub_cb(msg)

    def move_lin_vel_wrf_meca_conv_sub_cb(self, msg):
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

    def move_lin_vel_wrf_sub_cb(self, msg):
        """
        Send a move lin vel wrf command in [m/s] and [rad/s]
        """
        msg = self.twist_stamped_2_mecademic_twist(msg, self.wrf_frame_id)
        self.move_lin_vel_wrf_meca_conv_sub_cb(msg)

    def move_pose_meca_conv_srv_cb(self, req):
        """
        Add a move pose command to the robot queue
        """
        with self._robot_lock:
            self.robot.MovePose(
                [req.position.x, req.position.y, req.position.z],
                [req.orientation.x, req.orientation.y, req.orientation.z]
            )
        res = mecademic_msgs.srv.SetMecaPoseResponse()
        res.message = "MovePose Sent"
        res.success = True
        return res

    def move_pose_srv_cb(self, req):
        """
        Add a move lin command to the robot queue (ros pose stamped)
        """
        meca_req = self.set_pose_req_2_set_meca_pose_req(
            req, self.wrf_frame_id)
        return self.set_meca_pose_res_2_set_pose_res(self.move_pose_meca_conv_srv_cb(meca_req))

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
            rospy.loginfo("Sending SetBlending({})...".format(req.perc))
            self.robot.SetBlending(req.perc)
            rospy.loginfo("Sending SetBlending({}) Sent!".format(req.perc))
        res = mecademic_msgs.srv.SetPercResponse()
        res.message = "SetBlending({}) Sent".format(req.perc)
        res.success = True
        return res

    def set_cart_acc_srv_cb(self, req):
        """
        SetCartAcc cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending SetCartAcc({})...".format(req.perc))
            self.robot.SetCartAcc(req.perc)
            rospy.loginfo("Sending SetCartAcc({}) Sent!".format(req.perc))
        res = mecademic_msgs.srv.SetPercResponse()
        res.message = "SetCartAcc({}) Sent".format(req.perc)
        res.success = True
        return res

    def set_cart_ang_vel_meca_conv_srv_cb(self, req):
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

    def set_cart_lin_vel_meca_conv_srv_cb(self, req):
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
        # TODO
        with self._robot_lock:
            rospy.loginfo("Sending SetConf({},{},{})...".format(
                req.c1, req.c3, req.c5))
            self.robot.SetConf(c1=req.c1, c3=req.c3, c5=req.c5)
            rospy.loginfo("Sending SetConf({},{},{}) Sent!".format(
                req.c1, req.c3, req.c5))
        res = mecademic_msgs.srv.SetConfResponse()
        res.message = "SetConf({},{},{}) Sent".format(req.c1, req.c3, req.c5)
        res.success = True
        return res

    def set_joint_acc_srv_cb(self, req):
        """
        SetJointAcc cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending SetJointAcc({})...".format(req.perc))
            self.robot.SetJointAcc(req.perc)
            rospy.loginfo("Sending SetJointAcc({}) Sent!".format(req.perc))
        res = mecademic_msgs.srv.SetPercResponse()
        res.message = "SetJointAcc({}) Sent".format(req.perc)
        res.success = True
        return res

    def set_joint_vel_srv_cb(self, req):
        """
        SetJointVel cb
        """
        with self._robot_lock:
            rospy.loginfo("Sending SetJointVel({})...".format(req.perc))
            self.robot.SetJointVel(req.perc)
            rospy.loginfo("Sending SetJointVel({}) Sent!".format(req.perc))
        res = mecademic_msgs.srv.SetPercResponse()
        res.message = "SetJointVel({}) Sent".format(req.perc)
        res.success = True
        return res

    def set_trf_meca_conv_srv_cb(self, req):
        """
        Add a set trf command to the robot queue
        WARNING! THIS WILL IMMEDIATLY UPDATE THE TRF ON ROS
        """
        with self._robot_lock:
            rospy.loginfo("Sending SetTRF...")
            self.robot.SetTRF(
                [req.position.x, req.position.y, req.position.z],
                [req.orientation.x, req.orientation.y, req.orientation.z]
            )
            rospy.loginfo("SetTRF Sent!")
            self.update_tf_trf([req.position.x, req.position.y, req.position.z],
                               [req.orientation.x, req.orientation.y, req.orientation.z])
        res = mecademic_msgs.srv.SetMecaPoseResponse()
        res.message = "SetTRF Sent"
        res.success = True
        return res

    def set_trf_srv_cb(self, req):
        """
        Add a set trf command to the robot queue (input in ros pose)
        WARNING! THIS WILL IMMEDIATLY UPDATE THE TRF ON ROS
        """
        meca_req = self.set_pose_req_2_set_meca_pose_req(
            req, self.frf_frame_id)
        return self.set_meca_pose_res_2_set_pose_res(self.set_trf_meca_conv_srv_cb(meca_req))

    def set_wrf_meca_conv_srv_cb(self, req):
        """
        Add a set wrf command to the robot queue
        WARNING! THIS WILL IMMEDIATLY UPDATE THE WRF ON ROS
        """
        with self._robot_lock:
            rospy.loginfo("Sending SetWRF...")
            self.robot.SetWRF(
                [req.position.x, req.position.y, req.position.z],
                [req.orientation.x, req.orientation.y, req.orientation.z]
            )
            rospy.loginfo("SetWRF Sent!")
            self.update_tf_wrf([req.position.x, req.position.y, req.position.z],
                               [req.orientation.x, req.orientation.y, req.orientation.z])
        res = mecademic_msgs.srv.SetMecaPoseResponse()
        res.message = "SetWRF Sent"
        res.success = True
        return res

    def set_wrf_srv_cb(self, req):
        """
        Add a set wrf command to the robot queue (input in ros pose)
        WARNING! THIS WILL IMMEDIATLY UPDATE THE WRF ON ROS
        """
        meca_req = self.set_pose_req_2_set_meca_pose_req(
            req, self.brf_frame_id)
        return self.set_meca_pose_res_2_set_pose_res(self.set_wrf_meca_conv_srv_cb(meca_req))

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
    tf_prefix = rospy.get_param('~tf_prefix', "")

    try:
        mecademic_ros_driver = MecademicRobotROS_Driver(ip_address=ip_address)
        mecademic_ros_driver.robot_setup(
            activate=activate,
            home=home,
            high_performance=high_performance
        )
        mecademic_ros_driver.start_update_log_timer()
        mecademic_ros_driver.ros_setup(tf_prefix=tf_prefix)  
        
        rospy.spin()
    except Exception as e:
        rospy.logerr("MecademicDriver: {}".format(e))
    finally:
        mecademic_ros_driver.release_resources()
