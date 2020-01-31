#include "ros/ros.h"
#include "signal.h"
#include "mecademic_msgs/Joints.h"
#include "sensor_msgs/JointState.h"
#include "std_srvs/SetBool.h"

#define NUM_JOINTS 6
#define ZERO_VEL_COMMANDS_TO_SEND 10
#define DEFAULT_GAINS 3.0

/*
Patch to avoid c++11 depdendency
*/
namespace patch{
template <typename T>
std::string to_string(T const& value) {
    std::stringstream sstr;
    sstr << value;
    return sstr.str();
}
}

//Enabled
bool enabled;

//Params
double gains[NUM_JOINTS];

//joints command
double joints_command[NUM_JOINTS];
//joints measure
double joints_measure[NUM_JOINTS];

//User fncs
void mySigintHandler(int sig);
void pre_stop();
void pre_start();
void control_publish();

//Defined vars
mecademic_msgs::Joints vel_command_msg;
ros::Publisher pub_joints_velocity_command;
bool measure_arrived;

/*
Joints position command callbk
receive the joint position command from a topic and update the corresponding global var
call controller
*/
void joints_position_command_cb( const mecademic_msgs::Joints::ConstPtr& msg )
{
    if(enabled) //Parse joint commands only if enabled
    {
        for( int i=0; i<NUM_JOINTS; i++ )
            joints_command[i] = msg->joints[i];

        control_publish();
    }
}

/*
Joints position measure callbk
receive the joint position measure from a topic and update the corresponding global var
call controller
*/
void joints_position_measure_cb( const sensor_msgs::JointState::ConstPtr& msg )
{
    //TODO Parse joints name
    for( int i=0; i<NUM_JOINTS; i++ )
        joints_measure[i] = msg->position[i];

    measure_arrived = true;

    if(enabled)
        control_publish();
}

/*
Service Callbk - Set Enable
Set enable state of the Node
*/
bool srv_server_set_enabled_cb(
    std_srvs::SetBool::Request& request, 
    std_srvs::SetBool::Response& response
    )
{
    //On enable change, reset the node
    if (request.data != enabled)
    {
        if(request.data) //if enabled requested
        {
            pre_start();
        } else { //if disable requested
            pre_stop();
        }
        ROS_INFO("mecademic/joints_controller %s", (request.data ? "ENABLED" : "DISABLED"));
    }
    enabled = request.data;
    return true;
}

int main(int argc, char *argv[])
{
    //ROS INIT - Use a coustom signinthandler
    ros::init(argc,argv, "joints_controller",ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigintHandler);

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    //Disable the node
    enabled = false;

    //Init vars
    vel_command_msg.joints.resize(NUM_JOINTS);

    //Params
    //gains
    if(nh_private.hasParam("gains"))
    {
        std::vector<double> gains_std;
        if(!nh_private.getParam("gains", gains_std))
        {
            //Param is not std::vector<double>
            double gain_;
            if(!nh_private.getParam("gains", gain_))
            {
                ROS_ERROR("[MECA JOINTS CONTROLLER] INVALID PARAM TYPE: GAINS");
                ros::shutdown();
                exit(-1);
            }
            gains_std.push_back(gain_);
        }
        if(gains_std.size() == 1)
        {
            for(int i=1;i<NUM_JOINTS;i++)
                gains_std.push_back(gains_std[0]);
        } 
        else if(gains_std.size() != NUM_JOINTS)
        {
            ROS_ERROR("[MECA JOINTS CONTROLLER] INVALID PARAM: GAINS");
            ros::shutdown();
            exit(-1);
        }
        for(int i=0;i<NUM_JOINTS;i++)
            gains[i]=gains_std[i];
    }
    else
    {
        for(int i=0;i<NUM_JOINTS;i++)
            gains[i]=DEFAULT_GAINS;
    }

    //Print the params
    {
        std::string output = "[MECA JOINTS CONTROLLER] PARAMS\n";
        output += "\tGains: ";
        for(int i=0;i<NUM_JOINTS;i++)
            output += patch::to_string(gains[i]) + " ";
        ROS_INFO_STREAM(output);
    }

    //Subscribers
    ros::Subscriber sub_joint_position_command = 
        nh_public.subscribe("command/joint_position",1,joints_position_command_cb);
    ros::Subscriber sub_joint_position_measure = 
        nh_public.subscribe("state/joint_position",1,joints_position_measure_cb);

    //Publishers
    pub_joints_velocity_command = 
        nh_public.advertise<mecademic_msgs::Joints>("command/joints_vel",1);

    //Service Servers
    ros::ServiceServer srv_server_activate = 
        nh_public.advertiseService("joints_controller/set_enable",srv_server_set_enabled_cb);

    //SPIN
    ros::spin();

    return 0;
}

/*
Send ZERO VEL commands
*/
void pre_stop()
{
    //prepare zero vel
    for(int i=0; i<NUM_JOINTS; i++)
        vel_command_msg.joints[i] = 0.0;

    //send zero vel
    for(int i=0; i<ZERO_VEL_COMMANDS_TO_SEND; i++)
    {
        vel_command_msg.header.stamp = ros::Time::now();
        pub_joints_velocity_command.publish(vel_command_msg);
    }
}

/*
Wait for a measure
Set a ZERO error
*/
void pre_start()
{
    //Wait a measure
    measure_arrived = false;
    while(ros::ok() && !measure_arrived)
        ros::spinOnce();

    //Set an initial Zero Error
    for(int i=0; i<NUM_JOINTS; i++)
        joints_command[i] = joints_measure[i];

}

/*
Compute the control and publish it
*/
void control_publish()
{
    for(int i=0; i<NUM_JOINTS; i++)
        vel_command_msg.joints[i] = gains[i]*( joints_command[i] - joints_measure[i] );
    vel_command_msg.header.stamp = ros::Time::now();

    pub_joints_velocity_command.publish(vel_command_msg);
}

/*
    Handler for CTRL+C
*/
void mySigintHandler(int sig){ 
    if(enabled)
        pre_stop();
    enabled = false;
    ros::shutdown();
}