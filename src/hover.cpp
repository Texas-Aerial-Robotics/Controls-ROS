/**
 * @file takeoff.cpp
 * @August 2017 Competition Flight Code
 */

#include <fstream>

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h>
#include <cmath>
#include <iostream>
#define PI           3.14159265358979323846



mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

sensor_msgs::Range rngfnd;
void rng_cb(const sensor_msgs::Range::ConstPtr& msg){
    std::ofstream outRng("~/rng.log",std::ios::trunc);
    rngfnd = *msg;
    float rng = rngfnd.range;
    outRng << rng;
    outRng.close();
}
void nedSub(const geometry_msgs::PoseStamped::ConstPtr& msg)
{   
    geometry_msgs::PoseStamped nedPos;
    nedPos = *msg;
    std::cout << nedPos << std::endl;
}
mavros_msgs::AttitudeTarget set_attitude(int roll, int pitch, int yawRate, int thrust)
{

    //convert to quaternions
    int yaw = 0;
    double t0 = cos(yaw*(PI/180)); 
    double t1 = sin(yaw*(PI/180));
    double t2 = cos(roll*(PI/180));
    double t3 = sin(roll*(PI/180));
    double t4 = cos(pitch*(PI/180));
    double t5 = sin(pitch*(PI/180));
    
    double w = t0 * t2 * t4 + t1 * t3 * t5;
    double x = t0 * t3 * t4 - t1 * t2 * t5;
    double y = t0 * t2 * t5 + t1 * t3 * t4;
    double z = t1 * t2 * t4 - t0 * t3 * t5;
    
    mavros_msgs::AttitudeTarget att;
    att.orientation.x=x;
    att.orientation.y=y;
    att.orientation.z=z;
    att.orientation.w=w;
    att.body_rate.x=0;
    att.body_rate.y=0;
    att.body_rate.z=yawRate;
    att.thrust=thrust;

    
    
    return att;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
    ros::Subscriber rng_sub = nh.subscribe<sensor_msgs::Range>("mavros/rangefinder/rangefinder",1,rng_cb);
    ros::Subscriber ned_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",1, nedSub);

    ros::Publisher att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::AttitudeTarget att;
    att.orientation.x=0;
    att.orientation.y=0;
    att.orientation.z=0;
    att.orientation.w=0;
    att.body_rate.x=0;
    att.body_rate.y=0;
    att.body_rate.z=0;
    att.thrust=.55;
    

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
	att_pub.publish(att);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED_NOGPS";

    mavros_msgs::SetMode alt_set_mode;
    alt_set_mode.request.custom_mode = "LAND";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
//        std::cout << current_state.mode << std::endl;
 //        if( current_state.mode == "ACRO" && (ros::Time::now() - last_request > ros::Duration(5.0))){
 //            if( set_mode_client.call(offb_set_mode)){
 //                ROS_INFO("Offboard enabled");
 //            }
 //            last_request = ros::Time::now();
 //        } else {
 //            if( !current_state.armed && current_state.mode == "GUIDED_NOGPS" && (ros::Time::now() - last_request > ros::Duration(5.0))){
 //                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
 //                    ROS_INFO("Vehicle armed");
 //                }
 //                last_request = ros::Time::now();
 //            }
 //        }

	// att_pub.publish(att);
 
 //        if(rngfnd.range>1 && current_state.mode=="GUIDED_NOGPS"){
 //           ros::Time cmdStart = ros::Time::now();
 //           while(ros::Time::now() - cmdStart < ros::Duration(4.0))
 //            {
 //                att = set_attitude(0, 5, 0, .5);
 //                att_pub.publish(att);
 //                rate.sleep();
 //            }
           
 //        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
