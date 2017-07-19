/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/Range.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

sensor_msgs::Range rngfnd;
void rng_cb(const sensor_msgs::Range::ConstPtr& msg){
    rngfnd = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1, state_cb);
    ros::Subscriber rng_sub = nh.subscrube<sensor_msgs::Range>("mavros/rangefinder/rangefinder",1,rng_cb);

    ros::Publisher att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

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
    att.thrust=.63;
    

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
        std::cout << current_state.mode << std::endl;
        if( current_state.mode == "ALT_HOLD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode)){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && current_state.mode == "GUIDED_NOGPS" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

	att_pub.publish(att);
 
        if(rngfnd.range>1.5){
            if(set_mode_client.call(alt_set_mode) && alt_set_mode.response.success){
                ROS_INFO("land");
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
