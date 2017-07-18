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

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Publisher att_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_attitude/cmd_vel", 10);
    ros::Publisher att_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude", 10);
    ros::Publisher att_thr_pub = nh.advertise<std_msgs::Float64>("mavros/setpoint_attitude/att_throttle", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose1;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = 0;
    twist.twist.linear.y = 0;
    twist.twist.linear.z = 0;
    twist.twist.angular.x = 0;
    twist.twist.angular.y = 0;
    twist.twist.angular.z = 0;

    std_msgs::Float64 thr;
    thr.data = .63;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
	att_vel_pub.publish(twist);
        att_pos_pub.publish(pose1);
        att_thr_pub.publish(thr);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED_NOGPS";

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

	att_vel_pub.publish(twist);
        att_pos_pub.publish(pose1);
	att_thr_pub.publish(thr);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
