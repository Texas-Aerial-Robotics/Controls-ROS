/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <fstream>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/Range.h>

std_msgs::Float64 hdg_msg;
void hdg_cb(const std_msgs::Float64::ConstPtr& msg){
    std::ofstream outHDG("~/hdg.log",std::ios::trunc);
    hdg_msg = *msg;
    float hdg = hdg_msg.data;
    outHDG << hdg;
    outHDG.close();
}

sensor_msgs::Range rngfnd;
void rng_cb(const sensor_msgs::Range::ConstPtr& msg){
    std::ofstream outRng("~/rng.log",std::ios::trunc);
    rngfnd = *msg;
    float rng = rngfnd.range;
    outRng << rng;
    outRng.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber hdg_sub = nh.subscribe<std_msgs::Float64>("mavros/global_position/compass_hdg", 1, hdg_cb);
    ros::Subscriber rng_sub = nh.subscribe<sensor_msgs::Range>("mavros/rangefinder/rangefinder",1,rng_cb);

    return 0;
}
