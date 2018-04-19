#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <geometry_msgs/Pose2D.h>
#include <mavros_msgs/CommandTOL.h>
#include <time.h>
#include <cmath>
#include <math.h>
#include <ros/duration.h>

using namespace std;

//Set global variables
mavros_msgs::State current_state;
//nav_msgs::Odometry current_pose;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped pose;
std_msgs::Float64 current_heading;
float GYM_OFFSET;

//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
  bool connected = current_state.connected;
  bool armed = current_state.armed;
}
//get current position of drone
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose = *msg;
  ROS_INFO("x: %f y: %f z: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  // ROS_INFO("y: %f", current_pose.pose.position.y);
  // ROS_INFO("z: %f", current_pose.pose.position.z);
}
// void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
// {
//  current_pose = *msg;
//  ROS_INFO("x: %f y: %f z: %f", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
// }
//get compass heading
void heading_cb(const std_msgs::Float64::ConstPtr& msg)
{
  current_heading = *msg;
  //ROS_INFO("current heading: %f", current_heading.data);
}
//set orientation of the drone (drone should always be level)
void setHeading(float heading)
{
  heading = -heading + 90 - GYM_OFFSET;
  float yaw = heading*(M_PI/180);
  float pitch = 0;
  float roll = 0;

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  float qw = cy * cr * cp + sy * sr * sp;
  float qx = cy * sr * cp - sy * cr * sp;
  float qy = cy * cr * sp + sy * sr * cp;
  float qz = sy * cr * cp - cy * sr * sp;

  pose.pose.orientation.w = qw;
  pose.pose.orientation.x = qx;
  pose.pose.orientation.y = qy;
  pose.pose.orientation.z = qz;

}
// set position to fly to in the gym frame
void setDestination(float x, float y, float z)
{
  float deg2rad = (M_PI/180);
  float X = x*cos(-GYM_OFFSET*deg2rad) - y*sin(-GYM_OFFSET*deg2rad);
  float Y = x*sin(-GYM_OFFSET*deg2rad) + y*cos(-GYM_OFFSET*deg2rad);
  float Z = z;
  pose.pose.position.x = X;
  pose.pose.position.y = Y;
  pose.pose.position.z = Z;
  ROS_INFO("Destination set to x: %f y: %f z %f", X, Y, Z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Publisher set_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/global_position/pose", 10, pose_cb);
  ros::Subscriber currentHeading = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10, heading_cb);

  // allow the subscribers to initialize
  ROS_INFO("INITILIZING...");
  for(int i=0; i<100; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  while(current_state.mode != "GUIDED")
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  //set the orientation of the gym
  GYM_OFFSET = 0;
  for (int i = 1; i <= 30; ++i) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    GYM_OFFSET += current_heading.data;
    ROS_INFO("current heading%d: %f", i, GYM_OFFSET/i);
  }
  GYM_OFFSET /= 30;
  ROS_INFO("the N' axis is facing: %f", GYM_OFFSET);
  cout << GYM_OFFSET << "\n" << endl;

  // wait for FCU connection
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // arming
  ros::ServiceClient arming_client_i = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  mavros_msgs::CommandBool srv_arm_i;
  srv_arm_i.request.value = true;
  if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success)
    ROS_INFO("ARM sent %d", srv_arm_i.response.success);
  else
  {
    ROS_ERROR("Failed arming");
    return -1;
  }


  //request takeoff
  ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = 1.5;
  if(takeoff_cl.call(srv_takeoff)){
    ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
  }else{
    ROS_ERROR("Failed Takeoff");
    return -1;
  }

  sleep(10);


  //move foreward
  setHeading(0);
  setDestination(0, 2, 1.5);
  float tollorance = .35;
  if (local_pos_pub)
  {

    for (int i = 10000; ros::ok() && i > 0; --i)
    {

      local_pos_pub.publish(pose);
      // float percentErrorX = abs((pose.pose.position.x - current_pose.pose.position.x)/(pose.pose.position.x));
      // float percentErrorY = abs((pose.pose.position.y - current_pose.pose.position.y)/(pose.pose.position.y));
      // float percentErrorZ = abs((pose.pose.position.z - current_pose.pose.position.z)/(pose.pose.position.z));
      // cout << " px " << percentErrorX << " py " << percentErrorY << " pz " << percentErrorZ << endl;
      // if(percentErrorX < tollorance && percentErrorY < tollorance && percentErrorZ < tollorance)
      // {
      //   break;
      // }
      float deltaX = abs(pose.pose.position.x - current_pose.pose.position.x);
      float deltaY = abs(pose.pose.position.y - current_pose.pose.position.y);
      float deltaZ = abs(pose.pose.position.z - current_pose.pose.position.z);
      //cout << " dx " << deltaX << " dy " << deltaY << " dz " << deltaZ << endl;
      float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
      cout << dMag << endl;
      if( dMag < tollorance)
      {
        break;
      }
      ros::spinOnce();
      ros::Duration(0.5).sleep();
      if(i == 1)
      {
        ROS_INFO("Failed to reach destination. Stepping to next task.");
      }
    }
    ROS_INFO("Done moving foreward.");
  }

  //land
  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  mavros_msgs::CommandTOL srv_land;
  if (land_client.call(srv_land) && srv_land.response.success)
    ROS_INFO("land sent %d", srv_land.response.success);
  else
  {
    ROS_ERROR("Landing failed");
    ros::shutdown();
    return -1;
  }

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
