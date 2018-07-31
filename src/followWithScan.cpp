#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <geometry_msgs/Pose2D.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/LaserScan.h>
#include <time.h>
#include <cmath>
#include <math.h>
#include <ros/duration.h>

using namespace std;

//Set global variables
mavros_msgs::State current_state;
nav_msgs::Odometry current_pose;
geometry_msgs::PoseStamped waypoint;
sensor_msgs::LaserScan current_2D_scan;

std_msgs::String MODE;
float GYM_OFFSET;
float x_min;
float x_max;
float y_min;
float y_max;
float z_min;
float z_max;
float current_heading;
double RUN_START_TIME = 0;
bool currentlyAvoiding = false;
bool lidarPresent = true;

//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}
//get current position of drone
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose = *msg;
  float q0 = current_pose.pose.pose.orientation.w;
  float q1 = current_pose.pose.pose.orientation.x;
  float q2 = current_pose.pose.pose.orientation.y;
  float q3 = current_pose.pose.pose.orientation.z;
  float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  current_heading = -psi*(180/M_PI) + 90;
  ROS_INFO("Current Heading %f ", current_heading);
  // ROS_INFO("x: %f y: %f z: %f", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
}
void mode_cb(const std_msgs::String::ConstPtr& msg)
{
  MODE = *msg;
  //ROS_INFO("current mode: %s", MODE.data.c_str());
}
void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  current_2D_scan = *msg;
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

  waypoint.pose.orientation.w = qw;
  waypoint.pose.orientation.x = qx;
  waypoint.pose.orientation.y = qy;
  waypoint.pose.orientation.z = qz;
}
void setHeading_cb(const std_msgs::Float64::ConstPtr& msg)
{
  std_msgs::Float64 set_heading = *msg;
  setHeading(set_heading.data);
  // ROS_INFO("current heading: %f", current_heading.data);
}
// set position to fly to in the gym frame
void setDestination(float x, float y, float z)
{
  float deg2rad = (M_PI/180);
  float X = x*cos(-GYM_OFFSET*deg2rad) - y*sin(-GYM_OFFSET*deg2rad);
  float Y = x*sin(-GYM_OFFSET*deg2rad) + y*cos(-GYM_OFFSET*deg2rad);
  float Z = z;
  waypoint.pose.position.x = X;
  waypoint.pose.position.y = Y;
  waypoint.pose.position.z = Z;
  // ROS_INFO("Destination set to x: %f y: %f z: %f", X, Y, Z);
  cout << "Destination set to x: " << X << " y: " << Y << " z: " << Z << endl;
}

//initialize ROS
void init_ros()
{
  ROS_INFO("INITIALIZING ROS");
  for(int i=0; i<100; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
}

// update waypoint from strat but check inputs
void waypoint_update(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  geometry_msgs::PoseStamped waypoint_eval;
  waypoint_eval = *msg;
  float way_x = waypoint_eval.pose.position.x;
  float way_y = waypoint_eval.pose.position.y;
  float way_z = waypoint_eval.pose.position.z;

  if(!currentlyAvoiding)
  {
    if(way_z < z_max && way_z > z_min && way_x < x_max && way_x > x_min && way_y < y_max && way_y > y_min)
    {
      waypoint.pose.position.x = way_x;
      waypoint.pose.position.y = way_y;
      waypoint.pose.position.z = way_z;
      setDestination(way_x,way_y,way_z);
    }
    else
    {
      ROS_INFO("The waypoint passed is out of bounds at x,y,z = %f,%f,%f",way_x,way_y,way_z);
    }
  }
  else
  {
    ROS_INFO("Waypoint ignored. Obs avoidance in progress...");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle controlnode;

  ros::param::get("/contols_params/safety_limits/xlim/min", x_min);
  ros::param::get("/contols_params/safety_limits/xlim/max", x_max);
  ros::param::get("/contols_params/safety_limits/ylim/min", y_min);
  ros::param::get("/contols_params/safety_limits/ylim/max", y_max);
  ros::param::get("/contols_params/safety_limits/xlim/min", z_min);
  ros::param::get("/contols_params/safety_limits/xlim/max", z_max);

  ros::Rate rate(5.0);
  ros::Publisher gym_offset_pub = controlnode.advertise<std_msgs::Float64>("gymOffset", 1);
  ros::Publisher local_pos_pub = controlnode.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Publisher run_start_pub = controlnode.advertise<std_msgs::Float64>("runStartTime", 1);
  ros::Subscriber collision_sub = controlnode.subscribe<sensor_msgs::LaserScan>("/scan", 1, scan_cb);
  ros::Subscriber currentPos = controlnode.subscribe<nav_msgs::Odometry>("mavros/global_position/local", 10, pose_cb);
  ros::Subscriber heading_pub = controlnode.subscribe<std_msgs::Float64>("setHeading", 1, setHeading_cb);
  ros::Subscriber mode_sub = controlnode.subscribe<std_msgs::String>("mode", 10, mode_cb);
  ros::Subscriber state_sub = controlnode.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber waypointSubscrib = controlnode.subscribe<geometry_msgs::PoseStamped>("TARwaypoint", 10, waypoint_update);
  ros::ServiceClient arming_client = controlnode.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient land_client = controlnode.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  ros::ServiceClient set_mode_client = controlnode.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  ros::ServiceClient takeoff_client = controlnode.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

  // wait for FCU connection
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Connected to FCU");
    // wait for mode to be set to guided
  while (current_state.mode != "GUIDED")
  {
    ros::spinOnce();
    rate.sleep();
  }
  RUN_START_TIME = ros::Time::now().toSec();

  // Publish mission start time
  std_msgs::Float64 runStartTime;
  runStartTime.data = RUN_START_TIME;
  run_start_pub.publish(runStartTime);
  ROS_INFO("Run start time %f", RUN_START_TIME);
  ROS_INFO("Mode set to GUIDED");

  //set the orientation of the gym
  GYM_OFFSET = 0;
  for (int i = 1; i <= 30; ++i) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();

    float q0 = current_pose.pose.pose.orientation.w;
    float q1 = current_pose.pose.pose.orientation.x;
    float q2 = current_pose.pose.pose.orientation.y;
    float q3 = current_pose.pose.pose.orientation.z;
    float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) ); // yaw

    GYM_OFFSET += psi*(180/M_PI);
    // ROS_INFO("current heading%d: %f", i, GYM_OFFSET/i);
  }
  GYM_OFFSET /= -30;
  GYM_OFFSET += 90;
  ROS_INFO("the N' axis is facing: %f", GYM_OFFSET);
  cout << GYM_OFFSET << "\n" << endl;
  std_msgs::Float64 gymOffset;
  gymOffset.data = GYM_OFFSET;
  gym_offset_pub.publish(gymOffset);

  // arming
  mavros_msgs::CommandBool arm_request;
  arm_request.request.value = true;
  while (!current_state.armed && !arm_request.response.success)
  {
    ros::Duration(.1).sleep();
    arming_client.call(arm_request);
  }
  ROS_INFO("ARM sent %d", arm_request.response.success);


  //request takeoff
  mavros_msgs::CommandTOL takeoff_request;
  takeoff_request.request.altitude = 1.5;

  while (!takeoff_request.response.success)
  {
    ros::Duration(.1).sleep();
    takeoff_client.call(takeoff_request);
  }
  ROS_INFO("Takeoff initialized");
  sleep(10);

  setDestination(0,0,1.5);
  setHeading(0);

  // 2D spinning LIDAR params
  float tollorance = .35;
  float scanMinRange = .45;
  float scanMaxRange = 1.75;

  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();

    // 2D LIDAR obstacle avoidance
    int scanRayIndex = 2;
    int scanSize = current_2D_scan.ranges.size();
    while(lidarPresent && ((scanRayIndex+2) < scanSize)) {
      scanRayIndex++;
      ros::spinOnce();

      // check if 5 consecutive readings are in bounds we care about
      if((((current_2D_scan.ranges[scanRayIndex-2] < scanMaxRange) && (current_2D_scan.ranges[scanRayIndex-2] > scanMinRange)) &&
          ((current_2D_scan.ranges[scanRayIndex-1] < scanMaxRange) && (current_2D_scan.ranges[scanRayIndex-1] > scanMinRange)) &&
          ((current_2D_scan.ranges[scanRayIndex+0] < scanMaxRange) && (current_2D_scan.ranges[scanRayIndex+0] > scanMinRange)) &&
          ((current_2D_scan.ranges[scanRayIndex+1] < scanMaxRange) && (current_2D_scan.ranges[scanRayIndex+1] > scanMinRange)) &&
          ((current_2D_scan.ranges[scanRayIndex+2] < scanMaxRange) && (current_2D_scan.ranges[scanRayIndex+2] > scanMinRange))))
      {
        break; // if so, stop scanning and move on
      }
      // Prints if we got a few detections but not a full 5
      if(current_2D_scan.ranges[scanRayIndex] < scanMaxRange && current_2D_scan.ranges[scanRayIndex] > scanMinRange)
      {
        ROS_INFO("Index[%d]: %f", scanRayIndex, current_2D_scan.ranges[scanRayIndex]);
      }
      if(!lidarPresent || (scanRayIndex+3) == scanSize) // scanRayIndex+2 because checking multiple rays and +1 because less than in while loop
      {
        currentlyAvoiding = false;
      }
    }
    if(lidarPresent && (scanRayIndex < scanSize && scanRayIndex > 0 && current_2D_scan.ranges[scanRayIndex] < scanMaxRange && current_2D_scan.ranges[scanRayIndex] > scanMinRange))
    {
      double angle_of_obstacle_RAD = current_2D_scan.angle_increment*scanRayIndex;
      ROS_INFO("Obstacle sighted @%f (current_2D_scan.angle_increment: %f * scanRayIndex: %d)", angle_of_obstacle_RAD, current_2D_scan.angle_increment, scanRayIndex);
      ROS_INFO("-------------------SKRTT SKRTT-------------------");

      //get current position in gym frame
      float deg2rad = (M_PI/180);
      float X = current_pose.pose.pose.position.x*cos(GYM_OFFSET*deg2rad) - current_pose.pose.pose.position.y*sin(GYM_OFFSET*deg2rad);
      float Y = current_pose.pose.pose.position.x*sin(GYM_OFFSET*deg2rad) + current_pose.pose.pose.position.y*cos(GYM_OFFSET*deg2rad);
      float Z = 1.5;
      ROS_INFO("x: %f y: %f z: %f", X, Y, Z);

      double radius = current_2D_scan.ranges[scanRayIndex];
      double why = radius * sin(angle_of_obstacle_RAD);
      double ex = radius * cos(angle_of_obstacle_RAD);

      ROS_INFO("radius: %f, ex: %f, why: %f", radius, ex, why);

      //avoidance vector relative to the drone
      float avoidXdrone = (ex/radius)*(-1.328);
      float avoidYdrone = (why/radius)*(-1.328);

      //Transform to gym refernce frame
      float avoidXgym = avoidXdrone*cos(-(current_heading-GYM_OFFSET)*deg2rad) - avoidYdrone*sin(-(current_heading-GYM_OFFSET)*deg2rad);
      float avoidYgym = avoidXdrone*sin(-(current_heading-GYM_OFFSET)*deg2rad) + avoidYdrone*cos(-(current_heading-GYM_OFFSET)*deg2rad);

      ROS_INFO("goto x: %f, y: %f", avoidXgym, avoidYgym);
      setDestination((X + avoidXgym), (Y + avoidYgym), Z);
      currentlyAvoiding = true;

      rate.sleep();
      rate.sleep();
      rate.sleep();
    }


    // LAND if StratNode says to or ten min run is done (600 sec == ten min)
    if((MODE.data == "LAND" || ((ros::Time::now().toSec()-RUN_START_TIME) > 590)) && !currentlyAvoiding)
    {
      mavros_msgs::CommandTOL srv_land;
      if(land_client.call(srv_land) && srv_land.response.success)
        ROS_INFO("land sent %d", srv_land.response.success);
      else
      {
        ROS_ERROR("Landing failed");
      }
    }
    else if(MODE.data == "TAKEOFF")
    {
      while (current_state.armed && current_state.mode == "LAND")
      {
        rate.sleep();
        ros::spinOnce();
      }
      ros::Duration(5).sleep();

      while (current_state.mode == "LAND")
      {
        //set flight mode to guided
        mavros_msgs::SetMode srv_setMode;
        srv_setMode.request.base_mode = 0;
        srv_setMode.request.custom_mode = "GUIDED";
        if(set_mode_client.call(srv_setMode)){
          ROS_INFO("setmode send ok");
        }else{
          ROS_ERROR("Failed SetMode");
          return -1;
        }
        ros::Duration(.5).sleep();
        ros::spinOnce();
      }
      // arming
      mavros_msgs::CommandBool arm_request;
      arm_request.request.value = true;
      while (!current_state.armed && current_state.mode == "GUIDED")
      {
        arming_client.call(arm_request);
        ros::Duration(.1).sleep();
        ros::spinOnce();
      }
      ROS_INFO("ARM sent %d", arm_request.response.success);
      mavros_msgs::CommandTOL takeoff_request;
      takeoff_request.request.altitude = 1.5;

      while (!takeoff_request.response.success && current_state.mode == "GUIDED")
      {
        takeoff_client.call(takeoff_request);
        ros::Duration(.1).sleep();
        ros::spinOnce();
      }
      ROS_INFO("Takeoff sent");
      ros::Duration(5).sleep();
    }

    local_pos_pub.publish(waypoint);
    gym_offset_pub.publish(gymOffset);
  }
  return 0;
}
