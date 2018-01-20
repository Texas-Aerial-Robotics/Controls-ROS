/**
 * @file mavros_offboard_velocity_node.cpp
 * @brief MAVROS Offboard Control example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL & jMAVSIM.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <mavros_msgs/CommandTOL.h>
#include <time.h>
#include <ros/duration.h>
#include <cmath>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

const float POSITION_TOLERANCE = 5;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  // ROS_INFO("I'm in state_cb!");
  current_state = *msg;
  bool connected = current_state.connected;
  bool armed = current_state.armed;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr pose){
  // ROS_INFO("I'm in pose_cb!");
  current_pose = *pose;
}

float get_distance(double x1, double y1, double z1, double x2, double y2, double z2){
  return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2) + pow(z1-z2, 2));
}

bool go_to_position(ros::NodeHandle* nh, float x, float y, float z){
  ros::Publisher set_pos_pub = nh->advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
  mavros_msgs::PositionTarget pos;
  pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

  pos.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | 
              mavros_msgs::PositionTarget::IGNORE_AFY | 
              mavros_msgs::PositionTarget::IGNORE_AFZ | 
              mavros_msgs::PositionTarget::IGNORE_YAW | 
              mavros_msgs::PositionTarget::IGNORE_YAW_RATE | 
              mavros_msgs::PositionTarget::IGNORE_VX | 
              mavros_msgs::PositionTarget::IGNORE_VY | 
              mavros_msgs::PositionTarget::IGNORE_VZ;
  pos.position.x = x;
  pos.position.y = y;
  pos.position.z = z;
  // pos.velocity.x = 10.0f;
  // pos.velocity.y = 0.0f;
  // pos.velocity.z = 0.0f;
  pos.yaw_rate = 0.0f;

  if (set_pos_pub)
  {
    ROS_INFO("zero velocity");
    // for (int i = 5000; ros::ok() && i > 0; --i)
    // {
    //   set_pos_pub.publish(pos);
    //   ros::spinOnce();
    //   // rate.sleep();
    //   ros::Duration(0.01).sleep();
    // }

    printf("Going to position: %f, %f, %f\n", pos.position.x,pos.position.y, pos.position.z);

    while(ros::ok() && get_distance(x,y,z,current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z) > POSITION_TOLERANCE){
      float distance = get_distance(x,y,z,current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

      // printf("Current position: %f,%f,%f. Distance: %f\n", 
      //               current_pose.pose.position.x, 
      //               current_pose.pose.position.y, 
      //               current_pose.pose.position.z, 
      //               distance);
      set_pos_pub.publish(pos);
      ros::spinOnce();
      // rate.sleep();
      ros::Duration(0.01).sleep();
    }

    ROS_INFO("Done with zero velocity set");
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
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
    ROS_ERROR("Failed arming/disarming");
    //return -1;
  }

   ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  //ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "GUIDED";
  if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    ROS_INFO("OFFBOARD enabled");
  else
  {
    ROS_INFO("unable to switch to offboard");
    return -1;
  }

  sleep(5)

  ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = 2;
  // srv_takeoff.request.latitude = -35.3632607;
  // srv_takeoff.request.longitude = 149.1652351;
  srv_takeoff.request.min_pitch = 0;
  srv_takeoff.request.yaw = 0;
  if (takeoff_client.call(srv_takeoff) && srv_takeoff.response.success)
    ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
  else
  {
    ROS_ERROR("Failed Takeoff");
    return -1;
  }

  sleep(10);



  ROS_INFO("Going to position 1");

  go_to_position(&nh, 5,5,0);

  ros::Duration(5.0).sleep();

  ROS_INFO("Going to position 2");

  go_to_position(&nh, -5,0,0);
  ros::Duration(5.0).sleep();

  ROS_INFO("Going to position 3");

  go_to_position(&nh, 0,-5, 0);
  ros::Duration(5.0).sleep();

  ROS_INFO("Going to position 4");

  go_to_position(&nh, 5, 0, 0);
  ros::Duration(5.0).sleep();

  ROS_INFO("Going to position 5");

  go_to_position(&nh, 0, 5, 0);

//right 
  // pos.position.x = 0.0f;
  // pos.position.y = 5.0f;
  // pos.position.z = 0.0f;
  // pos.velocity.x = 0.0f;
  // pos.velocity.y = 0.0f;
  // pos.velocity.z = 0.0f;

  // if (set_vel_pub)
  // {
  //   for (int i = 500; ros::ok() && i > 0; --i)
  //   {
  //     set_vel_pub.publish(pos);
  //     ros::spinOnce();
  //     // rate.sleep();
  //     ros::Duration(0.01).sleep();
  //   }
  //   ROS_INFO("Done Right");
  // }

  // //foreward

  // pos.position.x =  5.0f;
  // pos.position.y = 0.0f;
  // pos.position.z = 0.0f;
  // pos.velocity.x = 0.0f;
  // pos.velocity.y = 0.0f;
  // pos.velocity.z = 0.0f;

  // if (set_vel_pub)
  // {
  //   for (int i = 500; ros::ok() && i > 0; --i)
  //   {
  //     set_vel_pub.publish(pos);
  //     ros::spinOnce();
  //     // rate.sleep();
  //     ros::Duration(0.01).sleep();
  //   }
  //   ROS_INFO("Done Foreward");
  // }


  // //left 

  // pos.position.x = 0.0f;
  // pos.position.y = -0.5f;
  // pos.position.z = 0.0f;
  // pos.velocity.x = 0.0f;
  // pos.velocity.y = 0.0f;
  // pos.velocity.z = 0.0f;

  // if (set_vel_pub)
  // {
  //   for (int i = 500; ros::ok() && i > 0; --i)
  //   {
  //     set_vel_pub.publish(pos);
  //     ros::spinOnce();
  //     // rate.sleep();
  //     ros::Duration(0.01).sleep();
  //   }
  //   ROS_INFO("Done Left");
  // }

  // //back

  // pos.position.x = -0.5f;
  // pos.position.y = 0.0f;
  // pos.position.z = 0.0f;
  // pos.velocity.x = 0.0f;
  // pos.velocity.y = 0.0f;
  // pos.velocity.z = 0.0f;

  // if (set_vel_pub)
  // {
  //   for (int i = 500; ros::ok() && i > 0; --i)
  //   {
  //     set_vel_pub.publish(pos);
  //     ros::spinOnce();
  //     // rate.sleep();
  //     ros::Duration(0.01).sleep();
  //   }
  //   ROS_INFO("Done Back");
  // }

  // ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  // mavros_msgs::CommandTOL srv_land{};
  // if (land_client.call(srv_land) && srv_land.response.success)
  //   ROS_INFO("land sent %d", srv_land.response.success);
  // else
  // {
  //   ROS_ERROR("Landing failed");
  //   ros::shutdown();
  //   return -1;
  // }

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}