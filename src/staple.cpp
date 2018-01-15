#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <geometry_msgs/Pose2D.h>
#include <mavros_msgs/CommandTOL.h>
#include <time.h>
#include <ros/duration.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
  bool connected = current_state.connected;
  bool armed = current_state.armed;
}
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
	current_pose = *msg;
	ROS_INFO("x: %f", current_pose.pose.position.x);
	ROS_INFO("y: %f", current_pose.pose.position.y);
	ROS_INFO("z: %f", current_pose.pose.position.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  // wait for FCU connection
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  mavros_msgs::SetMode srv_setMode;
  srv_setMode.request.base_mode = 0;
  srv_setMode.request.custom_mode = "GUIDED";
  if(set_mode_client.call(srv_setMode)){
    ROS_INFO("setmode send ok");
  }else{
    ROS_ERROR("Failed SetMode");
    return -1;
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

  ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = 5;
  if(takeoff_cl.call(srv_takeoff)){
    ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
  }else{
    ROS_ERROR("Failed Takeoff");
    return -1;
  }

  sleep(5);

  ros::Publisher set_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
  mavros_msgs::PositionTarget pos;
  pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0;
	pose.pose.position.y = 8;
	pose.pose.position.z = 3;

  pos.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  
 
  // pos.position.x = 0.0f;
  // pos.position.y = 0.0f;
  // pos.position.z = 0.0f;
  // pos.acceleration_or_force.x = 0.0f;
  // pos.acceleration_or_force.y = 0.0f;
  // pos.acceleration_or_force.z = 0.0f;

  // pos.velocity.x = 0.0f;
  // pos.velocity.y = 0.0f;
  // pos.velocity.z = 0.0;
  // pos.yaw_rate = 0.0f;
  // if (set_vel_pub)
  // {
  //   ROS_INFO("zero velocity");
  //   for (int i = 1000; ros::ok() && i > 0; --i)
  //   {
  //     set_vel_pub.publish(pos);
  //     ros::spinOnce();
  //     // rate.sleep();
  //     ros::Duration(0.01).sleep();
  //   }
  //   ROS_INFO("Done with zero velocity set");
  // }

  //ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "GUIDED";
  float tollorance = .05;
  if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    ROS_INFO("OFFBOARD enabled");
  else
  {
    ROS_INFO("unable to switch to offboard");
    return -1;
  }

  // pos.position.x = -0.5f;
  // pos.position.y = 0.0f;
  // pos.position.z = 0.0f;

  if (set_vel_pub)
  {

    for (int i = 10000; ros::ok() && i > 0; --i)
    // while (current_pose.x != -0.5f)
    {

      local_pos_pub.publish(pose);
      float percentErrorX = (pose.pose.position.x - current_pose.pose.position.x)/(pose.pose.position.x);
      float percentErrorY = (pose.pose.position.y - current_pose.pose.position.y)/(pose.pose.position.y);
      float percentErrorZ = (pose.pose.position.z - current_pose.pose.position.z)/(pose.pose.position.z);
      if(percentErrorX < tollorance && percentErrorY < tollorance && percentErrorZ < tollorance)
      {
      	break;
      }
      ros::spinOnce();
      // rate.sleep();
      ros::Duration(0.01).sleep();
    }
    ROS_INFO("Done side");
  }

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