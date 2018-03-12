#include <ros/ros.h>
#include <mavconn/interface.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/mavlink_convert.h>
#include <mavlink/v2.0/mavlink_helpers.h>

mavros_msgs::State current_state;


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_home_node");
  ros::NodeHandle home_handle;

  ros::Subscriber state_sub = home_handle.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
  ros::ServiceClient home_set = home_handle.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home",1);

  ros::Rate rate(20.0);

  while(ros::ok() && current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  ros::Duration(1.0).sleep();


  double latitude = 30.2672;
  double longitude = -97.7431;
  double altitude = 165.0;

  mavros_msgs::CommandHome set_home_req;
  set_home_req.request.current_gps = false;
  set_home_req.request.latitude = latitude;
  set_home_req.request.longitude = longitude;
  set_home_req.request.altitude = altitude;

  ros::service::call("/mavros/cmd/set_home", set_home_req);

  printf("Result was %d\n", set_home_req.response.result); 

}