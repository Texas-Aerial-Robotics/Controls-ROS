#include <ros/ros.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/State.h>

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

  mavros_msgs::CommandHome set_home_req;
  set_home_req.request.current_gps = false;
  set_home_req.request.latitude = 23;
  set_home_req.request.longitude = -98;
  set_home_req.request.altitude = 5;

  bool result = home_set.call(set_home_req);
  std::cout << result << std::endl;
}
