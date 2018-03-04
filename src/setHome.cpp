#include <ros/ros.h>
#include <mavconn/interface.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/mavlink_convert.h>
#include <mavlink/v2.0/mavlink_helpers.h>

mavros_msgs::State current_state;

uint8_t system_id = 255;
uint8_t component_id = 1;
uint8_t target_system = 1;

bool packMavlinkMessage(const mavlink::Message& mavMsg, mavros_msgs::Mavlink &rosMsg)
{
  mavlink::mavlink_message_t msg;
  mavlink::MsgMap map(msg);
  mavMsg.serialize(map);
  auto mi = mavMsg.get_message_info();

  mavlink::mavlink_status_t *status = mavlink::mavlink_get_channel_status(mavlink::MAVLINK_COMM_0);
  status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  mavlink::mavlink_finalize_message_buffer(&msg, system_id, component_id, status, mi.min_length, mi.length, mi.crc_extra);

  return mavros_msgs::mavlink::convert(msg, rosMsg);
}

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

  ros::Publisher origin_pub = home_handle.advertise<mavros_msgs::Mavlink>("mavlink/to", 1000);

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
  // double latitude = 0.0;
  // double longitude = 0.0;
  // double altitude = 0.0;


  mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN originMsg;
  originMsg.latitude = (uint32_t)(latitude * 10000000);
  originMsg.longitude = (uint32_t)(longitude * 10000000);
  originMsg.altitude = (uint32_t)(altitude * 1000);
  originMsg.target_system = target_system;

  mavros_msgs::Mavlink packedMsg;
  bool success = packMavlinkMessage(originMsg, packedMsg);

  if(success){
    printf("Pack mavlink message SUCCEEDED\n");
  }
  else{
    printf("Pack mavlink message FAILED\n");
  }

  origin_pub.publish(packedMsg);


  mavros_msgs::CommandHome set_home_req;
  set_home_req.request.current_gps = false;
  set_home_req.request.latitude = latitude;
  set_home_req.request.longitude = longitude;
  set_home_req.request.altitude = altitude;

  ros::service::call("/mavros/cmd/set_home", set_home_req);

  printf("Result was %d\n", set_home_req.response.result); 

}
