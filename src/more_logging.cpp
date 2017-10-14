#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Imu.h>

#include <fstream>

mavros_msgs::OpticalFlowRad opt_flow_msg;
std_msgs::Float64 comp_hdg_msg;
sensor_msgs::Imu imu_data_msg;
sensor_msgs::Temperature imu_temp_msg;

void opt_flow_cb(mavros_msgs::OpticalFlowRad::ConstPtr& msg)
{
  opt_flow_msg = *msg;
}

void comp_hdg_cb(std_msgs::Float64::ConstPtr& msg)
{
  comp_hdg_msg = *msg;
}

void imu_data_cb(sensor_msgs::Imu::ConstPtr& msg)
{
  imu_data_msg = *msg;
}

void imu_temp_cb(sensor_msgs::Temperature::ConstPtr& msg)
{
  imu_temp_msg = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "days_before_logs");
  ros::NodeHandle dbl;

  ros::Subscriber opt_flow = dbl.subscribe<mavros_msgs::OpticalFlowRad>("mavros/px4flow/raw/optical_flow_rad",1,opt_flow_cb);
  ros::Subscriber comp_hdg = dbl.subscribe<std_msgs::Float64>("mavros/global_position/compass_hdg",1,comp_hdg_cb);
  ros::Subscriber imu_data = dbl.subscribe<sensor_msgs::Imu>("mavros/imu/data",1,imu_data_cb);
  ros::Subscriber imu_temp = dbl.subscribe<sensor_msgs::Temperature>("mavros/imu/temperature",1,imu_temp_cb);

  std::ofstream outTimeHist("~/TimeHist.log",std::ios::app);

  while (ros::ok())
  {
    ros::spinOnce();
    std::ofstream out1Line("~/1Line.log",std::ios::trunc);

    float hdg = comp_hdg_msg.data;
    float temp = imu_temp_msg.temperature;
    

    int optflow_intTime = opt_flow_msg.integration_time_us;
    float optflow_intX = opt_flow_msg.integrated_x;
    float optflow_intY = opt_flow_msg.integrated_y;
    float optflow_intXgyro = opt_flow_msg.integrated_xgyro;
    float optflow_intYgyro = opt_flow_msg.integrated_ygyro;
    float optflow_intZgyro = opt_flow_msg.integrated_zgyro;
    float optflow_quality = opt_flow_msg.quality;
    float optflow_timeDel = opt_flow_msg.time_delta_distance_us;
    float optflow_dist = opt_flow_msg.distance;

    float imu_orientX = imu_data_msg.orientation.x;
    float imu_orientY = imu_data_msg.orientation.y;
    float imu_orientZ = imu_data_msg.orientation.z;
    float imu_orientW = imu_data_msg.orientation.w;
    float imu_angularX = imu_data_msg.angular_velocity.x;
    float imu_angularY = imu_data_msg.angular_velocity.y;
    float imu_angularZ = imu_data_msg.angular_velocity.z;
    float imu_linAccelX = imu_data_msg.linear_acceleration.x;
    float imu_linAccelY = imu_data_msg.linear_acceleration.y;
    float imu_linAccelZ = imu_data_msg.linear_acceleration.z;

  }
}
