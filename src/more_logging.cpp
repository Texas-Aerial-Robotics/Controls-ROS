#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Imu.h>

#include <string>
#include <fstream>
#include <time.h>
#include <ctime>

sensor_msgs::Range rng_msg;
mavros_msgs::OpticalFlowRad opt_flow_msg;
std_msgs::Float64 comp_hdg_msg;
sensor_msgs::Imu imu_data_msg;
sensor_msgs::Temperature imu_temp_msg;

void opt_flow_cb(const mavros_msgs::OpticalFlowRad::ConstPtr& msg)
{
  opt_flow_msg = *msg;
}

void comp_hdg_cb(const std_msgs::Float64::ConstPtr& msg)
{
  comp_hdg_msg = *msg;
}

void imu_data_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_data_msg = *msg;
}

void imu_temp_cb(const sensor_msgs::Temperature::ConstPtr& msg)
{
  imu_temp_msg = *msg;
}

void rng_cb(const sensor_msgs::Range::ConstPtr& msg)
{
  rng_msg = *msg;
}

void init_timeHist(std::string file_name)
{
  std::ofstream outTimeHist(file_name.c_str(),std::ios::app);

  outTimeHist << "Time(ms), Hdg(deg), Alt(m), Temp(C), optflow_intTime,optflow_intX,optflow_intY,optflow_intXgyro,";
  outTimeHist << "optflow_intYgyro,optflow_intZgyro,optflow_quality,optflow_timeDel,optflow_dist,";
  outTimeHist << "imu_orientX,imu_orientY,imu_orientZ,imu_orientW,imu_angularX(rad/sec),imu_angularY(rad/sec),imu_angularZ(rad/sec),";
  outTimeHist << "imu_linAccelX(m/s^2),imu_linAccelY(m/s^2),imu_linAccelZ(m/s^2)" << std::endl;

  outTimeHist.close();
}

const std::string currentDateTime()
{
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  strftime(buf,sizeof(buf),"%m_%d_%y-%H_%M", &tstruct);
  return buf;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "days_before_logs");
  ros::NodeHandle dbl;

  ros::Subscriber rng = dbl.subscribe<sensor_msgs::Range>("mavros/rangefinder/rangefinder",1,rng_cb);
  ros::Subscriber opt_flow = dbl.subscribe<mavros_msgs::OpticalFlowRad>("px4flow/px4flow/raw/optical_flow_rad",1,opt_flow_cb);
  ros::Subscriber comp_hdg = dbl.subscribe<std_msgs::Float64>("mavros/global_position/compass_hdg",1,comp_hdg_cb);
  ros::Subscriber imu_data = dbl.subscribe<sensor_msgs::Imu>("mavros/imu/data",1,imu_data_cb);
  ros::Subscriber imu_temp = dbl.subscribe<sensor_msgs::Temperature>("mavros/imu/temperature",1,imu_temp_cb);

  std::string date_file = "/sdCard/Logs/"+currentDateTime()+"_TimeHist.csv";

  init_timeHist(date_file);

  struct timeval start,stop;
  gettimeofday(&start,NULL);
  long int ms_start = start.tv_sec * 1000 + start.tv_usec / 1000;
  while (ros::ok())
  {
    ros::spinOnce();
    std::ofstream out1Line("/sdCard/Logs/1Line.csv",std::ios::trunc);
    std::ofstream outTimeHist(date_file.c_str(),std::ios::app);


    float hdg = comp_hdg_msg.data;
    float rng = rng_msg.range;
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

    out1Line << hdg << "," << rng <<","<< temp << ",";
    out1Line << optflow_intTime << "," << optflow_intX << "," << optflow_intY << ",";
    out1Line << optflow_intXgyro << "," << optflow_intYgyro << "," << optflow_intZgyro << ",";
    out1Line << optflow_quality << "," << optflow_timeDel << "," << optflow_dist << ",";
    out1Line << imu_orientX << "," << imu_orientY << "," << imu_orientZ << "," << imu_orientW << ",";
    out1Line << imu_angularX << "," << imu_angularY << "," << imu_angularZ << ",";
    out1Line << imu_linAccelX << "," << imu_linAccelY << "," << imu_linAccelZ;
    out1Line.close();

    gettimeofday(&stop,NULL);
    long int ms_stop = stop.tv_sec * 1000 + stop.tv_usec / 1000;
    outTimeHist << ms_stop-ms_start << "," << hdg << "," << rng << "," << temp << ",";
    outTimeHist << optflow_intTime << "," << optflow_intX << "," << optflow_intY << ",";
    outTimeHist << optflow_intXgyro << "," << optflow_intYgyro << "," << optflow_intZgyro << ",";
    outTimeHist << optflow_quality << "," << optflow_timeDel << "," << optflow_dist << ",";
    outTimeHist << imu_orientX << "," << imu_orientY << "," << imu_orientZ << "," << imu_orientW << ",";
    outTimeHist << imu_angularX << "," << imu_angularY << "," << imu_angularZ << ",";
    outTimeHist << imu_linAccelX << "," << imu_linAccelY << "," << imu_linAccelZ << std::endl;
    outTimeHist.close();
  }
}
