#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Float64.h"

using namespace std;

std_msgs::Float64 newHeading;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "setHeading");

  ros::NodeHandle setHeadingNode;

  ros::Rate rate(5.0);
  ros::Publisher heading_pub = setHeadingNode.advertise<std_msgs::Float64>("setHeading", 1);


  printf("desired heading: ");
  fflush( stdout );
  cin >> newHeading.data;

  heading_pub.publish(newHeading);
  return 0;
}
