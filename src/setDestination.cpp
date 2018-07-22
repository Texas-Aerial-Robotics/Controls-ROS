#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

geometry_msgs::PoseStamped waypoint;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "setDestination");

  ros::NodeHandle setDestinationNode;

  ros::Rate rate(5.0);
  ros::Publisher chatter_pub = setDestinationNode.advertise<geometry_msgs::PoseStamped>("TARwaypoint", 1);

  printf("desired x: ");
  fflush( stdout );
  cin >> waypoint.pose.position.x;
  printf("desired y: ");
  fflush( stdout );
  cin >> waypoint.pose.position.y;
  printf("desired z: ");
  fflush( stdout );
  cin >> waypoint.pose.position.z;

  for (int i = 0; i < 20; ++i)
  {
    chatter_pub.publish(waypoint);
    ros::Duration(0.01).sleep();
  }
  return 0;
}
