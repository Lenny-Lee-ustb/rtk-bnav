#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
 
#include <iostream>
#include <math.h>
#include <string.h>
#include <sstream>
 
void chatterCallback(const sensor_msgs::NavSatFixConstPtr& msg)
{
  float x,y;
  x = msg->latitude;
  y = msg->longitude;
  ROS_INFO("The position now is x:[%f] y:[%f]",x,y);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtk_test");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("fix",1000,chatterCallback);
  ros::spin();

  return 0;
}
 

