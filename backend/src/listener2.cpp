#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"

void chatterCallback_string(const std_msgs::String::ConstPtr& msg)
{
 ROS_INFO("I heard anothe msg: [%s]", msg->data.c_str());
}

void chatterCallback_int(const std_msgs::Int16::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]", msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener2");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter_msg2", 1000, chatterCallback_string);

  ros::spin();

  return 0;
}
