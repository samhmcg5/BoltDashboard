//file ex.
//msg1 5
//msg1 3
//msg2 2
//...
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  //ros::Publisher chatter_pub = n.advertise<std_msgs::Int16>("chatter", 1000);
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher chatter_pub_msg1 = n.advertise<std_msgs::String>("chatter_msg1", 1000);
  ros::Publisher chatter_pub_msg2 = n.advertise<std_msgs::String>("chatter_msg2", 1000);
  
  ros::Rate loop_rate(10);

  std::ifstream inputStream;
  inputStream.open(argv[1]);
  if(inputStream.is_open())
    std::cout << "file is open: " << argv[1] << std::endl;
  else if(argc < 2)
    std::cout << "please specify file" << std::endl;
  else
    std::cout << "file did not open" << std::endl;
  
  while (ros::ok() && !inputStream.fail())
    {
      
      //std_msgs::Int16 msg;
      //short randVale = 5;
      //msg.data = randValue;

      //ROS_INFO("%d", msg.data);

      std::string name;
      std::string value;
      
      std_msgs::String msg;
      std::stringstream stream;
      
      inputStream >> name >> value;
      msg.data = name + " " + value;

      if(name == "msg1"){
	//ROS_INFO("msg1: %s", msg.data);
	std::cout << "send msg1: " << msg.data << std::endl;
	chatter_pub_msg1.publish(msg);
      }
      else if(name == "msg2"){
	std::cout << "send msg2: " << msg.data << std::endl;
	chatter_pub_msg2.publish(msg);
	
      }
      else{
	std::cout << "End of File: exiting" << std::endl;
	exit(0);
      }
      
      std::cin.ignore();
      

      
      ros::spinOnce();

      loop_rate.sleep();
    }


  return 0;
}
