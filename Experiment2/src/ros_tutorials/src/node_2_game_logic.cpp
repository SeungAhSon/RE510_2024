#include "ros/ros.h"
#include "ros_tutorials/Counts.h"
#include "ros_tutorials/IsClap.h"
#include <cmath>

ros::Publisher result_pub;

void numberCallback(const ros_tutorials::Counts::ConstPtr& msg) {
  int number = msg->data;
  std::stringstream ss;
  ss << number;
  std::string result = (ss.str().find('3') != std::string::npos ||
                        ss.str().find('6') != std::string::npos ||
                        ss.str().find('9') != std::string::npos) ? "Clap" : "Pass";
  
  ros_tutorials::IsClap msg2;
  msg2.data = number;
  msg2.isclap = result;
  result_pub.publish(msg2);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "game_logic_node");
  ros::NodeHandle nh;
  result_pub = nh.advertise<ros_tutorials::IsClap>("result_topic", 100);
  ros::Subscriber sub = nh.subscribe("chatter", 1000, numberCallback);
  ros::spin();

  return 0;
}
