#include "ros/ros.h"
#include "ros_tutorials/IsClap.h"
#include "std_msgs/String.h"

void resultCallback(const ros_tutorials::IsClap::ConstPtr& msg) {
  int number = msg->data;
  std::string result_msg = msg->isclap;

  std::stringstream ss_result;
  ss_result << number << " is a " << result_msg;

  ROS_INFO("%s", ss_result.str().c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "game_output_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("result_topic", 100, resultCallback);
  
  ros::spin();
  return 0;
}
