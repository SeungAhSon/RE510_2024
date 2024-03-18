#include "ros/ros.h"
#include "ros_tutorials/Counts.h"
#include <cmath>

int main(int argc, char **argv) {
  ros::init(argc, argv, "number_generator_node");
  ros::NodeHandle nh;
  
  ros::Publisher pub = nh.advertise<ros_tutorials::Counts>("chatter", 100);
  ros::Rate loop_rate(1);  

  ros_tutorials::Counts msg;
  int count = 0;
  while (ros::ok()) {
    msg.stamp = ros::Time::now();
    msg.data = count;

    pub.publish(msg);
    loop_rate.sleep();
    count++;
  }

  return 0;
}
