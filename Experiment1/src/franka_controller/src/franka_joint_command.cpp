#include "ros/ros.h"

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>


sensor_msgs::JointState DesiredJointState_;
bool desired_joint_states_callback_ {false};

int teleoperation_mode_;

void DesriedJointStateCallback(const sensor_msgs::JointStateConstPtr& msg){
  desired_joint_states_callback_ = true;
  
  DesiredJointState_.position = msg->position;
  DesiredJointState_.velocity = msg->velocity;
  DesiredJointState_.effort = msg->effort;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "franka_panda_controller_node");

    ros::NodeHandle n;
    ros::NodeHandlePtr pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));
    n.param("teleoperation_mode", teleoperation_mode_, 1);
    ROS_INFO("Joint Command: %d",teleoperation_mode_);

    std::string robot_namespace;
    pnode_->param("robot_name", robot_namespace, std::string("franka"));

    ros::Subscriber desreid_jointstate_sub = n.subscribe(robot_namespace+"/desired_joint_state",100,DesriedJointStateCallback);

    ros::Publisher joint1_pos_cmd_pub = n.advertise<std_msgs::Float64>(robot_namespace+"/joint1_position_controller/command", 10);
    ros::Publisher joint2_pos_cmd_pub = n.advertise<std_msgs::Float64>(robot_namespace+"/joint2_position_controller/command", 10);
    ros::Publisher joint3_pos_cmd_pub = n.advertise<std_msgs::Float64>(robot_namespace+"/joint3_position_controller/command", 10);
    ros::Publisher joint4_pos_cmd_pub = n.advertise<std_msgs::Float64>(robot_namespace+"/joint4_position_controller/command", 10);
    ros::Publisher joint5_pos_cmd_pub = n.advertise<std_msgs::Float64>(robot_namespace+"/joint5_position_controller/command", 10);
    ros::Publisher joint6_pos_cmd_pub = n.advertise<std_msgs::Float64>(robot_namespace+"/joint6_position_controller/command", 10);
    ros::Publisher joint7_pos_cmd_pub = n.advertise<std_msgs::Float64>(robot_namespace+"/joint7_position_controller/command", 10);


    printf("franka_panda_controller_node start!..\n");

    ros::Rate loop_rate(1000);

    while (n.ok())
    {


        std_msgs::Float64 joint1_pos_cmd;
        std_msgs::Float64 joint2_pos_cmd;
        std_msgs::Float64 joint3_pos_cmd;
        std_msgs::Float64 joint4_pos_cmd;
        std_msgs::Float64 joint5_pos_cmd;
        std_msgs::Float64 joint6_pos_cmd;
        std_msgs::Float64 joint7_pos_cmd;


        if(desired_joint_states_callback_){

            // Teleoperation
            // position[0],[1] are finger joints

            joint1_pos_cmd.data = DesiredJointState_.position[2];
            joint2_pos_cmd.data = DesiredJointState_.position[3];
            joint3_pos_cmd.data = DesiredJointState_.position[4];
            joint4_pos_cmd.data = DesiredJointState_.position[5];
            joint5_pos_cmd.data = DesiredJointState_.position[6];
            joint6_pos_cmd.data = DesiredJointState_.position[7];
            joint7_pos_cmd.data = DesiredJointState_.position[8];

            joint1_pos_cmd_pub.publish(joint1_pos_cmd);
            joint2_pos_cmd_pub.publish(joint2_pos_cmd);
            joint3_pos_cmd_pub.publish(joint3_pos_cmd);
            joint4_pos_cmd_pub.publish(joint4_pos_cmd);
            joint5_pos_cmd_pub.publish(joint5_pos_cmd);
            joint6_pos_cmd_pub.publish(joint6_pos_cmd);
            joint7_pos_cmd_pub.publish(joint7_pos_cmd);

        }


        ros::spinOnce();
        loop_rate.sleep();
    }
  return 0;
}
