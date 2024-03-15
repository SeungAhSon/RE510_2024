#include "ros/ros.h"


#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/tree.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>


class FrankaKinematicsSolver{
public:


	ros::NodeHandlePtr node_;
	ros::NodeHandlePtr pnode_;

	geometry_msgs::PoseStamped target_ee_pose_;

	ros::Publisher desired_joint_state_pub_;
	ros::Publisher updated_ee_pose_pub_;

	ros::Subscriber franka_jointstate_sub_;
	ros::Subscriber target_ee_pose_sub_;

	std::string robot_name_space_;
	std::string robot_name_;

	tf::TransformListener listener_;

	bool JointStateGet_{false};

	KDL::Chain franka_chain_;

	sensor_msgs::JointState current_jointstate_;

	int teleoperation_mode_;


	void TargetEEPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg){

		target_ee_pose_ = *msg; // Target End-effector Pose.

		if(!JointStateGet_) return;


        sensor_msgs::JointState current_joint_state;


        // Get current franka joint state information for joint1 to joint7
        // jointstate's [0],[1] data is finger joints, and we don't need it
        for(unsigned int i=0; i<7;i++){
            current_joint_state.name.push_back(current_jointstate_.name[i+2]);
            current_joint_state.position.push_back(current_jointstate_.position[i+2]);
            current_joint_state.velocity.push_back(current_jointstate_.velocity[i+2]);
            current_joint_state.effort.push_back(current_jointstate_.effort[i+2]);
        }

        // Get Current joint angle q_current by current_joint_state
        KDL::JntArray q_current;
        JointStateMsgToKDLJntArray(current_joint_state,q_current);


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        sensor_msgs::JointState desired_joint_state; // Put your desired joint values to this variable.



        //--- Implement your code here ---//  implement inverse kinematics by using included functions in this c++ class
        KDL::Frame target_frame;
        PoseStampedMsgToKDLFrame(target_ee_pose_, target_frame);

        KDL::JntArray q_out(q_current.rows());
        int ik_result = KDLInverseKinematics(franka_chain_, target_frame, q_current, q_out);

        if (ik_result >= 0) {
            std::cout <<"Y : Found a solution." << "\n";
            KDLJntArrayToJointStateMsg(q_out, desired_joint_state);
        }else {
            std::cout <<"N : Inverse kinematics failed to find a solution." << "\n";
            KDLJntArrayToJointStateMsg(q_current, desired_joint_state);
        }
        

        /////////////////////////////////roslaunch franka_controller franka_controller.launch

        // Publish final joint state.
        sensor_msgs::JointState final_joint_state; // Including finger joints

        final_joint_state = current_jointstate_;

        // Updating desired joint positions
        for(unsigned int i=0; i<7;i++){
            final_joint_state.name[i+2] = desired_joint_state.name[i];
            final_joint_state.position[i+2] = desired_joint_state.position[i];
            final_joint_state.velocity[i+2] = desired_joint_state.velocity[i];
            final_joint_state.effort[i+2] = desired_joint_state.effort[i];
        }

        final_joint_state.header.stamp = ros::Time::now();
        desired_joint_state_pub_.publish(final_joint_state);


        // Forward Kinematics To check the result
        geometry_msgs::PoseStamped updated_ee_pose;
        KDL::JntArray q_now;

        JointStateMsgToKDLJntArray(desired_joint_state,q_now);

        KDL::Frame ee_frame;
        int fk_res = KDLForwardKinematics(franka_chain_,q_now,ee_frame);
        KDLFrameToPoseStampedMsg(ee_frame,updated_ee_pose);
        updated_ee_pose.header.frame_id = "panda_link0";
        updated_ee_pose.header.stamp = msg->header.stamp;

        updated_ee_pose_pub_.publish(updated_ee_pose);
	}



	// You can use 'franka_chain_' variable as input for 'robot_chain'.
	// 'franka_chain_' variable is already figured out in the 'init' function.
	int KDLForwardKinematics(const KDL::Chain robot_chain, const KDL::JntArray joint_in, KDL::Frame& cart_pos_out){

	    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(robot_chain);
	    int fk_result = fksolver.JntToCart(joint_in,cart_pos_out); // if < 0 something went wrong
	    return fk_result;
	}

	int KDLInverseKinematics(const KDL::Chain robot_chain, const KDL::Frame target_frame,
	        const KDL::JntArray q_current, KDL::JntArray& q_out){

        // Joint Limit
        KDL::JntArray q_min(q_current.rows());
        KDL::JntArray q_max(q_current.rows());

        // Set Joint Limit for each joint (Below is franka joint limits)
        q_min(0) = -2.8973;
        q_max(0) = 2.8973;
        q_min(1) = -1.7628;
        q_max(1) = 1.7628;
        q_min(2) = -2.8973;
        q_max(2) = 2.8973;
        q_min(3) = -3.0718;
        q_max(3) = 3.0718;
        q_min(4) = -2.8973;
        q_max(4) = 2.8973;
        q_min(5) = -0.0175;
        q_max(5) = 3.7525;
        q_min(6) = -2.8973;
        q_max(6) = 2.8973;


        // Creation of the solvers
        KDL::ChainFkSolverPos_recursive fksolver(franka_chain_);//Forward position solver
        KDL::ChainIkSolverVel_pinv iksolver_vel(franka_chain_);//Inverse velocity solver
        KDL::ChainIkSolverPos_NR_JL iksolver_pos_nr_jl(franka_chain_,q_min,q_max,fksolver,iksolver_vel,200,1e-3);//Maximum 100 iterations, stop at accuracy 1e-3

        // Solving IK
        int ik_result = iksolver_pos_nr_jl.CartToJnt(q_current,target_frame,q_out);
        return ik_result;
	}

	void KDLFrameToPoseStampedMsg(const KDL::Frame kdl_frame_in, geometry_msgs::PoseStamped& pose_stamped_out){
        pose_stamped_out.pose.position.x = kdl_frame_in.p.x();
        pose_stamped_out.pose.position.y = kdl_frame_in.p.y();
        pose_stamped_out.pose.position.z = kdl_frame_in.p.z();

        double qx,qy,qz,qw;
        kdl_frame_in.M.GetQuaternion(qx,qy,qz,qw);

        pose_stamped_out.pose.orientation.x = qx;
        pose_stamped_out.pose.orientation.y = qy;
        pose_stamped_out.pose.orientation.z = qz;
        pose_stamped_out.pose.orientation.w = qw;
	}

    void PoseStampedMsgToKDLFrame(const geometry_msgs::PoseStamped pose_stamped_in, KDL::Frame& kdl_frame_out){
        double ee_roll,ee_pitch,ee_yaw;
        tf::Pose pose;
        tf::poseMsgToTF(pose_stamped_in.pose, pose);
        pose.getBasis().getRPY(ee_roll, ee_pitch, ee_yaw);

        double ee_x = pose_stamped_in.pose.position.x;
        double ee_y = pose_stamped_in.pose.position.y;
        double ee_z = pose_stamped_in.pose.position.z;

        kdl_frame_out = KDL::Frame(KDL::Rotation::RPY(ee_roll, ee_pitch, ee_yaw), KDL::Vector(ee_x, ee_y, ee_z));
    }

    void KDLJntArrayToJointStateMsg(const KDL::JntArray jnt_array_in, sensor_msgs::JointState& jointstate_out){
        for (unsigned int i = 0; i < jnt_array_in.rows(); i++){
            jointstate_out.position.push_back(jnt_array_in(i));
        }
        jointstate_out.name.resize(jnt_array_in.rows());
        jointstate_out.velocity.resize(jnt_array_in.rows());
        jointstate_out.effort.resize(jnt_array_in.rows());
	}

    void JointStateMsgToKDLJntArray(const sensor_msgs::JointState jointstate_in, KDL::JntArray& jnt_array_out){
        jnt_array_out.resize(jointstate_in.position.size());
        for (unsigned int i = 0; i < jointstate_in.position.size(); i++){
            jnt_array_out(i) = jointstate_in.position[i];
        }
    }


    void CurrentJointStateCallback(const sensor_msgs::JointStateConstPtr &msg){
        JointStateGet_ = true;
        current_jointstate_ = *msg;
    }

	void publish(){
		ros::Rate loop_rate(100);
		while (node_->ok())
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
	}

    int init(){


        node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        pnode_->param("robot_namespace", robot_name_space_, std::string("franka"));
        pnode_->param("robot_name", robot_name_, std::string("panda"));
        node_->param("teleoperation_mode", teleoperation_mode_, 1);
        ROS_INFO("Teleoperation Mode: %d",teleoperation_mode_);


        desired_joint_state_pub_ = node_->advertise<sensor_msgs::JointState>(robot_name_space_+"/desired_joint_state", 10);

        updated_ee_pose_pub_ =  node_->advertise<geometry_msgs::PoseStamped>("/updated_target_ee_pose", 10);
        target_ee_pose_sub_ = node_->subscribe("ee_target_pose",10,&FrankaKinematicsSolver::TargetEEPoseCallback,this);
        franka_jointstate_sub_ = node_->subscribe(robot_name_space_+"/joint_states",10,&FrankaKinematicsSolver::CurrentJointStateCallback,this);

        JointStateGet_ = false;


        // Get KDL_tree information from ROS parameter server
        KDL::Tree franka_kdl_tree;
        std::string robot_desc_string;
        node_->param("robot_description", robot_desc_string, std::string());

        if (!kdl_parser::treeFromString(robot_desc_string, franka_kdl_tree)){
            std::cout <<robot_desc_string << "\n";
            ROS_ERROR("Failed to construct kdl tree");
            return -1;
        }
        else{
            ROS_INFO("Successfully construct kdl tree from robot_description");
        }


        unsigned int num_joints = 0;
        num_joints = franka_kdl_tree.getNrOfJoints();
        ROS_INFO("Number of joints: %d\n",num_joints);


        // // Chain Creation
        bool ret0 = franka_kdl_tree.getChain("panda_link0","panda_hand",franka_chain_);
        ROS_INFO("GetChain Result: %d", ret0);

        return 0;
    }

};



int main(int argc, char **argv)
{

	ros::init(argc, argv, "franka_kinematics_solver_node");

    FrankaKinematicsSolver franka_kinematics;
	if (franka_kinematics.init())
	{
		ROS_FATAL("franka_kinematics_solver_node initialization failed");
		return -1;
	}

    franka_kinematics.publish();

	
	return 0;
}
