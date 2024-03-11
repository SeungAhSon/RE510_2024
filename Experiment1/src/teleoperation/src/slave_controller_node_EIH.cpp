#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
using namespace std;

inline Eigen::Vector3d vex(const Eigen::Matrix3d &m) {
    return 0.5 * Eigen::Vector3d{
                         m(2,1)-m(1,2),
                         m(0,2)-m(2,0),
                         m(1,0)-m(0,1)
    };
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
    Eigen::Matrix3d skew_mat;
    skew_mat << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
    return skew_mat;
}

class TeleSlaveController
{
public:

	std::string robot_name_;

	ros::NodeHandlePtr node_;
	ros::NodeHandlePtr pnode_;

	ros::Publisher target_pose_pub_;

	ros::Subscriber master_command_sub_;

	geometry_msgs::PoseStamped target_pose_;

	tf::StampedTransform T_BaseToTip_;
	tf::StampedTransform T_BaseToMas_;
	tf::StampedTransform T_BaseToCam_;
	tf::StampedTransform T_BaseToEE_;

	Eigen::Matrix3d R_CamToScr_;
	Eigen::Matrix3d R_BaseToCam_;
	Eigen::Affine3d T_BaseToEE_Eig_;

    int teleoperation_mode_;

	bool is_slave_state_;
	bool is_initial_slave_ee_pose_get_;

	tf::TransformListener CamPoseListener;

	TeleSlaveController(){
        is_slave_state_ = false;
        is_initial_slave_ee_pose_get_ = false;
	}

	void MasterCommandCallback(const geometry_msgs::PoseStampedConstPtr &msg){

	    geometry_msgs::PoseStamped master_cmd = *msg;

        // Dt
        ros::Time t1;
        t1 = ros::Time::now();
        static ros::Time t2 = t1;
        double Dt = (t1-t2).toSec();

        // Filtering large Dt
        static double prev_Dt = Dt;
        if(Dt > 5.0*prev_Dt && prev_Dt > 0){
            Dt = 0.0;
        }
        prev_Dt = Dt;
        t2 = t1;
//        ROS_INFO("Dt: %f",Dt);


        // Target Pose Update after getting the initial pose of slave manipulator
        if(!is_initial_slave_ee_pose_get_) return;

		// Get current master orientation
		tf::Pose mst_pose;
		tf::poseMsgToTF(master_cmd.pose,mst_pose);
		tf::Matrix3x3 mst_rot_tf = mst_pose.getBasis();
		Eigen::Matrix3d mst_rot_eig;
		tf::matrixTFToEigen(mst_rot_tf,mst_rot_eig); // We get master rotation matrix now.
		
		// Get current slave orientation
		tf::Pose current_pose;
		tf::poseMsgToTF(target_pose_.pose,current_pose);
		tf::Matrix3x3 current_rot_tf = current_pose.getBasis();
		Eigen::Matrix3d current_rot_eig;
		tf::matrixTFToEigen(current_rot_tf,current_rot_eig); // We get master rotation matrix now.

		// Get current camera orientation
		CamPoseListener.lookupTransform("world","base_camera_link",ros::Time(0), T_BaseToCam_);
		tf::Matrix3x3 R_temp = T_BaseToCam_.getBasis();
		tf::matrixTFToEigen(R_temp,R_BaseToCam_);

		// Example of matrix calculation based on Eigen library
        Eigen::MatrixXd T(3,3); // matrix T(3x3)
        Eigen::MatrixXd T_T = T.transpose(); // transpose of T
        Eigen::MatrixXd T_inv = T.inverse(); // inverse of T
        double t01 = T(0,1); // value for the row:1; col:2 component of matrix T

        Eigen::MatrixXd A(3,3), B(3,3); // matrix A and B
        Eigen::MatrixXd AB = A*B; // possible to multiply matrices
		
		
        // The value of 'teleoperation_mode_' varaible is defined by the 'teleoperation_mode' parameter in the 'teleoperation.launch' file
        // 1.Position to Position : publish the increments command
        if(teleoperation_mode_ == 1){

            // Implement your controller

            // Update Desired End-effector Pose to the 'target_pose_' variable.

        }

        // 2.Position to Velocity : publish the position command
        else if(teleoperation_mode_ == 2){

            // Implement your controller

            // Update Desired End-effector Pose to the 'target_pose_' variable.

        }

        // Publish Target End-effector pose or velocity
        target_pose_.header.stamp = msg->header.stamp;
        target_pose_.header.frame_id = robot_name_+"_link0";
        target_pose_pub_.publish(target_pose_);
	}

  // Load parameters etc
	int init()
	{
		node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
		pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

		pnode_->param("robot_name", robot_name_, std::string("panda"));

        node_->param("teleoperation_mode", teleoperation_mode_, 1);

        target_pose_pub_ = node_->advertise<geometry_msgs::PoseStamped>("ee_target_pose",10);

		master_command_sub_ = node_->subscribe("master_command", 10, &TeleSlaveController::MasterCommandCallback, this);


		return 0;
	}


	// Publish data
	void publish()
	{
		tf::TransformListener Listener;
		// ros::Rate loop_rate(100);
		while (node_->ok()) {

			// Get Initial End-effector pose of slave manipulator
			if(!is_initial_slave_ee_pose_get_){
				try{
					Listener.lookupTransform(robot_name_+"_link0", robot_name_+"_link8",ros::Time(0), T_BaseToTip_);
					is_slave_state_ = true;
					// ROS_INFO("tf get success");

					Listener.lookupTransform("world","base_camera_link",ros::Time(0), T_BaseToCam_);
					tf::Matrix3x3 R_temp = T_BaseToCam_.getBasis();
					tf::matrixTFToEigen(R_temp,R_BaseToCam_);
				}
				catch (tf::TransformException ex){
					ROS_ERROR("%s",ex.what());
					is_slave_state_ = false;
					ros::Duration(0.5).sleep();
				}
				if(is_slave_state_){
					geometry_msgs::TransformStamped transform;
					tf::transformStampedTFToMsg(T_BaseToTip_,transform);

			        target_pose_.pose.position.x = transform.transform.translation.x;
			        target_pose_.pose.position.y = transform.transform.translation.y;
			        target_pose_.pose.position.z = transform.transform.translation.z;
					target_pose_.pose.orientation = transform.transform.rotation;

					is_initial_slave_ee_pose_get_ = true;
				}
			}

			ros::spinOnce();
			// loop_rate.sleep();
		}
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "tele_slave_controller_node_EIH");

	TeleSlaveController slave;
	if(slave.init())
	{
		ROS_FATAL("tele_slave_controller_node initialization failed");
		return -1;
	}

	slave.publish();

	return 0;
}


