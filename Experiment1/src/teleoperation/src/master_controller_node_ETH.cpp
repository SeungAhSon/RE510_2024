#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

#include <virtual_master_device/master_dev_state.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class TeleMasterController
{
public:

	ros::NodeHandlePtr node_;
	ros::NodeHandlePtr pnode_;


    ros::Publisher master_cmd_pub_;


	ros::Subscriber master_device_state_sub_;

    int teleoperation_mode_;


	void MasterDevStateCallback(const virtual_master_device::master_dev_stateConstPtr &msg){

        // Dt
        ros::Time t1;
        t1 = ros::Time::now();
        static ros::Time t2 = t1;
        double Dt = (t1-t2).toSec();
        t2 = t1;
        if(Dt == 0) return;
        
        
        // Master Position
        double m_px = msg->pos.pose.position.x;
        double m_py = msg->pos.pose.position.y;
        double m_pz = msg->pos.pose.position.z;

        // Master Orientation
        Eigen::Quaternionf m_rot(
            msg->pos.pose.orientation.w,
            msg->pos.pose.orientation.x,
            msg->pos.pose.orientation.y,
            msg->pos.pose.orientation.z);

        // Indexing trigger
        bool clutch = msg->button;

        static virtual_master_device::master_dev_state old_msg = *msg;

        geometry_msgs::PoseStamped master_command;


        // The value of 'teleoperation_mode_' varaible is defined by the 'teleoperation_mode' parameter in the 'teleoperation.launch' file
        // 1.Position to Position : publish the increments command
        if(teleoperation_mode_ == 1){
            
            // Make your increments command

            // Translation
            master_command.pose.position.x = 0.0; // replace '0.0' to your command value
            master_command.pose.position.y = 0.0; // replace '0.0' to your command value
            master_command.pose.position.z = 0.0; // replace '0.0' to your command value

            // Orientation
            master_command.pose.orientation.x = 0.0; // replace value to your command value
            master_command.pose.orientation.y = 0.0; // replace value to your command value
            master_command.pose.orientation.z = 0.0; // replace value to your command value
            master_command.pose.orientation.w = 1.0; // replace value to your command value
        }

        // 2.Position to Velocity : publish the position command
        else if(teleoperation_mode_ == 2){

            // Translation
            master_command.pose.position.x = 0.0; // replace '0.0' to your command value
            master_command.pose.position.y = 0.0; // replace '0.0' to your command value
            master_command.pose.position.z = 0.0; // replace '0.0' to your command value

            // Orientation
            master_command.pose.orientation.x = 0.0; // replace value to your command value
            master_command.pose.orientation.y = 0.0; // replace value to your command value
            master_command.pose.orientation.z = 0.0; // replace value to your command value
            master_command.pose.orientation.w = 1.0; // replace value to your command value
        }


        // Publish Master Command
        // Changed : Publish only when clutch is on
        if (clutch)
        {
            master_command.header = msg->pos.header;
            master_cmd_pub_.publish(master_command);
        }
        
    

        // Update old_msg
        old_msg = *msg;
	}

	// Smoothing the noisy signal. previous: x(t-1), current: x(t).
	double lowpass_filter(double previous, double current){
	    double k = 0.95; // can adjust the gain
        current = k*current + (1.0-k)*previous;
        return current;
	}


  // Load parameters etc
	int init()
	{
		node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
		pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        node_->param("teleoperation_mode", teleoperation_mode_, 1);

        master_device_state_sub_ = node_->subscribe("master_device/state",10,&TeleMasterController::MasterDevStateCallback,this);

        master_cmd_pub_ = node_->advertise<geometry_msgs::PoseStamped>("master_command",10);

		return 0;
	}

	// Publish data
	void publish()
	{
		// ros::Rate loop_rate(100);
		while (node_->ok()) {
		  ros::spinOnce();
		  // loop_rate.sleep();
		}
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "tele_master_controller_node_ETH");

	TeleMasterController master;
	if(master.init())
	{
		ROS_FATAL("tele_master_controller_node initialization failed");
		return -1;
	}

	master.publish();

	return 0;
}


