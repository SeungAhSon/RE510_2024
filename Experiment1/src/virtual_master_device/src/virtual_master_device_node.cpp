#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <std_msgs/Bool.h>

#include <Eigen/Core>

#include <virtual_master_device/master_dev_state.h>

ros::NodeHandlePtr node_;
ros::NodeHandlePtr pnode_;

visualization_msgs::Marker clutch_marker;
visualization_msgs::InteractiveMarker int_marker;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

ros::Publisher master_state_pub_;
ros::Publisher clutch_marker_pub_;


double tl = 0.3; //translational limit
double rl = 60.0*(M_PI/180); //rotational limit

virtual_master_device::master_dev_state master_state_;

std::string master_frame = "master";

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
{

    switch ( feedback->event_type )
    {
        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
//            ROS_INFO_STREAM( ": menu item " << feedback->menu_entry_id << " clicked" << "." );
            if(feedback->menu_entry_id == 1){
//                ROS_INFO("Clutch On");
                master_state_.button = true;
            }
            else if(feedback->menu_entry_id == 2){
//                ROS_INFO("Clutch Off");
                master_state_.button = false;
                geometry_msgs::Pose pose;
                pose.position.x = 0;
                pose.position.y = 0;
                pose.position.z = 0;
                pose.orientation.w = 1;
                pose.orientation.x = 0;
                pose.orientation.y = 0;
                pose.orientation.z = 0;

                server->setPose(feedback->marker_name, pose);
                server->applyChanges();
            }
            break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            break;
    }

    tf::Pose pose;
    tf::poseMsgToTF(feedback->pose, pose);
    double rolling,pitching,yawing;
    pose.getBasis().getRPY(rolling, pitching, yawing);

    bool translation_ws_limit = (fabs(feedback->pose.position.x) > tl || fabs(feedback->pose.position.y) > tl || fabs(feedback->pose.position.z) > tl);
    bool rotation_ws_limit = (fabs(rolling) > rl || fabs(pitching) > rl || fabs(yawing) > rl);

    if(translation_ws_limit || rotation_ws_limit) {
//        ROS_ERROR("workspace limit");
    }
    else{
        server->applyChanges();

        clutch_marker.pose = feedback->pose;

        master_state_.pos.pose.position = feedback->pose.position;
        master_state_.pos.pose.orientation = feedback->pose.orientation;

        master_state_.pos.header.frame_id = master_frame;
        master_state_.pos.header.stamp = ros::Time::now();

        master_state_pub_.publish(master_state_);
    }
}

void Simple6DOFInteractiveMarker(){


    int_marker.header.frame_id = master_frame;
    int_marker.header.stamp=ros::Time::now();
    int_marker.name = "Master_Device_Marker";
    int_marker.description = "6-DOF Controller";

    int_marker.scale = 0.5;

    visualization_msgs::Marker marker;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.3;

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(marker);

    box_control.name = "menu control";

    // add the control to the interactive marker
    int_marker.controls.push_back( box_control );
    int_marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;


    visualization_msgs::InteractiveMarkerControl control;

    std::string mode_text;

    int_marker.name += "Master Device";
    bool show_6dof = true;
    int_marker.description = "Master Device";

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;

    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    server->insert(int_marker, &processFeedback);

    server->applyChanges();
    menu_handler.apply( *server, int_marker.name );

}

void ClutchMarkerInit(){

    clutch_marker.type = visualization_msgs::Marker::CUBE;
    clutch_marker.header.frame_id = master_frame;
    clutch_marker.ns = "my_namespace";
    clutch_marker.action =  visualization_msgs::Marker::ADD;
    clutch_marker.scale.x = 0.1;
    clutch_marker.scale.y = 0.1;
    clutch_marker.scale.z = 0.1;
    clutch_marker.color.r = 1.0;
    clutch_marker.color.g = 0.0;
    clutch_marker.color.b = 0.0;
    clutch_marker.color.a = 1.0;
}

// Load parameters etc
int init()
{
    node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
    pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    clutch_marker_pub_ = node_->advertise<visualization_msgs::Marker>("clutch_marker",10);
    master_state_pub_ = node_->advertise<virtual_master_device::master_dev_state>("master_device/state",10);
    master_state_.button = false;
    return 0;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "virtual_master_device_node");

    init();

    server.reset( new interactive_markers::InteractiveMarkerServer("master_device_controls","",false) );

    menu_handler.insert( "Clutch On", &processFeedback);
    menu_handler.insert( "Clutch Off", &processFeedback);

    ClutchMarkerInit();
    Simple6DOFInteractiveMarker();

    ros::Rate loop_rate(100);

    while (node_->ok()) {


        static bool old_button = master_state_.button;
        if(old_button != master_state_.button){
            if(master_state_.button){
                clutch_marker.color.r = 0.0;
                clutch_marker.color.g = 1.0;
                clutch_marker.color.b = 0.0;
            }
            else{
                clutch_marker.color.r = 1.0;
                clutch_marker.color.g = 0.0;
                clutch_marker.color.b = 0.0;
            }
            old_button = master_state_.button;
        }

        clutch_marker.header.stamp = ros::Time::now();
        clutch_marker_pub_.publish(clutch_marker);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

