#include <ros/ros.h>
#include <cmath>
#include "lidarPoint.h"
#include <string>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <gnc_functions.hpp> // Intelligent Quads mavros API

//TO DO: make callback use the appropriate message type for /prop_local_coords topic
//

class WPSender {
public:
    WPSender() : nh_(""), private_nh_("~")
    {
        prop_local_coords_sub_ = nh_.subscribe("/prop_local_coords", 1, &WPSender::wpCallback, this);
        setpoint_velocity_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:

    void wpCallback(const geometry_msgs::Vector3::ConstPtr& vector_msg)
    {
        geometry_msgs::PoseStamped waypoint;
        //geometry_msgs::Vector3 vector_msg = *msg;
        waypoint.pose.position.x = vector_msg->x;
        waypoint.pose.position.y = vector_msg->y;
        waypoint.pose.position.z = vector_msg->z;
        waypoint.pose.orientation.w = 0.5;
        waypoint.pose.orientation.x = 0.5;
        waypoint.pose.orientation.y = 0.5;
        waypoint.pose.orientation.z = 0.5;


        setpoint_velocity_pub_.publish(waypoint);
    }

    ros::NodeHandle nh_;
    ros::Subscriber prop_local_coords_sub_;
    ros::Publisher setpoint_velocity_pub_;
    ros::NodeHandle private_nh_;


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wp_sender_node");
    WPSender wp_sender;
    
    // wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	// arm boat 
	arm();

    wp_sender.spin();
    return 0;
}