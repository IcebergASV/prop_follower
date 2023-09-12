#include <ros/ros.h>
#include <cmath>
#include "lidarPoint.h"
#include <string>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <gnc_functions.hpp> // Intelligent Quads mavros API

//TO DO: make callback use the appropriate message type for /prop_local_coords topic
//

class VelSender {
public:
    VelSender() : nh_(""), private_nh_("~")
    {
        prop_local_coords_sub_ = nh_.subscribe("/prop_local_coords", 1, &VelSender::velCallback, this);
        setpoint_velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);

        //load parameter for scaling velocity. 1.0 is the default value 
        private_nh_.param<double>("vel_scale_factor", vel_scale_factor, 1.0); 
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:

    void velCallback(const geometry_msgs::Vector3::ConstPtr& vector_msg)
    {
        geometry_msgs::Twist twist_msg;
        //geometry_msgs::Vector3 vector_msg = *msg;
        twist_msg.linear.x = vel_scale_factor * vector_msg->x;
        twist_msg.linear.y = vel_scale_factor * vector_msg->y;
        twist_msg.linear.z = vel_scale_factor * vector_msg->z;
        setpoint_velocity_pub_.publish(twist_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber prop_local_coords_sub_;
    ros::Publisher setpoint_velocity_pub_;
    ros::NodeHandle private_nh_;
    
    double vel_scale_factor;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vel_sender_node");
    VelSender vel_sender;
    
    // wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	// arm boat 
	arm();

    vel_sender.spin();
    return 0;
}