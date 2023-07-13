
#define vel_scale_factor 
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <navigation_pkg/PropInProgress.h>
#include <cmath>
#include <vector>
#include <stdexcept>
#include "lidarPoint.h"
#include <string>
#include <iostream>
#include <geometry_msgs>

class Vel_sender {
public:
    Vel_sender()
    {
        prop_local_coords_sub_ = nh_.subscribe("/prop_local_coords", 1, &Vel_sender::vel_callback, this);
        setpoint_velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:

    void vel_callback(const navigation_pkg::yolo::ConstPtr& msg)
    {

        setpoint_velocity_pub_.publish(prop_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber prop_local_coords_sub_;
    ros::Publisher setpoint_velocity_pub_;
    double x_min;
    double x_max;
    double const realsense_fov = 1.204277184; //radians - 69 degrees
    double const fov_end = (M_PI / 2) + (realsense_fov / 2 );
    int const realsense_res_x = 1920;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "angle_finder_node");
    AngleFinder angle_finder;
    angle_finder.spin();
    return 0;
}