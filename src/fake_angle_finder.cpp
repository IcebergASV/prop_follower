#include <ros/ros.h>
#include <prop_follower/PropAngleRange.h>
#include <cmath>


void fake_bbox_angles() {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<prop_follower::PropAngleRange>("prop_angle_range", 1);
    ros::Rate rate(10);
    prop_follower::PropAngleRange msg;
    msg.prop_label = "buoy";
    msg.theta_1 = 0; 
    msg.theta_2 = M_PI; 

    while (ros::ok()) {
        ROS_INFO_STREAM(msg);
        pub.publish(msg);
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_angle_finder");
    try {
        fake_bbox_angles();
    }
    catch (ros::Exception& e) {
        ROS_ERROR_STREAM("An exception occurred in the fake_angle_finder node: " << e.what());
    }
    return 0;
}