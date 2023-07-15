#include <ros/ros.h>
#include <prop_follower/PropAngleRange.h>
#include <cmath>

/**
 * @brief Node to publish fake angle ranges for testing without camera
 *
 * Publishes a prop labelled buoy with an angle range of 0 to 180 degrees.
 *
 * Set object_detection to false to run this node. 
 */
void fake_bbox_angles() {
    ros::NodeHandle nh("");
    ros::NodeHandle private_nh_("~");

    // Get params
    double theta_1;
    double theta_2;
    private_nh_.param<double>("theta_1", theta_1, 0.0);
    private_nh_.param<double>("theta_2", theta_2, M_PI);

    ros::Publisher pub = nh.advertise<prop_follower::PropAngleRange>("prop_angle_range", 1);
    ros::Rate rate(10);
    prop_follower::PropAngleRange msg;

    // Message
    msg.prop_label = "buoy";
    msg.theta_1 = theta_1; 
    msg.theta_2 = theta_2;

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