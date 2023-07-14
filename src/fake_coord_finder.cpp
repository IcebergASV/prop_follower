#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>

void fake_coord_finder() {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("prop_local_coords", 1);
    private_nh_.param<double>("points", points);
    ros::Rate rate(10);
    prop_follower::PropAngleRange msg;
    geometry_msgs::Vector3 prop_coords_msg;

    std::vector<geometry_msg::Vector3> points_vec;

    // Iterate over each point
    for (const auto& p : points) {
      // Access the 'x', 'y', and 'z' coordinates
      double x = p["x"].as<double>();
      double y = p["y"].as<double>();
      double z = p["z"].as<double>();

      // Create a geometry_msgs::Vector3 message
      geometry_msgs::Vector3 point;
      point.x = x;
      point.y = y;
      point.z = z;

      points_vec.push_back(point);
    }

    prop_coords_msg.x = ; //North
    prop_coords_msg.y = ; //East 
    prop_coords_msg.z = 0; //Down

    while (ros::ok()) {
        ROS_DEBUG_STREAM(prop_coords_msg);
        pub.publish(prop_coords_msg);
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_coord_finder");
    try {
        fake_bbox_angles();
    }
    catch (ros::Exception& e) {
        ROS_ERROR_STREAM("An exception occurred in the fake_coord_finder node: " << e.what());
    }
    return 0;
}