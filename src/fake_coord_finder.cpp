#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <vector>

/**
 * @brief Node to publish fake coordinates for testing velocity and waypoint sender without camera or lidar readings
 *
 * Coordinates are loaded in from fake_coords.yaml
 * 
 * Set lidar-distance-angle-measurement to false to run this node 
 */
void fake_coord_finder() {
    ros::NodeHandle nh("~");

    // Coordinate parameters
    std::vector<double> x_coords;
    std::vector<double> y_coords;
    std::vector<double> z_coords;
    nh.getParam("x", x_coords);
    nh.getParam("y", y_coords);
    nh.getParam("z", z_coords);

    // Vector to store the coordinates
    std::vector<geometry_msgs::Vector3> points_vec;

    ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("prop_local_coords", 1);

    ros::Rate rate(10);

    // Message to publish
    geometry_msgs::Vector3 prop_coords_msg;

    
    int i = 0;
    // use the x coordinates to iterate over all coordinates
    for (const auto& coord : x_coords) {
      // Access the 'x', 'y', and 'z' coordinates
      double x = x_coords[i];
      double y = y_coords[i];
      double z = z_coords[i];

      // Create a geometry_msgs::Vector3 message
      prop_coords_msg.x = x;
      prop_coords_msg.y = y;
      prop_coords_msg.z = z;

      points_vec.push_back(prop_coords_msg);
      i++;
    }

    int j = 0;

    while (ros::ok()) {
        // Publish the messages in points_vec one by one - loop after reaching the end
        j = j%(points_vec.size());
        pub.publish(points_vec[j]);
        j++;
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_coord_finder");
    try {
        fake_coord_finder();
    }
    catch (ros::Exception& e) {
        ROS_ERROR_STREAM("An exception occurred in the fake_coord_finder node: " << e.what());
    }
    return 0;
}