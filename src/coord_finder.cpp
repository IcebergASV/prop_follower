#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <prop_follower/PropAngleRange.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <vector>
#include <stdexcept>
#include "lidarPoint.h"
#include <string>
#include <iostream>
#include <ros/console.h>

class CoordFinder {
public:
    CoordFinder() : nh_(""), private_nh_("~") {
        // get ROS parameters
        private_nh_.param<std::string>("prop_topic", prop_topic_, "/prop_angle_range");
        private_nh_.param<std::string>("scan_topic", scan_topic_, "/scan");
        private_nh_.param<double>("max_range", max_range_, 10.0);


        sub_scan_ = nh_.subscribe(scan_topic_, 1, &CoordFinder::scanCallback, this);
        sub_prop_ = nh_.subscribe(prop_topic_, 1, &CoordFinder::propCallback, this);
        pub_prop_closest_ = nh_.advertise<geometry_msgs::Vector3>("/prop_local_coords", 1);
        private_nh_.param<double>("angle_error_adjustment", angle_error_adjustment, 0.0);
        private_nh_.param<double>("marker_base_diameter_for_filtering", marker_diameter_for_filtering, 0.0 );
        private_nh_.param<double>("max_lidar_range", max_lidar_range, 0.0 );
        private_nh_.param<double>("min_lidar_range", min_lidar_range, 0.0 );

    }

    void spin() {
        ros::Rate rate(2); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }
    

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_scan_;
    ros::Subscriber sub_prop_;
    ros::Publisher pub_prop_closest_;
    std::string prop_topic_;
    std::string scan_topic_;
    double max_range_;
    double laser_angle_min;
    double laser_angle_max;
    double laser_angle_increment;
    double angle_error_adjustment;
    double marker_diameter_for_filtering; 
    double max_lidar_range;
    double min_lidar_range;
    prop_follower::PropAngleRange prop_msg_;
    sensor_msgs::LaserScan scan_msg;

    void propCallback(const prop_follower::PropAngleRange::ConstPtr& msg) {
        // save the PropInProgress message for later use
        prop_msg_ = *msg;
        ROS_DEBUG_STREAM("Received PropInProgress message with theta_1=" << prop_msg_.theta_1
            << " and theta_2=" << prop_msg_.theta_2);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        laser_angle_min = scan_msg.angle_min;
        laser_angle_max = scan_msg.angle_max;

        // save the scan message for later use
        scan_msg = *msg;
        laser_angle_increment = scan_msg.angle_increment;

        // check if the PropInProgress message is valid
        if (prop_msg_.prop_label.empty()) {
            ROS_WARN("Invalid PropInProgress message received - Prop type is empty");
            return;
        }
        if (std::isnan(prop_msg_.theta_1)) {
            ROS_WARN("Invalid PropInProgress message received - theta 1 is empty");
            return;
        }
        if (std::isnan(prop_msg_.theta_2)) {
            ROS_WARN("Invalid PropInProgress message received - theta 2 is empty");
            return;
        }

        //add a safety range onto the bounding box angles
        double index1_angle = prop_msg_.theta_1 + angle_error_adjustment;
        double index2_angle = prop_msg_.theta_2 - angle_error_adjustment;
        // calculate the range indexes for the given theta angles
        double steps = (laser_angle_max * 2) / laser_angle_increment; 
        int index1 = (int)(((index1_angle + (laser_angle_max - (M_PI/2))) / (laser_angle_max*2))* steps);
        int index2 = (int)(((index2_angle + (laser_angle_max - (M_PI/2))) / (laser_angle_max*2))* steps);
        ROS_DEBUG_STREAM("Coord Finder: index1 :" << index1 << " index2: " << index2);
        ROS_DEBUG_STREAM("Coord Finder: size of scan message ranges " << scan_msg.ranges.size());
        // check that the range indexes are within the range of the scan message and that index1 > index2
        if (index1 < 0 || index2 < 0 || index1 >= scan_msg.ranges.size() || index2 >= scan_msg.ranges.size() || index1 >= index2) {
            ROS_WARN("PropInProgress message range indexes are out of bounds for the given scan message");
            return;
        }

        //create a 2D vector containing distance angle pairs for points detected by lidar within the range provided by yolo
               //starting angle for lidar scan 
        ROS_DEBUG_STREAM("Laser angle min" << laser_angle_min);
        ROS_DEBUG_STREAM("Laser angle increment" << laser_angle_increment);
        double starting_angle = laser_angle_min + (M_PI/2.0);
        std::vector<lidarPoint> scanPoints = CoordFinder::createLidarPoints(scan_msg.ranges, starting_angle, laser_angle_increment);
        if (scanPoints.size()<1){
            ROS_WARN("No points added to scanPoints vector");
            return;
        }


        //create a smaller vector of only points within the camera provided range
        std::vector<lidarPoint> selectedPoints;
        for (int i = index1; i <= index2; i++) {

            selectedPoints.push_back(scanPoints[i]);
            ROS_DEBUG_STREAM("Pushing back points within camera range: " << scanPoints[i]);
        }
        if (selectedPoints.size()<1){
            ROS_WARN("No points added to vector containing points within camera range ");
            return;
        }

        // find the distance from the center of closest point and angle within the given range

        int i = 0;
        double closest_distance = selectedPoints[i].getDistance();
        double closest_angle = selectedPoints[i].getAngle();
        for (int i = 0; i < selectedPoints.size(); ++i) {
            if (std::isnan(selectedPoints[i].getDistance())) {
                continue; 
            }
            if(selectedPoints[i].getDistance() < closest_distance){
                closest_distance = selectedPoints[i].getDistance(); 
                closest_angle = selectedPoints[i].getAngle();       
            }
        }
        ROS_DEBUG_STREAM("closest_distance " << closest_distance);
        ROS_DEBUG_STREAM("closest angle " << closest_angle);

        geometry_msgs::Vector3 prop_coords_msg;
        prop_coords_msg.x = closest_distance*sin(closest_angle); //North
        prop_coords_msg.y = closest_distance*cos(closest_angle); //East 
        prop_coords_msg.z = 0; //Down
        pub_prop_closest_.publish(prop_coords_msg);
    }


    static std::vector<lidarPoint> createLidarPoints(const std::vector<float>& distances, double startAngle, double angleIncrement) {
        std::vector<lidarPoint> lidarPoints;
        ROS_DEBUG_STREAM("start angle: " << startAngle);
        // Add the first Lidar point
        lidarPoint firstPoint(distances[0], startAngle);
        lidarPoints.push_back(firstPoint);

        // Add the remaining Lidar points
        double currentAngle = startAngle + angleIncrement;
        for (size_t i = 1; i < distances.size(); i++) {
            double distance = distances[i];
            lidarPoint point(distance, currentAngle);
            lidarPoints.push_back(point);

            currentAngle += angleIncrement;
        }

    return lidarPoints;
    }
  

};
int main(int argc, char** argv) {
    ros::init(argc, argv, "coord_finder");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();
    CoordFinder coord_finder;

    coord_finder.spin();
    return 0;
}