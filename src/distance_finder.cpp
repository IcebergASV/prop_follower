#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <navigation_pkg/PropInProgress.h>
#include <cmath>
#include <vector>
#include <stdexcept>
#include "lidarPoint.h"
#include <string>
#include <iostream>
#include <ros/console.h>

class DistanceFinder {
public:
    DistanceFinder() : nh_(""), private_nh_("~") {
        // get ROS parameters
        private_nh_.param<std::string>("prop_topic", prop_topic_, "/prop_angle_range");
        private_nh_.param<std::string>("scan_topic", scan_topic_, "/scan");
        private_nh_.param<double>("max_range", max_range_, 10.0);


        sub_scan_ = nh_.subscribe(scan_topic_, 1, &DistanceFinder::scanCallback, this);
        sub_prop_ = nh_.subscribe(prop_topic_, 1, &DistanceFinder::propCallback, this);
        pub_prop_closest_ = nh_.advertise<navigation_pkg::PropInProgress>("/prop_closest_point", 1);
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
    navigation_pkg::PropInProgress prop_msg_;
    sensor_msgs::LaserScan scan_msg;

    void propCallback(const navigation_pkg::PropInProgress::ConstPtr& msg) {
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
        if (prop_msg_.prop_type.empty()) {
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
        ROS_DEBUG_STREAM("Distance Finder: index1 :" << index1 << " index2: " << index2);
        ROS_DEBUG_STREAM("Distance Finder: size of scan message ranges " << scan_msg.ranges.size());
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
        std::vector<lidarPoint> scanPoints = DistanceFinder::createLidarPoints(scan_msg.ranges, starting_angle, laser_angle_increment);
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

        //filter out points not likely to be part of the marker
        std::vector<lidarPoint> filteredPoints = filterPoints(selectedPoints, marker_diameter_for_filtering);
        ROS_DEBUG_STREAM("filteredPoints Vector created");
        if (filteredPoints.size()<1){
            ROS_WARN("No points passed through filter");
            return;
        }
        // Iterate through the vector and print each element
        for (const auto& point : filteredPoints) {
            ROS_DEBUG_STREAM("Filtered Points: " << point);
            
        }


        //calculate the radius of the prop
        double radius = calculateRadius(filteredPoints);
        std::string radius_str = std::to_string(radius);
        ROS_DEBUG_STREAM("calculated radius" << radius_str);
        //Could add in check here to exclude the prop if the measured radius doesn't match expected radius

        // find the distance from the center of closest point and angle within the given range

        int i = 0;
        double closest_distance = filteredPoints[i].getDistance();
        double closest_angle = filteredPoints[i].getAngle();
        for (int i = 0; i < filteredPoints.size(); ++i) {
            if (std::isnan(filteredPoints[i].getDistance())) {
                continue; 
            }
            if(filteredPoints[i].getDistance() < closest_distance){
                closest_distance = filteredPoints[i].getDistance(); 
                closest_angle = filteredPoints[i].getAngle();       
            }
        }
        ROS_DEBUG_STREAM("closest_distance " << closest_distance);
        ROS_DEBUG_STREAM("closest angle " << closest_angle);

        navigation_pkg::PropInProgress closest_prop_msg;
        closest_prop_msg.prop_type = prop_msg_.prop_type;
        closest_prop_msg.theta_1 = prop_msg_.theta_1;
        closest_prop_msg.theta_2 = prop_msg_.theta_2;
        closest_prop_msg.closest_pnt_dist = closest_distance;
        closest_prop_msg.closest_pnt_angle = closest_angle;
        pub_prop_closest_.publish(closest_prop_msg);
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

    std::vector<int> smallerVector(const std::vector<int>& largerVector, int startIndex, int endIndex) {
 
        if (startIndex > endIndex) {
            throw std::invalid_argument("Start index cannot be greater than end index");
        }

        std::vector<int> smallerVector;

        // Add the values from startIndex to endIndex (inclusive) to the smaller vector
        for (int i = startIndex; i <= endIndex; i++) {
            smallerVector.push_back(largerVector[i]);
        }

    return smallerVector;
    }

    std::vector<lidarPoint> filterPoints(const std::vector<lidarPoint>& inputVector, double range) {
        //removes points that dont belong to prop by removing points not close to the closest detected point
        //note - this is no good if the closest detected point is not a part of the marker
        if (inputVector.size()<1){
            ROS_WARN("No points passed into filter");
        }
        std::vector<lidarPoint> outputVector;

        double smallest_distance = max_lidar_range;
        ROS_DEBUG_STREAM("max_lidar_range" << max_lidar_range);
        ROS_DEBUG_STREAM("inputVector.size " << inputVector.size());
        for (lidarPoint point : inputVector) {

            if (std::isnan(point.getDistance())) {
                continue; 
                ROS_WARN("no distance in point");
            }
            if(point.getDistance() < smallest_distance){
                smallest_distance = point.getDistance();
                ROS_DEBUG_STREAM("Changing smallest distance to : " << smallest_distance);       
            }
            else {
                ROS_DEBUG_STREAM("smallest_distance : " << smallest_distance << " inputVector[i].getDISTANCE : " << point.getDistance());
            }
        }

        // Add points to the output vector if their distance is within the range
        for (lidarPoint point : inputVector) {
            if ((point.getDistance() >= smallest_distance)  && (point.getDistance() <= (smallest_distance + range)) && (point.getDistance() < max_lidar_range) ) {
                outputVector.push_back(point);
                //ROS_DEBUG_STREAM("Just pushed back: " << point << " Into filtered Points Vector");
            }
        }

    return outputVector;
    }
    
    double calculateRadius(const std::vector<lidarPoint>& points) {
        // Check that we have at least 6 points
        //if (points.size() < 4) {
        //    throw std::runtime_error("At least 6 points are required to calculate the radius of a cylinder.");
        //}

        // Convert angles to x,y coordinates on a unit circle
        std::vector<double> x_coords(points.size());
        std::vector<double> y_coords(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            x_coords[i] = points[i].getDistance() * std::cos(points[i].getAngle());
            y_coords[i] = points[i].getDistance() * std::sin(points[i].getAngle());
            ROS_DEBUG_STREAM("xy converted points: " << x_coords[i] << ", " << y_coords[i]);
        }


        //insert radius code here
        return 0.03;//temporary
 


        //ROS_DEBUG_STREAM("Marker radius: " << radius);
        //return radius;
    }

};
int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_finder_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();
    DistanceFinder distance_finder;

    distance_finder.spin();
    return 0;
}