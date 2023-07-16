#include <ros/ros.h>
#include <prop_follower/PropAngleRange.h>
#include <cmath>
#include <ros/console.h>
#include <string>



class AngleFinder {
public:
    AngleFinder() : nh_(""), private_nh_("~") 
    {
        yolo_sub_ = nh_.subscribe("/bounding_boxes", 1, &AngleFinder::yoloCallback, this);
        prop_pub_ = nh_.advertise<prop_follower::PropAngleRange>("/prop_angle_range", 1);
        private_nh_.param<double>("realsense_fov", realsense_fov, 0.0);
        private_nh_.param<int>("realsense_res_x", realsense_res_x, 0);
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:

    void yoloCallback(const prop_follower::BoundingBoxes::ConstPtr& msg)
    {
        //prop_follower::BoundingBoxes boxes = msg;
        for (prop_follower::BoundingBox box : msg->bounding_boxes){
            ROS_DEBUG_STREAM(TAG << "Received BoundingBox message with xmin = " << box.xmin
            << " and xmax = " << box.xmax);

            // Make sure bounding box message is valid
            if (box.xmax >= box.xmin || box.xmin < 0 || box.xmax > realsense_res_x){
                ROS_WARN(TAG, "Invalid bounding box");
            }

            //get the position of the bounding box
            x_min = box.xmin;
            x_max = box.xmax;

            // Calculate the angle range for the prop
            double theta_right = fov_end - ((box.xmax / realsense_res_x) * realsense_fov); 
            double theta_left = fov_end - ((box.xmin / realsense_res_x) * realsense_fov);

            // Create and publish the Prop message with the prop coordinates
            prop_follower::PropInProgress prop_msg;
            prop_msg.prop_type = box.label; //assign object classification label to the prop
            prop_msg.theta_small = theta_right;
            prop_msg.theta_small = theta_left; 

            ROS_DEBUG_STREAM(TAG << "Publishing prop_msg with theta_small = " << prop_msg.theta_small  << " and theta_large =" << prop_msg.theta_small);
            prop_pub_.publish(prop_msg);
        }

    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber yolo_sub_;
    ros::Publisher prop_pub_;
    double x_min;
    double x_max;
    double realsense_fov;
    double fov_end = (M_PI / 2) + (realsense_fov / 2 );
    int realsense_res_x;
    std::string TAG = "ANGLE_FINDER: ";
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "angle_finder");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();
    AngleFinder angle_finder;
    angle_finder.spin();
    return 0;
}