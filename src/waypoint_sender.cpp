#include <ros/ros.h>
#include <cmath>
#include "lidarPoint.h"
#include <string>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <gnc_functions.hpp> // Intelligent Quads mavros API
#include <ros/console.h>

std::string TAG = "WP_SENDER: ";
class WPSender {
public:
    WPSender() : nh_(""), private_nh_("~")
    {
        prop_local_coords_sub_ = nh_.subscribe("/prop_local_coords", 1, &WPSender::wpCallback, this);
        setpoint_waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
        mavros_state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &WPSender::mavrosStateCallback, this);
        current_pos_sub_ = nh_.subscribe("/mavros/global_position/local", 10, &WPSender::poseCallback, this);
        arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

    //set orientation of the drone (drone should always be level) 
    // Heading input should match the ENU coordinate system
    /**
    \ingroup control_functions
    This function is used to specify the drone’s heading in the local reference frame. Psi is a counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone. 
    @returns n/a
    */
    void set_heading(float heading)
    {
      local_desired_heading_g = heading; 
      heading = heading + correction_heading_g + local_offset_g;
    
      ROS_INFO_STREAM(TAG << "Desired Heading " << local_desired_heading_g);
      float yaw = heading*(M_PI/180);
      float pitch = 0;
      float roll = 0;

      float cy = cos(yaw * 0.5);
      float sy = sin(yaw * 0.5);
      float cr = cos(roll * 0.5);
      float sr = sin(roll * 0.5);
      float cp = cos(pitch * 0.5);
      float sp = sin(pitch * 0.5);

      float qw = cy * cr * cp + sy * sr * sp;
      float qx = cy * sr * cp - sy * cr * sp;
      float qy = cy * cr * sp + sy * sr * cp;
      float qz = sy * cr * cp - cy * sr * sp;

      waypoint_.pose.orientation.w = qw;
      waypoint_.pose.orientation.x = qx;
      waypoint_.pose.orientation.y = qy;
      waypoint_.pose.orientation.z = qz;
    }
    // set position to fly to in the local frame
    /**
    \ingroup control_functions
    This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone. 
    @returns n/a
    */
    void set_destination(float x, float y, float z, float psi)
    {
    	set_heading(psi);
    	//transform map to local
    	float deg2rad = (M_PI/180);
    	float Xlocal = x*cos((correction_heading_g + local_offset_g - 90)*deg2rad) - y*sin((correction_heading_g + local_offset_g - 90)*deg2rad);
    	float Ylocal = x*sin((correction_heading_g + local_offset_g - 90)*deg2rad) + y*cos((correction_heading_g + local_offset_g - 90)*deg2rad);
    	float Zlocal = z;

    	x = Xlocal + correction_vector_g.position.x + local_offset_pose_g.x;
    	y = Ylocal + correction_vector_g.position.y + local_offset_pose_g.y;
    	z = Zlocal + correction_vector_g.position.z + local_offset_pose_g.z;
    	ROS_INFO_STREAM(TAG << "Destination set to x " << x << "y: " << y << "z: " << z );

    	waypoint_.pose.position.x = x;
    	waypoint_.pose.position.y = y;
    	waypoint_.pose.position.z = z;

    	setpoint_waypoint_pub_.publish(waypoint_);
    
    }
    /**
    Wait for connect is a function that will hold the program until communication with the FCU is established.
    @returns 0 - connected to fcu 
    @returns -1 - failed to connect to drone
    */
    int wait4connect()
    {
    	ROS_INFO_STREAM(TAG << "Waiting for FCU connection");
    	// wait for FCU connection
    	while (ros::ok() && !current_state_.connected)
    	{
            ROS_DEBUG_STREAM(TAG << "Inside wait for connect loop");
    		ros::spinOnce(); //THIS LINE CAUSES MAVPROXY CONNECTION TO CLOSE BUT I NEED IT TO UPDATE STATE
    		ros::Duration(0.01).sleep();
    	}
    	if(current_state_.connected)
    	{
    		ROS_INFO_STREAM(TAG << "Connected to FCU");	
    		return 0;
    	}else{
    		ROS_INFO_STREAM(TAG << "Error connecting to drone");
    		return -1;	
    	}
    
        ROS_DEBUG_STREAM(TAG << " End of wait4connect function");
    }

    /**
    Wait for strat will hold the program until the user signals the FCU to enther mode guided. This is typically done from a switch on the safety pilot’s remote or from the ground control station.
    @returns 0 - mission started
    @returns -1 - failed to start mission
    */
    int wait4start()
    {
    	ROS_INFO_STREAM(TAG << "Waiting for user to set mode to GUIDED");
    	while(ros::ok() && current_state_.mode != "GUIDED")
    	{
    	    ros::spinOnce();
    	    ros::Duration(0.01).sleep();

            ROS_DEBUG_STREAM(TAG << "Stuck in wiat4start loop");
      	}
        ROS_DEBUG_STREAM(TAG << "exited while loop");
      	if(current_state_.mode == "GUIDED")
    	{
    		ROS_INFO_STREAM(TAG << "Mode set to GUIDED. Mission starting");
    		return 0;
    	}else{
    		ROS_INFO_STREAM(TAG << "Error starting mission!!");
    		return -1;	
    	}

        ROS_DEBUG_STREAM(TAG << "End of wait4start function");
    }

    /**
    This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to.
    @returns 0 - frame initialized
    */
    int initialize_local_frame()
    {
    	//set the orientation of the local reference frame
    	ROS_INFO_STREAM(TAG << "Initializing local coordinate system");
    	local_offset_g = 0;
    	for (int i = 1; i <= 30; i++) {
    		ros::spinOnce();
    		ros::Duration(0.1).sleep();

    

    		float q0 = current_pose_g.pose.pose.orientation.w;
    		float q1 = current_pose_g.pose.pose.orientation.x;
    		float q2 = current_pose_g.pose.pose.orientation.y;
    		float q3 = current_pose_g.pose.pose.orientation.z;
    		float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) ); // yaw

    		local_offset_g += psi*(180/M_PI);

    		local_offset_pose_g.x = local_offset_pose_g.x + current_pose_g.pose.pose.position.x;
    		local_offset_pose_g.y = local_offset_pose_g.y + current_pose_g.pose.pose.position.y;
    		local_offset_pose_g.z = local_offset_pose_g.z + current_pose_g.pose.pose.position.z;
    		// ROS_INFO(TAG, "current heading%d: %f", i, local_offset_g/i);
    	}
    	local_offset_pose_g.x = local_offset_pose_g.x/30;
    	local_offset_pose_g.y = local_offset_pose_g.y/30;
    	local_offset_pose_g.z = local_offset_pose_g.z/30;
    	local_offset_g /= 30;
    	ROS_INFO_STREAM(TAG << "Coordinate offset set");
    	ROS_INFO_STREAM(TAG << "the X' axis is facing: " << local_offset_g);
    	return 0;
    }

    int arm()
    {
    	//intitialize first waypoint of mission
    	set_destination(0,0,0,0);
    	for(int i=0; i<100; i++)
    	{
            ROS_DEBUG_STREAM(TAG << "before publishing wp to arm");
    		setpoint_waypoint_pub_.publish(waypoint_);
            ROS_DEBUG_STREAM(TAG << "after publishing wp to arm");

    		ros::spinOnce();
    		ros::Duration(0.01).sleep();
            ROS_DEBUG_STREAM(TAG << "bottom of loop");
    	}
    	// arming
    	ROS_INFO_STREAM(TAG << "Arming drone");
    	mavros_msgs::CommandBool arm_request;
    	arm_request.request.value = true;
    	while (!current_state_.armed && !arm_request.response.success && ros::ok())
    	{
    		ros::Duration(.1).sleep();
    		arming_client.call(arm_request);
    		setpoint_waypoint_pub_.publish(waypoint_);
    	}
    	if(arm_request.response.success)
    	{
    		ROS_INFO_STREAM(TAG << "Arming Successful");	
    		return 0;
    	}else{
    		ROS_INFO_STREAM(TAG << "Arming failed with " << arm_request.response.success);
    		return -1;	
    	}
    }

private:
    float current_heading_;
    float local_offset_;

    void wpCallback(const geometry_msgs::Vector3::ConstPtr& vector_msg)
    {
        //geometry_msgs::PoseStamped waypoint;
        //geometry_msgs::Vector3 vector_msg = *msg;
        //waypoint.pose.position.x = vector_msg->x;
        //waypoint.pose.position.y = vector_msg->y;
        //waypoint.pose.position.z = vector_msg->z;
        //waypoint.pose.orientation.w = 0.5;
        //waypoint.pose.orientation.x = 0.5;
        //waypoint.pose.orientation.y = 0.5;
        //waypoint.pose.orientation.z = 0.5;
//
//
        //setpoint_velocity_pub_.publish(waypoint);

        if (current_state_.armed){
            ROS_DEBUG_STREAM( TAG << "wpCAllback is armed");
            set_destination(vector_msg->x, vector_msg->y, vector_msg->z, 0);
            //set_destination(4,6,0,0);
        }
            

    }

    void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg)
    {
        current_state_ = *msg;
    }

    ros::NodeHandle nh_;
    ros::Subscriber prop_local_coords_sub_;
    ros::Subscriber current_pos_sub_;
    ros::Subscriber mavros_state_sub_;
    ros::Publisher setpoint_waypoint_pub_;
    ros::NodeHandle private_nh_;
    ros::ServiceClient arming_client;

    mavros_msgs::State current_state_;
    nav_msgs::Odometry current_pos_;
    geometry_msgs::PoseStamped waypoint_;

    //get current position of drone
    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      current_pos_ = *msg;
      enu_2_local(current_pos_);
      float q0 = current_pos_.pose.pose.orientation.w;
      float q1 = current_pos_.pose.pose.orientation.x;
      float q2 = current_pos_.pose.pose.orientation.y;
      float q3 = current_pos_.pose.pose.orientation.z;
      float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
      //ROS_INFO(TAG, "Current Heading %f ENU", psi*(180/M_PI));
      //Heading is in ENU
      //IS YAWING COUNTERCLOCKWISE POSITIVE?
      current_heading_ = psi*(180/M_PI) - local_offset_g;
      //ROS_INFO(TAG, "Current Heading %f origin", current_heading_g);
      //ROS_INFO(TAG, "x: %f y: %f z: %f", current_pose_g.pose.pose.position.x, current_pose_g.pose.pose.position.y, current_pose_g.pose.pose.position.z);
    }





};


int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "wp_sender_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();
    WPSender wp_sender;

    // wait for FCU connection
    ROS_WARN_STREAM(TAG << "Test WARN Message");
    wp_sender.wait4connect();
    ROS_INFO_STREAM(TAG << "Test INFO Message");
	//wait for user to switch to mode GUIDED
	wp_sender.wait4start();
    ROS_DEBUG_STREAM(TAG << "Test DEBUG Message");
	//create local reference frame 
	wp_sender.initialize_local_frame();

	// arm boat 
	wp_sender.arm();

    wp_sender.spin();
    return 0;

}
