// include appropriate things
#include <ros/ros.h>
#include <turtlebot_tinyslam/turtlebot_tinyslam.h>
#include "turtlebot_tinyslam.cpp"
#include <stdint.h>
#include <math.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"


// variables for public use
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_ = 0.0;
double angle_max_ = 0.0;
double angle_increment_ = 0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
int nb_points_ = 0;
std::vector<float> ranges_;

double x_odom_, y_odom_;
double theta_odom_; //radian
double theta_odom_deg_; //degree


// Structs for tinySLAM, initialized to 0,0,0
//ts_position_t pos = {0, 0, 0};
ts_position_t pos_; //  = pos;
ts_scan_t scan_; 
ts_map_t map_;

 
// TODO input all values and begin testing -- 


// Figure out how to display the map, publish it as a topic
// Top prioirity ^^^^^^^^^^^^^^^^^^^^^^^^^^^

// TODO Figure out how to use the localization


// callbacks for subscriptions
void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        ROS_INFO("angle_min_ = %f", angle_min_);
        ROS_INFO("angle_increment_ = %f", angle_increment_);
        ROS_INFO("LIDAR setup: ping_index = %d",ping_index_);
        
        nb_points_ = (int) ((angle_max_ - angle_min_) / angle_increment_);
        ranges_ = laser_scan.ranges;
        // store intensities somewhere
        
        scan_.nb_points = nb_points_;
        
        
	}
	
	// decide if we want to store the scan based on a timer, always update, or only when we ask for it
	// maybe check if the values are the same/similar to the previous, 
	// and only call the slam if they are sufficiently different
	
	
	// First compare value of new set of ranges to old set
	// Then, once compared, convert the new set of ranges into x,y, values and store into the struct scan_
	
	// Code to convert to a struct
	
	for(int i = 0; i < nb_points_; i++){
		double angle_in_rad = angle_min_ + (i * angle_increment_);
		
		if(isnan(laser_scan.ranges[i])){ // check if is nan, maybe need to check inf?
			scan_.value[i] = TS_NO_OBSTACLE;
		}
		else{
			scan_.value[i] = TS_OBSTACLE;
			scan_.x[i] = x_odom_ + (laser_scan.ranges[i] * cos(angle_in_rad));
			scan_.y[i] = y_odom_ + (laser_scan.ranges[i] * sin(angle_in_rad)); 
		}
		// Unsure if the x,y are in relation to the robot or the world, I will assume the world for now
		
	}
	// update the map with the new scan
	ts_map_update(scan_, map_, pos_, 1);
	
}

void odomCallback(const nav_msgs::Odometry& odom) {
	//ROS_INFO("odom: [%f,%f]", odom.twist.twist.linear.x, odom.pose.pose.position.y);

	x_odom_ = odom.pose.pose.position.x;
	y_odom_ = odom.pose.pose.position.y;
	
	//ROS_INFO("x position: %d", x_odom_);
	///ROS_INFO("y position: %d", y_odom_);
	
	// convert quat to theta in degrees 
	double x = odom.pose.pose.orientation.x;
	double y = odom.pose.pose.orientation.y;
	double z = odom.pose.pose.orientation.z;
	double w = odom.pose.pose.orientation.w;
	
	double siny_cosp = +2.0 * (w * z + x * y);
	double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);  
	
	//ROS_INFO("x orientation: %f", x);
	//ROS_INFO("y orientation: %f", y);
	//ROS_INFO("z orientation: %f", z);
	//ROS_INFO("w orientation: %f", w);
	
    theta_odom_ = atan2(siny_cosp, cosy_cosp); // from -PI to PI in radian
    //ROS_INFO("Theta from odom: %f", theta_odom_); 
    
    theta_odom_deg_ = (theta_odom_ * 180 / M_PI);
   	//ROS_INFO("Theta from odom degree: %f", theta_odom_deg_); 
	
    //ROS_INFO("siny_cosp %d", siny_cosp);
    //ROS_INFO("cosy_cosp %d", cosy_cosp);
    
    // Update value of the position struct
	// pos_ = {x_odom_, y_odom_, theta_odom_deg_}; 
    pos_.x = x_odom_;
    pos_.y = y_odom_;
    pos_.theta = theta_odom_deg_;
}

// on init, get values for laser scan, calculate max min, nb_points etc


// define a main
// in main, get laser scan, then convert it to x,y values iterating along each scan.
int main(int argc, char** argv){
	ros::init(argc, argv, "tiny_node");
	ros::NodeHandle n;
	
	 //sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
	ros::Publisher map_meta_publisher = n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
	ros::Publisher occupancy_publsiher = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
	
	ros::Subscriber odom_subscriber = n.subscribe("/odom", 1000, odomCallback);
	ros::spinOnce();
	ros::Subscriber lidar_subscriber = n.subscribe("/scan", 1, laserCallback);
	ROS_WARN("BEFORE MAP");
	ros::Duration(1).sleep();
	ts_map_init(map_);
	ROS_WARN("AFTER MAP");
	// in while(ros::ok()){
	// have a sleep timer that periodically calls for updates, or check for new data
	ros::spin();

}
