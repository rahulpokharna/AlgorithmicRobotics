// include appropriate things
#include <turtlebot_tinyslam/turtlebot_tinyslam.h>
#include <stdint.h>
#include <math.h>
#include <cmath>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>


// variables for public use
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
int nb_points = 0;
float32[] ranges_;

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
        
        nb_points = (int) ((angle_max_ - angle_min_) / angle_increment_);
        ranges_ = laser_scan.ranges;
        // store intensities somewhere
	}
	
	// decide if we want to store the scan based on a timer, always update, or only when we ask for it
	// maybe check if the values are the same/similar to the previous, 
	// and only call the slam if they are sufficiently different
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
	ROS_INFO("dom: [%f,%f]", odom->twist.twist.linear.x, odom->pose.pose.position.y);

	//double x_position = odom.pose.pose.position.x;
	//double y_position = odom.pose.pose.position.y;
}

// on init, get values for laser scan, calculate max min, nb_points etc


// define a main
// in main, get laser scan, then convert it to x,y values iterating along each scan.
int main(int argc, char ** argv){
	ros::init(argc, argv, "tiny_node");
	ros::NodeHandle n;
	
	ros::Subscriber lidar_subscriber = n.subscribe("/scan", 1, laserCallback);
	ros::Subscriber odom_subscriber = n.subscribe("/odom", 1000, odomCallback);
	
	
	// in while(ros::ok()){
	// have a sleep timer that periodically calls for updates, or check for new data

}
