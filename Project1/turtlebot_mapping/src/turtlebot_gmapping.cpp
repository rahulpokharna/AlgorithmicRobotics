#include "turtlebot_mapping.h"
#include <iostream>
#include <time.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/MapMetaData.h>


// read laser scans and odometry to compute a map
