#ifndef _TINYSLAM_H_
#define _TINYSLAM_H_

#ifndef M_PI
#define M_PI 3.141592653589793238466
#endif

#define TS_SCAN_SIZE 639 //8192
#define TS_MAP_SIZE 4096  //2048
#define TS_MAP_SCALE 0.1
#define TS_DISTANCE_NO_DETECTION 10000
#define TS_NO_OBSTACLE 65500
#define TS_OBSTACLE 0
#define TS_HOLE_WIDTH 600

#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "ros/ros.h"
#include <vector>
#include <set>
#include <math.h>

typedef signed char ts_map_pixel_t; //int8_t

typedef struct {
	// ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE ];
	// std::vector<int> map(TS_MAP_SIZE * TS_MAP_SIZE);
	std::vector<signed char> map; 
} ts_map_t;

typedef struct {
	double x[TS_SCAN_SIZE], y[TS_SCAN_SIZE];
	int value[TS_SCAN_SIZE];
	int nb_points;
} ts_scan_t;

typedef struct{
	double x, y;	//in mm
	double theta;	//in degrees
} ts_position_t;

///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////

void ts_map_init (ts_map_t &map ) ;
int ts_distance_scan_to_map(ts_scan_t &scan, ts_map_t &map, ts_position_t &pos);
void ts_map_update(ts_scan_t &scan , ts_map_t &map, ts_position_t &position, int quality);

#endif //_TINYSLAM_H_
