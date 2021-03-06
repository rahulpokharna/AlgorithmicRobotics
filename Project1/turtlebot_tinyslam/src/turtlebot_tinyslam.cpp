#include <turtlebot_tinyslam/turtlebot_tinyslam.h>
#include <stdint.h>
#include <math.h>
#include <cmath>

// Any other includes

int ts_distance_scan_to_map(ts_scan_t &scan, ts_map_t &map, ts_position_t &pos) 
{
	double c, s;
	int i, x, y, nb_points = 0;
	int64_t sum;
	
	
	
	c = cos(pos.theta * M_PI / 180);
	s = sin(pos.theta * M_PI / 180);

	//Translate and rotate scan to robot position and compute the distance

	for(i = 0, sum = 0; i != scan.nb_points; i++) {
		if(scan.value[i] != TS_NO_OBSTACLE) {
			x = (int) floor((pos.x + c * scan.x[i] - s * scan.y[i]) * TS_MAP_SCALE + 0.5);
			y = (int) floor((pos.y + s * scan.x[i] + c * scan.y[i]) * TS_MAP_SCALE + 0.5);
			//Check boundaries
			if(x >= 0 && x < TS_MAP_SIZE && y >= 0 && y < TS_MAP_SIZE) {
				sum += map.map[y * TS_MAP_SIZE + x];
				nb_points++;
			}
		}
	}
	if (nb_points) sum = sum *1024 / nb_points;
	else sum = 2000000000;
	return (int) sum;
}

void ts_map_init(ts_map_t &map)
{
	std::vector<signed char> map2(TS_MAP_SIZE * TS_MAP_SIZE);
	map.map = map2;
	
	int x, y, initval;
	// ts_map_pixel_t *ptr;
	initval = (TS_OBSTACLE + TS_NO_OBSTACLE) / 2;
	for( y = 0; y < TS_MAP_SIZE; y++) { //ptr = map.map,
		for(x = 0; x < TS_MAP_SIZE; x++) { // , ptr++
			// *ptr = initval;
			map.map[x + y*TS_MAP_SIZE] = initval;
		}
	}
}

#define SWAP(x, y) (x^= y ^= x ^= y)

void ts_map_laser_ray(ts_map_t &map, int x1, int y1, int x2, int y2, int xp, int yp, int value, int alpha) {
	int x2c, y2c, dx, dy, dxc, dyc, error, errorv, derrorv, x;
	int incv, sincv, incerrorv, incptrx, incptry, pixval, horiz, diago;
	
	int ptr; 
	if( x1 < 0 || x1 >= TS_MAP_SIZE || y1 < 0 || y1 >= TS_MAP_SIZE)
		return; // Robot is out of map
	
	x2c = x2; 
	y2c = y2; // Clipping
	
	if(x2c < 0) {
		if(x2c == x1) return;
		y2c += (y2c-y1)*(-x2c) / (x2c-x1);
		x2c = 0;
	}
	if( x2c >= TS_MAP_SIZE) {
		if(x1 == x2c) return;
		y2c += (y2c-y1)*(TS_MAP_SIZE-1-x2c) / (x2c-x1);
		x2c = TS_MAP_SIZE-1;
	}
	if(y2c < 0) {
		if(y1 == y2c) return;
		x2c += (x1-x2c)*(-y2c) / (y1-y2c);
		y2c = 0;
	}
	if( y2c >= TS_MAP_SIZE) {
		if(y1 == y2c) return;
		x2c += (x1-x2c)*(TS_MAP_SIZE-1-y2c) / (y1-y2c);
		y2c = TS_MAP_SIZE-1;
	}
	dx = std::abs(x2-x1);
	dy = std::abs(y2-y1);
	dxc = std::abs(x2c-x1);
	dyc = std::abs(y2c-y1);
	incptrx = (x2 > x1) ? 1 : -1;
	incptry = (y2 > y1) ? TS_MAP_SIZE : -TS_MAP_SIZE;
	sincv = (value > TS_NO_OBSTACLE) ? 1 : -1;
	if(dx > dy) {
		derrorv = std::abs(xp-x2);
	}else{
		SWAP(dx, dy); 
		SWAP(dxc, dyc); 
		SWAP(incptrx, incptry);
		derrorv = std::abs(yp-y2);
	}
	error = 2*dyc-dxc;
	horiz = 2*dyc;
	diago = 2*(dyc-dxc);
	errorv = derrorv / 2;
	incv = (value-TS_NO_OBSTACLE) /derrorv;
	incerrorv = value-TS_NO_OBSTACLE-derrorv*incv;
	ptr = y1*TS_MAP_SIZE + x1; // equivalent to ptr is looking at map.map[y1*TS_MAP_SIZE + x1]
	pixval = TS_NO_OBSTACLE;
	for(x = 0; x <= dxc; x++, ptr += incptrx) { //  at each round, increment how far in by 
		if(x > dx-2*derrorv) {
			if(x <= dx-derrorv) {
				pixval += incv;
				errorv += incerrorv;
				if(errorv > derrorv) {
					pixval += sincv;
					errorv-= derrorv;
				}
			}
			else{
				pixval-= incv;
				errorv-= incerrorv;
				if(errorv < 0) {
					pixval-= sincv;
					errorv += derrorv;
				}
			}
		}
		
		// update pixel value at x y
		// Integration into the map
		map.map[ptr] = ((256-alpha)*(map.map[ptr]) + alpha*pixval) >> 8;
		if(error > 0) {
			ptr += incptry;
			error += diago;
		} else error += horiz;
	}
}


void ts_map_update(ts_scan_t &scan, ts_map_t &map, ts_position_t &pos,int quality)
{
	double c, s, q;
	double x2p, y2p;
	int i, x1, y1, x2, y2, xp, yp, value;
	double add, dist;
	c=cos(pos.theta*M_PI / 180);
	s=sin(pos.theta*M_PI / 180);
	x1 = (int)floor(pos.x*TS_MAP_SCALE +0.5);
	y1 = (int)floor(pos.y*TS_MAP_SCALE +0.5);
	// Translate and rotate scan to robot position
	
	ROS_INFO("X, Y of robot: %f, %f", pos.x, pos.y);
	for( i = 0; i != scan.nb_points; i++) {
		
		// this changes the entire way that it is processed, it is all done ahead of time
		x2p = scan.x[i]; // c*scan.x [i] - s*scan.y [i];
		y2p = scan.y[i]; //s*scan.x [i] + c*scan.y [i];
		
		xp = (int)floor((pos.x + x2p)*TS_MAP_SCALE + 0.5);
		yp = (int)floor((pos.y + y2p)*TS_MAP_SCALE + 0.5);
		
		// alternative way to get xp, yp, this fix helped
		// xp = (int)floor((pos.x + scan.x[i])*TS_MAP_SCALE + 0.5);
		// yp = (int)floor((pos.y + scan.y[i])*TS_MAP_SCALE + 0.5);
		
		
		dist = sqrt(x2p*x2p + y2p*y2p);
		add = TS_HOLE_WIDTH / 2 / dist;
		x2p *= TS_MAP_SCALE*(1 + add);
		y2p *= TS_MAP_SCALE*(1 + add);
		x2 = (int)floor(pos.x*TS_MAP_SCALE + x2p + 0.5);
		y2 = (int)floor(pos.y*TS_MAP_SCALE + y2p + 0.5);
		
		// alternative calculation for xy, y2, this did not help as it is the same
		//x2 = (int)floor(pos.x*TS_MAP_SCALE + x2p + 0.5);
		//y2 = (int)floor(pos.y*TS_MAP_SCALE + y2p + 0.5);
		
		
		if(scan.value[i] == TS_NO_OBSTACLE) {
			q = quality / 2; 
			value = TS_NO_OBSTACLE;
		} else {
			q = quality;
			value = TS_OBSTACLE;
		}
		ts_map_laser_ray(map, x1, y1, x2, y2, xp, yp, value, q);
	}
}
