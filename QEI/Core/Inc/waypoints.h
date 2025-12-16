/*
 * waypoints.h
 *
 *  Created on: Jun 4, 2025
 *      Author: ioonz
 */

#ifndef INC_WAYPOINTS_H_
#define INC_WAYPOINTS_H_
#define MAX_WAYPOINT 200

#include "main.h"
#include "arm_math.h"
#include <math.h>



typedef struct {
	char cmd[4];      // Command (G0, G1, etc.)
	float x;          // X coordinate
	float y;          // Y coordinate
	float z;          // Z coordinate
	float feedrate;   // Feedrate (F)
} WAYPOINT;

//float calculateWaypoints(PrismaticControlTypeHandler *Prismatic,
//		RevoluteControlTypeHandler *Revolute,
//		WAYPOINT *points, int num_points);


#endif /* INC_WAYPOINTS_H_ */
