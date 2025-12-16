/*
 * TrajectoryGenerator.h
 *
 *  Created on: May 8, 2025
 *      Author: Kaitniyom
 */

#ifndef INC_TRAJECTORYGENERATOR_H_
#define INC_TRAJECTORYGENERATOR_H_

#include "stm32g4xx_hal.h"
#include "math.h"
#include "stdio.h"
#include "stdbool.h"

typedef enum {Current ,Previous,Prior};

typedef struct
{
	float ang_pos;
	float ang_velo;
	float ang_acc;
	float desire_av;
	float desire_ac;
	float dir;

	float Init_pos;
	float Init_Velo;

	float t_b; //desired t for accele phase
	float t_c; //t for constant phase
	float t_f; // t for deaccele phase
	float t_s; // diff time
	float delta_pos;
	float b_pos;

	uint64_t t[2];
	float traj_time;
	uint8_t active;

}Trajectory;

void init_trapezoidal(Trajectory* trj, float init_pos, float final_pos);
void generate_trapezoidal(Trajectory* trj); // real time generate
void activate_generator(Trajectory* trj);
void deactivate_generator(Trajectory* trj);

uint8_t is_active(Trajectory* trj);
int sign(float x);

#endif /* INC_TRAJECTORYGENERATOR_H_ */
