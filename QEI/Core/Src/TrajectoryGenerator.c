/*
 * TrajectoryGenerator.c
 *
 *  Created on: May 8, 2025
 *      Author: Kaitniyom
 */
#include "TrajectoryGenerator.h"

void init_trapezoidal(Trajectory* trj, float init_pos, float final_pos);
void generate_trapezoidal(Trajectory* trj);

void activate_generator();
void deactivate_generator();

uint8_t is_active();

extern uint64_t micros(void);

void init_trapezoidal(Trajectory* trj, float init_pos, float final_pos){
	trj->Init_pos = init_pos;
	trj->delta_pos = final_pos - trj->Init_pos;
	trj->dir = (float)sign(trj->delta_pos);
	trj->delta_pos = fabs(trj->delta_pos);

	trj->t_b = trj->desire_av / trj->desire_ac;
	trj->b_pos = 0.5 * trj->desire_ac * trj->t_b * trj->t_b;

	if(trj->delta_pos <= 2 * trj->b_pos){
		// Triangular profile
		trj->t_c = 0;
		trj->t_b = sqrt(trj->delta_pos / trj->desire_ac);
		trj->t_f = 2 * trj->t_b;
	}
	else{
		// Trapezoidal profile
		trj->t_c = (trj->delta_pos - 2 * trj->b_pos) / trj->desire_av;
		trj->t_f = 2 * trj->t_b + trj->t_c;
	}
	trj->ang_pos = trj->Init_pos;
	trj->ang_velo = 0;
	trj->ang_acc = 0;
	trj->t[Current] = 0;
	trj->t[Previous] = 0;
	trj->t_s = 0;
}

void generate_trapezoidal(Trajectory* trj){
	if(trj->t[Previous] == 0){
		trj->t[Previous] = micros();
		trj->t[Current] = micros();
	}

	trj->t[Current] = micros();
	double dt = (float)(trj->t[Current] - trj->t[Previous]) * 1e-6;
	trj->t_s += dt;
	trj->t[Previous] = trj->t[Current];

	if(trj->t_c == 0){
		// Triangular profile
		if(trj->t_s < trj->t_b){
			// Acceleration phase
			trj->ang_acc = trj->desire_ac * trj->dir;
			trj->ang_velo += trj->ang_acc * dt;
			trj->ang_pos += trj->ang_velo * dt + 0.5 * trj->ang_acc * dt * dt;
		}
		else if(trj->t_s <= trj->t_f){
			// Deceleration phase
			trj->ang_acc = -trj->desire_ac * trj->dir;
			trj->ang_velo += trj->ang_acc * dt;
			trj->ang_pos += trj->ang_velo * dt + 0.5 * trj->ang_acc * dt * dt;
		}
		else {
			// Done
			trj->ang_pos = trj->Init_pos + trj->delta_pos * trj->dir;
			trj->ang_velo = 0;
			trj->ang_acc = 0;
		}
	}
	else{
		// Trapezoidal
		if(trj->t_s < trj->t_b){
			trj->ang_acc = trj->desire_ac * trj->dir;
			trj->ang_velo += trj->ang_acc * dt;
			trj->ang_pos += trj->ang_velo * dt + 0.5 * trj->ang_acc * dt * dt;
		}
		else if(trj->t_s < trj->t_b + trj->t_c){
			trj->ang_acc = 0;
			trj->ang_velo = trj->desire_av * trj->dir;;
			trj->ang_pos += trj->ang_velo * dt;
		}
		else if(trj->t_s <= trj->t_f){
			trj->ang_acc = -trj->desire_ac * trj->dir;
			trj->ang_velo += trj->ang_acc * dt;
			trj->ang_pos += trj->ang_velo * dt + 0.5 * trj->ang_acc * dt * dt;
		}
		else{
			trj->ang_pos = trj->Init_pos + trj->delta_pos * trj->dir;
			trj->ang_velo = 0;
			trj->ang_acc = 0;
		}
	}
}

void activate_generator(Trajectory* trj){
	trj->active = 1;
}
void deactivate_generator(Trajectory* trj){
	trj->active = 0;
}

uint8_t check_status(Trajectory* trj){
	return trj->active;
}

int sign(float x) {
    if(x == 0){
    	return 0.0f;
    }
    else if(x < 0){
    	return -1.0f;
    }
    else if(x > 0){
		return 1.0f;
	}
}
