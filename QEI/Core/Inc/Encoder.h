/*
 * Encoder.h
 *
 *  Created on: May 6, 2025
 *      Author: ioonz
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"
#include "math.h"

//#define CPR        4294967295U
//#define HALF_CPR   (CPR/2U)


typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t count[2];
	float rad;
	float velocity;
	float position_per_round;
	float degree;

	uint32_t ppr;

} ENCODER;

enum {
	NOW, PREV
};

void Encoder_Init(ENCODER *enc, TIM_HandleTypeDef *htim, uint32_t ppr);
void Encoder_GetCount(ENCODER *enc);
uint32_t GetCount(ENCODER *enc);
void Encoder_Compute(ENCODER *enc);
void Encoder_Reset(ENCODER *enc);


#endif /* INC_ENCODER_H_ */
