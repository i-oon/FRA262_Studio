/*
 * Encoder.c
 *
 *  Created on: May 6, 2025
 *      Author: ioonz
 */
#include "main.h"
#include "Encoder.h"

static uint64_t last_time_us = 0;
static int a = 0;

extern uint64_t micros(void);

void Encoder_Init(ENCODER *enc, TIM_HandleTypeDef *htim, uint32_t ppr) {
	enc->htim = htim;
	// Reset hardware counter
//	__HAL_TIM_SET_COUNTER(enc->htim, 0);
	// Start QEI interface
	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);

	// Initialize software state
	enc->count[NOW] = 0;
	enc->count[PREV] = 0;
	enc->rad = 0.0f;
	enc->velocity = 0.0f;
	enc->position_per_round = 0;
	enc->degree=0;
	a = 1;
	last_time_us = micros();

	enc->ppr = ppr;
}
//void Encoder_GetCount(ENCODER *enc){
//    // Read current timer count
//	enc->count[PREV] = enc->count[NOW];
//    enc->count[NOW] = __HAL_TIM_GET_COUNTER(enc->htim);
//
//}

uint32_t GetCount(ENCODER *enc) {
	// Read current timer count
	enc->count[PREV] = enc->count[NOW];
	enc->count[NOW] = __HAL_TIM_GET_COUNTER(enc->htim);
//	enc->position_per_round = enc->count[NOW]/8192.0;
	return enc->count[NOW];

}

void Encoder_Compute(ENCODER *enc) {
//collect data

	enc->count[NOW] = __HAL_TIM_GET_COUNTER(enc->htim);

	int32_t diff_count = enc->count[NOW] - enc->count[PREV];

	enc->position_per_round = enc->count[NOW] % enc->ppr;


	// Handle wrap-around
	if (diff_count > (4294967295 / 2))
		diff_count -= 4294967295;
	if (diff_count < -(4294967295 / 2))
		diff_count += 4294967295;
//	if (diff_count > (4294967295 / 2))
//		diff_count = -((enc->count[PREV]-0)+(4294967295-enc->count[NOW]));
//	if (diff_count < -(4294967295 / 2))
//		diff_count = (enc->count[NOW]-0)+(4294967295-enc->count[PREV]);

	// Compute angle [rad] and angular velocity [rad/s]
	enc->rad += (diff_count * 2.0f * M_PI) / 8192.0f;
	enc->velocity = (diff_count * 1000 * 2.0f * M_PI) / (float)enc->ppr;
	enc->degree =enc->rad/ M_PI*180.0;

	// Shift current to previous for next iteration
	enc->count[PREV] = enc->count[NOW];
}

void Encoder_Reset(ENCODER *enc) {
	 HAL_TIM_Encoder_Stop(enc->htim, TIM_CHANNEL_ALL);    // Stop encoder before reset
	__HAL_TIM_SET_COUNTER(enc->htim, 0);                  // Reset the counter to 0
	enc->count[NOW] = 0;
	enc->count[PREV] = 0;
	enc->rad = 0.0f;
	enc->velocity = 0.0f;
	enc->position_per_round = 0.0f;
	enc->degree = 0.0f;
//	HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL);

}


