/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "math.h"
#include "stdio.h"
#include "velocity_control.h"
#include "MotorDriver.h"
#include "Encoder.h"
#include "pid.h"
#include "TrajectoryGenerator.h"
#include "kalman_filter.h"
#include "kalman_prs.h"
#include "feedforward.h"
#include "waypoints.h"

#include "VelocityFormPID.h"

#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EPSILON 0.0001
#define RAD2TICKS (8192.0f / (2.0f * M_PI))
#define PRISMATIC_RESET_BUTTON HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)
#define REVOLUTE_RESET_BUTTON HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)
#define JOY_MEM_JOG_BUTTON HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)
#define JOY_RESET_BUTTON HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)
#define JOY_RUN_JOG_BUTTON HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)
#define JOY_ADD_VELO_BUTTON HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)

#define EMERGENCY_BUTTON HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)
//#define SERVO HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int EMERGENCY_STATE = 0;

int ok =0;

typedef struct{
	uint8_t reset;

	float prev_set_pos;
	float prev_set_velo;

	float set_pos;
	float set_velo;
	float prev_setpos;

	//control variable
	float Pwm;
	float Vin;

	float pos_error;
	float velo_error;

	float disturbance;
	VelocityFormPIDHandler pos_PID;
	VelocityFormPIDHandler velo_PID;

	MOTOR motor;
	ENCODER encoder;

}RevoluteControlTypeHandler;

typedef struct{
	uint8_t reset;
	uint8_t reset_param;

	float prev_set_pos;
	float prev_set_velo;
	float prev_set_dis;

	float set_pos;
	float set_velo;
	float set_dis;
	float prev_setpos;

	//control variable
	float Pwm;
	float Vin;

	float pos_error;
	float dis_error;
	float velo_error;

	float disturbance;
	VelocityFormPIDHandler pos_PID;
	VelocityFormPIDHandler velo_PID;

	MOTOR motor;
	ENCODER encoder;
}PrismaticControlTypeHandler;


RevoluteControlTypeHandler Revolute = {0};
PrismaticControlTypeHandler Prismatic = {0};

SERVO Servo = {0};
float Volt = 0.0;


int servo_command = 0;

int32_t joy_analog[2];
int check = 0;
int joy_mid_pris = 0;
int joy_mid_rev = 0;

int Emergency= 0 ;
int Reset_button = 0;

int green_light=0;
int red_light=0;

int servo_check1=1;
int servo_check2=1;

int servo_PWM;
int state = 0;

kalman k_rev;
kalman_prs k_pris;

float Rev_Dis;
float Pris_Dis;

float rev_pos_error = 0;
float rev_vel_error = 0;

int count = 0;

float test = 0;

float velocity = 0;
float setvelocity = 0;
float test_speed = 0;
float direction = 1;

uint8_t Pris_reset;

uint64_t _micros = 0;

Trajectory rev_traj = {0};
Trajectory pris_traj = {0};

uint8_t reset_stage = 0;
uint32_t reset_timer = 0;

uint32_t reset_done_time = 0;
uint8_t waiting_after_reset = 0;


float JOY_PRISMATIC[2] = {0.0, 0.0};
float JOY_REVOLUTE[2] = {0.0, 0.0};



// Array to store G-code commands
WAYPOINT gCode[MAX_WAYPOINT] = {
    {"G21", 0.0f, 0.0f, 0.0f, 0.0f},         // G21 - mm mode
    {"G0", 44.8466f, 148.7895f, 5.0f, 1000.0f}, // G0 - Lift pen, move to X44.8466 Y148.7895
    {"G0", 44.8466f, 148.7895f, 0.0f, 1000.0f}, // G0 - Activate Pen at Z0
    {"G1", 40.8713f, 148.7895f, 0.0f, 1000.0f}, // G1 - Drawing to X40.8713 Y148.7895
    {"G1", 40.3237f, 148.9217f, 0.0f, 1000.0f}, // G1 - Continue drawing
    {"G1", 39.9781f, 149.1881f, 0.0f, 1000.0f}, // G1
    {"G1", 39.7506f, 149.5604f, 0.0f, 1000.0f}, // G1
    {"G1", 39.6713f, 149.9895f, 0.0f, 1000.0f}, // G1
    {"G1", 39.6713f, 179.1677f, 0.0f, 1000.0f}, // G1
    {"G1", 39.8035f, 179.7153f, 0.0f, 1000.0f}, // G1
    {"G1", 40.0699f, 180.0609f, 0.0f, 1000.0f}, // G1
    {"G1", 40.4422f, 180.2884f, 0.0f, 1000.0f}, // G1
    {"G1", 40.8713f, 180.3677f, 0.0f, 1000.0f}, // G1
    {"G1", 56.4983f, 180.3677f, 0.0f, 1000.0f}, // G1
    {"G1", 56.9038f, 180.2971f, 0.0f, 1000.0f}, // G1
    {"G1", 57.2808f, 180.0775f, 0.0f, 1000.0f}, // G1
    {"G1", 57.5544f, 179.7375f, 0.0f, 1000.0f}, // G1
    {"G1", 57.6883f, 179.3222f, 0.0f, 1000.0f}, // G1
    {"G1", 58.0799f, 176.3065f, 0.0f, 1000.0f}, // G1
    {"G1", 58.0829f, 176.0225f, 0.0f, 1000.0f}, // G1
    {"G1", 57.9577f, 175.6044f, 0.0f, 1000.0f}, // G1
    {"G1", 57.6913f, 175.2588f, 0.0f, 1000.0f}, // G1
    {"G1", 57.3190f, 175.0313f, 0.0f, 1000.0f}, // G1
    {"G1", 56.8899f, 174.9520f, 0.0f, 1000.0f}, // G1
    {"G1", 46.0466f, 174.9520f, 0.0f, 1000.0f}, // G1
    {"G1", 46.0466f, 167.8348f, 0.0f, 1000.0f}, // G1
    {"G1", 55.1079f, 167.8348f, 0.0f, 1000.0f}, // G1
    {"G1", 55.6555f, 167.7026f, 0.0f, 1000.0f}, // G1
    {"G1", 56.0011f, 167.4362f, 0.0f, 1000.0f}, // G1
    {"G1", 56.2286f, 167.0639f, 0.0f, 1000.0f}, // G1
    {"G1", 56.3079f, 166.6348f, 0.0f, 1000.0f}, // G1
    {"G1", 56.3079f, 163.4624f, 0.0f, 1000.0f}, // G1
    {"G1", 56.1757f, 162.9148f, 0.0f, 1000.0f}, // G1
    {"G1", 55.9093f, 162.5692f, 0.0f, 1000.0f}, // G1
    {"G1", 55.5370f, 162.3417f, 0.0f, 1000.0f}, // G1
    {"G1", 55.1079f, 162.2624f, 0.0f, 1000.0f}, // G1
    {"G1", 46.0466f, 162.2624f, 0.0f, 1000.0f}, // G1
    {"G1", 46.0466f, 149.9895f, 0.0f, 1000.0f}, // G1
    {"G1", 45.9144f, 149.4419f, 0.0f, 1000.0f}, // G1
    {"G1", 45.6480f, 149.0963f, 0.0f, 1000.0f}, // G1
    {"G1", 45.2757f, 148.8688f, 0.0f, 1000.0f}, // G1
    {"G1", 44.8466f, 148.7895f, 0.0f, 1000.0f}, // G1
    {"G0", 0.0f, 0.0f, 5.0f, 0.0f}             // G0 - Lift pen to z-safe
};
int num_points = 38;
int count_now = 1;
int count_prev = 0;
int draw_command = 0;

//typedef enum {
//	DRAW_DONE, DRAWING
//}DRAW_STATE;
int DRAW_STATE = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void reset_prismatic();
void reset_revolute();
void reset_pris();
void reset_rev();
void calculateWaypoints();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc2, joy_analog, 2);

  MotorDriver_Init(&Revolute.motor, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2);
  Encoder_Init(&Revolute.encoder, &htim5, 8192);
  InitializePositionFormPID(&Revolute.pos_PID, 5.0, 0.0001f, 1.0f, 0.01, 3.8);
  InitializeVelocityFormPID(&Revolute.velo_PID, 1.0, 10.0f, 0.0f, 0.001, 12);

  MotorDriver_Init(&Prismatic.motor, &htim1, TIM_CHANNEL_3, TIM_CHANNEL_4);
  Encoder_Init(&Prismatic.encoder, &htim2, 8192);
  InitializePositionFormPID(&Prismatic.pos_PID, 30.0f, 0.05, 0.1f, 0.01, 40);
  InitializeVelocityFormPID(&Prismatic.velo_PID, 0.01f, 0.9f, 0.0f, 0.001, 12);

//  ServoDriver_Init(&Servo, &htim8, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim8);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);



  Kalman_Init(&k_rev);
  Kalman_Init_prs(&k_pris);

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim6);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);

	rev_traj.desire_av = 3.0;
	rev_traj.desire_ac = 1.5;
	rev_traj.active = 0;

	pris_traj.desire_av = 3.0;
	pris_traj.desire_ac = 1.5;
	pris_traj.active = 0;

	HAL_Delay(100);
	joy_mid_rev = joy_analog[0];
	joy_mid_pris = joy_analog[1];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if (JOY_MEM_JOG_BUTTON == 1) {state = 1;}
		if (JOY_RESET_BUTTON == 1) {state = 2; }
		if (JOY_RUN_JOG_BUTTON == 1) {state = 3;}
		if (JOY_ADD_VELO_BUTTON == 1) {state = 4;}

		JOY_PRISMATIC[0] = ((float)(joy_analog[1]-joy_mid_pris)/(joy_mid_pris+35))*200.0;

		JOY_REVOLUTE[0] = (joy_analog[0]-joy_mid_rev)/63.0*200.0;


//		if (servo_command == 1) {
//			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, Volt); //in
//		}
//
//		count = EMERGENCY_BUTTON;
		if (EMERGENCY_STATE == 0 && EMERGENCY_BUTTON) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
			EMERGENCY_STATE = 1;
		}
		else if (EMERGENCY_STATE = 1 && !EMERGENCY_BUTTON) {
			Prismatic.reset = 1;
			EMERGENCY_STATE = 2;
		}
		else {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
		}



}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3) {
		_micros += UINT16_MAX;
	}

	if (htim == &htim6) { // 1000 Hz - Velocity loop (inner)

		if ((Prismatic.reset == 1 || Prismatic.reset == 3) && !waiting_after_reset) {
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) != 0) {
				MotorDriver_Set(&Prismatic.motor, -225);
				reset_pris();
				reset_rev();
			} else {
				MotorDriver_Set(&Prismatic.motor, 0);
				Encoder_Reset(&Prismatic.encoder);

				Prismatic.pos_error = 0;
				if(Prismatic.reset == 1){
					Prismatic.reset = 2;
				}
				else{
					waiting_after_reset = 1;
					reset_done_time = HAL_GetTick();
					Prismatic.reset = 0;
				}
				// Start 1-second wait
				return;
			}
		}

		if (Prismatic.reset == 2 && !waiting_after_reset) {
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) != 0) {
				MotorDriver_Set(&Revolute.motor, -150);
				reset_pris();
				reset_rev();
			} else {
				MotorDriver_Set(&Revolute.motor, 0);
				Encoder_Reset(&Revolute.encoder);
				Revolute.pos_error = 0;
//				Revolute.reset = 0;
				Prismatic.reset = 3;
				return;
			}
		}

		// Wait 1 second after reset before continuing
		if (waiting_after_reset) {
			if (HAL_GetTick() - reset_done_time < 2000) {
				Encoder_Reset(&Prismatic.encoder);
				reset_pris();
				reset_rev();
				return;
			}
			HAL_TIM_Encoder_Start(Prismatic.encoder.htim, TIM_CHANNEL_ALL);
			HAL_TIM_Encoder_Start(Revolute.encoder.htim, TIM_CHANNEL_ALL);
			waiting_after_reset = 0;  // Done waiting, resume control
		}

		if (Prismatic.reset != 0) return; // Skip position loop while resetting

		Encoder_Compute(&Revolute.encoder);
		Encoder_Compute(&Prismatic.encoder);

		Kalman_Filter(&k_rev,Revolute.Vin, Revolute.encoder.rad);
		Kalman_Filter_prs(&k_pris,Prismatic.Vin, Prismatic.encoder.rad);
		//=============================================================================

		//=============================================================================

		ComputeVelocityFormPID_C(&Prismatic.velo_PID, Prismatic.velo_error, k_pris.omega_kalman_prs);
		ComputeVelocityFormPID_C(&Revolute.velo_PID, Revolute.velo_error, k_rev.omega_kalman);
		Prismatic.Vin = (Prismatic.velo_PID.output[Current] + Prismatic.disturbance);
		Revolute.Vin = (Revolute.velo_PID.output[Current] + Revolute.disturbance);
		if(Prismatic.Vin > 12){ Prismatic.Vin = 12; }
		else if(Prismatic.Vin < -12){ Prismatic.Vin = -12; }

		if(Revolute.Vin > 12){ Revolute.Vin = 12; }
		else if(Revolute.Vin < -12){ Revolute.Vin = -12; }
		Prismatic.Pwm = Prismatic.Vin * 999 / 12;
		Revolute.Pwm = Revolute.Vin * 999 / 12;
		if (fabs(Prismatic.dis_error) < 0.1f && !pris_traj.active) {
			Prismatic.Pwm = 0;
			ResetPositionFormPID(&Prismatic.pos_PID);
			ResetVelocityFormPID(&Prismatic.velo_PID);
		}
		MotorDriver_Set(&Prismatic.motor, Prismatic.Pwm);
	//		}
			//-------------------------------------------------------------------------


		if (fabs(Revolute.pos_error) < 0.01f && !rev_traj.active) {
			Revolute.Pwm = 0;
			ResetPositionFormPID(&Revolute.pos_PID);
			ResetVelocityFormPID(&Revolute.velo_PID);
		}
		MotorDriver_Set(&Revolute.motor, Revolute.Pwm);

	}


	if (htim == &htim4) { // 100 Hz - Position loop (outer)
//		/*Trajectory generate phase*/
		//----------------------------------------------------------------------------------

		if (Prismatic.reset != 0) return;

//		if(ok==1){
//			Revolute.set_pos=3.14;
//			Prismatic.prev_set_dis=0;
//			Prismatic.set_pos = 2.0 * M_PI * Prismatic.set_dis / 40.0 + Revolute.set_pos;
//			ok=0;
//		}
//		if (draw_command && DRAW_STATE == 0) {
//			calculateWaypoints();
//		}
		Prismatic.set_pos = 2.0 * M_PI * Prismatic.set_dis / 40.0 + Revolute.set_pos;
		//=========================================================

			if (fabs(Revolute.set_pos - Revolute.prev_set_pos) > EPSILON) {
				Revolute.prev_set_pos = Revolute.set_pos;
				float init_rad_rev = (float)(Revolute.encoder.rad);
				init_trapezoidal(&rev_traj, init_rad_rev, Revolute.set_pos);
				rev_traj.active = 1;
			}
			if (fabs(Prismatic.set_pos - Prismatic.prev_set_pos) > EPSILON) {
				Prismatic.prev_set_pos = Prismatic.set_pos ;
				float init_rad_prs = (float)(Prismatic.encoder.rad);
				init_trapezoidal(&pris_traj, init_rad_prs, Prismatic.set_pos);
				pris_traj.active = 1;
			}
			//----------------------------------------------------------------------------------
			if(rev_traj.active && rev_traj.t_s <= rev_traj.t_f){
				generate_trapezoidal(&rev_traj);
			}else{
				rev_traj.active = 0;
			}

			if(pris_traj.active && pris_traj.t_s <= pris_traj.t_f){
				generate_trapezoidal(&pris_traj);
			}else{
				pris_traj.active = 0;

			}

		//=========================================================
		/*Position Control phase*/

		Revolute.pos_error = rev_traj.ang_pos - k_rev.theta_kalman;
		Prismatic.pos_error = pris_traj.ang_pos - k_pris.theta_kalman_prs;

		Prismatic.dis_error = Prismatic.pos_error * 6.0f;

		ComputePositionFormPID(&Revolute.pos_PID, Revolute.pos_error, k_rev.theta_kalman, 100);
		ComputePositionFormPID(&Prismatic.pos_PID, Prismatic.pos_error, k_pris.theta_kalman_prs, 100);

		Revolute.velo_error = Revolute.pos_PID.output[Current] + rev_traj.ang_velo - k_rev.omega_kalman;
		Prismatic.velo_error = Prismatic.pos_PID.output[Current] + pris_traj.ang_velo - k_pris.omega_kalman_prs;

		Revolute.disturbance = Rev_dis(Revolute.encoder.rad, Prismatic.encoder.rad, -1);
		Prismatic.disturbance = 0;

//		if (fabs(Prismatic.dis_error) < 0.1f && !pris_traj.active
//				&& fabs(Revolute.pos_error) < 0.01f && !rev_traj.active && draw_command) {
//		 count_prev = count_now;
//		 count_now++;
//		 DRAW_STATE = 0;
//	   }
	}

}

uint64_t micros() {
	return __HAL_TIM_GET_COUNTER(&htim3) + _micros;
}

void calculateWaypoints() {
    if (count_prev < num_points && DRAW_STATE == 0) {
		WAYPOINT start = gCode[count_prev];
		WAYPOINT end = gCode[count_now];

		float delta_x = end.x - start.x;
		float delta_y = end.y - start.y;
		float distance = sqrt((delta_x * delta_x) + (delta_y * delta_y)); // Euclidean distance
		float heading = atan2(delta_y, delta_x);  // Angle to move towards the next point
//		float time_to_move = distance / 3.0f;  // 40 is max speed for now

		Prismatic.set_dis = distance;  // Move prismatic along X
		Revolute.set_pos = heading;    // Move the revolute joint to the calculated heading
		DRAW_STATE = 1;
//        }
//        count_prev = count_now;
    	}
//    else if (DRAW_STATE == 1) {
//
//		 if (fabs(Prismatic.set_pos-pris_traj.ang_pos) < EPSILON && fabs(Revolute.set_pos-rev_traj.ang_pos) < EPSILON) {
//			 count_prev = count_now;
//			 count_now++;
//			 DRAW_STATE = 0;
//		   }
//		}

    else if (count_prev >= num_points) {draw_command = 0;}


}


void reset_pris() {
    Prismatic.set_dis = 0.0f;
    Prismatic.set_pos = 0.0f;
    Prismatic.prev_set_dis = 0.0f;
    Prismatic.prev_set_pos = 0.0f;
    Prismatic.Pwm = 0.0f;
    Prismatic.Vin = 0.0f;
    Prismatic.velo_error = 0.0f;
    Prismatic.pos_error = 0.0f;

    ResetPositionFormPID(&Prismatic.pos_PID);
    ResetVelocityFormPID(&Prismatic.velo_PID);
    init_trapezoidal(&pris_traj, 0.0f, 0.0f);

//    Kalman_Init(&k_rev);
//    Kalman_Init_prs(&k_pris);
    k_pris.current_kalman_prs = 0.0f;
    k_pris.omega_kalman_prs = 0.0f;
    k_pris.theta_kalman_prs = 0.0f;
    k_pris.torque_kalman_prs = 0.0f;

    Kalman_Filter_prs(&k_pris,Prismatic.Vin, Prismatic.encoder.rad);
}

void reset_rev() {
	Revolute.set_pos = 0.0f;
	Revolute.prev_set_pos = 0.0f;
	Revolute.Pwm = 0.0f;
	Revolute.Vin = 0.0f;
	Revolute.velo_error = 0.0f;
	Revolute.pos_error = 0.0f;

    ResetPositionFormPID(&Revolute.pos_PID);
    ResetVelocityFormPID(&Revolute.velo_PID);
    init_trapezoidal(&rev_traj, 0.0f, 0.0f);

//    Kalman_Init(&k_rev);
//    Kalman_Init_prs(&k_pris);

    k_rev.theta_kalman = 0.0f;
	k_rev.omega_kalman = 0.0f;
    k_rev.torque_kalman = 0.0f;
    k_rev.current_kalman = 0.0f;
    k_rev.theta_kalman_prev = 0.0f;

    Kalman_Filter(&k_rev,Revolute.Vin, Revolute.encoder.rad);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
