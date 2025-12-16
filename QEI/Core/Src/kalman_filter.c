/*
 * kalman_filter.c
 *
 *  Created on: May 12, 2025
 *      Author: User
 */
#include "kalman_filter.h"

// Kalman Variables

kalman KALMAN;
//Kalman Filter
// Error covariance matrix P (2x2)
float32_t P_f32[16] = {
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};

arm_matrix_instance_f32 P;


//Identity Matrix
float32_t I_f32[16] = {
		1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1
};

arm_matrix_instance_f32 I;


arm_matrix_instance_f32 Xsensor;

//start Matrix
//float32_t X_hat_f32[16] = {
//		1,1,1,1,
//		1,1,1,1,
//		1,1,1,1,
//		1,1,1,1
//};
//arm_matrix_instance_f32 X_hat;

float32_t X_f32[4] = {
	0,
	0,
	0,
	0
};
arm_matrix_instance_f32 X;

//Pao A
//float32_t A_f32[16] = {
//		1,	0.001,	-0.0017,	0.0000,
//		0,	0.9933,	-3.3221,	0.0000,
//		0,	0,		1.0000,		0.0000,
//		0, -0.0026,	0.0046,		0.6873
//};

//from ioon*****
//float32_t A_f32[16] = {
//		1,	0.001,	-0.00,	0.0000,
//		0,	0.9985,	-0.0747,	0.0000,
//		0,	0,		1.0000,		0.0000,
//		0, -0.4001,	0.0223,		0.0312
//};

float32_t A_f32[16] = {
		1,	0.001,	-0.00,	0.0000,
		0,	0.9999,	-0.4752,	0.0055,
		0,	0,		1.0000,		0.0000,
		0, -0.32,	0.02,		0.0272
};

//Revolute A
//float32_t A_f32[16] = {
//		1,	0.001,	0.0000,	0.0000,
//		0,	0.9989,	-0.0080,	0.0000,
//		0,	0,		1.0000,		0.0000,
//		0, -0.4402,	0.0019,		0.7135
//};

//Prismatic A
//float32_t A_f32[16] = {
//		1,	0.001,	-0.0005,	0.0000,
//		0,	0.9989,	-1.0231,	0.0000,
//		0,	0,		1.0000,		0.0000,
//		0, -0.0060,	0.0031,		0.9051
//};
arm_matrix_instance_f32 A;

//Prismatic A zland motor
//float32_t A_f32[16] = {
//		1,	0.001,	-0.0005,	0.0000,
//		0,	0.9989,	-1.0231,	0.0000,
//		0,	0,		1.0000,		0.0000,
//		0, -0.0060,	0.0031,		0.9051
//};
//arm_matrix_instance_f32 A;

//Revolute A zland motor
//float32_t A_f32[16] = {
//		1,	0.001,  	-0.0,		0.0,
//		0,	0.9989,		-0.0647,	0.0,
//		0,	0.00000,	1.0000,		0.0000,
//		0, -0.4001,		0.0223,		0.0312
//};
//arm_matrix_instance_f32 A;

//Pao B
//float32_t B_f32[4] = {
//		0,
//		0,
//		0,
//		0.2316
//};

//Revolute B
//float32_t B_f32[4] = {
//		0,
//		0,
//		0,
//		0.2622
//};

//Prismatic B
//float32_t B_f32[4] = {
//		0,
//		0,
//		0,
//		//0.0173
//		0.05f
//};

//Revolute B zland motor

//from ioon *****
//float32_t B_f32[4] = {
//		0.0,
//		0.0,
//		0.0,
//		1.5216
//};

float32_t B_f32[4] = {
		0.0,
		0.0,
		0.0,
		1.5095
};

//Prismatic B zland motor
//float32_t B_f32[4] = {
//		0,
//		0,
//		0,
//		//0.0173
//		0.05f
//};
arm_matrix_instance_f32 B;

arm_matrix_instance_f32 U;

float32_t Y_f32[1] = {
		1
};

arm_matrix_instance_f32 Y;

float32_t C_f32[4] = {
		1,0,0,0
};
arm_matrix_instance_f32 C;

//process noise
float32_t Q_f32[16] = {
		0.8,0,0,0,		//10
		0,0.8,0,0,
		0,0,0.8,0,
		0,0,0,0.8
};
arm_matrix_instance_f32 Q;

// measurement noise lower
float32_t R_f32[1] = {
		//0.9
//		0.000001
		0.2
};
arm_matrix_instance_f32 R;

float32_t AT_f32[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 AT;

float32_t CT_f32[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 CT;

float32_t KT_f32[4]={
		1,1,1,1,
};
arm_matrix_instance_f32 KT;

float32_t AX_f32[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 AX;

float32_t BU_f32[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 BU;

float32_t AX_BU_f32[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 AX_BU;

//float32_t AX_BU_Q_f32[4];
//arm_matrix_instance_f32 AX_BU_Q;

float32_t CXsensor_f32[1]={
		1
};
arm_matrix_instance_f32 CXsensor;//???

float32_t CX_f32[1]={
		1
};
arm_matrix_instance_f32 CX;

float32_t AP_f32[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 AP;

float32_t APAT_f32[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 APAT;

float32_t APAT_Q_f32[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 APAT_Q;

float32_t K_f32[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 K;

float32_t PCT_f32[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 PCT;

float32_t CP_f32[4]={
		1,1,1,1,
};
arm_matrix_instance_f32 CP;

float32_t CPCT_f32[1]={
		1
};
arm_matrix_instance_f32 CPCT;

float32_t CPCT_R_f32[1]={
		1
};
arm_matrix_instance_f32 CPCT_R;

float32_t CPCT_R_INV_f32[1]={
		1
};
arm_matrix_instance_f32 CPCT_R_INV;

float32_t PCPCT_R_INV_f32[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 PCPCT_R_INV;

float32_t Y_CX_f32[1]={
		1
};
arm_matrix_instance_f32 Y_CX;

float32_t KY_KCX_f32[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 KY_KCX;

float32_t X_KY_KCX_f32[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 X_KY_KCX;

float32_t KC_f32[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 KC;

float32_t KR_f32[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 KR;

float32_t KRKT_f32[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 KRKT;

float32_t I_KC_f32[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 I_KC;

float32_t IP_KCP_f32[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 IP_KCP;

float32_t I_KC_trans_f32[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 I_KC_trans;

float32_t IP_KCPI_KC_trans_f32[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 IP_KCPI_KC_trans;

float32_t IP_KCPI_KC_trans_KRKT_f32[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 IP_KCPI_KC_trans_KRKT;

volatile arm_status CalcSt;


// Kalman initialization function
void Kalman_Init(kalman* KALMAN)
{
	//start Matrix
		arm_mat_init_f32(&P, 4, 4, P_f32);  // assuming 2x2 based on values
		// 1x4 or 2x2 matrix?
		arm_mat_init_f32(&I, 4, 4, I_f32);  // assuming 2x2 based on values

		arm_mat_init_f32(&Xsensor, 1, 1, KALMAN->Xsensor_f32);
		//arm_mat_init_f32(&X_hat, 4, 1, X_hat_f32);
	    arm_mat_init_f32(&X, 4, 1, X_f32);

		arm_mat_init_f32(&A, 4, 4, A_f32);  // assumed 4x4, although only 12 values provided?
		arm_mat_init_f32(&B, 4, 1, B_f32);  // same here

		arm_mat_init_f32(&U, 1, 1, KALMAN->U_f32);
		arm_mat_init_f32(&Y, 1, 1, Y_f32);

		arm_mat_init_f32(&C, 1, 4, C_f32);  // 1x4 matrix assumed from 4 values

		arm_mat_init_f32(&Q, 4, 4, Q_f32);
		arm_mat_init_f32(&R, 1, 1, R_f32);

		arm_mat_init_f32(&AT, 4, 4, AT_f32);
		arm_mat_init_f32(&CT, 4, 1, CT_f32);  // transpose of 1x4 = 4x1
		arm_mat_init_f32(&KT, 1, 4, KT_f32);  // assumed 4x1

		arm_mat_init_f32(&AX, 4, 1, AX_f32);
		arm_mat_init_f32(&BU, 4, 1, BU_f32);
		arm_mat_init_f32(&AX_BU, 4, 1, AX_BU_f32);

		arm_mat_init_f32(&CXsensor, 1, 1, CXsensor_f32);  // assumed scalar
		arm_mat_init_f32(&CX, 1, 1, CX_f32);

		arm_mat_init_f32(&AP, 4, 4, AP_f32);
		arm_mat_init_f32(&APAT, 4, 4, APAT_f32);
		arm_mat_init_f32(&APAT_Q, 4, 4, APAT_Q_f32);

		arm_mat_init_f32(&K, 4, 1, K_f32);
		arm_mat_init_f32(&PCT, 4, 1, PCT_f32);
		arm_mat_init_f32(&CP, 1, 4, CP_f32);
		arm_mat_init_f32(&CPCT, 1, 1, CPCT_f32);
		arm_mat_init_f32(&CPCT_R, 1, 1, CPCT_R_f32);
		arm_mat_init_f32(&CPCT_R_INV, 1, 1, CPCT_R_INV_f32);
		arm_mat_init_f32(&PCPCT_R_INV, 4, 1, PCPCT_R_INV_f32);
		arm_mat_init_f32(&Y_CX, 1, 1, Y_CX_f32);
		arm_mat_init_f32(&KY_KCX, 4, 1, KY_KCX_f32);
		arm_mat_init_f32(&X_KY_KCX, 4, 1, X_KY_KCX_f32);

		arm_mat_init_f32(&KC, 4, 4, KC_f32);
		arm_mat_init_f32(&KR, 4, 1, KR_f32);
		arm_mat_init_f32(&KRKT, 4, 4, KRKT_f32);

		arm_mat_init_f32(&I_KC, 4, 4, I_KC_f32);
		arm_mat_init_f32(&IP_KCP, 4, 4, IP_KCP_f32);
		arm_mat_init_f32(&I_KC_trans, 4, 4, I_KC_trans_f32);
		arm_mat_init_f32(&IP_KCPI_KC_trans, 4, 4, IP_KCPI_KC_trans_f32);
		arm_mat_init_f32(&IP_KCPI_KC_trans_KRKT, 4, 4, IP_KCPI_KC_trans_KRKT_f32);
    // ...
}

// Kalman filter function
void Kalman_Filter(kalman* KALMAN, float voltage, float encoder) //input
{
	KALMAN->U_f32[0] = voltage;
	KALMAN->Xsensor_f32[0] = encoder * 4.0;

    // X^ = AX + BU
    arm_mat_mult_f32(&A, &X , &AX);
    arm_mat_mult_f32(&B, &U , &BU);
    arm_mat_add_f32(&AX, &BU, &X);

    // P = AP * AT + Q
    arm_mat_mult_f32(&A, &P , &AP);
    arm_mat_trans_f32(&A, &AT);
    arm_mat_mult_f32(&AP, &AT , &APAT);
    arm_mat_add_f32(&APAT, &Q, &P);

    // K = P*HT*INV(H*P*HT+R)
    arm_mat_trans_f32(&C, &CT);
    arm_mat_mult_f32(&C, &P , &CP);
    arm_mat_mult_f32(&CP, &CT , &CPCT);
    arm_mat_add_f32(&CPCT, &R , &CPCT_R);
    arm_mat_inverse_f32(&CPCT_R, &CPCT_R_INV);
    arm_mat_mult_f32(&P, &CT, &PCT);
    arm_mat_mult_f32(&PCT, &CPCT_R_INV , &K);

    // Z = CXsensor
    arm_mat_mult_f32(&C, &Xsensor , &CXsensor);

    // X^ = X^ + K(z - CX^)
    arm_mat_mult_f32(&C, &X , &CX);
    arm_mat_sub_f32(&CXsensor, &CX, &Y_CX);
    arm_mat_mult_f32(&K, &Y_CX , &KY_KCX);
    arm_mat_add_f32(&X, &KY_KCX , &X);

    // P = (I - KH)*P*transpose(I - KH) + KRKT
    arm_mat_mult_f32(&K, &C , &KC);
    arm_mat_mult_f32(&K, &R , &KR);
    arm_mat_trans_f32(&K, &KT);
    arm_mat_mult_f32(&KR, &KT , &KRKT);
    arm_mat_sub_f32(&I, &KC , &I_KC);
    arm_mat_trans_f32(&I_KC, &I_KC_trans);
    arm_mat_mult_f32(&I_KC, &P , &IP_KCP);
    arm_mat_mult_f32(&IP_KCP, &I_KC_trans , &IP_KCPI_KC_trans);
    arm_mat_add_f32(&IP_KCPI_KC_trans, &KRKT , &P);

    // Extract state
    KALMAN->theta_kalman = X_f32[0]/4.0;
    KALMAN->omega_kalman = X_f32[1] /4.0;
    KALMAN->torque_kalman = X_f32[2];
    KALMAN->current_kalman = X_f32[3];
}



