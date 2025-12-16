/*
 * kalman_prs.c
 *
 *  Created on: May 19, 2025
 *      Author: gunda
 */

#include "kalman_prs.h"

// Kalman Variables

kalman_prs KALMAN_prs;
//Kalman Filter
// Error covariance matrix P (2x2)
float32_t P_f32_prs[16] = {
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};

arm_matrix_instance_f32 P_prs;


//Identity Matrix
float32_t I_f32_prs[16] = {
		1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1
};

arm_matrix_instance_f32 I_prs;


arm_matrix_instance_f32 Xsensor_prs;

//start Matrix
//float32_t X_hat_f32[16] = {
//		1,1,1,1,
//		1,1,1,1,
//		1,1,1,1,
//		1,1,1,1
//};
//arm_matrix_instance_f32 X_hat;

float32_t X_f32_prs[4] = {
	0,
	0,
	0,
	0
};
arm_matrix_instance_f32 X_prs;

//Pao A
//float32_t A_f32[16] = {
//		1,	0.001,	-0.0017,	0.0000,
//		0,	0.9933,	-3.3221,	0.0000,
//		0,	0,		1.0000,		0.0000,
//		0, -0.0026,	0.0046,		0.6873
//};


//Revolute A zland motor
float32_t A_f32_prs[16] = {
		1,	0.001,		-0.0038,		0.0002,
		0,	0.9964,		-7.6673,	0.3787,
		0,	0.00000,	1.0000,		0.0000,
		0, -0.0072,		0.0287,		0.8173

};
arm_matrix_instance_f32 A_prs;


//Revolute B zland motor
float32_t B_f32_prs[4] = {
		0.0,
		0.0245,
		0.0,
		0.1132
};


arm_matrix_instance_f32 B_prs;

arm_matrix_instance_f32 U_prs;

float32_t Y_f32_prs[1] = {
		1
};

arm_matrix_instance_f32 Y_prs;

float32_t C_f32_prs[4] = {
		1,0,0,0
};
arm_matrix_instance_f32 C_prs;

//process noise
float32_t Q_f32_prs[16] = {
		0.1,0,0,0,
		0,0.1,0,0,
		0,0,0.1,0,
		0,0,0,0.1
};
arm_matrix_instance_f32 Q_prs;

// measurement noise lower
float32_t R_f32_prs[1] = {
		//0.9
		80.0};
arm_matrix_instance_f32 R_prs;

float32_t AT_f32_prs[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 AT_prs;

float32_t CT_f32_prs[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 CT_prs;

float32_t KT_f32_prs[4]={
		1,1,1,1,
};
arm_matrix_instance_f32 KT_prs;

float32_t AX_f32_prs[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 AX_prs;

float32_t BU_f32_prs[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 BU_prs;

float32_t AX_BU_f32_prs[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 AX_BU_prs;

//float32_t AX_BU_Q_f32[4];
//arm_matrix_instance_f32 AX_BU_Q;

float32_t CXsensor_f32_prs[1]={
		1
};
arm_matrix_instance_f32 CXsensor_prs;//???

float32_t CX_f32_prs[1]={
		1
};
arm_matrix_instance_f32 CX_prs;

float32_t AP_f32_prs[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 AP_prs;

float32_t APAT_f32_prs[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 APAT_prs;

float32_t APAT_Q_f32_prs[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 APAT_Q_prs;

float32_t K_f32_prs[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 K_prs;

float32_t PCT_f32_prs[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 PCT_prs;

float32_t CP_f32_prs[4]={
		1,1,1,1,
};
arm_matrix_instance_f32 CP_prs;

float32_t CPCT_f32_prs[1]={
		1
};
arm_matrix_instance_f32 CPCT_prs;

float32_t CPCT_R_f32_prs[1]={
		1
};
arm_matrix_instance_f32 CPCT_R_prs;

float32_t CPCT_R_INV_f32_prs[1]={
		1
};
arm_matrix_instance_f32 CPCT_R_INV_prs;

float32_t PCPCT_R_INV_f32_prs[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 PCPCT_R_INV_prs;

float32_t Y_CX_f32_prs[1]={
		1
};
arm_matrix_instance_f32 Y_CX_prs;

float32_t KY_KCX_f32_prs[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 KY_KCX_prs;

float32_t X_KY_KCX_f32_prs[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 X_KY_KCX_prs;

float32_t KC_f32_prs[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 KC_prs;

float32_t KR_f32_prs[4]={
		1,
		1,
		1,
		1
};
arm_matrix_instance_f32 KR_prs;

float32_t KRKT_f32_prs[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 KRKT_prs;

float32_t I_KC_f32_prs[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 I_KC_prs;

float32_t IP_KCP_f32_prs[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 IP_KCP_prs;

float32_t I_KC_trans_f32_prs[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 I_KC_trans_prs;

float32_t IP_KCPI_KC_trans_f32_prs[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 IP_KCPI_KC_trans_prs;

float32_t IP_KCPI_KC_trans_KRKT_f32_prs[16]={
		1,1,1,1,
		1,1,1,1,
		1,1,1,1,
		1,1,1,1
};
arm_matrix_instance_f32 IP_KCPI_KC_trans_KRKT_prs;

volatile arm_status CalcSt_prs;


// Kalman initialization function
void Kalman_Init_prs(kalman_prs* KALMAN)
{
    arm_mat_init_f32(&P_prs, 4, 4, P_f32_prs);
    arm_mat_init_f32(&I_prs, 4, 4, I_f32_prs);

    arm_mat_init_f32(&Xsensor_prs, 1, 1, KALMAN->Xsensor_f32_prs);
    arm_mat_init_f32(&X_prs, 4, 1, X_f32_prs);

    arm_mat_init_f32(&A_prs, 4, 4, A_f32_prs);
    arm_mat_init_f32(&B_prs, 4, 1, B_f32_prs);

    arm_mat_init_f32(&U_prs, 1, 1, KALMAN->U_f32_prs);
    arm_mat_init_f32(&Y_prs, 1, 1, Y_f32_prs);

    arm_mat_init_f32(&C_prs, 1, 4, C_f32_prs);

    arm_mat_init_f32(&Q_prs, 4, 4, Q_f32_prs);
    arm_mat_init_f32(&R_prs, 1, 1, R_f32_prs);

    arm_mat_init_f32(&AT_prs, 4, 4, AT_f32_prs);
    arm_mat_init_f32(&CT_prs, 4, 1, CT_f32_prs);
    arm_mat_init_f32(&KT_prs, 1, 4, KT_f32_prs);

    arm_mat_init_f32(&AX_prs, 4, 1, AX_f32_prs);
    arm_mat_init_f32(&BU_prs, 4, 1, BU_f32_prs);
    arm_mat_init_f32(&AX_BU_prs, 4, 1, AX_BU_f32_prs);

    arm_mat_init_f32(&CXsensor_prs, 1, 1, CXsensor_f32_prs);
    arm_mat_init_f32(&CX_prs, 1, 1, CX_f32_prs);

    arm_mat_init_f32(&AP_prs, 4, 4, AP_f32_prs);
    arm_mat_init_f32(&APAT_prs, 4, 4, APAT_f32_prs);
    arm_mat_init_f32(&APAT_Q_prs, 4, 4, APAT_Q_f32_prs);

    arm_mat_init_f32(&K_prs, 4, 1, K_f32_prs);
    arm_mat_init_f32(&PCT_prs, 4, 1, PCT_f32_prs);
    arm_mat_init_f32(&CP_prs, 1, 4, CP_f32_prs);
    arm_mat_init_f32(&CPCT_prs, 1, 1, CPCT_f32_prs);
    arm_mat_init_f32(&CPCT_R_prs, 1, 1, CPCT_R_f32_prs);
    arm_mat_init_f32(&CPCT_R_INV_prs, 1, 1, CPCT_R_INV_f32_prs);
    arm_mat_init_f32(&PCPCT_R_INV_prs, 4, 1, PCPCT_R_INV_f32_prs);
    arm_mat_init_f32(&Y_CX_prs, 1, 1, Y_CX_f32_prs);
    arm_mat_init_f32(&KY_KCX_prs, 4, 1, KY_KCX_f32_prs);
    arm_mat_init_f32(&X_KY_KCX_prs, 4, 1, X_KY_KCX_f32_prs);

    arm_mat_init_f32(&KC_prs, 4, 4, KC_f32_prs);
    arm_mat_init_f32(&KR_prs, 4, 1, KR_f32_prs);
    arm_mat_init_f32(&KRKT_prs, 4, 4, KRKT_f32_prs);

    arm_mat_init_f32(&I_KC_prs, 4, 4, I_KC_f32_prs);
    arm_mat_init_f32(&IP_KCP_prs, 4, 4, IP_KCP_f32_prs);
    arm_mat_init_f32(&I_KC_trans_prs, 4, 4, I_KC_trans_f32_prs);
    arm_mat_init_f32(&IP_KCPI_KC_trans_prs, 4, 4, IP_KCPI_KC_trans_f32_prs);
    arm_mat_init_f32(&IP_KCPI_KC_trans_KRKT_prs, 4, 4, IP_KCPI_KC_trans_KRKT_f32_prs);
}


//void Kalman_Init_prs(kalman_prs* KALMAN)
//{
//    // Zero all data buffers
//    memset(X_f32_prs, 0, sizeof(X_f32_prs));
//    memset(P_f32_prs, 0, sizeof(P_f32_prs));
//    memset(I_f32_prs, 0, sizeof(I_f32_prs));
//    memset(KALMAN->Xsensor_f32_prs, 0, sizeof(KALMAN->Xsensor_f32_prs));
//    memset(KALMAN->U_f32_prs, 0, sizeof(KALMAN->U_f32_prs));
//
//    // Identity matrix I (4x4)
//    for (int i = 0; i < 4; i++) {
//        I_f32_prs[i * 4 + i] = 1.0f;
//    }
//
//    // Initial state covariance P (4x4): small uncertainty
//    for (int i = 0; i < 4; i++) {
//        P_f32_prs[i * 4 + i] = 0.01f;
//    }
//
//    // State transition matrix A (example: adjust if needed)
//    memset(A_f32_prs, 0, sizeof(A_f32_prs));
//    A_f32_prs[0] = 1.0f;  A_f32_prs[1] = 0.001f;
//    A_f32_prs[5] = 1.0f;  A_f32_prs[6] = 0.001f;
//    A_f32_prs[10] = 1.0f; A_f32_prs[15] = 1.0f;
//
//    // Input matrix B (adjust as needed)
//    memset(B_f32_prs, 0, sizeof(B_f32_prs));
//    B_f32_prs[2] = 0.001f; // Example: torque influenced by input
//
//    // Measurement matrix C (only measuring theta)
//    memset(C_f32_prs, 0, sizeof(C_f32_prs));
//    C_f32_prs[0] = 1.0f;
//
//    // Process noise covariance Q (low process noise)
//    memset(Q_f32_prs, 0, sizeof(Q_f32_prs));
//    for (int i = 0; i < 4; i++) {
//        Q_f32_prs[i * 4 + i] = 1e-4f;
//    }
//
//    // Measurement noise covariance R
//    R_f32_prs[0] = 1e-2f;
//
//    // Initialize all matrices
//    arm_mat_init_f32(&P_prs, 4, 4, P_f32_prs);
//    arm_mat_init_f32(&I_prs, 4, 4, I_f32_prs);
//
//    arm_mat_init_f32(&Xsensor_prs, 1, 1, KALMAN->Xsensor_f32_prs);
//    arm_mat_init_f32(&X_prs, 4, 1, X_f32_prs);
//
//    arm_mat_init_f32(&A_prs, 4, 4, A_f32_prs);
//    arm_mat_init_f32(&B_prs, 4, 1, B_f32_prs);
//
//    arm_mat_init_f32(&U_prs, 1, 1, KALMAN->U_f32_prs);
//    arm_mat_init_f32(&Y_prs, 1, 1, Y_f32_prs);
//
//    arm_mat_init_f32(&C_prs, 1, 4, C_f32_prs);
//
//    arm_mat_init_f32(&Q_prs, 4, 4, Q_f32_prs);
//    arm_mat_init_f32(&R_prs, 1, 1, R_f32_prs);
//
//    // Temporary and intermediate matrices
//    arm_mat_init_f32(&AT_prs, 4, 4, AT_f32_prs);
//    arm_mat_init_f32(&CT_prs, 4, 1, CT_f32_prs);
//    arm_mat_init_f32(&KT_prs, 1, 4, KT_f32_prs);
//
//    arm_mat_init_f32(&AX_prs, 4, 1, AX_f32_prs);
//    arm_mat_init_f32(&BU_prs, 4, 1, BU_f32_prs);
//    arm_mat_init_f32(&AX_BU_prs, 4, 1, AX_BU_f32_prs);
//
//    arm_mat_init_f32(&CXsensor_prs, 1, 1, CXsensor_f32_prs);
//    arm_mat_init_f32(&CX_prs, 1, 1, CX_f32_prs);
//
//    arm_mat_init_f32(&AP_prs, 4, 4, AP_f32_prs);
//    arm_mat_init_f32(&APAT_prs, 4, 4, APAT_f32_prs);
//    arm_mat_init_f32(&APAT_Q_prs, 4, 4, APAT_Q_f32_prs);
//
//    arm_mat_init_f32(&K_prs, 4, 1, K_f32_prs);
//    arm_mat_init_f32(&PCT_prs, 4, 1, PCT_f32_prs);
//    arm_mat_init_f32(&CP_prs, 1, 4, CP_f32_prs);
//    arm_mat_init_f32(&CPCT_prs, 1, 1, CPCT_f32_prs);
//    arm_mat_init_f32(&CPCT_R_prs, 1, 1, CPCT_R_f32_prs);
//    arm_mat_init_f32(&CPCT_R_INV_prs, 1, 1, CPCT_R_INV_f32_prs);
//    arm_mat_init_f32(&PCPCT_R_INV_prs, 4, 1, PCPCT_R_INV_f32_prs);
//    arm_mat_init_f32(&Y_CX_prs, 1, 1, Y_CX_f32_prs);
//    arm_mat_init_f32(&KY_KCX_prs, 4, 1, KY_KCX_f32_prs);
//    arm_mat_init_f32(&X_KY_KCX_prs, 4, 1, X_KY_KCX_f32_prs);
//
//    arm_mat_init_f32(&KC_prs, 4, 4, KC_f32_prs);
//    arm_mat_init_f32(&KR_prs, 4, 1, KR_f32_prs);
//    arm_mat_init_f32(&KRKT_prs, 4, 4, KRKT_f32_prs);
//
//    arm_mat_init_f32(&I_KC_prs, 4, 4, I_KC_f32_prs);
//    arm_mat_init_f32(&IP_KCP_prs, 4, 4, IP_KCP_f32_prs);
//    arm_mat_init_f32(&I_KC_trans_prs, 4, 4, I_KC_trans_f32_prs);
//    arm_mat_init_f32(&IP_KCPI_KC_trans_prs, 4, 4, IP_KCPI_KC_trans_f32_prs);
//    arm_mat_init_f32(&IP_KCPI_KC_trans_KRKT_prs, 4, 4, IP_KCPI_KC_trans_KRKT_f32_prs);
//
//    // Optional: reset outputs
//    KALMAN->theta_kalman_prs = 0.0f;
//    KALMAN->omega_kalman_prs = 0.0f;
//    KALMAN->torque_kalman_prs = 0.0f;
//    KALMAN->current_kalman_prs = 0.0f;
//}


void Kalman_Filter_prs(kalman_prs* KALMAN, float voltage, float encoder)
{
    KALMAN->U_f32_prs[0] = voltage;
    KALMAN->Xsensor_f32_prs[0] = encoder;

    // Prediction: X̂ = AX + BU
    arm_mat_mult_f32(&A_prs, &X_prs , &AX_prs);
    arm_mat_mult_f32(&B_prs, &U_prs , &BU_prs);
    arm_mat_add_f32(&AX_prs, &BU_prs, &X_prs);

    // P = APAᵀ + Q
    arm_mat_mult_f32(&A_prs, &P_prs , &AP_prs);
    arm_mat_trans_f32(&A_prs, &AT_prs);
    arm_mat_mult_f32(&AP_prs, &AT_prs , &APAT_prs);
    arm_mat_add_f32(&APAT_prs, &Q_prs , &P_prs);

    // Kalman Gain: K = PCT / (CPCT + R)
    arm_mat_trans_f32(&C_prs, &CT_prs);
    arm_mat_mult_f32(&C_prs, &P_prs , &CP_prs);
    arm_mat_mult_f32(&CP_prs, &CT_prs , &CPCT_prs);
    arm_mat_add_f32(&CPCT_prs, &R_prs , &CPCT_R_prs);
    arm_mat_inverse_f32(&CPCT_R_prs, &CPCT_R_INV_prs);
    arm_mat_mult_f32(&P_prs, &CT_prs, &PCT_prs);
    arm_mat_mult_f32(&PCT_prs, &CPCT_R_INV_prs , &K_prs);

    // Z = CXsensor
    arm_mat_mult_f32(&C_prs, &Xsensor_prs , &CXsensor_prs);

    // Update: X̂ = X̂ + K(Z - CX̂)
    arm_mat_mult_f32(&C_prs, &X_prs , &CX_prs);
    arm_mat_sub_f32(&CXsensor_prs, &CX_prs, &Y_CX_prs);
    arm_mat_mult_f32(&K_prs, &Y_CX_prs , &KY_KCX_prs);
    arm_mat_add_f32(&X_prs, &KY_KCX_prs , &X_prs);

    // Update P: P = (I - KC)P(I - KC)ᵀ + KRKT
    arm_mat_mult_f32(&K_prs, &C_prs , &KC_prs);
    arm_mat_mult_f32(&K_prs, &R_prs , &KR_prs);
    arm_mat_trans_f32(&K_prs, &KT_prs);
    arm_mat_mult_f32(&KR_prs, &KT_prs , &KRKT_prs);
    arm_mat_sub_f32(&I_prs, &KC_prs , &I_KC_prs);
    arm_mat_trans_f32(&I_KC_prs, &I_KC_trans_prs);
    arm_mat_mult_f32(&I_KC_prs, &P_prs , &IP_KCP_prs);
    arm_mat_mult_f32(&IP_KCP_prs, &I_KC_trans_prs , &IP_KCPI_KC_trans_prs);
    arm_mat_add_f32(&IP_KCPI_KC_trans_prs, &KRKT_prs , &P_prs);

    // Store estimated state
    KALMAN->theta_kalman_prs = X_f32_prs[0];
    KALMAN->omega_kalman_prs = X_f32_prs[1];
    KALMAN->torque_kalman_prs = X_f32_prs[2];
    KALMAN->current_kalman_prs = X_f32_prs[3];
}

