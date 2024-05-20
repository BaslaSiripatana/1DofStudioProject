/*
 * kalman_filter.h
 *
 *  Created on: May 20, 2024
 *      Author: buzz
 */

#ifndef INC_KALMAN_FILTER_H_
#define INC_KALMAN_FILTER_H_

#include "arm_math.h"

// Define the Kalman filter structure
typedef struct {
    arm_matrix_instance_f32 A;   // State transition matrix
    arm_matrix_instance_f32 B;   // Control input matrix
    arm_matrix_instance_f32 C;   // Observation matrix
    arm_matrix_instance_f32 G;   // Process noise gain matrix
    arm_matrix_instance_f32 Q;   // Process noise covariance
    arm_matrix_instance_f32 R;   // Measurement noise covariance
    arm_matrix_instance_f32 P;   // Error covariance
    arm_matrix_instance_f32 x;   // State estimate
    arm_matrix_instance_f32 K;   // Kalman gain
    arm_matrix_instance_f32 y;   // Measurement residual
    arm_matrix_instance_f32 S;   // Innovation covariance
    arm_matrix_instance_f32 temp1;
    arm_matrix_instance_f32 temp2;
    arm_matrix_instance_f32 temp3;
    arm_matrix_instance_f32 temp4;
    arm_matrix_instance_f32 temp5;
    arm_matrix_instance_f32 temp6;
} KalmanFilter;

// Function to initialize the Kalman filter
void KalmanFilter_Init(KalmanFilter* kf, float32_t* A_data, float32_t* B_data, float32_t* C_data,
                       float32_t* G_data, float32_t* Q_data, float32_t* R_data, float32_t* P_data,
                       float32_t* x_data, float32_t* K_data, float32_t* temp1_data,
                       float32_t* temp2_data, float32_t* temp3_data, float32_t* temp4_data,
                       float32_t* temp5_data, float32_t* temp6_data, float32_t* S_data);

// Function to perform the prediction step
void KalmanFilter_Predict(KalmanFilter* kf, float32_t* u_data, float32_t* w_data);

// Function to perform the update step
void KalmanFilter_Update(KalmanFilter* kf, float32_t* y_data);

#endif /* INC_KALMAN_FILTER_H_ */
