/*
 * kalman_filter.c
 *
 *  Created on: May 20, 2024
 *      Author: buzz
 */

#include "kalman_filter.h"

void KalmanFilter_Init(KalmanFilter* kf, float32_t* A_data, float32_t* B_data, float32_t* C_data,
                       float32_t* G_data, float32_t* Q_data, float32_t* R_data, float32_t* P_data,
                       float32_t* x_data, float32_t* K_data, float32_t* temp1_data,
                       float32_t* temp2_data, float32_t* temp3_data, float32_t* temp4_data,
                       float32_t* temp5_data, float32_t* temp6_data, float32_t* S_data) {
    arm_mat_init_f32(&kf->A, 3, 3, A_data);
    arm_mat_init_f32(&kf->B, 3, 1, B_data);
    arm_mat_init_f32(&kf->C, 1, 3, C_data);
    arm_mat_init_f32(&kf->G, 3, 1, G_data); // Process noise gain matrix is now 3x1
    arm_mat_init_f32(&kf->Q, 1, 1, Q_data); // Process noise covariance is now 1x1
    arm_mat_init_f32(&kf->R, 1, 1, R_data);
    arm_mat_init_f32(&kf->P, 3, 3, P_data);
    arm_mat_init_f32(&kf->x, 3, 1, x_data);
    arm_mat_init_f32(&kf->K, 3, 1, K_data); // Initialize K matrix
    arm_mat_init_f32(&kf->temp1, 3, 3, temp1_data);
    arm_mat_init_f32(&kf->temp2, 3, 3, temp2_data);
    arm_mat_init_f32(&kf->temp3, 3, 1, temp3_data);
    arm_mat_init_f32(&kf->temp4, 1, 1, temp4_data);
    arm_mat_init_f32(&kf->temp5, 3, 1, temp5_data);
    arm_mat_init_f32(&kf->temp6, 1, 3, temp6_data);
    arm_mat_init_f32(&kf->S, 1, 1, S_data);  // Initialize S matrix
}

void KalmanFilter_Predict(KalmanFilter* kf, float32_t* u_data, float32_t* w_data) {
    arm_matrix_instance_f32 u;
    arm_matrix_instance_f32 w;
    arm_mat_init_f32(&u, 1, 1, u_data);
    arm_mat_init_f32(&w, 1, 1, w_data);

    // x = A * x + B * u + G * w
    arm_mat_mult_f32(&kf->A, &kf->x, &kf->temp3);
    arm_mat_mult_f32(&kf->B, &u, &kf->temp5);
    arm_mat_add_f32(&kf->temp3, &kf->temp5, &kf->temp3);
    arm_mat_mult_f32(&kf->G, &w, &kf->temp5);
    arm_mat_add_f32(&kf->temp3, &kf->temp5, &kf->x);

    // P = A * P * A^T + G * Q * G^T
    arm_mat_mult_f32(&kf->A, &kf->P, &kf->temp1);
    arm_mat_trans_f32(&kf->A, &kf->temp2);
    arm_mat_mult_f32(&kf->temp1, &kf->temp2, &kf->P);
    arm_mat_mult_f32(&kf->G, &kf->Q, &kf->temp3);
    arm_mat_trans_f32(&kf->G, &kf->temp6);
    arm_mat_mult_f32(&kf->temp3, &kf->temp6, &kf->temp1);
    arm_mat_add_f32(&kf->P, &kf->temp1, &kf->P);
}

void KalmanFilter_Update(KalmanFilter* kf, float32_t* y_data) {
    // Innovation residual: y - C * x
    arm_matrix_instance_f32 y;
    arm_mat_init_f32(&y, 1, 1, y_data);
    arm_mat_mult_f32(&kf->C, &kf->x, &kf->temp4);
    arm_mat_sub_f32(&y, &kf->temp4, &y);

    // Innovation covariance: S = C * P * C^T + R
    arm_mat_mult_f32(&kf->C, &kf->P, &kf->temp6);
    arm_mat_trans_f32(&kf->C, &kf->temp3);
    arm_mat_mult_f32(&kf->temp6, &kf->temp3, &kf->S);
    arm_mat_add_f32(&kf->S, &kf->R, &kf->S);

    // Kalman gain: K = P * C^T * S^-1
    arm_mat_inverse_f32(&kf->S, &kf->temp4);
    arm_mat_mult_f32(&kf->P, &kf->temp3, &kf->temp5);
    arm_mat_mult_f32(&kf->temp5, &kf->temp4, &kf->K);

    // Corrected state estimate: x = x + K * y
    arm_mat_mult_f32(&kf->K, &y, &kf->temp3);
    arm_mat_add_f32(&kf->x, &kf->temp3, &kf->x);

    // Corrected estimate covariance: P = (I - K * C) * P
    arm_matrix_instance_f32 I;
    float32_t I_data[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    arm_mat_init_f32(&I, 3, 3, I_data);
    arm_mat_mult_f32(&kf->K, &kf->C, &kf->temp1);
    arm_mat_sub_f32(&I, &kf->temp1, &kf->temp2);
    arm_mat_mult_f32(&kf->temp2, &kf->P, &kf->P);
}
