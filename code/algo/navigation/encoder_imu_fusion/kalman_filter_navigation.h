#ifndef _KALMAN_FILTER_NAVIGATION_H_
#define _KALMAN_FILTER_NAVIGATION_H_

#include "zf_common_headfile.h"

#define STATE_SIZE 6
#define MEASUREMENT_SIZE 4

typedef struct
{
    float x[STATE_SIZE];                         // state vector
    float P[STATE_SIZE][STATE_SIZE];             // state covariance matrix
    float A[STATE_SIZE][STATE_SIZE];             // state transition matrix
    float B[STATE_SIZE];                         // control matrix
    float H[MEASUREMENT_SIZE][STATE_SIZE];       // measurement matrix
    float Q[STATE_SIZE][STATE_SIZE];             // process noise covariance matrix
    float R[MEASUREMENT_SIZE][MEASUREMENT_SIZE]; // measurement noise covariance matrix
    float u[1];                                  // control vector
} kalman_filter_navigation_t;

void kalman_filter_navigation_init(kalman_filter_navigation_t *kf);
void kalman_filter_navigation_predict(kalman_filter_navigation_t *kf);
void kalman_filter_navigation_update(kalman_filter_navigation_t *kf, float z[MEASUREMENT_SIZE]);

#endif /* CODE_ALGORITHM_NAVIGATION_ENCODER_IMU_FUSION_KALMAN_FILTER_NAVIGATION_H_ */
