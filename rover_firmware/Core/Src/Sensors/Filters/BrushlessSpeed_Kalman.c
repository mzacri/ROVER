/*****************************************************************************//**
  * @file    KalmanFilter.c
  * @author  M'barek ZACRI
  * @date    1 mai 2020
  * @brief 	Kalman filtering of the brushless motor's speed. IMU's X axis
  * accelerometer data is fusioned with the speed sensor's data.
 *******************************************************************************/
/* Include Files */
#include <Sensors/Filters/BrushlessSpeed_Kalman.h>

/* Variable Definitions */
static double x_est[2];
static double P_est[4];

/* Function Definitions */

float KalmanVariance(void)
{
	  return (float)P_est[3];
}

double KalmanFilter(double ax, double zm)
{
  double K[2];
  int i0;
  double d0;
  static const double a[4] = { 1.0, 0.0, 0.02, 1.0 };

  double b_a[4];
  double b;
  double y;
  static const double Q[4] = { 0.0, 0.0, 0.0, 5e-4 };

  double dv0[4];

  K[0] = (x_est[0] + 0.02 * x_est[1]);
  K[1] = x_est[1] + 0.02 * ax;
  for (i0 = 0; i0 < 2; i0++) {
    x_est[i0] = K[i0];
    d0 = a[i0 + 2];
    b_a[i0] = a[i0] * P_est[0] + d0 * P_est[1];
    b_a[i0 + 2] = a[i0] * P_est[2] + d0 * P_est[3];
  }

  /* Update: */
  d0 = 0.0;
  for (i0 = 0; i0 < 2; i0++) {
    P_est[i0] = b_a[i0] + b_a[i0 + 2] * 0.02;
    P_est[i0 + 2] = (b_a[i0 + 2]) + Q[i0 + 2];
    d0 += (double)i0 * x_est[i0];
  }

  b = zm - d0;
  y = 1.0 / (P_est[3] +
             0.013);
  for (i0 = 0; i0 < 2; i0++) {
    d0 = P_est[i0 + 2] * y;
    K[i0] = d0;
    x_est[i0] += d0 * b;
    b_a[i0 + 2] = K[i0];
    d0 = b_a[i0 + 2];
    dv0[i0] = P_est[i0] - d0 * P_est[1];
    dv0[i0 + 2] = P_est[i0 + 2] - d0 * P_est[3];
  }

  P_est[0] = dv0[0];
  P_est[1] = dv0[1];
  P_est[2] = dv0[2];
  P_est[3] = dv0[3];
  return x_est[1];
}

void KalmanFilter_init(void)
{
  x_est[0] = 0.0;
  x_est[1] = 0.0;
  P_est[0] = 0.0;
  P_est[1] = 0.0;
  P_est[2] = 0.0;
  P_est[3] = 0.0;
}

