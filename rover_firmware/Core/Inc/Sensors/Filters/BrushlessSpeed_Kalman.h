/*****************************************************************************//**
  * @file    KalmanFilter.h
  * @author  M'barek ZACRI
  * @date    1 mai 2020
  * @brief 	Kalman filtering of the brushless motor's speed. IMU's X axis
  * accelerometer data is fusioned with the speed sensor's data.
 *******************************************************************************/
#ifndef INC_KALMANFILTER_H_
#define INC_KALMANFILTER_H_

/* Include Files */
#include <stddef.h>
#include <stdlib.h>

/**
 * @addtogroup Filters
 * @{
 */

/**
 * @defgroup Kalman Kalman Filtering
 * @brief Filter speed sensor's output.
 * @{
 */
/**
 * @brief  Filter speed sensor's output using the IMU X axis acceleration data.
 *
 * @param  ax: X axis acceleration.
 *
 * @param  zm: speed to filter.
 *
 * @retval 	Filtered speed
 */
extern double KalmanFilter(double ax, double zm);

/**
 * @brief  Kalman filter initialisation.
 */
extern void KalmanFilter_init(void);

/**
 * @brief  return the covariance matrix of the Kalman state.
 */
extern float KalmanVariance(void);
/**
 * @}
 */
/**
 * @}
 */

#endif /* INC_KALMANFILTER_H_ */
