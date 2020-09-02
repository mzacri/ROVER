/*****************************************************************************//**
  * @file    compFilter.h
  * @author  M'barek ZACRI
  * @date    1 mai 2020
  * @brief 	Complementary filtering to compute Eulers angles by fusing
  * accelerometer's and gyroscope's noisy data.
 *******************************************************************************/
#ifndef INC_COMPFILTER_H_
#define INC_COMPFILTER_H_
#include <Sensors/MPU6050/sd_hal_mpu6050.h>

/**
 * @addtogroup Filters
 * @{
 */

/**
 * @defgroup CompFilter Complementary Filtering
 * @brief Fusion Accelerometer and gyroscope to estimate Euler's angles.
 * @{
 */
/**
 * @brief  Euler's angles data structure.
 */
typedef struct EULER_ANGLES{
	float accel_roll;
	float accel_pitch;

	float gyro_roll;
	float gyro_pitch;
	float gyro_yaw;

	float roll;
	float pitch;
	float yaw;
} EULER_ANGLES;

/**
 * @brief  Compute Accelerometer's estimation of Euler's angles.
 *
 * @param  angles: Pointer to Euler's angles structure.
 *
 * @param  mpu: mpu6050 handle.
 */
void computeAccelAngles(EULER_ANGLES *angles,SD_MPU6050 mpu);
/**
 * @brief  Integrate gyroscope rates to  estimate Euler's angles.
 *
 * @param  angles: Pointer to Euler's angles structure.
 *
 * @param  mpu: mpu6050 handle.
 *
 * @param  deltaT: discretisation period.
 */
void computeGyroAngles(EULER_ANGLES *angles,SD_MPU6050 mpu,float deltaT);

/**
 * @brief  Compute Euler's angles estimation based on both gyroscope and accelerometer datas.
 *
 * @param  angles: Pointer to Euler's angles structure.
 *
 * @param  alpha: Complementary coefficient.
 */
void computeAngles(EULER_ANGLES *angles,float alpha);

/**
 * @brief  Init gyroscope angles with accelerometer estimation.
 *
 * @param  angles: Pointer to Euler's angles structure.
 *
 * @param  mpu: mpu6050 handle.
 */
void initAngles(EULER_ANGLES *angles,SD_MPU6050 mpu);
/**
 * @}
 */
/**
 * @}
 */
#endif /* INC_COMPFILTER_H_ */
