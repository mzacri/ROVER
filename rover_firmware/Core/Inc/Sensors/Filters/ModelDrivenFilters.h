/*****************************************************************************//**
  * @file    ModelDrivenFilters.h
  * @author  M'barek ZACRI
  * @date    6 juil. 2020
  * @brief 	 Model driven filters
 *******************************************************************************/

#ifndef INC_SENSORS_FILTERS_MODELDRIVENFILTERS_H_
#define INC_SENSORS_FILTERS_MODELDRIVENFILTERS_H_

/**
 * @addtogroup 	Filters Model driven filters
 * @brief	 	Sensors model driven filtering.
 * @{
 */

/**
 * @brief  If servoPose is different by far from servo model output,
 *         the servoPose value is rejected and last value is repeated.
 *
 * @param  servoPose: adapted ADC output value.
 *
 * @param  servoControl: value of servo control to inject in the model.
 *
 * @retval 	Filtered sservo pose
 */
float ServoPoseFilter(float servoPose,float servoControl);

/**
 * @brief  If speed is different by far from  speed model output,
 *         the speed value is rejected and last value is repeated.
 *
 * @param  speed: speed sensor output value.
 *
 * @param  brushlessControl: value of brushless motor control to inject in the model.
 *
 * @retval 	Filtered speed
 */

float SpeedFilter(float speed, float brushlessControl);

#endif /* INC_SENSORS_FILTERS_MODELDRIVENFILTERS_H_ */
