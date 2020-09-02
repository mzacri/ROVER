/*****************************************************************************//**
  * @file    lowPassFilters.h
  * @author  M'barek ZACRI
  * @date    25 mai 2020
  * @brief 	 Numeric low pass filters
 *******************************************************************************/
#ifndef INC_SENSORS_FILTERS_LOWPASSFILTERS_H_
#define INC_SENSORS_FILTERS_LOWPASSFILTERS_H_

/**
 * @defgroup 	Filters Filters
 * @brief	 	Sensors low pass filtering.
 * @{
 */
/**
 * @defgroup LPF Low pass filters
 * @{
 */
/**
 * @note  Due to static variables, different filters needs to be declared for every filtered variable.
 */
/**
 * @brief  Speed filtering at bandwidth corresponding to 80 ms.
 * @param  speed: speed value to filter.
 *
 * @retval 	Filtered speed
 */
float LowPassFilter1(float speed);

/**
 * @brief  Servo Position low pass filtering of third order at bandwidth corresponding to 30 ms .
 * @param  servoPose: servo position value to filter.
 *
 * @retval 	Filtered servo position
 */

float LowPassFilter2(float servoPose);
/**
 * @brief  battery Voltage filtering at bandwidth corresponding to 150 ms.
 * @param  servoPose: servo position value to filter.
 *
 * @retval 	Filtered battery voltage
 */
float LowPassFilter3(float batteryVolt);
/**
 * @}
 */
/**
 * @}
 */

#endif /* INC_SENSORS_FILTERS_LOWPASSFILTERS_H_ */
