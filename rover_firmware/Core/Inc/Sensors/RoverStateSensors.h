/*****************************************************************************//**
  * @file    sensors.h
  * @author  M'barek ZACRI
  * @date    11 mai 2020
  * @brief 	 Related declarations to the state sensors of the rover actuators.
 *******************************************************************************/
#include "stdint.h"
#ifndef INC_CONTROL_SENSORS_H_
#define INC_CONTROL_SENSORS_H_
/**
 * @defgroup  Sensors Sensors
 * @brief     Drivers of the rover state's sensors.
 * @{
 */

/**
 * @defgroup BLDCSense Brushless motor speed
 * @brief    Different speed sensing strategies of the BLDC motor
 * @{
 */

/**
 * @brief  Based on period measurement of XOR combination output of three HALL sensors.The rising edge of XOR signal is responsible of:
 * 			- Capturing the counter value of the timer TIM5 channel 1 configured as input.
 * 			- enabling a DMA request which transfers to memory the captured value.
 * 			- Reseting the timer TIM5 counter (Reset mode).
 * 		   The speed value is then computed using the captured value.
 *
 * @note   Direction of the motor rotation is estimated using the control value. This solution is unfaithful if
 * 		   direction changes instantly from one way to the other.
 * 		   However, Kalman filtering plus low pass filtering correct this issue quiet well for instant changes of direction between small speed values ( up to 2 m/s).
 *
 * @param  command: Motor control value
 *
 * @param  timestamp: Pointer to the counter's captured value.
 *
 * @retval 	Brute speed value
 */
float Sensor3_speed(int command, volatile uint32_t *timestamp);
/**
 * @brief  Based on period measurement of XOR combination output of three HALL sensors.The rising edge of XOR signal is responsible of:
 * 			- Capturing the counter value of the timer TIM5 channel 1 configured as input.
 * 			- enabling a DMA request which transfers to memory the captured value.
 * 			- Reseting the timer TIM5 counter (Reset mode).
 * 		   The speed value is then computed using the captured value.
 *
 * @note   Direction of the motor rotation is calculated using two consecutive states of HALL sensors.
 *		   Every toggling of XOR combination triggers a DMA transfer of the GPIOA pins state. Combinational logic function deduct the rotation direction.
 *		   This solution is the most faithful to reality but comes with a drawback: the calculation of direction may falsify the speed value due to errors.
 *		   A first order low pass filter is implemented to correct this issue, but higher order is needed.
 *
 * @note   Motor control value is used to stop computing rotation direction while braking. It helps improving the result since direction won't change if control is null.
 *
 * @param  command: Motor control value
 *
 * @param  timestamp: Pointer to the counter's captured value.
 *
 * @param  inGPIOA:	Pointer to two captured GPIOA pins state transfered by DMA.
 *
 * @retval 	Brute speed value
 */
float Sensor4_speed(int command, volatile uint32_t *timestamp,volatile uint16_t* inGPIOA);
/**
 * @}
 */

/**
 * @defgroup ServoSense Servo motor position
 * @brief    Sensors of the front wheels orientation
 * @{
 */

/**
 * @brief  Compute servo position based on its internal potentiometer.
 *         An ADC conversion of the potentiometer signal is triggered by TIM3 every 20 ms.
 *         Then, the completed conversion triggers a DMA stream to transfer the ADC Data register to memory (inServo).
 *
 * @note   The output needs to be filtered due to noisy ADC conversion. Low pass filtering is used.
 *
 * @param  inServo: potentiometer value converted by ADC.
 *
 * @retval 	Brute servo position
 */
float SenseServoPos(volatile uint16_t inServo);

/**
 * @brief  Estimate servo position using its motion model.
 *
 * @param servoControl: servo control value.
 *
 * @retval 	estimated servo position in radian
 */
float EstimateServoPos(float servoControl);


/**
 * @brief  Returns the battery's voltage measurement issued from the ADC conversion.
 *
 * @param  inBattery: Converted ADC value.
 *
 * @retval 	Battery voltage
 */
float SenseBatteryVoltage(volatile uint16_t inBattery);
/**
 * @}
 */

/**
 * @}
 */

/**
 * @brief  Rover state structure
 */
typedef struct ROVER_STATE{
	float rearSpeed;  /*! Brushless motor speed */
	float servoPose;   /*! Servo position */
	float x;              /*! x coordinate */
	float y;             /*! y coordinate */
	float theta;          /*! rover orientation */
	float covariance[9];  /*! Covariance Matrix */
	float batteryVolt; /*! Battery voltage */
	float w; /*vitesse de rotation modèle diff drive*/
	float speed_covariance[4]; /*Matrice de covariance sur rearSpeed,w */
} ROVER_STATE;

/**
 * @}
 */
#endif /* INC_CONTROL_SENSORS_H_ */
