/*****************************************************************************//**
  * @file    speedControl.h
  * @author  M'barek ZACRI
  * @date    27 avr. 2020
  * @brief 	 Related declarations to the control of the rover actuators.
  *
  * @note	Different control methods are possible for the brushless motor.Since the brushless driver is highly non linear around small
  * speeds, non linear control is needed for more accurate results.
  *	Higher Speed reduction ratio may help shifting the usable window
  * of speed to the most linear area of the driver.
  *
 *******************************************************************************/

#ifndef INC_SPEEDCONTROL_H_
#define INC_SPEEDCONTROL_H_

#include <RoverConfig.h>
#include <Sensors/RoverStateSensors.h>
/**
 * @defgroup Control Control
 * @brief   State control strategies
 * @{
 */

/**
 * @defgroup BLDCControl Brushless motor control
 * @{
 */

#if	CONTROL== RE
/**
 * @brief  Compute BLDC closed loop control using a state feedback.
 * Returned value is bounded between -1500 and 1500. The control value
 * is then mapped to the 2ms window of the pulse width that controls the
 * motor driver. TIM3 CH3 generates the appropriate control pulse based on
 * the output of this function.
 *
 * @note  A minimal control value is assured by the dead zone management.
 *
 * @param  re_command: speed command to reach.
 *
 * @param  re_speed: feedback value of speed.
 *
 * @retval 	Brushless control
 */
int Re(float re_command,float re_speed);
#elif	CONTROL == OL
/**
 * @brief  Open loop control.
 *
 * @note   Control value is computed using the motor model.
 * @param  ol_command: speed command to reach.
 *
 * @retval 	Brushless control
 */
int OpenLoop(float ol_command);
#endif
/**
 * @}
 */

/**
 * @brief  If internal command is chosen in @ref RoverConfig, this function alternate
 * between two values of command every 2 seconds. For the BLDC, it alternates between COMMAND and -COMMAND.
 * For the servo, it alternates between 1 and -1.
 *
 * @param  command: Pointer to speed command.
 *
 * @param  servoSpeed: Pointer to servoSpeed command.
 *
 * @retval 	Brushless control
 */
void intCommandAlternator(float* command, float* servoSpeed);

/**
 * @defgroup  ServoControl Servo motor control
 * @{
 */

/**
 * @brief   Transform servo Speed to a position of the servo. The control value
 * is then mapped to the 2ms window of the pulse width that controls the
 * servo motor. TIM3 CH2 generates accordingly.
 *
 * @param  servoCommand: Servo command to execute.
 *
 * @retval 	Servo control
 */
int Servo_command(float servoCommand);
/**
 * @}
 */
#endif /* INC_SPEEDCONTROL_H_ */

