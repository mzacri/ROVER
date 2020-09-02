/*****************************************************************************//**
  * @file    Raspberry.h
  * @author  M'barek ZACRI
  * @date    25 mai 2020
  * @brief 	 Functions related to communication with the raspberry pi.
 *******************************************************************************/
#ifndef INC_COMMUNICATION_RASPBERRY_H_
#define INC_COMMUNICATION_RASPBERRY_H_

#include <Sensors/RoverStateSensors.h>
#include "main.h"

/**
 * @defgroup Communication Communication
 * @brief    Functions to interface communication with PC.
 * @{
 */

/**
 * @brief   Update commands ( Servo and BLDC ) using the received data buffer.
 *
 * @param	pData: pointer to Rx data buffer.
 *
 * @param	bldcCommand: pointer to BLDC command variable.
 *
 * @param	servoCommand: pointer to servo speed variable.
 */
void Update_command(uint8_t* pData, float* bldcCommand, float* servoCommand);

/**
 * @brief   Send the rover state structure  using the tx data buffer via SPI at 1 Mb/s.
 *
 * @param	state: rover state strucure to send.
 *
 * @param	hspi: SPI handle.
 *
 * @param	txComData: pointer to tx data buffer.
 */
void Send_state_SPI(ROVER_STATE state, uint8_t* txComData);

/**
 * @}
 */
#endif /* INC_COMMUNICATION_RASPBERRY_H_ */
