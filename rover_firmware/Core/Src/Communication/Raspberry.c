/*****************************************************************************//**
  * @file    Raspberry.h
  * @author  M'barek ZACRI
  * @date    25 mai 2020
  * @brief 	 Functions related to communication with the raspberry pi.
 *******************************************************************************/

#include <string.h>
#include <Sensors/RoverStateSensors.h>
#include <Sensors/RoverStateSensors.h>
#include "main.h"
#include "RoverConfig.h"

//Command update:
void Update_command(uint8_t* pData, float* bldcCommand, float* servoCommand){
			memcpy(bldcCommand, pData, sizeof(float));
			if (*bldcCommand > MAX_COMMAND || *bldcCommand < -MAX_COMMAND){
				*bldcCommand=0;
			}
			memcpy(servoCommand, (pData+4), sizeof(float));
}

//Send state:
void Send_state_SPI(ROVER_STATE state,uint8_t* txComData){
	static uint16_t com_id=1;
	float servoPose = state.servoPose;
	float batteryVolt = state.batteryVolt;
	float* covariance = state.covariance;
	float speed = state.rearSpeed;
	float x = state.x;
	float y = state.y;
	float theta = state.theta;
	float* cov = state.speed_covariance;
	float w=state.w;

	memcpy(txComData,&batteryVolt,sizeof(float)); //Copy Battery voltage
	memcpy((txComData+4),&servoPose,sizeof(float)); //Copy servoPose
	memcpy((txComData+8),&x,sizeof(float));
	memcpy((txComData+12),&y,sizeof(float));
	memcpy((txComData+16),&theta,sizeof(float));
	memcpy((txComData+20),covariance,9* sizeof(float));
	memcpy((txComData+56),&speed,sizeof(float));
	memcpy((txComData+60),&w,sizeof(float));  //Copy speed
	memcpy((txComData+64),cov,4*sizeof(float));
	memcpy((txComData+80),&com_id,sizeof(uint16_t));
	if (com_id==65530){
		com_id=1;
	} else {
		com_id++;
	}
    //tx registers are sent via DMA once a SPI transaction is initiated by the Raspberry
}

