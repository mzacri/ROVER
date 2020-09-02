/*****************************************************************************//**
  * @file    sensors.c
  * @author  M'barek ZACRI
  * @date    11 mai 2020
  * @brief 	 Related definitions to the state sensors of the rover actuators.
  * Every sensor relies on different concept.
 *******************************************************************************/
#include <RoverConfig.h>
#include "main.h"

//Servo pos in rad:
float SenseServoPos(volatile uint16_t inServo){
    float ret=0;
	ret=9.4682875e-4*inServo-0.98;
	if(ret>0.4){
		ret=0.4;
	} else if (ret < -0.4) {
		ret=-0.4;
	}
	return ret;
}

float EstimateServoPos(float servoControl){
	static float so=0,so0=0,sc0=0,sc=0;
	sc=-servoControl;
	//so=(-5.1198257e-5 * (sc+sc0) - so0)/-1.22; //100 ms
	so=(-9.3087740e-05 * (sc+sc0) - so0)/-1.4	; //60 ms
	sc0=sc;
	so0=so;
	return so;
}
//Sense Battery Voltage:
float SenseBatteryVoltage(volatile uint16_t inBattery){
	return (0.00894562 * (int)inBattery);
}


//BLDC speed:
float Sensor3_speed(int command,uint32_t *timestamp){
	static float speed=0;
	static int sign=0, first=1;
	if (command > 0){
		sign=1;
	} else if (command < 0) {
		sign=-1;
	} else {
		sign=0;
	}
	if(timestamp[0] >0 && (TIM5->SR & TIM_SR_UIF)==0){
		if(first == 0){
			speed= sign*((float)20/RATIO)*((float)HAL_RCC_GetHCLKFreq()/((TIM5->PSC+1)*timestamp[0]));
		} else {
			first=0;
		}
	} else {
		first=1;
		TIM5->SR &= ~ (TIM_SR_UIF);
		timestamp[0]=0;
		speed= 0;
	}
	return speed;
}

float Sensor4_speed(int command, volatile uint32_t *timestamp,volatile uint16_t* inGPIOA){
	static float speed=0;
	static int sign=0, first=1;
	//Direction calculation:
	if(command != 0){
		if (DMA2->LISR & DMA_LISR_TCIF1){
			DMA2->LIFCR|=DMA_LIFCR_CTCIF1;
			uint8_t a,b,c,d,e,f;
			a=(uint8_t)((inGPIOA[0] & GPIO_IDR_ID0)>>0);
			b=(uint8_t)((inGPIOA[0] & GPIO_IDR_ID1)>>1);
			c=(uint8_t)((inGPIOA[0] & GPIO_IDR_ID2)>>2);
			d=(uint8_t)((inGPIOA[1] & GPIO_IDR_ID0)>>0);
			e=(uint8_t)((inGPIOA[1] & GPIO_IDR_ID1)>>1);
			f=(uint8_t)((inGPIOA[1] & GPIO_IDR_ID2)>>2);
			sign= -(2*(int)(((c&d) | (a&e)) | (b&f))  - 1);
		}
	}
	//Speed calculation:
		if(timestamp[0] > MINTIMESTAMP && (TIM5->SR & TIM_SR_UIF)==0){
			if(first == 0){
				speed= sign*((float)20/RATIO)*((float)HAL_RCC_GetHCLKFreq()/((TIM5->PSC+1)*timestamp[0]));
			} else {
				first=0;
			}
		} else {
			first=1;
			TIM5->SR &= ~ (TIM_SR_UIF);
			timestamp[0]=0;
			speed= 0;
		}
	return speed;

}
