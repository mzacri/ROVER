/*****************************************************************************//**
  * @file   timersConfig.c
  * @author  M'barek ZACRI
  * @date    27 avr. 2020
  * @brief 	Timers configuration.
 *******************************************************************************/
#ifndef SRC_TIMERSCONFIG_C_
#define SRC_TIMERSCONFIG_C_

#include <RoverConfig.h>
#include "main.h"

void TIM2_Init(void){

	//------RCC enable
	  RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;

	//------TIM2 Config : Input Compare mode(Rear Speed):
	  TIM2->PSC=0;
	  TIM2->ARR=0xffffffff;
}

void TIM5_Init(void){

	//------RCC enable
	  RCC->APB1ENR|=RCC_APB1ENR_TIM5EN;
	  RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN; //Enable GPIOA

	//------GPIO:
	//PA0,1,2 (TIM5):
	  GPIOA->MODER|=GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1 | GPIO_MODER_MODE2_1; //Alternate function mode for PA0 + PA1 + PA2
	  GPIOA->AFR[0]|=GPIO_AFRL_AFRL0_1 | GPIO_AFRL_AFRL1_1 | GPIO_AFRL_AFRL2_1 ;  //TIM5_CH1,2,3 as Alternate function for PA0,1,2

	//------TIM5 Config : Input Compare mode(Rear Speed):
	  TIM5->PSC=0; //Max precision
	  TIM5->ARR=MAXTIMESTAMP;  //Correspond to minimum detectable speed
	  TIM5->DCR|=TIM_DCR_DBA_3|TIM_DCR_DBA_2|TIM_DCR_DBA_0;  //DMA base Address : CCR1
	  TIM5->SMCR|=TIM_SMCR_TS_2 | TIM_SMCR_TS_0; // Tl1FP1 Edge Detector as Trigger
	  TIM5->SMCR|=TIM_SMCR_SMS_2; // Reset mode
	  TIM5->CR1|=TIM_CR1_URS; //UEV only on overflow/underflow of the counter
	  //***Channel1 (Speed):
	  TIM5->CCMR1|=TIM_CCMR1_IC1F; //Filter fclk/32 8 ticks
	  TIM5->CR2|=TIM_CR2_TI1S; //Enable XOR combination
	  TIM5->DIER|=TIM_DIER_CC1DE;  //Enable DMA request on channel 1
	  TIM5->CCMR1|=TIM_CCMR1_CC1S_0 ; //Input mode: Map input to Tl1
	  //TIM5->CCMR1|=TIM_CCMR1_IC1F; //Filter fclk/32 8 ticks
	  TIM5->CCER|=TIM_CCER_CC1E; //Enable TIM5 channel 1 capture rising
}


void TIM3_Init(void){
	//----RCC Enable:
	  RCC->APB1ENR|= RCC_APB1ENR_TIM3EN;
	  RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN; //Enable GPIOB
	  RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN; //Enable GPIOA
	  RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN; //Enable GPIOC
	//----GPIO:
	  //PA6 (TIM3 CH1 -> Control interrupt):
	  GPIOA->MODER|=GPIO_MODER_MODE6_1 ; //Alternate function mode for PA6
	  GPIOA->AFR[0]|=GPIO_AFRL_AFRL6_1;  //AF2:TIM3_CH1  as Alternate function for PA6

	  //PB5 (TIM3 CH2 -> Servo control):
	  GPIOB->MODER|=GPIO_MODER_MODE5_1 ; //Alternate function mode for PB5
	  GPIOB->AFR[0]|=GPIO_AFRL_AFRL5_1;  //AF2:TIM3_CH2  as Alternate function for PB5

	  //PC8 (TIM3 CH3 -> BLDC control):
	  GPIOC->MODER|=GPIO_MODER_MODE8_1; //AF for PC8
	  GPIOC->AFR[1]|=GPIO_AFRH_AFRH0_1;  //AF2 TIM3_CH3  as Alternate function for PC8

	//----TIM3 Config:
	  HAL_NVIC_SetPriority(TIM3_IRQn, 5, 5); //Set TIM3 IRQ priority : When using FreeRTOS, this priority must be less or equal to configured max ISR priority
	  NVIC_EnableIRQ(TIM3_IRQn); //Enable TIM3 Interrupt for NVIC controller
	  TIM3->CR1|=TIM_CR1_ARPE; //Enable preload ARR register
	  TIM3->PSC=27;
	  uint16_t ARR=60000; //16 bit
	  TIM3->ARR=ARR;  //for 20ms period ~ 50Hz

	  //CH1: Control interrupt generation
	  //TIM3->DIER|=TIM_DIER_CC1DE;  //Enable DMA request on channel 1
	  TIM3->DIER|=TIM_DIER_CC1IE;  //Enable IRQ on CC1 event
	  TIM3->CCMR1|=TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; //PWM mode 1 chosen
	  TIM3->CCER|=TIM_CCER_CC1E; //Enable TIM3 channel 1 output
	  TIM3->CCMR1|=TIM_CCMR1_OC1PE;//Enable preload
	  TIM3->CCR1=46000;   //Interruption generation shifted according to CH2 and CH3  (1 ms before rising edge of CH2 & CH3)


	  //CH2: PWM generation ( Servo control)
	  TIM3->CCMR1|=TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; //PWM mode 1 chosen
	  TIM3->CCER|=TIM_CCER_CC2E; //Enable TIM3 channel 2 output
	  TIM3->CCMR1|=TIM_CCMR1_OC2PE;//Enable preload
	  TIM3->CCR2=SERVO_ZERO_POSE;

	  //CH3: PWM generation ( BLDC control)
	  TIM3->CCMR2|=TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; //PWM mode 1 chosen
	  TIM3->CCER|=TIM_CCER_CC3E; //Enable TIM3 channel 3 output
	  TIM3->CCMR2|=TIM_CCMR2_OC3PE;//Enable preload
	  TIM3->CCR3=4500;
	  TIM3->EGR|=TIM_EGR_UG;
}

void TIM1_Init(void){

	//-----RCC Enable:
	RCC->APB2ENR|=RCC_APB2ENR_TIM1EN ; //Enable TIM1 (Rear direction DMA2 request trigger)
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN; //Enable GPIOA
	//-----GPIO:
	//PA8 (TIM1 CH1):
	GPIOA->MODER|=GPIO_MODER_MODE8_1; //AF for PA8
	GPIOA->AFR[1]|=GPIO_AFRH_AFRH0_0;  //AF1 TIM1_CH1  as Alternate function for PA8
	//PA9 (TIM1 CH2):
    GPIOA->MODER|=GPIO_MODER_MODE9_1; //AF for PA9
    GPIOA->AFR[1]|=GPIO_AFRH_AFRH1_0;  //AF1 TIM1_CH2  as Alternate function for PA9
    //PA10 (TIM1 CH3):
    GPIOA->MODER|=GPIO_MODER_MODE10_1; //AF for PA10
    GPIOA->AFR[1]|=GPIO_AFRH_AFRH2_0;  //AF1 TIM1_CH3  as Alternate function for PA10

	//-----TIM1 Config : Input Compare mode ***XOR Channel1 (Rear Direction):
	TIM1->DIER|=TIM_DIER_CC1DE;  //Enable DMA request on channel 1
	TIM1->CR2|=TIM_CR2_TI1S; //Enable XOR combination
	TIM1->CCMR1|=TIM_CCMR1_CC1S_0; //Input mode: Map input to Tl1
	TIM1->CCER|=TIM_CCER_CC1E | TIM_CCER_CC1P |TIM_CCER_CC1NP; //Enable TIM1 channel 1 capture both edges
	//TIM1->CCMR1|=TIM_CCMR1_IC1F; //Filter f/32 8 ticks
	TIM1->PSC=0;
	TIM1->ARR=0xffff;

}

#endif /* INC_TIMERSCONFIG_C_ */
