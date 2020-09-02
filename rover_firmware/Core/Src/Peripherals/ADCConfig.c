/*****************************************************************************//**
  * @file    ADCConfig.c
  * @author  M'barek ZACRI
  * @date    May 22, 2020
  * @brief 	 ADC configuration.
 *******************************************************************************/

#include <RoverConfig.h>
#include "main.h"

void ADC1_init(void){

	//-----------RCC Enable:
	  RCC->APB2ENR|=RCC_APB2ENR_ADC1EN; //Enable ADC1
	  RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN; //Enable GPIOA
	  RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN; //Enable GPIOC
	//----------GPIO configuration:
	 //PA7 (ADC1_IN7):
	 GPIOA->MODER|=GPIO_MODER_MODE7_1 | GPIO_MODER_MODE7_0 ;  //Analog mode PA7
	 //PC4 (ADC1_IN14):
	 GPIOC->MODER|=GPIO_MODER_MODE4_1 | GPIO_MODER_MODE4_0 ;  //Analog mode PC4
	//-----------ADC configuration:
	  ADC1->CR2|=ADC_CR2_EXTEN_0; //External event polarity: Rising
	  ADC1->CR2|=ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2; //TIM3 CC2 event as external trigger
	  ADC1->CR2|=ADC_CR2_DDS; //DMA Request issued as long as data are converted
	  ADC1->CR2|=ADC_CR2_DMA; //DMA Enable
	  ADC1->CR1|=ADC_CR1_SCAN; //Scan mode
	  ADC1->SQR1|=ADC_SQR1_L_0; //Two conversions ADC1_IN7 and ADC_IN14
	  ADC1->SQR3|=0x7 | (0xe  << 5);//Channel ADC1_IN7 and ADC1_IN14
}

