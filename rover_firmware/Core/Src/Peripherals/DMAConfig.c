/*****************************************************************************//**
  * @file   DMAConfig.c
  * @author  M'barek ZACRI
  * @date     27 avr. 2020
  * @brief 	 ADC configuration.
 *******************************************************************************/
#ifndef SRC_DMACONFIG_C_
#define SRC_DMACONFIG_C_

#include <RoverConfig.h>
#include "main.h"
//Servo position and Battery ADC:
void DMA2_Stream0_Init(volatile uint16_t *memAddress){
	//-----------RCC Enable:
	  RCC->AHB1ENR|=RCC_AHB1ENR_DMA2EN; //Enable DMA2
	//DMA2_Stream0 (Servo position):
	  DMA2_Stream0->CR&=~(DMA_SxCR_EN); //Disable for configuration
	  while(DMA2_Stream0->CR & DMA_SxCR_EN); //wait for stream disable
	  DMA2_Stream0->NDTR|=0x0002; //Two items to transfer.
	  DMA2_Stream0->PAR=(ADC1_BASE + 0x4CUL); //Peripheral address: ADC1_JDR
	  DMA2_Stream0->M0AR=(uint32_t)memAddress;
	  DMA2_Stream0->CR|=DMA_SxCR_MINC; //Increment memory
	  //Channel 0 for stream 0: ADC1
	  DMA2_Stream0->CR|=DMA_SxCR_PL_0; //Medium priority
	  DMA2_Stream0->CR|=DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0; //half-word (16 bit) for peripheral and memory sizes
	  DMA2_Stream0->CR|=DMA_SxCR_CIRC; //Circular mode
}
//Rear speed
void DMA1_Stream2_Init(volatile uint32_t *memAddress){

	//-----------RCC Enable:
	  RCC->AHB1ENR|=RCC_AHB1ENR_DMA1EN; //Enable DMA1
	//-----------DMA coonfiguration:
	  //DMA1_Stream2: (BLDC Speed)
	  DMA1_Stream2->CR&=~(DMA_SxCR_EN); //Disable for configuration
	  while(DMA1_Stream2->CR & DMA_SxCR_EN); //wait for stream disable
	  DMA1_Stream2->NDTR|=0x0001; //1 item to transfer.
	  DMA1_Stream2->PAR=(TIM5_BASE + 0x4CUL); //Peripheral address: TIM5 offset 0x4C ==  TIM5_DMAR
	  DMA1_Stream2->M0AR=(uint32_t)memAddress;
	  DMA1_Stream2->CR|=DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1; //Channel 6 for stream 2: TIM5_CH1
	  DMA1_Stream2->CR|=DMA_SxCR_PL_1; //High priority for stream 2
	  DMA1_Stream2->CR|=DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1; //word (32 bit) for peripheral and memory sizes
	  DMA1_Stream2->CR|=DMA_SxCR_CIRC; //Circular mode
}
//Rear direction
void DMA2_Stream1_Init(volatile uint16_t *memAddress){
#if SENSOR == 4
	  //-----------RCC Enable:
	  RCC->AHB1ENR|=RCC_AHB1ENR_DMA2EN; //Enable DMA2
	  //DMA2_Stream1 (Rear direction) :
	  DMA2_Stream1->CR&=~(DMA_SxCR_EN); //Disable for configuration
	  while(DMA2_Stream1->CR & DMA_SxCR_EN); //wait for stream disable
	  DMA2_Stream1->NDTR|=0x0002; //Two items to transfer.
	  DMA2_Stream1->PAR=(GPIOA_BASE + 0x10UL); //Peripheral address: GPIOA_IDR
	  DMA2_Stream1->M0AR=(uint32_t)memAddress;
	  DMA2_Stream1->CR|=DMA_SxCR_MINC; //Increment memory
	  DMA2_Stream1->FCR|=DMA_SxFCR_DMDIS;  //Disable FIFO direct mode
	  DMA2_Stream1->FCR|=DMA_SxFCR_FTH_0;  //1/2 FIFO threshold
	  DMA2_Stream1->CR|=DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1; //Channel 6 for stream 1: TIM1_CH1
	  DMA2_Stream1->CR|=DMA_SxCR_PL_1; //High priority
	  DMA2_Stream1->CR|=DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0;//half-word (16 bit) for peripheral and memory sizes
	  DMA2_Stream1->CR|=DMA_SxCR_CIRC; //Circular mode
#endif

}
#endif /* SRC_DMACONFIG_C_ */
