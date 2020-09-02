/*****************************************************************************//**
  * @file   DMAConfig.h
  * @author  M'barek ZACRI
  * @date     27 avr. 2020
  * @brief 	 DMA configuration.
 *******************************************************************************/

#ifndef INC_DMACONFIG_H_
#define INC_DMACONFIG_H_

/**
 * @addtogroup Peripherals
 * @{
 */
/**
 * @defgroup DMAConfig DMA configuration
 * @brief	 Direct Memory Acces configuration
 * @{
 */

/**
 * @brief   DMA2 Stream0 configuration to retrieve servo motor potentiometer voltage.
 * @details The DMA request is triggered by ADC conversion complete flag .
 */
void DMA2_Stream0_Init(volatile uint16_t *memAddress);
/**
 * @brief   DMA1 Stream2 configuration to retrieve TIM5 CH1 captured value for BLDC speed comptaion.
 * @details If SENSOR1, SENSOR3 or SENSOR4 are chosen, the DMA request is triggered
 *  		by TIM5 CH1 capture event of a rising edge. The number of items to transfer for each DMA transfer
 *  		depends of the SENSOR strategy ( cf @ref Sensors).
 *  		If SENSOR2 is chosen, the DMA request is triggered by TIM3 CH1 every 20 ms. It is possible to do
 *  		the same as SENSOR2 for the SENSOR4.
 */
void DMA1_Stream2_Init(volatile uint32_t *memAddress);

/**
 * @brief   DMA2 Stream1 configuration to retrieve GPIOA state for BLDC direction computation.
 * @details If SENSOR1 is chosen, the DMA request is triggered by TIM5 CH1 capture event of a rising edge.
 * 			The number of items to transfer is one since direction is computed using an other HALL sensor's state
 * 			when the triggering HALL sensor is captured. If SENSOR4 is chosen, the DMA request is triggered
 * 			by the XOR combination of HALL sensors on both edges. The number of items to transfer is two
 * 			since we need two consecutive states to compute direction.
 */
void DMA2_Stream1_Init(volatile uint16_t *memAddress);
/**
 * @}
 */
/**
 * @}
 */
#endif /* INC_DMACONFIG_H_ */
