/*****************************************************************************//**
  * @file    ADCConfig.h
  * @author  M'barek ZACRI
  * @date    May 22, 2020
  * @brief 	 ADC configuration.
 *******************************************************************************/
#ifndef INC_PERIPHERALS_ADCCONFIG_H_
#define INC_PERIPHERALS_ADCCONFIG_H_

/**
 * @defgroup Peripherals Peripherals
 * @brief	 STM32F446RE integrated peripherals bare metal configuration.
 * @{
 */
/**
 * @defgroup ADCConfig ADC configuration
 * @brief	 Analog to Digital Converters configuration
 * @{
 */

/**
 * @brief   ADC1 configuration to retrieve servo motor potentiometer voltage.
 *          Voltage to position conversion is done in @ref Sensors.
 * @details The ADC conversion is triggered by TIM3 CH2 every 20 ms. The conversion
 * 			complete flag trigger the DMA transfer which sends the converted value
 * 			to memory.
 */
void ADC1_init(void);

/**
 * @}
 */
/**
 * @}
 */

#endif /* INC_PERIPHERALS_ADCCONFIG_H_ */
