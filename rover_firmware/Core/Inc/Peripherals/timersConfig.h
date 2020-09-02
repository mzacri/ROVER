/*****************************************************************************//**
  * @file   timersConfig.h
  * @author  M'barek ZACRI
  * @date    27 avr. 2020
  * @brief 	Timers configuration.
 *******************************************************************************/
#ifndef INC_TIMERSCONFIG_H_
#define INC_TIMERSCONFIG_H_

/**
 * @addtogroup Peripherals
 * @{
 */
/**
 * @defgroup TIMConfig Timers configuration
 * @{
 */

/**
 * @brief	TIMER TIM5 configuration on capture mode to retrieve the BLDC speed.
 *
 * @details	If SENSOR1 is chosen, the timer capture the counter on a rising edge of the HALL sensor signal connected to CH1.
 * 			If SENSOR3 or SENSOR4 are chosen, the timer captures the counter on a rising edge of the HALL sensors XOR combination .
 */
void TIM5_Init(void);

/**
 * @brief	TIMER TIM1 configuration on capture mode to retrieve the BLDC direction.
 *
 * @details	If SENSOR1 or SENSOR4 are chosen, the timer trigger a DMA transfer of the GPIOA.
 *
 * @note The main motivation of the use of this timer is that GPIO ports are not connected to DMA1.
 * 		 In result we can't trigger DMA transfer of GPIOA using TIM5 flags.
 */
void TIM1_Init(void);

/**
 * @brief	TIMER TIM3 configuration on output mode to generate PWM control pulses.
 * @details	-CH1: generates the control interrupt that enable the control Task every 20 ms.
 * 				  The interrupt is triggered on the falling edge of the generated pulse.
 * 			-CH2: generates the servo control PWM.
 * 			-CH3: generates the BLDC control PWM.
 *
 * @note    CH1 interrupt is triggered before the rising edge of the CH2 and CH3 PWM. The time shift is controlled by the TIM3->CCR1 value.
 * 			In that way, the calculated controls will update most immediately the PWM pulses after calculation of command.
 * 			To characterize the control computation time and shift the control interrupt to the left accordingly,
 * 			Execution time variables can be used (cf control Task).
 */
void TIM3_Init(void);
/**
 * @}
 */
/**
 * @}
 */

void TIM2_Init(void);
#endif /* INC_TIMERSCONFIG_H_ */
