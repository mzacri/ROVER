/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body.
  * @details 		:
  *
  * @mainpage		 Main program body
  * @section       intro_sec Introduction
  * 			   Rover STM32 firmware is able to receive a speed and orientation command from a PC and execute it
  * 			   according to the configuration file RoverConfig.h. The rover state is then sent to the PC.
  * 			   -# The project tree consists of the following bricks:
  * 				  - Communication: Communication with the PC.
  * 				  - Control: Control of the rover actuators.
  * 				  - Peripherals: STM32 peripherals configuration which interface the rover actuators.
  * 				  - Sensors: Drivers of the sensors and the peripherals data.
  *			       -# Two FreeRTOS tasks are scheduled:
  * 				- StartDefaultTask: background task.
  * 				- startControlTask: The main task triggered by a signal from
  * 				TIM3 ISR handle.
  * @author 		: M'barek ZACRI
  *******************************************************************************/
/*
  * attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <time.h>
#include "math.h"

#include <Control/Control.h>

#include <Sensors/AckermannOdom.h>
#include <Sensors/Filters/BrushlessSpeed_Kalman.h>
#include <Sensors/Filters/EulerOrientationCompFilter.h>
#include <Sensors/Filters/LowPassFilters.h>
#include <Sensors/Filters/ModelDrivenFilters.h>

#include <Sensors/MPU6050/sd_hal_mpu6050.h>
#include <Sensors/RoverStateSensors.h>

#include "Communication/Raspberry.h"

#include "Peripherals/timersConfig.h"
#include "Peripherals/DMAConfig.h"
#include "Peripherals/ADCConfig.h"


#if defined SYSTEMVIEW
#include "../SystemView/SEGGER/SEGGER_SYSVIEW.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId defaultTaskHandle;
osThreadId controlTaskHandle;
osMutexId command_mutexHandle;
/* USER CODE BEGIN PV */

//Global variables (For SWV)
float batteryVolt=0;
int battery_dead=0;
float speed=0;
float filtered_speed=0;
float servoCommand=0;
float bldcCommand=0;
int servoControl=0,brushlessControl=0;

//DMA memory segments:
uint8_t rxComData[82];
uint8_t txComData[82];

volatile uint16_t inADC[2];  //ADCs input ( inADC[0] ~ servo pose; inADC[1] ~  Battery voltage measurement )   (Linked to DMA2)

#if SENSOR == 3     //Reset mode + estimated direction
volatile uint32_t timestamp;  //BLDC speed (Linked to DMA1)
#elif SENSOR == 4    //Reset mode + calculated direction
volatile uint32_t timestamp;  //BLDC speed (Linked to DMA1)
volatile uint16_t inGPIOA[2];  //BLDC direction (Linked to DMA2)
#endif

//Structures:
SD_MPU6050 mpu1;
EULER_ANGLES angles;
ROVER_STATE	state;

//Debug:
uint16_t start=0,end=0,total=0; //Execution time evaluation using timers
float brushlessSpeed_brut=0,servoPose_brut=0; //testing

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void const * argument);
void startControlTask(void const * argument);

/* USER CODE BEGIN PFP */
static void Start_Peripherals(void);
static void Brushless_Calibration(void);
static void Timers_config(void);
static void DMA_Config(volatile uint16_t *inADC,volatile uint16_t *inGPIOA,volatile uint32_t *timestamp);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//-------------------Interrupt handlers:
//TIM3 CH1 Control interrupt:
void TIM3_IRQHandler(void)
  {
	//Clear compare flag signal:
	TIM3->SR&=0x00;

	osSignalSet (controlTaskHandle, 0x0001); //Triggering signal of control task

  }


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  //----------Peripherals enable:
  RCC->APB1ENR|=RCC_APB1ENR_PWREN; //Enable powering APB1 bus

  //----------Inits:
  AckermannOdom_init();
  //Init Kalman:
  KalmanFilter_init();
  //Init MPU:
  mpuInit(&mpu1,&hi2c1);  //if init fails, led2 stays on
  //Init complimentary filter:
  initAngles(&angles,mpu1);


  //----------Timers configuration:
  Timers_config();

  //----------ADC configuration:
  ADC1_init();

  //----------Peripherals DMA configuration:
  DMA_Config(inADC,inGPIOA,&timestamp);

  //----------DMA for Communication:
#if COMMAND_SOURCE == RASP
  HAL_SPI_TransmitReceive_DMA(&hspi2, txComData, rxComData, sizeof(rxComData));  //SPI RX/TX DMA1 (RX: stream3 ; TX: stream4)
  DMA1_Stream3->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_DMEIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE); //Disable interrupt on DMA1_Stream3 ( SPI Rx )
  DMA1_Stream4->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_DMEIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE); //Disable interrupt on DMA1_Stream4 ( SPI Tx )
#endif


  /* USER CODE END 2 */
  /* Create the mutex(es) */
  /* definition and creation of command_mutex */
  osMutexDef(command_mutex);
  command_mutexHandle = osMutexCreate(osMutex(command_mutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of controlTask */
  osThreadDef(controlTask, startControlTask, osPriorityHigh, 0, 1024);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
#if defined SYSTEMVIEW
  //SEGGER VIEWER:
  SEGGER_SYSVIEW_Conf();
  vSetVarulMaxPRIGROUPValue();  //For SystemView to work, you need to modify portmacro.h and port.c after every generation of code. Cf Core/SystemView/portsPatch.txt
  SEGGER_SYSVIEW_Start();
  TIM2->CR1|=TIM_CR1_CEN;  //SysTick
#endif

  //-------------Start peripherals (DMA, timers...):
  Start_Peripherals();
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //-------------Calibration of the brushless
  //Brushless_Calibration();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//Functions:

void Start_Peripherals(void){
  //Enable DMA1
    DMA1_Stream2->CR|=DMA_SxCR_EN;  //Rear speed
  #if SENSOR == 4
    DMA2_Stream1->CR|=DMA_SxCR_EN;  //Rear direction
  #endif
    DMA2_Stream0->CR|=DMA_SxCR_EN;  //Servo position
  //Enable ADC1:
    ADC1->CR2|=ADC_CR2_ADON;
  //START timers
    TIM5->CR1|=TIM_CR1_CEN;  //Speed
  #if SENSOR == 4
    TIM1->CR1|=TIM_CR1_CEN;  //Direction
  #endif
    TIM3->CR1|=TIM_CR1_CEN;  //Control execution
}

void DMA_Config(volatile uint16_t *inADC,volatile uint16_t *inGPIOA,volatile uint32_t *timestamp){
	  DMA2_Stream0_Init(inADC);   //ADC1 stream of the two converted analog inputs
	#if SENSOR == 3
	  DMA1_Stream2_Init(&timestamp);   //Rear speed
	#elif SENSOR == 4
	  DMA1_Stream2_Init(timestamp);   //Rear speed
	  DMA2_Stream1_Init(inGPIOA);   //Rear direction
	#endif
}


void Timers_config(void){
	#if SENSOR == 4
	  TIM1_Init();
	#endif
	  TIM3_Init();
	  TIM5_Init();
	#if defined SYSTEMVIEW
	  TIM2_Init();
	#endif
}

static void Brushless_Calibration(void){
	  HAL_Delay(2000);
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  TIM3->CCR3=4500;
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  TIM3->CCR3=6000;
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  TIM3->CCR3=3000;
	  HAL_Delay(2000);
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  TIM3->CCR3=4700;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

#if	COMMAND_SOURCE == INT
	  intCommandAlternator(&bldcCommand,&servoCommand);
	  HAL_Delay(2500);
#endif
	  osMutexWait(command_mutexHandle, osWaitForever);
	  //Stop led blinking when battery is dead:
	  if (!battery_dead){
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  }
	  osMutexRelease(command_mutexHandle);
	  osDelay(500);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_startControlTask */
/**
* @brief Function implementing the controlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startControlTask */
void startControlTask(void const * argument)
{
  /* USER CODE BEGIN startControlTask */
  /* Infinite loop */
  for(;;)
	{
	  //Wait for TIM3 ISR signal:
      osSignalWait (0x0001, osWaitForever);
	  start=TIM3->CNT;  //Execution time
		//--------------Update command:
	  #if COMMAND_SOURCE == RASP
	  	  Update_command(rxComData,&bldcCommand,&servoCommand);
	  	  if(battery_dead){  //If battery is drained, negative speed may be altered. Stop moving is more adequate.
	  		  bldcCommand=0;
	  	  }
	  #endif

	  //--------------Servo control:
	  servoControl=Servo_command(servoCommand);
	  int adjust_zero_pose=0;
	  if(servoCommand==0){
		  if(state.servoPose>0){
			  adjust_zero_pose=ADJUST_POSITIVE;
		  } else if (state.servoPose<0){
			  adjust_zero_pose=ADJUST_NEGATIVE;
		  }
	  }
	  TIM3->CCR2=servoControl+SERVO_ZERO_POSE+adjust_zero_pose;

	  //--------------Servo position:
	  #if ESTIMATE_SERVO_POSE
			state.servoPose=EstimateServoPos(servoControl);
	  #else
			float infiltered_servoPose=0;
			infiltered_servoPose=SenseServoPos(inADC[0]);  //Computed servo position using ADC.
			servoPose_brut=infiltered_servoPose;
			infiltered_servoPose=ServoPoseFilter(infiltered_servoPose,servoControl);
			//state.servoPose=infiltered_servoPose;
			state.servoPose=LowPassFilter2(infiltered_servoPose);  //third order low pass filter bandwidth at 30ms
	  #endif
	  //---------------Compute speed:
	  #if SENSOR == 3
			float infiltered_speed=0;
			infiltered_speed=Sensor3_speed(brushlessControl,&timestamp);
      #elif SENSOR == 4
			float infiltered_speed=0;
			infiltered_speed=Sensor4_speed(brushlessControl,&timestamp,inGPIOA);
	  #endif
		brushlessSpeed_brut=infiltered_speed;  //testing
		infiltered_speed=SpeedFilter(infiltered_speed,brushlessControl); //Model driven filter
		speed=LowPassFilter1(infiltered_speed); //Low pass filter at 31 ms bandwidth
		if(speed>MAX_COMMAND){
			TIM3->CCR3=4500; //Safety
		}

		//-------------Sense IMU:
		SD_MPU6050_ReadGyroscope(&hi2c1,&mpu1);
		SD_MPU6050_ReadAccelerometer(&hi2c1,&mpu1);

		//-------------Kalman filtering of speed:
		double Ax=0;
		Ax=mpu1.Accelerometer_X*mpu1.Acce_Mult;
		filtered_speed=(float)KalmanFilter(Ax,speed);

		//-------------BLDC control:
		//TODO: Stack analysis when using a feedback loop
	  #if CONTROL == RE
			brushlessControl=Re(bldcCommand,filtered_speed);
			TIM3->CCR3=brushlessControl+4500;
	  #elif CONTROL == OL
			brushlessControl=OpenLoop(bldcCommand);
			TIM3->CCR3=brushlessControl+4500;
	  #endif

		//-------------Compute Roll,Pitch,Yaw:
		///Complimentary filter:
		computeAccelAngles(&angles,mpu1);
		computeGyroAngles(&angles,mpu1,0.02);
		computeAngles(&angles,0.2);

		//--------------Update state:
		state.rearSpeed=filtered_speed;
        //Battery management
		batteryVolt=SenseBatteryVoltage(inADC[1]);
		state.batteryVolt=LowPassFilter3(batteryVolt);
		osMutexWait(command_mutexHandle, osWaitForever);
		if (state.batteryVolt < BATTERY_VOLTAGE_THRESHOLD){
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			battery_dead=1;
		} else {
			battery_dead=0;
		}
		osMutexRelease(command_mutexHandle);

		//-------------Compute Odometry using Ackermann car model:
		float v_var=0.2*state.rearSpeed*state.rearSpeed;
		float steering_var=0.0076;
		float w=0;
		float M[4];
		AckermannOdom(state.rearSpeed,state.servoPose,v_var,steering_var,&w,M);
		Ackermann_RetrieveOdom(&state); //update state
		state.w=w;
		state.speed_covariance[0]=M[0];
		state.speed_covariance[1]=M[1];
		state.speed_covariance[2]=M[2];
		state.speed_covariance[3]=M[3];

		///------------Send state:
		Send_state_SPI(state,txComData);


		end=TIM3->CNT;
		total=(end-start);  //Execution time  ~2ms with openLoop
  }
  /* USER CODE END startControlTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
