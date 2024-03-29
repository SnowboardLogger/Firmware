/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "liquidcrystal_i2c.h"
#include "driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern volatile uint16_t Timer1, Timer2;
extern screenStates state; // Can't declare volatile for some reason, might need to wry about this
extern screenStates prevState; // Same issue as state
extern volatile uint8_t isFixed;
extern volatile uint8_t buzEnable; // Buzzer Enable
extern volatile uint8_t IMU_DATA_FLAG;
extern volatile uint8_t isLogging;

extern volatile gpsData GPSData;
extern volatile char gpsDataBuffer[100];//max chars of 70 from gpgga
volatile int gpsBufferIndex;
extern volatile char bufferByte[1];
extern volatile int gpsParseFlag;

uint32_t counter = 0;
uint32_t timer6 = 0;
uint8_t buttonActive = 0;
uint8_t activeButtonCounter = 0;
volatile uint32_t buttonCounter = 0;
extern volatile uint32_t adcOutput;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim16;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern volatile uint8_t IMU_Flag;
extern volatile uint8_t StopLog;
extern volatile uint8_t GPSNoFix;
extern volatile uint8_t GPSTimeout;
extern volatile uint8_t IMUTimeout;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  if(Timer1 > 0)
	  Timer1--;
  if(Timer2 > 0)
	  Timer2--;
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
  // NOTE: ALL INTERRUPTS SUBJECT TO CHANGE/REORDERING
  if (buttonActive == 0) {
	  buttonActive = 1;
	  btnFourIRQ(&state, &prevState);
  }
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Button_GPIOB4_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles ADC1 global interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_IRQn 1 */
  adcOutput = HAL_ADC_GetValue(&hadc1);
//  HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END ADC1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  // NOTE: ALL INTERRUPTS SUBJECT TO CHANGE/REORDERING
  if (buttonActive == 0) {
	  buttonActive = 1;
	  btnNineToFiveIRQ(&state, &prevState, &isLogging);
  }
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Button_GPIOB5_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
  // DO we want to check if signals are primed before calling a interrupt?
  // TODO Maybe, but I don't think it's necessary

  // If Buzzer is enabled, oscillates at 2 kHz
  if (buzEnable == 1) {
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
	  buttonCounter++;
	  if (buttonCounter > 500) {
		  buzEnable = 0;
		  buttonCounter = 0;
	  }
  }

  // Set a IMU data flag to read data
  if (counter % 8) {
	  IMU_DATA_FLAG = 1;
  } else {
	  IMU_DATA_FLAG = 0;
  }

  if (buttonActive == 1) {
	  activeButtonCounter++;
	  if (activeButtonCounter % 60) {
		  activeButtonCounter = 0;
		  buttonActive = 0;
	  }
  }
  counter = (counter > 60000) ? 0 : counter+1;

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */
  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  bufferByte[0] = 0;
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  //if (GPSData.bufferIndex < BUFFER_SIZE){
	//GPSData.dataBuffer[GPSData.bufferIndex++] = *bufferByte;
  gpsDataBuffer[gpsBufferIndex] = *bufferByte;
  ++gpsBufferIndex;
  /*} else {
	GPSData.bufferIndex = 0;
  }*/

	if(*bufferByte == '\n'){
		//GPSData.bufferIndex = 0;
		gpsBufferIndex = 0;
		gpsParseFlag = 1;
	}

	HAL_UART_Receive_IT(&huart1, (uint8_t *) bufferByte, 1);

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  if (buttonActive == 0) {
	  buttonActive = 1;
	  // NOTE: ALL INTERRUPTS SUBJECT TO CHANGE/REORDERING
	  btnFifteenToTenIEQ(&state, &prevState, &isLogging);
  }
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Button_GPIO_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC channel1 and channel2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
  if (GPSTimeout == 1) {
	  if (GPSData.fix != 0) {
		  state = prevState;
		  GPSTimeout = 0;
		  isFixed = 1;
	  } else if (timer6 >= 5) { // change back to 30
		  state = stopLog;
		  GPSNoFix = 1;
		  GPSTimeout = 0;
		  timer6 = 0;
	  }
	  timer6++;
  }

  if (StopLog == 1) {
	  if (timer6 >= 5) {
		  state = save;
		  timer6 = 0;
		  StopLog = 0;
	  } else {
		  timer6++;
	  }
  }

  /*
  if (IMUTimeout == 1) {
	  if (timer6 >= 300) {
		  timer6 = 0;
		  state = error;
		  IMUTimeout = 0;
	  } else {
		  timer6++;
	  }
  }
  */

  timer6 = (timer6 < 40000) ? timer6 : 0;

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
