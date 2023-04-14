/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "liquidcrystal_i2c.h"
#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include "stm32l4xx_hal_uart.h"

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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

volatile screenStates state = speed;
volatile uint8_t isLogging = 0;
volatile screenStates prevState;

#define USART1_ADDR = 0x40013800
#define USART1_CR1_OFFSET = 0x0
#define USART1_BRR _OFFSET = 0xC
#define USART1_CR2_OFFSET = 0x4
#define USART1_CR3_OFFSET = 0x8

const int BUFFER_SIZE = 100;

double maxSpeed = 0;
double maxAltitude = 0;
double longestRun = 0;
double tallestRun = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void btSendData(uint8_t str[], uint32_t size);
void printScreen(screenStates s, screenStates p);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile gpsData gps_data = {
		  0,
		  "hh",
		  0,
		  "mm",
		  0,
		  "ss.ss",
		  "ASDFADFHDFGASDFASDFSDFHDASDF", //dataBuffer
	  	  0,//bufferIndex
		  "lllll.ll",//latitudeChar[]
		  0,//latitude
		  'A',//latDir
		  "yyyyy.yy",//longitudeChar
		  0,//longitude
		  'A',//longDir
		  0,//fix
		  0,//numSatellites
		  "xxx",//numSatellitesChar
		  0,//hdop
		  "x.x",//hdopChar[]
		  0,//altitude
		  "x.x",//altitudeChar[]
		  'M',//altitudeUnits
		  'V',//validity
		  "xxx",//speedCharKnots
		  0, //speedMph
			};
uint8_t bytesReceived;
volatile char bufferByte[1];

runData run_data = {
	0,0,0,//start lat, long, alt
	0,0,0 //stop lat, long, alt
};

gpsData parseGps(gpsData data){

	int dataElementIndex = 0;
	int dataElementNum = 0;
	int timeCount = 0;

	char dataType[7] = "XXXXXX";

	for(uint8_t i = 0; i < BUFFER_SIZE; ++i){
		char letter = *(data.dataBuffer+i);

		if(letter == ','){
			++dataElementNum;
			dataElementIndex = 0;
		}

		if(dataElementNum == 0 && letter != ','){
			//datatype, either GPGGA or GPRMC
			dataType[dataElementIndex] = letter;
			++dataElementIndex;
		}


		if(strcmp(dataType,"$GPGGA") == 0 && letter != ','){
			if (dataElementNum == 1 ){

				if (timeCount <= 1){
					data.hoursChar[dataElementIndex] = letter;
					++dataElementIndex;
					if(timeCount == 1){
						data.hours = atoi(data.hoursChar);
						dataElementIndex = 0;
					}

				}else if(timeCount <= 3){
					data.minsChar[dataElementIndex] = letter;
					++dataElementIndex;
					if (timeCount == 3){
						data.mins = atoi(data.minsChar);
						dataElementIndex = 0;
					}

				}else if(timeCount > 3){
					data.secsChar[dataElementIndex] = letter;
					++dataElementIndex;
					if (*(data.dataBuffer+i+1) == ','){
						data.secs = atof(data.secsChar);
						dataElementIndex = 0;
						data.timeInSecs = data.secs + (data.mins * 60) + (data.hours * 24 * 60);
					}

				}

				++timeCount;

			} else if (dataElementNum == 2 ){
				data.latitudeChar[dataElementIndex] = letter;
				++dataElementIndex;
				if(*(data.dataBuffer+i+1) == ','){
					data.latitude = atof(data.latitudeChar);
				}

			} else if (dataElementNum == 3){
				data.latDir = letter;

			} else if (dataElementNum == 4){
				data.longitudeChar[dataElementIndex] = letter;
				++dataElementIndex;
				if(*(data.dataBuffer+i+1) == ','){
					data.longitude = atof(data.longitudeChar);
				}

			} else if (dataElementNum == 5){
				data.longDir = letter;

			} else if (dataElementNum == 6){
				data.fix = (uint8_t) (letter - '0');

			} else if (dataElementNum == 7){
				data.numSatellitesChar[dataElementIndex] = letter;
				++dataElementIndex;
				if(*(data.dataBuffer+i+1) == ','){
					data.numSatellites = atoi(data.numSatellitesChar);
				}

			} else if (dataElementNum == 8){
				data.hdopChar[dataElementIndex] = letter;
				++dataElementIndex;
				if(*(data.dataBuffer+i+1) == ','){
					data.hdop = atof(data.hdopChar);
				}

			} else if (dataElementNum == 9){
				data.altitudeChar[dataElementIndex] = letter;
				++dataElementIndex;
				if(*(data.dataBuffer+i+1) == ','){
					data.altitude = atof(data.altitudeChar);
				}

			} else if (dataElementNum == 10){
				data.altitudeUnits = letter;

			} else if (dataElementNum == 11){
				break;
			}

		} else if(strcmp(dataType,"$GPRMC") == 0 && letter != ','){
			if (dataElementNum == 2){
				data.validity = letter;

			} else if (dataElementNum == 7){
				data.speedCharKnots[dataElementIndex] = letter;
				++dataElementIndex;
				if(*(data.dataBuffer+i+1) == ','){
					data.speedMph = 1.15077945 * atof(data.speedCharKnots);
				}

			} else if(dataElementNum >= 8){
				break;
			}
		}

	}
	return data;
}

float calcDistance(float lat1, float long1, float lat2, float long2){


}

void determineMax(gpsData data){
	if(data.speedMph > maxSpeed){
		maxSpeed = data.speedMph;
	}

	if(data.altitude > maxAltitude){
		maxAltitude = data.altitude;
	}

	float runLength = calcDistance(run_data.startLat,run_data.startLong,run_data.stopLat,run_data.stopLong);
	if(runLength > longestRun){
		longestRun = runLength;
	}

	float runHeight = run_data.startAlt - run_data.stopAlt;
	if(runHeight > tallestRun){
		tallestRun = runHeight;
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{


	//if(data.dataBuffer[data.bufferIndex] != '/n'){

		if (gps_data.bufferIndex < BUFFER_SIZE){
			gps_data.dataBuffer[gps_data.bufferIndex++] = *bufferByte;

		} else{
			gps_data.bufferIndex = 0;

		}

		if(*bufferByte == '\n'){
			gps_data.bufferIndex = 0;

		}

		HAL_UART_Receive_IT(huart, (uint8_t *) bufferByte, 1);

	//}


	//++bytesReceived;
}

runStates runStatus;
float runStartTimeInSecs;
float stopStartTimeInSecs;
//int firstTimeOver;
const float THRESHOLD_SPEED = 3.0;

void checkRunStatus(gpsData data){

	if(runStatus == running && data.speedMph <= THRESHOLD_SPEED){
		if(firstTimeOver == 0){
			//Keep track of the time when we first stopped
			stopStartTimeInSecs = data.timeInSecs;
			++firstTimeOver;
		}

		//Stopped for 15 seconds
		if(data.timeInSecs - stopStartTimeInSecs >= 15){
			runStatus = notRunning;
		}

	} else if(runStatus == running && data.speedMph > THRESHOLD_SPEED){
		firstTimeOver = 0;
		//Moving right now so reset firstTimeOver because we didn't stay in place for 15 secs
	}

	if(runStatus == notRunning && data.speedMph > THRESHOLD_SPEED){
		if(firstTimeOver == 0){
			//Keep track of the time when we first started moving
			runStartTimeInSecs = data.timeInSecs;
			++firstTimeOver;
		}

		//Moving for 7 seconds
		if(data.timeInSecs - runStartTimeInSecs >= 7){
			runStatus = notRunning;
		}

	} else if(runStatus == notRunning && data.speedMph <= THRESHOLD_SPEED){
		firstTimeOver = 0;
		//Stopped right now so reset firstTimeOver
	}

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize LCD
  HD44780_Init(2); // Initialize
  HD44780_Clear(); // Clear buffer
  HD44780_Display(); // Display characters
  HD44780_NoBlink(); // Don't blink cursor
  HD44780_NoCursor(); // Don't show cursor

  //enable GGA (contains the precision data) and RMC (contains all the minimum navigation info)
	//data on the GPS
	char inputBuffer[] = "PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2C\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t *) inputBuffer, sizeof(inputBuffer), 100);

	//Change GPS update frequency to every 3000 milliseconds
	char inputBuffer[] = "$PMTK220,3000*1F\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t *) inputBuffer, sizeof(inputBuffer), 100);

	bytesReceived = 0;

	runStatus = notRunning;
	startTimeInSecs = 0;
	//firstTimeOver = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//if(isLogging){
	  HAL_UART_Receive_IT(&huart1, (uint8_t *) bufferByte, 1);

	  gps_data = parseGps(gps_data);

	  checkRunStatus(gps_data);

	  determineMax(gps_data);


	//}

	printScreen(state, prevState);

	btSendData("hello world\r\n", sizeof("hello world\r\n"));

	printf("Time: %u:%u:%f | ", gps_data.hours, gps_data.mins, gps_data.secs);
	printf("Latitude: %f", gps_data.latitude);
	printf(" %c", gps_data.latDir);
	printf(" | Longitude: %f", gps_data.longitude);
	printf(" %c", gps_data.longDir);
	printf(" | GPS fix?: %u", gps_data.fix);
	printf(" | Satellites: %u", gps_data.numSatellites);
	printf(" | HDOP: %f", gps_data.hdop);
	printf(" | altitude: %f", gps_data.altitude);
	printf(" %c", gps_data.altitudeUnits);
	printf(" | Validity: %c", gps_data.validity);
	printf(" | Speed: %f", gps_data.speedMph);
	printf("\n");
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_GPIO_Port, LED_GPIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_SPI_SCK_Pin SD_SPI_MISO_Pin SD_SPI_MOSI_Pin */
  GPIO_InitStruct.Pin = SD_SPI_SCK_Pin|SD_SPI_MISO_Pin|SD_SPI_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : QiEnable_GPIO_Pin */
  GPIO_InitStruct.Pin = QiEnable_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(QiEnable_GPIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_GPIO_Pin */
  GPIO_InitStruct.Pin = Button_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GPIO_Pin */
  GPIO_InitStruct.Pin = LED_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_GPIOB4_Pin Button_GPIOB5_Pin */
  GPIO_InitStruct.Pin = Button_GPIOB4_Pin|Button_GPIOB5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
	//Enable printf
	#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
	#else
	  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
	#endif /* __GNUC__ */
	PUTCHAR_PROTOTYPE
	{
	  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	  return ch;
	}

	// Blue tooth Functions
	void btSendData(uint8_t str[], uint32_t size) {
	  //	uint8_t s[20] = (uint8_t)str;
		HAL_UART_Transmit(&huart2, str, size, 1000);
	 }

	// LCD Functions
	void printScreen(screenStates s, screenStates p) {
		HD44780_SetCursor(0, 0);
		char temp[16];
		switch(s) {
			case speed: {
				HD44780_PrintStr("Max Speed (m/s):");
				HD44780_SetCursor(1, 0);
				sprintf(temp, "%f", maxSpeed);
				HD44780_PrintStr(temp);
				break; }
			case alt: {
				HD44780_PrintStr("Max Alt. (m):   ");
				HD44780_SetCursor(1, 0);
				sprintf(temp, "%f", maxAltitude);
				HD44780_PrintStr(temp);
				break; }
			case longest: {
				HD44780_PrintStr("Longest Run (m):");
				break; }
			case tallest: {
				HD44780_PrintStr("Tallest Run (m):");
				break; }
			case startLog: {
				HD44780_PrintStr("Starting log:   ");
				break; }
			case pauseLog: {
				HD44780_PrintStr("Pausing log:    ");
				break; }
			case stopLog: {
				HD44780_PrintStr("Stopping log:   ");
				break; }
			case save: {
				HD44780_PrintStr("Saving log:     ");
				break; }
			default: {
				HD44780_PrintStr("Error           ");
				break; }
		}
	}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
