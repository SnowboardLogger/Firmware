/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	int hours;
	char hoursChar[8];
	int mins;
	char minsChar[8];
	float secs;
	char secsChar[8];
	float timeInSecs;

	volatile char dataBuffer[100];//max chars of 70 from gpgga
	volatile int bufferIndex;

	char latitudeChar[15];
	float latitude;

	char latDir;//N or S

	char longitudeChar[15];
	float longitude;

	char longDir;//E or W

	uint8_t fix;//0 = invalid, 1 = GPS fix, 2 = Dif. GPS fix

	uint8_t numSatellites;
	char numSatellitesChar[6];

	float hdop;//Horizontal Dilution of Precision
	char hdopChar[8];

	float altitude;
	char altitudeChar[8];

	char altitudeUnits;//M = meters
	//they have a checksum, do we need to use it?

	char validity;

	char speedCharKnots[10];
	float speedMph;

	char checksumgga[6];
	int ggaGood;

	char checksumrmc[6];
	int rmcGood;
}gpsData;

typedef struct {

	float startLat;
	float startLong;
	float startAlt;

	float stopLat;
	float stopLong;
	float stopAlt;

} runData;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

typedef enum {
	running,
	notRunning
} runStates;

typedef enum {
	speed,
	alt,
	longest,
	tallest,
	startLog,
	stopLog,
	pauseLog,
	resumeLog,
	save,
	error
} screenStates;
volatile extern screenStates state;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SD_SPI_SCK_Pin GPIO_PIN_1
#define SD_SPI_SCK_GPIO_Port GPIOA
#define BT_UART_TX_Pin GPIO_PIN_2
#define BT_UART_TX_GPIO_Port GPIOA
#define BT_UART_RX_Pin GPIO_PIN_3
#define BT_UART_RX_GPIO_Port GPIOA
#define SD_SPI_MISO_Pin GPIO_PIN_6
#define SD_SPI_MISO_GPIO_Port GPIOA
#define SD_SPI_MOSI_Pin GPIO_PIN_7
#define SD_SPI_MOSI_GPIO_Port GPIOA
#define QiEnable_GPIO_Pin GPIO_PIN_1
#define QiEnable_GPIO_GPIO_Port GPIOB
#define GPS_UART_TX_Pin GPIO_PIN_9
#define GPS_UART_TX_GPIO_Port GPIOA
#define GPS_UART_RX_Pin GPIO_PIN_10
#define GPS_UART_RX_GPIO_Port GPIOA
#define Button_GPIO_Pin GPIO_PIN_11
#define Button_GPIO_GPIO_Port GPIOA
#define Button_GPIO_EXTI_IRQn EXTI15_10_IRQn
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LED_GPIO_Pin GPIO_PIN_3
#define LED_GPIO_GPIO_Port GPIOB
#define Button_GPIOB4_Pin GPIO_PIN_4
#define Button_GPIOB4_GPIO_Port GPIOB
#define Button_GPIOB4_EXTI_IRQn EXTI4_IRQn
#define Button_GPIOB5_Pin GPIO_PIN_5
#define Button_GPIOB5_GPIO_Port GPIOB
#define Button_GPIOB5_EXTI_IRQn EXTI9_5_IRQn
#define Display_I2C_SCL_Pin GPIO_PIN_6
#define Display_I2C_SCL_GPIO_Port GPIOB
#define Display_I2C_SDA_Pin GPIO_PIN_7
#define Display_I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
