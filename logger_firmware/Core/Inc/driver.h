#ifndef __DRIVER_H
#define __DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include "Math.h"
#include <string.h>
#include "liquidcrystal_i2c.h"
#include "stm32l4xx_hal.h"
#include "fatfs.h"

extern volatile uint8_t buzEnable;

extern volatile char gpsDataBuffer[100];//max chars of 70 from gpgga
extern volatile int gpsParseFlag;
extern volatile uint8_t GPSTimeout;
extern volatile uint8_t IMUTimeout;
extern volatile uint8_t curRun;
extern volatile uint8_t curData;
extern volatile float startTime;
extern volatile float endTime;

typedef struct {
	int hours;
	char hoursChar[8];
	int mins;
	char minsChar[8];
	float secs;
	char secsChar[8];
	float timeInSecs;

	char latitudeChar[15];
	float latitude[2];

	char latDir;//N or S

	char longitudeChar[15];
	float longitude[2];

	char longDir;//E or W

	uint8_t fix;//0 = invalid, 1 = GPS fix, 2 = Dif. GPS fix

	uint8_t numSatellites;
	char numSatellitesChar[6];

	float hdop;//Horizontal Dilution of Precision
	char hdopChar[8];

	float altitude[3];
	char altitudeChar[8];

	char altitudeUnits;//M = meters
	//they have a checksum, do we need to use it?

	char validity;

	char speedCharKnots[10];
	float speedMph[3];

	char checksumgga[3];
	int ggaGood;

	char checksumrmc[3];
	int rmcGood;
} gpsData;

typedef struct {
	float startLat;
	float startLong;
	float startAlt;

	float stopLat;
	float stopLong;
	float stopAlt;
} runData;

typedef enum {
	speed,
	alt,
	longest,
	tallest,
	steepest,
	startLog,
	stopLog,
	pauseLog,
	resumeLog,
	save,
	battery,
	error
} screenStates;

typedef enum {
	running,
	notRunning
} runStates;

typedef struct {
	float GPSspeed;
	float altitude;
	float longitude;
	float latitude; // NE pos SW neg
} GPSRequired;

typedef struct {
	uint8_t elapsedTime;
	uint8_t verticalDistance;
	uint8_t averageSpeed;
	uint8_t horizontalDistance;
	uint8_t numberOfPoints;
	GPSRequired data[200];
//	float IMUspeed[6000][3];
} Run;

typedef struct {
	float maxSpeed;
	float maxAltitude;
	float tallestRun;
	float longestRun;
	float steepestRun;
	uint8_t numberOfRuns;
	Run run[10]; 	// Can have a max of 10 runs per log
} Log ;

extern volatile Log recordedData;

typedef struct {
	uint16_t accel_offset_x;
	uint16_t accel_offset_y;
	uint16_t accel_offset_z;
	uint16_t mag_offset_x;
	uint16_t mag_offset_y;
	uint16_t mag_offset_z;
	uint16_t gyro_offset_x;
	uint16_t gyro_offset_y;
	uint16_t gyro_offset_z;
	uint16_t accel_radius;
	uint16_t mag_radius;
} IMU_OFFSET;

extern volatile uint8_t buzEnable;
extern volatile runStates runStatus;
extern volatile int firstTimeOver;
extern volatile float runStartTimeInSecs;
extern volatile float stopStartTimeInSecs;

// Declaration of both states
extern screenStates state;
extern screenStates prevState;
extern IMU_OFFSET offset_cal_data;

#define BUFFER_SIZE 100
#define THRESHOLD_SPEED 3.0

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

// UART
#define USART1_ADDR = 0x40013800
#define USART1_CR1_OFFSET = 0x0
#define USART1_BRR _OFFSET = 0xC
#define USART1_CR2_OFFSET = 0x4
#define USART1_CR3_OFFSET = 0x8

// IMU
#define I2C_Addr 0x50
#define PI 3.14159265359
/* PAGE ZERO
 * At power-on Page 0 is selected, PAGE_ID register
 * can be used to identify the current selected page and
 * change between page 0 and page 1
 */
#define MAG_RADIUS_MSB 0x6A
#define MAG_RADIUS_LSB 0x69
#define ACC_RADIUS_MSB 0x68
#define ACC_RADIUS_LSB 0x67
#define GYR_OFFSET_Z_MSB 0x66
#define GYR_OFFSET_Z_LSB 0x65
#define GYR_OFFSET_Y_MSB 0x64
#define GYR_OFFSET_Y_LSB 0x63
#define GYR_OFFSET_X_MSB 0x62
#define GYR_OFFSET_X_LSB 0x61
#define MAG_OFFSET_Z_MSB 0x60
#define MAG_OFFSET_Z_LSB 0x5F
#define MAG_OFFSET_Y_MSB 0x5E
#define MAG_OFFSET_Y_LSB 0x5D
#define MAG_OFFSET_X_MSB 0x5C
#define MAG_OFFSET_X_LSB 0x5B
#define ACC_OFFSET_Z_MSB 0x5A
#define ACC_OFFSET_Z_LSB 0x59
#define ACC_OFFSET_Y_MSB 0x58
#define ACC_OFFSET_Y_LSB 0x57
#define ACC_OFFSET_X_MSB 0x56
#define ACC_OFFSET_X_LSB 0x55

#define AXIS_MAP_SIGN 0x42
#define AXIS_MAP_CONFIG 0x41
#define TEMP_SOURCE 0x40
#define SYS_TRIGGER 0x3F
#define PWR_MODE 0x3E
#define OPR_MODE 0x3D
#define UNIT_SEL 0x3B
#define SYS_ERR 0x3A
#define SYS_STATUS 0x39
#define SYS_CLK_STATUS 0x38
#define INT_STA 0x37
#define ST_RESULT 0x36
#define CALIB_STAT 0x35
#define TEMP 0x34
// Gravity Vector
#define GRV_DATA_Z_MSB 0x33
#define GRV_DATA_Z_LSB 0x32
#define GRV_DATA_Y_MSB 0x31
#define GRV_DATA_Y_LSB 0x30
#define GRV_DATA_X_MSB 0x2F
#define GRV_DATA_X_LSB 0x2E
// Linear Acceleration
#define LIA_DATA_Z_MSB 0x2D
#define LIA_DATA_Z_LSB 0x2C
#define LIA_DATA_Y_MSB 0x2B
#define LIA_DATA_Y_LSB 0x2A
#define LIA_DATA_X_MSB 0x29
#define LIA_DATA_X_LSB 0x28
// Quaternion
#define QUA_DATA_Z_MSB 0x27
#define QUA_DATA_Z_LSB 0x26
#define QUA_DATA_Y_MSB 0x25
#define QUA_DATA_Y_LSB 0x24
#define QUA_DATA_X_MSB 0x23
#define QUA_DATA_X_LSB 0x22
#define QUA_DATA_W_MSB 0x21
#define QUA_DATA_W_LSB 0x20
// Pitch
#define EUL_PITCH_MSB 0x1F
#define EUL_PITCH_LSB 0x1E
// Roll
#define EUL_ROLL_MSB 0x1D
#define EUL_ROLL_LSB 0x1C
// Heading
#define EUL_HEADING_MSB 0x1B
#define EUL_HEADING_LSB 0x1A
// Sensor Data
#define GYR_DATA_Z_MSB 0x19
#define GYR_DATA_Z_LSB 0x18
#define GYR_DATA_Y_MSB 0x17
#define GYR_DATA_Y_LSB 0x16
#define GYR_DATA_X_MSB 0x15
#define GYR_DATA_X_LSB 0x14

#define MAG_DATA_Z_MSB 0x13
#define MAG_DATA_Z_LSB 0x12
#define MAG_DATA_Y_MSB 0x11
#define MAG_DATA_Y_LSB 0x10
#define MAG_DATA_X_MSB 0xF
#define MAG_DATA_X_LSB 0xE

#define ACC_DATA_Z_MSB 0xD
#define ACC_DATA_Z_LSB 0xC
#define ACC_DATA_Y_MSB 0xB
#define ACC_DATA_Y_LSB 0xA
#define ACC_DATA_X_MSB 0x9
#define ACC_DATA_X_LSB 0x8

// Extra
#define PAGE_ID 0x7
#define BL_REV_ID 0x6
#define SW_REV_ID_MSB 0x5
#define SW_REV_ID_LSB 0x4
#define GYR_ID 0x3
#define MAG_ID 0x2
#define ACC_ID 0x1
#define CHIP_ID 0x0
/*
 * Page One. Use the PAGE_ID register to
 * switch and see values between the two pages
*/
#define GYR_AM_SET 0x1F
#define GYR_AM_THRES 0x1E
#define GYR_DUR_Z 0x1D
#define GYR_HR_Z_SET 0x1C
#define GYR_DUR_Y 0x1B
#define GYR_HR_Y_SET 0x1A
#define GYR_DUR_X 0x19
#define GYR_HR_X_SET 0x18
#define GYR_INT_SETING 0x17
#define ACC_NM_SET 0x16
#define ACC_NM_THRE 0x15
#define ACC_HG_THRES 0x14
#define ACC_HG_DURATION 0x13
#define ACC_INT_SETTINGS 0x12
#define ACC_AM_THRES 0x11
#define INT_EN 0x10
#define INT_MSK 0xF
#define GYR_SLEEP_CONFIG 0xD
#define ACC_SLEEP_CONFIG 0xC
#define GYR_CONFIG_1 0xB
#define GYR_CONFIG_0 0xA
#define MAG_CONFIG 0x9
#define ACC_CONFIG 0x8
#define PAGE_ID 0x7

// State Control
void switchToErrorState(screenStates* s);

// LCD Stuff
void LCD_printFlt(float data);
void LCD_printFltDecLim(float data);

// Button Interrupt
void btnFourIRQ(screenStates* state, screenStates* prevState);
void btnNineToFiveIRQ(screenStates* state, screenStates* prevState, uint8_t* isLogging);
void btnFifteenToTenIEQ(screenStates* state, screenStates* prevState, uint8_t* isLogging);

// GPS Stuff
void parseGps(gpsData *data, char dataBuffer[]);
void determineMax(gpsData* GPSData, Log* Log, runData* run_data);
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef huart1);
float calcDistance(float lat1, float long1, float lat2, float long2);
uint16_t calcCheckSum(char buffer[100]);
void checkRunStatus(gpsData* data, runData* run_data);
void stopGPS(UART_HandleTypeDef* huart1);
void startGPS(UART_HandleTypeDef* huart1);

// Bluetooth Stuff
void BT_sendData(UART_HandleTypeDef *huart2, uint8_t* str, uint32_t size);
void printScreen(screenStates* s, Log* log);
uint8_t readSDsendBT(UART_HandleTypeDef *huart2);

// IMU Stuff
void IMU_Config(I2C_HandleTypeDef* hi2c1);
void IMU_CalibrateRegisters(I2C_HandleTypeDef* hi2c1);
void IMU_Calibrate(I2C_HandleTypeDef* hi2c1);
void IMU_GET_EUL(I2C_HandleTypeDef* hi2c1, float* Euler[]);
void IMU_GET_LIA(I2C_HandleTypeDef* hi2c1, float* LIA[]);
float IMU_GET_TEMP(I2C_HandleTypeDef* hi2c1);
void IMU_PRINT_DATA(I2C_HandleTypeDef* hi2c1, float* Data[]);
void IMU_POWER_ON(I2C_HandleTypeDef* hi2c1);
void IMU_POWER_OFF(I2C_HandleTypeDef* hi2c1);
void IMU_SAVE_OFFSET(I2C_HandleTypeDef* hi2c1, IMU_OFFSET* offset_type);
void IMU_SET_OFFSET(I2C_HandleTypeDef* hi2c1, IMU_OFFSET* offset_type);
float IMU_GET_ROLL(I2C_HandleTypeDef* hi2c1);
void IMU_Determine_Max_Slope(I2C_HandleTypeDef* hi2c1, float curSlope, float prevSlope, Log* log);
float IMU_GET_ORIENTATION_FOR_SLOPE(I2C_HandleTypeDef* hi2c1); // After this, device cannot move
float getBatteryPercentage(float temp, uint32_t adcVal);

// SD Card
uint8_t SD_write(Log* log);

#ifdef __cplusplus
}
#endif

#endif
