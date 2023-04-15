#include "driver.h"

void btnFourIRQ(screenStates* state, screenStates* prevState) {

	switch(*state) {
		case speed:
			*prevState = *state;
			*state = alt;
			break;
		case alt:
			*prevState = *state;
			*state = longest;
			break;
		case longest:
			*prevState = *state;
			*state = tallest;
			break;
		case tallest:
			*prevState = *state;
			*state = speed;
			break;
		case startLog:
			*state = *prevState;
			break;
		case stopLog:
			*state = *prevState;
			break;
		case pauseLog:
			break;
		case resumeLog:
			*state = *prevState;
			break;
		case save:
			break;
		case battery:
			*state = *prevState;
			break;
		default:
			*prevState = *state;
			*state = error;
			break;
	}
}

void btnNineToFiveIRQ(screenStates* state, screenStates* prevState, uint8_t isLogging) {
	switch(*state) {
		case speed:
		case alt:
		case longest:
		case tallest:
			*prevState = *state;
			*state = isLogging ? stopLog : startLog;
			break;
		case startLog:
			isLogging = 1;
			break;
		case stopLog:
			isLogging = 0;
			break;
		case pauseLog:
		case resumeLog:
			isLogging = 0;
			*state = stopLog;
			break;
		case save:
			break;
		case battery:
			*state = isLogging ? stopLog : startLog;
			break;
		default:
			break;
	}
}

void btnFifteenToTenIEQ(screenStates* state, screenStates* prevState, uint8_t isLogging) {
	switch(*state) {
		case speed:
		case alt:
		case longest:
		case tallest:
			*state = isLogging ? pauseLog : battery;
			break;
		case startLog:
			*state = pauseLog;
			break;
		case stopLog:
			*state = *prevState;
			break;
		case pauseLog:
			*state = resumeLog;
			break;
		case resumeLog:
			*state = *prevState;
			break;
		case save:
			break;
		case battery:
			break;
		default:
			break;
	}
}

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
			}		}

	}
	return data;
}

void determineMax(gpsData* GPSData, Log* Log) {
	Log->maxSpeed = (GPSData->speedMph > Log->maxSpeed) ? GPSData->speedMph : Log->maxSpeed;
	Log->maxAltitude = (GPSData->altitude > Log->maxAltitude) ? GPSData->altitude : Log->maxAltitude;
}

 // void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart1, gpsData* GPSData, char bufferByte[1]) { HAL_UART_Receive_IT(huart1, (uint8_t *) bufferByte, 1);}

// Bluetooth Functions
void btSendData(UART_HandleTypeDef *huart2, uint8_t* str, uint32_t size) {
	HAL_UART_Transmit(huart2, str, size, 1000);
}

// LCD Functions
void printScreen(screenStates* s, Log* log) {
	HD44780_SetCursor(0, 0);
	char temp[16];
	switch(*s) {
		case speed:
			HD44780_PrintStr("Max Speed (m/s):");
			HD44780_SetCursor(1, 0);
			sprintf(temp, "%f", log->maxSpeed);
			HD44780_PrintStr("                ");
			HD44780_PrintStr(temp);
			break;
		case alt:
			HD44780_PrintStr("Max Alt. (m):   ");
			HD44780_SetCursor(1, 0);
			sprintf(temp, "%f", log->maxAltitude);
			HD44780_PrintStr("                ");
			HD44780_PrintStr(temp);
			break;
		case longest:
			HD44780_PrintStr("Longest Run (m):");
			HD44780_SetCursor(1, 0);
			sprintf(temp, "%f", log->longestRun);
			HD44780_PrintStr("                ");
			HD44780_PrintStr(temp);
			break;
		case tallest:
			HD44780_PrintStr("Tallest Run (m):");
			HD44780_SetCursor(1, 0);
			sprintf(temp, "%f", log->tallestRun);
			HD44780_PrintStr("                ");
			HD44780_PrintStr(temp);
			break;
		case startLog:
			HD44780_PrintStr("Starting log    ");
			HD44780_SetCursor(1, 0);
			HD44780_PrintStr("                ");
			// delay - while gps !connected
			state = prevState;
			break;
		case stopLog:
			HD44780_PrintStr("Stopping log    ");
			HD44780_SetCursor(1, 0);
			HD44780_PrintStr("                ");
			// delay - 2 secs-ish
			state = save;
			break;
		case pauseLog:
			HD44780_PrintStr("Log paused      ");
			HD44780_SetCursor(1, 0);
			HD44780_PrintStr("                ");
			// pasue logging
			break;
		case resumeLog:
			HD44780_PrintStr("Resuming log... ");
			HD44780_SetCursor(1, 0);
			HD44780_PrintStr("                ");
			// restart logging
			state = prevState;
			break;
		case save:
			HD44780_PrintStr("Log Saved       ");
			HD44780_SetCursor(1, 0);
			HD44780_PrintStr("                ");
			// save data to sd
			state = prevState;
			break;
		case battery:
			HD44780_PrintStr("Battery %:      ");
			HD44780_SetCursor(1, 0);
			HD44780_PrintStr("                ");
			HD44780_PrintStr("battery percent ");
			break;
		default:
			HD44780_PrintStr("Error           ");
			HD44780_SetCursor(1, 0);
			HD44780_PrintStr("                ");
			break;
	}
}

float calcDistance(float lat1, float long1, float lat2, float long2) {
	int R = 6371000;
	float phi1 = lat1 * M_PI/180;
	float phi2 = lat2 * M_PI/180;
	float delPhi = (lat2-lat1) * M_PI/180;
	float delLam = (long2-long1) * M_PI/180;

	float a = (sin(delPhi/2)*sin(delPhi/2)) + (cos(phi1)*cos(phi2)*sin(delLam/2)*sin(delLam/2));
	float c = 2 * atan2(sqrt(a), sqrt(1-a));

	return R * c;
}


void IMU_Config(I2C_HandleTypeDef* hi2c1) {
	uint8_t buf[2];
	buf[0] = OPR_MODE;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);
	HAL_Delay(30);

	/* Reset */
	buf[0] = SYS_TRIGGER;
	buf[1] = 0x20;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);
	HAL_Delay(30);

	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 1, 1000);
	while (buf[0] != 0xA0) {
		HAL_Delay(10);
		HAL_I2C_Master_Receive(hi2c1, I2C_Addr, &buf[0], 1, 1000);
	}

	HAL_Delay(50);

	/* Normal Power Mode */
	buf[0] = PWR_MODE;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);
	HAL_Delay(10);

	/* Set Page Number to 0 */
	buf[0] = PAGE_ID;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	/* Reset 8 */
	buf[0] = SYS_TRIGGER;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);
	HAL_Delay(10);

	/* Set Operation Mode */
	buf[0] = OPR_MODE;
	buf[1] = 0x0C;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);
	HAL_Delay(30);
}

void IMU_CalibrateRegisters(I2C_HandleTypeDef* hi2c1) {
	uint8_t buf[2];
	buf[0] = ACC_OFFSET_X_LSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_OFFSET_X_MSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_OFFSET_Y_LSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_OFFSET_Y_MSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_OFFSET_Z_LSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_OFFSET_Z_MSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_X_LSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_X_MSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_Y_LSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_Y_MSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_Z_LSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_Z_MSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_X_LSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_X_MSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_Y_LSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_Y_MSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_Z_LSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_Z_MSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_RADIUS_LSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_RADIUS_MSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_RADIUS_LSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_RADIUS_MSB;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);
}

void IMU_Calibrate(I2C_HandleTypeDef* hi2c1) {
	uint8_t system, gyro, accel, mg = 0;
	uint8_t buf[1];
	buf[0] = CALIB_STAT;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, I2C_Addr, &buf[0], 1, 1000);
	system = (buf[0] >> 6) & 0x03;
	gyro = (buf[0] >> 4) & 0x03;
	accel = (buf[0] >> 2) & 0x03;
	mg = buf[0] & 0x03;
	// Stays in calibration until IMU is calibrated
	while ((accel + gyro + mg + system) < 12) {
		printf("Calibrating... aCal: %d gCal: %d mCal: %d sCal: %d \n\r", accel, gyro, mg, system);
		HAL_Delay(100);
	}
}

void IMU_GET_EUL(I2C_HandleTypeDef* hi2c1, float Euler[]) {
	int16_t MEuler[3] = {0, 0, 0};
	uint8_t buf[6];
	buf[0] = EUL_HEADING_LSB;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, I2C_Addr, &buf[0], 6, 1000);
	MEuler[0] = buf[0] | (buf[1] << 8); // Heading
	MEuler[1] = buf[2] | (buf[3] << 8); // Roll
	MEuler[2] = buf[4] | (buf[5] << 8); // Pitch

	// Might need to check this code
	Euler[0] = fmod((float) MEuler[0] / 16.0, 360.0);
	Euler[1] = fmod((float) MEuler[1] / 16.0, 360.0);
	Euler[2] = fmod((float) MEuler[2] / 16.0, 360.0);
}
void IMU_GET_LIA(I2C_HandleTypeDef* hi2c1, float LIA[]) {
	uint8_t buf[6];
	int16_t MLIA[3] = {0, 0, 0};
	buf[0] = LIA_DATA_X_LSB;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, I2C_Addr, &buf[0], 6, 1000);
	MLIA[0] = buf[0] | (buf[1] << 8);
	MLIA[1] = buf[2] | (buf[3] << 8);
	MLIA[2] = buf[4] | (buf[5] << 8);

	LIA[0] = (float) MLIA[0] / 100.0;
	LIA[1] = (float) MLIA[1] / 100.0;
	LIA[2] = (float) MLIA[2] / 100.0;
}

void IMU_PRINT_DATA(I2C_HandleTypeDef* hi2c1, float Data[]) {
	printf("%f %f %f \n\r", Data[0], Data[1], Data[2]);
}