#include "driver.h"

void btnFourIRQ(screenStates* state, screenStates* prevState) {
	buzEnable = 1;
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
			*state = steepest;
			break;
		case steepest:
			*state = *prevState;
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

void btnNineToFiveIRQ(screenStates* state, screenStates* prevState, uint8_t* isLogging) {
	buzEnable = 1;
	switch(*state) {
		case speed:
			*prevState = *state;
			*state = *isLogging ? stopLog : startLog;
			break;
		case alt:
			*prevState = *state;
			*state = *isLogging ? stopLog : startLog;
			break;
		case longest:
			*prevState = *state;
			*state = *isLogging ? stopLog : startLog;
			break;
		case tallest:
			*prevState = *state;
			*state = *isLogging ? stopLog : startLog;
			break;
		case steepest:
			*prevState = *state;
			*state = *isLogging ? stopLog : startLog;
			break;
		case startLog:
			*isLogging = 1;
			break;
		case stopLog:
			*isLogging = 0;
			break;
		case pauseLog:
			*isLogging = 0;
			*state = stopLog;
			break;
		case resumeLog:
			*isLogging = 0;
			*state = stopLog;
			break;
		case save:
			break;
		case battery:
			*state = *isLogging ? stopLog : startLog;
			break;
		default:
			break;
	}
}

void btnFifteenToTenIEQ(screenStates* state, screenStates* prevState, uint8_t* isLogging) {
	buzEnable = 1;
	switch(*state) {
		case speed:
			*state = *isLogging ? pauseLog : battery;
			break;
		case alt:
			*state = *isLogging ? pauseLog : battery;
			break;
		case longest:
			*state = *isLogging ? pauseLog : battery;
			break;
		case tallest:
			*state = *isLogging ? pauseLog : battery;
			break;
		case steepest:
			*state = *isLogging ? pauseLog : battery;
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

uint16_t calcCheckSum(char buffer[100]){
	int i = 1;
	char letter = *(buffer);
	uint16_t sum = 0;

	while(letter != '*'){

		letter = *(buffer+i);

		if(letter != '$' && letter != '*'){
			sum = sum ^ letter;
		}

		++i;
	}
	return sum;
}

void swap(float* xp, float* yp){
	//Adapted from GeeksForGeeks
    float temp = *xp;
    *xp = *yp;
    *yp = temp;
}



void parseGps(gpsData *data, char inputBuffer[]){

	int dataElementIndex = 0;
	int dataElementNum = 0;
	int timeCount = 0;

	char dataType[7] = "XXXXXX";

	for(uint8_t i = 0; i < BUFFER_SIZE; ++i){
		char letter = *(inputBuffer+i);

		if(letter == ','){
			++dataElementNum;
			dataElementIndex = 0;
		}

		/*if(letter == '\n'){
			++dataElementNum;
			dataElementIndex = 0;
		}*/

		if(dataElementNum == 0 && letter != ','){
			//datatype, either GPGGA or GPRMC
			dataType[dataElementIndex] = letter;
			++dataElementIndex;
		}


		if(strcmp(dataType,"$GPGGA") == 0 && letter != ','){
			if (dataElementNum == 1 ){

				if (timeCount <= 1){
					data->hoursChar[dataElementIndex] = letter;
					++dataElementIndex;
					if(timeCount == 1){
						data->hours = atoi(data->hoursChar);
						dataElementIndex = 0;
					}

				}else if(timeCount <= 3){
					data->minsChar[dataElementIndex] = letter;
					++dataElementIndex;
					if (timeCount == 3){
						data->mins = atoi(data->minsChar);
						dataElementIndex = 0;
					}

				}else if(timeCount > 3){
					data->secsChar[dataElementIndex] = letter;
					++dataElementIndex;
					if (*(inputBuffer+i+1) == ','){
						data->secs = strtof(data->secsChar, NULL);
						dataElementIndex = 0;
						data->timeInSecs = data->secs + (data->mins * 60) + (data->hours * 24 * 60);
					}

				}

				++timeCount;

			} else if (dataElementNum == 2 ){
				data->latitudeChar[dataElementIndex] = letter;
				++dataElementIndex;
				if(*(inputBuffer+i+1) == ','){
					data->latitude[0] = data->latitude[1];
					float tempLat = (strtof(data->latitudeChar, NULL))/100.0;
					float degL = (float)((int)tempLat);
					degL += (tempLat - (float)degL)*10/6;
					data->latitude[1] = tempLat;
				}

			} else if (dataElementNum == 3){
				data->latDir = letter;
				if(data->latDir == 'S'){
					data->latitude[1] *= -1;
				}

			} else if (dataElementNum == 4){
				data->longitudeChar[dataElementIndex] = letter;
				++dataElementIndex;
				if(*(inputBuffer+i+1) == ','){
					data->longitude[0] = data->longitude[1];
					float tempLong = (strtof(data->longitudeChar, NULL))/100.0;
					float degO = (float)((int)tempLong);
					degO += (tempLong - (float)degO)*10/6;
					data->longitude[1] = tempLong;
				}

			} else if (dataElementNum == 5){
				data->longDir = letter;

				if(data->longDir == 'W'){
					data->longitude[1] *= -1;
				}

				(data->speedMph[0]) = (data->speedMph[1]);
				(data->speedMph[1]) = (data->speedMph[2]);
				(data->speedMph[2]) = (calcDistance(data->latitude[0], data->longitude[0],data->latitude[1], data->longitude[1])/3.0)*2.237;//m/s->mph

			} else if (dataElementNum == 6){
				data->fix = (uint8_t) (letter - '0');

			} else if (dataElementNum == 7){
				data->numSatellitesChar[dataElementIndex] = letter;
				++dataElementIndex;
				if(*(inputBuffer+i+1) == ','){
					data->numSatellites = atoi(data->numSatellitesChar);
				}

			} else if (dataElementNum == 8){
				data->hdopChar[dataElementIndex] = letter;
				++dataElementIndex;
				if(*(inputBuffer+i+1) == ','){
					data->hdop = strtof(data->hdopChar, NULL);
				}

			} else if (dataElementNum == 9){
				data->altitudeChar[dataElementIndex] = letter;
				++dataElementIndex;
				if(*(inputBuffer+i+1) == ','){
					float alt = strtof(data->altitudeChar, NULL);

					//Either we are on a plane or this is wrong because mt everest is 8849 M tall rn
					if(alt <= 8900 ){
						//Make sure this value isn't already in the array
						if(alt != data->altitude[2] && alt != data->altitude[1] && alt != data->altitude[0]){
							//Init the array with first alt for median
							if(data->altitude[2] == 0){
								data->altitude[0] = alt;
								data->altitude[1] = alt;
								data->altitude[2] = alt;
							}
							swap(&data->altitude[1], &data->altitude[0]);
							swap(&data->altitude[2], &data->altitude[1]);
							//What was in [0] gets replaced by the new value in [2]
							data->altitude[2] = alt;
						}
					}else {
						if(alt != data->altitude[2] && alt != data->altitude[1] && alt != data->altitude[0]){
							if(data->altitude[2] == 0){
								data->altitude[0] = -1;
								data->altitude[1] = -1;
								data->altitude[2] = -1;
							}
							swap(&data->altitude[1], &data->altitude[0]);
							swap(&data->altitude[2], &data->altitude[1]);
							//What was in [0] gets replaced by the new value in [2]
							data->altitude[2] = -1;
						}
					}
				}

			} else if (dataElementNum == 10){
				data->altitudeUnits = letter;

			} /*else if (dataElementNum == 14){

				//ignore the *
				if(letter != '*' && letter != '\r' && letter != '\n'){
					data->checksumgga[dataElementIndex] = letter;
					++dataElementIndex;
				}else if (letter == '*'){
					dataElementIndex = 0;
				}

				if(dataElementIndex == 2){
					uint16_t check = calcCheckSum(inputBuffer);
					char checkHex[3];
					sprintf(checkHex, "%02X", check);

					if(strcmp(data->checksumgga, checkHex)==0){
						//data is good
						data->ggaGood = 1;
					}else {
						data->ggaGood = 0;
					}

				}

			}*/
			if(letter == '*'){
				data->checksumgga[0] = *(inputBuffer+i+1);
				data->checksumgga[1] = *(inputBuffer+i+2);

				uint16_t check = calcCheckSum(inputBuffer);
				char checkHex[3];
				sprintf(checkHex, "%02X", check);

				if(strcmp(data->checksumgga, checkHex)==0){
					//data is good
					data->ggaGood = 1;
				}else {
					data->ggaGood = 0;
				}
			}

		} /*else if(strcmp(dataType,"$GPRMC") == 0 && letter != ','){
			if (dataElementNum == 2){
				data->validity = letter;

			} else if (dataElementNum == 7){
				data->speedCharKnots[dataElementIndex] = letter;
				++dataElementIndex;
				if(*(inputBuffer+i+1) == ','){
					float speed = 1.15077945 * strtof(data->speedCharKnots, NULL);


					//Either we are on a military jet or this is wrong because the land speed record is 1200mph and jetliners fly at like 600-800mph
					if(speed <= 100){
						if(speed != data->speedMph[2] && speed != data->speedMph[1] && speed != data->speedMph[0]){
							//Init 0 and 1 to the first speed so median works good
							if(data->speedMph[2] == 0){
								data->speedMph[0] = speed;
								data->speedMph[1] = speed;
								data->speedMph[2] = speed;
							}

							swap(&data->speedMph[1], &data->speedMph[0]);
							swap(&data->speedMph[2], &data->speedMph[1]);
							//What was in [0] gets replaced by the new value in [2]
							data->speedMph[2] = speed;
						}

					}else {
						if(speed != data->speedMph[2] && speed != data->speedMph[1] && speed != data->speedMph[0]){
							if(data->speedMph[2] == 0){
								data->speedMph[0] = -1;
								data->speedMph[1] = -1;
								data->speedMph[2] = -1;
							}

							swap(&data->speedMph[1], &data->speedMph[0]);
							swap(&data->speedMph[2], &data->speedMph[1]);
							//What was in [0] gets replaced by the new value in [2]
							data->speedMph[2] = -1;
						}
					}


				}

			} /*else if (dataElementNum == 12){

				//ignore the *
				if(letter != '*' && letter != '\r' && letter != '\n'){
					data->checksumrmc[dataElementIndex] = letter;
					++dataElementIndex;
				}else if (letter == '*'){
					dataElementIndex = 0;
				}

				if(dataElementIndex == 2){
					uint16_t check = calcCheckSum(inputBuffer);
					char checkHex[3];
					sprintf(checkHex, "%02X", check);

					if(strcmp(data->checksumrmc, checkHex)==0){
						//data is good
						data->rmcGood = 1;
					}else {
						data->rmcGood = 0;
					}

				}

			}
			if(letter == '*'){
				data->checksumrmc[0] = *(inputBuffer+i+1);
				data->checksumrmc[1] = *(inputBuffer+i+2);

				uint16_t check = calcCheckSum(inputBuffer);
				char checkHex[3];
				sprintf(checkHex, "%02X", check);

				if(strcmp(data->checksumrmc, checkHex)==0){
					//data is good
					data->rmcGood = 1;
				}else {
					data->rmcGood = 0;
				}
			}
		}*/


		if(letter == '\n' || letter == '\r'){
			break;
		}
	}
}

void stopGPS(UART_HandleTypeDef *huart1){
	//$PMTK161,0*28<CR><LF>
	//Put GPS into standby, power saving mode
	char inputBuffer[] = "$PMTK161,0*28\r\n";
	HAL_UART_Transmit(huart1, (uint8_t *) inputBuffer, sizeof(inputBuffer), 100);
}

void startGPS(UART_HandleTypeDef *huart1){
	//$PMTK225,0*2B<CR><LF>
	char inputBuffer[] = "$PMTK225,0*2B\r\n";
	HAL_UART_Transmit(huart1, (uint8_t *) inputBuffer, sizeof(inputBuffer), 100);
}

void bubbleSort(float arr[], int n){
	//sort (bubble)
	//Paraphrased from GeeksForGeeks
	for(uint8_t i = 0; i < 2; ++i){
		for(int j = 0; j < 1; ++j){
			if(arr[j] > arr[j+1]){
				swap(&arr[j], &arr[j+1]);
			}
		}
	}
}

float medianThree(float nums[3]){
	//bubbleSort(nums, 3);
	//return nums[1];
	float median;
	int maxIndex;
	int medianIndex=2;
	int minIndex;
	float max = -100;
	float min = 100;
	for(int i = 0; i < 2; i++){
		if(nums[i]>max){
			max=nums[i];
			maxIndex = i;
		}
		if(nums[i]<min){
			min=nums[i];
			minIndex = i;
		}
	}
	for(int i = 0; i < 2; i++){
		if(i != maxIndex && i != minIndex){
			medianIndex = i;
		}
	}
	return nums[medianIndex];
}

void determineMax(gpsData* GPSData, Log* Log) {
	float speed = medianThree(GPSData->speedMph);
	Log->maxSpeed = (speed > Log->maxSpeed && GPSData->ggaGood == 1 && GPSData->numSatellites >= 4) ? speed : Log->maxSpeed;

	float alt = medianThree(GPSData->altitude);
	Log->maxAltitude = (alt > Log->maxAltitude && GPSData->ggaGood == 1 && GPSData->numSatellites >= 4) ? alt : Log->maxAltitude;
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
			// TODO This should timeout using a timer
			/* uint8_t count = 0;
			 * while (gps_data.fix != 1) {
			 *	++count;
			 * }
			 *
			 * state = (count >= 1000000) ? stopLog : prevState;
			 */
			state = prevState;
			break;
		case stopLog:
			HD44780_PrintStr("Stopping log    ");
			HD44780_SetCursor(1, 0);
			HD44780_PrintStr("                ");
			// delay - 2 secs-ish
			/* HAL_Delay(2000); */
			state = save;
			break;
		case pauseLog:
			HD44780_PrintStr("Log paused      ");
			HD44780_SetCursor(1, 0);
			HD44780_PrintStr("                ");
			// pause logging
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
//			sdTest();
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

	// Setting up writing
	buf[1] = (0 << 7) | // Orientation = Android
	         (0 << 4) | // Temperature = Celsius
	         (0 << 2) | // Euler = Degrees
	         (1 << 1) | // Gyro = Rads
	         (0 << 0);  // Accelerometer = m/s^2
	buf[0] = UNIT_SEL;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	// Setup temp source as accelerometer
	buf[0] = TEMP_SOURCE;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

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
	// TODO Times out after 100 seconds or smth
	// Stays in calibration until IMU is calibrated
	while ((accel + gyro + mg + system) < 12) {
		printf("Calibrating... aCal: %d gCal: %d mCal: %d sCal: %d \n\r", accel, gyro, mg, system);
		HAL_Delay(100);
	}
}

void IMU_GET_EUL(I2C_HandleTypeDef* hi2c1, float* Euler[]) {
	int16_t MEuler[3] = {0, 0, 0};
	uint8_t buf[6];
	buf[0] = EUL_HEADING_LSB;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, I2C_Addr, &buf[0], 6, 1000);
	MEuler[0] = buf[0] | (buf[1] << 8); // Heading
	MEuler[1] = buf[2] | (buf[3] << 8); // Roll
	MEuler[2] = buf[4] | (buf[5] << 8); // Pitch

	// TODO Might need to check this code
	*Euler[0] = fmod((float) MEuler[0] / 16.0, 360.0);
	*Euler[1] = fmod((float) MEuler[1] / 16.0, 360.0);
	*Euler[2] = fmod((float) MEuler[2] / 16.0, 360.0);
}

void IMU_GET_LIA(I2C_HandleTypeDef* hi2c1, float* LIA[]) {
	uint8_t buf[6];
	int16_t MLIA[3] = {0, 0, 0};
	buf[0] = LIA_DATA_X_LSB;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, I2C_Addr, &buf[0], 6, 1000);
	MLIA[0] = buf[0] | (buf[1] << 8);
	MLIA[1] = buf[2] | (buf[3] << 8);
	MLIA[2] = buf[4] | (buf[5] << 8);

	*LIA[0] = (float) MLIA[0] / 100.0;
	*LIA[1] = (float) MLIA[1] / 100.0;
	*LIA[2] = (float) MLIA[2] / 100.0;
}

float IMU_GET_TEMP(I2C_HandleTypeDef* hi2c1) {
	uint8_t buf[1];
	buf[0] = TEMP;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, I2C_Addr, &buf[0], 1, 1000);
	return (( (float) buf[0] - 21.6) * 9 / 5) + 32;
}

void IMU_PRINT_DATA(I2C_HandleTypeDef* hi2c1, float* Data[]) {
	printf("%f %f %f \n\r", *Data[0], *Data[1], *Data[2]);
}

void IMU_POWER_ON(I2C_HandleTypeDef* hi2c1) {
	uint8_t buf[2];
	buf[0] = OPR_MODE;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);
	HAL_Delay(25);

	IMU_SET_OFFSET(hi2c1, &offset_cal_data);

	buf[0] = PWR_MODE;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = OPR_MODE;
	buf[1] = 0x0C;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);
	HAL_Delay(20);
}

// TODO putting sensor into suspend mode ruins calibration
void IMU_POWER_OFF(I2C_HandleTypeDef* hi2c1) {
	uint8_t buf[2];
	buf[0] = OPR_MODE;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);
	HAL_Delay(25);

	IMU_SAVE_OFFSET(hi2c1, &offset_cal_data);

	buf[0] = PWR_MODE;
	buf[1] = 0x02;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = OPR_MODE;
	buf[1] = 0x0C;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);
	HAL_Delay(20);
}

void IMU_SAVE_OFFSET(I2C_HandleTypeDef* hi2c1, IMU_OFFSET* offset_type) {
	IMU_Config(hi2c1);
	uint8_t buf[30];

	buf[0] = ACC_OFFSET_X_LSB;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, I2C_Addr, &buf[0], 22, 1000);
	offset_type->accel_offset_x = (buf[0]) | (buf[1] << 8);
	offset_type->accel_offset_y = (buf[2]) | (buf[3] << 8);
	offset_type->accel_offset_z = (buf[4]) | (buf[5] << 8);
	offset_type->mag_offset_x = (buf[6]) | (buf[7] << 8);
	offset_type->mag_offset_y = (buf[8]) | (buf[9] << 8);
	offset_type->mag_offset_z = (buf[10]) | (buf[11] << 8);
	offset_type->gyro_offset_x = (buf[12]) | (buf[13] << 8);
	offset_type->gyro_offset_y = (buf[14]) | (buf[15] << 8);
	offset_type->gyro_offset_z = (buf[16]) | (buf[17] << 8);
	offset_type->accel_radius = (buf[18]) | (buf[19] << 8);
	offset_type->mag_radius = (buf[20]) | (buf[21] << 8);
}

void IMU_SET_OFFSET(I2C_HandleTypeDef* hi2c1, IMU_OFFSET* offset_type) {
	uint8_t buf[30];

	buf[0] = MAG_RADIUS_MSB;
	buf[1] = (offset_type->mag_radius >> 8) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_RADIUS_LSB;
	buf[1] = (offset_type->mag_radius) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_RADIUS_MSB;
	buf[1] = (offset_type->accel_radius >> 8) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_RADIUS_LSB;
	buf[1] = (offset_type->accel_radius) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_Z_MSB;
	buf[1] = (offset_type->gyro_offset_z >> 8) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_Z_LSB;
	buf[1] = (offset_type->gyro_offset_z) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_Y_MSB;
	buf[1] = (offset_type->gyro_offset_y >> 8) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_Y_LSB;
	buf[1] = (offset_type->gyro_offset_y) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_X_MSB;
	buf[1] = (offset_type->gyro_offset_x >> 8) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_X_LSB;
	buf[1] = (offset_type->gyro_offset_x) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_Z_MSB;
	buf[1] = (offset_type->mag_offset_z >> 8) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_Z_LSB;
	buf[1] = (offset_type->mag_offset_z) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_Y_MSB;
	buf[1] = (offset_type->mag_offset_y >> 8) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_Y_LSB;
	buf[1] = (offset_type->mag_offset_y) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_X_MSB;
	buf[1] = (offset_type->mag_offset_x >> 8) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_X_LSB;
	buf[1] = (offset_type->mag_offset_x) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_OFFSET_Z_MSB;
	buf[1] = (offset_type->accel_offset_z >> 8) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_OFFSET_Z_LSB;
	buf[1] = (offset_type->accel_offset_z) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_OFFSET_Y_MSB;
	buf[1] = (offset_type->accel_offset_y >> 8) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_OFFSET_Y_LSB;
	buf[1] = (offset_type->accel_offset_y) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_OFFSET_X_MSB;
	buf[1] = (offset_type->accel_offset_x >> 8) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_OFFSET_X_LSB;
	buf[1] = (offset_type->accel_offset_x) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c1, I2C_Addr, &buf[0], 2, 1000);
}

void sdTest(Log* log) {

	  FATFS       FatFs;                //Fatfs handle
	  FIL         fil;                  //File handle
	  FRESULT     fres;                 //Result after operations

	  fres = f_mount(&FatFs, "", 1);    //1=mount now
	  if (fres != FR_OK) {
		return;
	  }

	  fres = f_open(&fil, "logs.txt", FA_WRITE | FA_OPEN_ALWAYS);
	  if (fres != FR_OK) {
//		  state = error;
		  return;
	  }
	  char t[100];

	  f_puts("max speed: ", &fil);
	  sprintf(t, "%f", log->maxSpeed);
	  f_puts(t, &fil);

	  f_puts(" | max alt: ", &fil);
	  sprintf(t, "%f", log->maxAltitude);
	  f_puts(t, &fil);

	  f_puts(" | tallest: ", &fil);
	  sprintf(t, "%f", log->tallestRun);
	  f_puts(t, &fil);

	  f_puts(" | longest: ", &fil);
	  sprintf(t, "%f", log->longestRun);
	  f_puts(t, &fil);

	  f_puts(" | num runs: ", &fil);
	  sprintf(t, "%i", log->numberOfRuns);
	  f_puts(t, &fil);

	  f_puts("\n", &fil);

	  for (uint8_t i = 0; i < log->numberOfRuns; ++i) {

	   	f_puts("\trun ", &fil);
	   	sprintf(t, "%i", i);
		f_puts(t, &fil);
	   	f_puts(" - ", &fil);

	 	f_puts(" elapsed time: ", &fil);
	    	sprintf(t, "%i", log->run[i].elapsedTime);
	  	f_puts(t, &fil);

	    	f_puts(" | vert dist: ", &fil);
	    	sprintf(t, "%i", log->run[i].verticalDistance);
	    	f_puts(t, &fil);

	    	f_puts(" | horiz dist: ", &fil);
	    	sprintf(t, "%i", log->run[i].horizontalDistance);
	    	f_puts(t, &fil);

	    	f_puts(" | avg speed: ", &fil);
	    	sprintf(t, "%i", log->run[i].averageSpeed);
	    	f_puts(t, &fil);

		f_puts(" | num points: ", &fil);
	    	sprintf(t, "%i", log->run[i].numberOfPoints);
	    	f_puts(t, &fil);

		f_puts("\n", &fil);

		for (uint8_t j = 0; j < log->run[i].numberOfPoints; ++j) {

			f_puts("\t\tdata point ", &fil);
	    		sprintf(t, "%d", j);
	    		f_puts(t, &fil);
			f_puts(" - ", &fil);

			f_puts(" speed: ", &fil);
	    		sprintf(t, "%i", log->run[i].data[j].GPSspeed);
	    		f_puts(t, &fil);

			f_puts(" | altitude: ", &fil);
	    		sprintf(t, "%i", log->run[i].data[j].altitude);
	    		f_puts(t, &fil);

			f_puts(" | longitude: ", &fil);
	    		sprintf(t, "%i", log->run[i].data[j].longitude);
	    		f_puts(t, &fil);

			f_puts(" | latitude: ", &fil);
	    		sprintf(t, "%i", log->run[i].data[j].latitude);
	    		f_puts(t, &fil);

			f_puts("\n", &fil);

		}

//		for (uint8_t k = 0; k < 6000; ++k) {
//			f_puts("\t\t ", &fil);
//			for (uint8_t a = 0; a < 3; ++a) {
//				sprintf(t, "%f", log->run[i].IMUspeed[k][a]);
//	    			f_puts(t, &fil);
//				f_puts(", ", &fil);
//			}
//			f_puts("\n", &fil);
//		}

	   }
	  f_close(&fil);
	  f_mount(NULL, "", 0);
}

float getBatteryPercentage(float temp, uint32_t adcVal){
	int batteryCalcMatrix[3][2] = {{5,2},{-5,5},{-100,10}}; //each row is a temp band, going lowest to highest
	float delta = 2.0f*((adcVal+350)*2.0/2260.0) - 2.8;
//	float delta = (adcVal*2)/2260.0;
	float percentage = -999;
	if(delta <= 0){
		percentage = 0;
		return percentage;
	}
	else if(delta >=1.2f){
		percentage = 100;
		return percentage;
	}
	for(int i = 0; i < 3; i++){
		if(temp > batteryCalcMatrix[i][0]){
			if(delta <= 0.2f){
				percentage = batteryCalcMatrix[i][1]*(delta/0.2);
			}
			else{
				percentage = (100.0 - batteryCalcMatrix[i][1])*(delta - 0.2) + batteryCalcMatrix[i][1];
			}
		}
	}

	return percentage;
}
