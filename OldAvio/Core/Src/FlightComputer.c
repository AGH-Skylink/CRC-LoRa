/*
 * FlightComputer.c
 *
 *  Created on: May 11, 2026
 *      Authors: Julia Brąglewicz
 *
 */

#include <math.h>
#include "FlightComputer.h"
#include "pid.h"
//#include "servos.h"
//#include "power.h"

#define ADXL345_SCALE_FACTOR  0.0039f      // 3.9 mg / LSB
#define GRAVITY_EARTH         9.80665f     // m/s^2
#define MPU_GYRO_SCALE_2000  16.4f

// Współczynniki dla domyślnego zakresu +/- 1.3 Gauss
#define HMC5883L_SCALE_XY   1090.0f
#define HMC5883L_SCALE_Z    980.0f
#define GAUSS_TO_UT         100.0f  // 1 Gauss = 100 uT

#define FRAME_TIME 50

// Local state enum (maps into flight_computer->state)
typedef enum {
	STATE_WAITING = 0,
	STATE_POWERED_ASCENT = 1,
	STATE_UNPOWERED_ASCENT = 2,
	STATE_DESCENT = 3,
	STATE_LANDED = 4,
	STATE_ABORT = 5
} StateName;

static PID_HandleTypedef pidX;
static PID_HandleTypedef pidZ;
static double angleX = 0.0;
static double angleZ = 0.0;

extern uint8_t uart_rx_buffer[40];
extern volatile uint8_t new_data_flag;

#define APOGEE_PRESSURE_WINDOW_SIZE 10
//#define ACC_CALIBRATION_SAMPLES 40
#define PRESSURE_CALIBRATION_SAMPLES 40

// 0 - dane z czujnikow, 1 - dane po uarcie
#define MODE 0

#define PRESSURE_SEA_LEVEL 101325.0f

static float FlightComputer_updateApogeePressureAverage(FlightComputer* flight_computer, float pressure) {
	uint8_t index = flight_computer->barothermo.apogee_pressure_window_index;

	if (flight_computer->barothermo.apogee_pressure_window_count < APOGEE_PRESSURE_WINDOW_SIZE) {
		flight_computer->barothermo.apogee_pressure_window_count++;
	} else {
		flight_computer->barothermo.apogee_pressure_window_sum -= flight_computer->barothermo.apogee_pressure_window[index];
	}

	flight_computer->barothermo.apogee_pressure_window[index] = pressure;
	flight_computer->barothermo.apogee_pressure_window_sum += pressure;
	flight_computer->barothermo.apogee_pressure_window_index = (uint8_t)((index + 1) % APOGEE_PRESSURE_WINDOW_SIZE);

	return flight_computer->barothermo.apogee_pressure_window_sum / flight_computer->barothermo.apogee_pressure_window_count;
}

void FlightComputer_scaleAccelerometer(FlightComputer* flight_computer) {
    // Przeliczenie bezpośrednio na m/s^2
    flight_computer->imu.accelerometer.x_scaled = (float)flight_computer->imu.accelerometer.x * ADXL345_SCALE_FACTOR * GRAVITY_EARTH;
    flight_computer->imu.accelerometer.y_scaled = (float)flight_computer->imu.accelerometer.y * ADXL345_SCALE_FACTOR * GRAVITY_EARTH;
    flight_computer->imu.accelerometer.z_scaled = (float)flight_computer->imu.accelerometer.z * ADXL345_SCALE_FACTOR * GRAVITY_EARTH;
    /*flight_computer->imu.accelerometer.x_biased = flight_computer->imu.accelerometer.x_scaled - flight_computer->imu.accelerometer.bias_x;
    flight_computer->imu.accelerometer.y_biased = flight_computer->imu.accelerometer.y_scaled - flight_computer->imu.accelerometer.bias_y;
    flight_computer->imu.accelerometer.z_biased = flight_computer->imu.accelerometer.z_scaled - flight_computer->imu.accelerometer.bias_z;*/
}

void FlightComputer_scaleGyroscope(FlightComputer* flight_computer) {
    // Przeliczenie surowych wartości na stopnie na sekundę (°/s)
    flight_computer->imu.gyroscope.x_scaled = (float)flight_computer->imu.gyroscope.x / MPU_GYRO_SCALE_2000;
    flight_computer->imu.gyroscope.y_scaled = (float)flight_computer->imu.gyroscope.y / MPU_GYRO_SCALE_2000;
    flight_computer->imu.gyroscope.z_scaled = (float)flight_computer->imu.gyroscope.z / MPU_GYRO_SCALE_2000;
}

void FlightComputer_scaleMagnetometer(FlightComputer* flight_computer) {
    // Przeliczenie na Gausy, a następnie na mikrotesle (uT)
    flight_computer->imu.magnetometer.x_scaled = ((float)flight_computer->imu.magnetometer.x / HMC5883L_SCALE_XY) * GAUSS_TO_UT;
    flight_computer->imu.magnetometer.y_scaled  = ((float)flight_computer->imu.magnetometer.y / HMC5883L_SCALE_XY) * GAUSS_TO_UT;
    flight_computer->imu.magnetometer.z_scaled  = ((float)flight_computer->imu.magnetometer.z / HMC5883L_SCALE_Z) * GAUSS_TO_UT;
}

/*void calculateAccelerometerBias(FlightComputer* flight_computer){
	float x_sum, y_sum, z_sum;
	int16_t x, y, z;
	uint8_t read_data[6];

	for(int i=0; i<ACC_CALIBRATION_SAMPLES; i++){
		if (HAL_I2C_Mem_Read(flight_computer->hi2c, 0x53 << 1, 0x32, 1, read_data, 6, 100) == HAL_OK) {
			x = (int16_t)(read_data[1] << 8 | read_data[0]);
			y = (int16_t)(read_data[3] << 8 | read_data[2]);
			z = (int16_t)(read_data[5] << 8 | read_data[4]);
			x_sum = x_sum + ((float)x * ADXL345_SCALE_FACTOR * GRAVITY_EARTH);
			y_sum = y_sum + ((float)y * ADXL345_SCALE_FACTOR * GRAVITY_EARTH);
			z_sum = z_sum + ((float)z * ADXL345_SCALE_FACTOR * GRAVITY_EARTH);
		}else{
			i = i-1;
		}
		HAL_Delay(50);
	}
	flight_computer->imu.accelerometer.bias_x = (float)(x_sum / ACC_CALIBRATION_SAMPLES);
	flight_computer->imu.accelerometer.bias_y = (float)(y_sum / ACC_CALIBRATION_SAMPLES);
	flight_computer->imu.accelerometer.bias_z = (float)(z_sum / ACC_CALIBRATION_SAMPLES);
}*/

void calculatePressureReference(FlightComputer* flight_computer){
	float pressure;
	float p_sum = 0;
	uint8_t read_data[6];
	int32_t pressure_raw, temperature_raw, t_fine;

	for(int i=0; i<PRESSURE_CALIBRATION_SAMPLES; i++){
		if (HAL_I2C_Mem_Read(flight_computer->hi2c, 0x76 << 1, 0xF7, 1, read_data, 6, 100) == HAL_OK) {
			pressure_raw = (int32_t)((read_data[0] << 12) | (read_data[1] << 4) | (read_data[2] >> 4));
			temperature_raw = (int32_t)((read_data[3] << 12) | (read_data[4] << 4) | (read_data[5] >> 4));
			t_fine= BaroThermo_calculate_t_fine(flight_computer, temperature_raw);
			p_sum = p_sum + BaroThermo_convertPressure(flight_computer, pressure_raw, t_fine);
		}else{
			i = i-1;
		}
		HAL_Delay(50);
	}

	flight_computer->barothermo.pressure_reference = (float)(p_sum / PRESSURE_CALIBRATION_SAMPLES);
}

void FlightComputer_calculateVectorLengths(FlightComputer* flight_computer) {
    // Accelerometer
    float ax = flight_computer->imu.accelerometer.x_scaled;
    float ay = flight_computer->imu.accelerometer.y_scaled;
    float az = flight_computer->imu.accelerometer.z_scaled;

    flight_computer->imu.accelerometer.acc_total = sqrtf((ax * ax) + (ay * ay) + (az * az));

    // Magnetometer
    float mx = flight_computer->imu.magnetometer.x_scaled;
    float my = flight_computer->imu.magnetometer.y_scaled;
    float mz = flight_computer->imu.magnetometer.z_scaled;

    flight_computer->imu.magnetometer.mag_total = sqrtf((mx * mx) + (my * my) + (mz * mz));
}

void Sensors_read(FlightComputer* flight_computer){
	uint8_t read_data[6];

	// ADXL345 accelerometer @ 0x53, reg 0x32
	if (HAL_I2C_Mem_Read(flight_computer->hi2c, 0x53 << 1, 0x32, 1, read_data, 6, 100) == HAL_OK) {
		flight_computer->imu.accelerometer.x = (int16_t)(read_data[1] << 8 | read_data[0]);
		flight_computer->imu.accelerometer.y = (int16_t)(read_data[3] << 8 | read_data[2]);
		flight_computer->imu.accelerometer.z = (int16_t)(read_data[5] << 8 | read_data[4]);
		// copy to telemetry frame positions 17-22 (legacy ordering)
		flight_computer->telemetry_frame[17] = read_data[1];
		flight_computer->telemetry_frame[18] = read_data[0];
		flight_computer->telemetry_frame[19] = read_data[3];
		flight_computer->telemetry_frame[20] = read_data[2];
		flight_computer->telemetry_frame[21] = read_data[5];
		flight_computer->telemetry_frame[22] = read_data[4];
	}

	FlightComputer_scaleAccelerometer(flight_computer);

	// MPU/gyro @ 0x68, reg 0x1D (read 6 bytes)
	if (HAL_I2C_Mem_Read(flight_computer->hi2c, 0x68 << 1, 0x1D, 1, read_data, 6, 100) == HAL_OK) {
		flight_computer->imu.gyroscope.x = (int16_t)(read_data[0] << 8 | read_data[1]);
		flight_computer->imu.gyroscope.y = (int16_t)(read_data[2] << 8 | read_data[3]);
		flight_computer->imu.gyroscope.z = (int16_t)(read_data[4] << 8 | read_data[5]);
		flight_computer->telemetry_frame[23] = read_data[0];
		flight_computer->telemetry_frame[24] = read_data[1];
		flight_computer->telemetry_frame[25] = read_data[2];
		flight_computer->telemetry_frame[26] = read_data[3];
		flight_computer->telemetry_frame[27] = read_data[4];
		flight_computer->telemetry_frame[28] = read_data[5];
	}

	FlightComputer_scaleGyroscope(flight_computer);

	// Magnetometer @ 0x1E reg 0x03 (6 bytes)
	if (HAL_I2C_Mem_Read(flight_computer->hi2c, 0x1E << 1, 0x03, 1, read_data, 6, 100) == HAL_OK) {
		flight_computer->imu.magnetometer.x = (int16_t)(read_data[0] << 8 | read_data[1]);
		flight_computer->imu.magnetometer.y = (int16_t)(read_data[4] << 8 | read_data[5]);
		flight_computer->imu.magnetometer.z = (int16_t)(read_data[2] << 8 | read_data[3]);
		// legacy telemetry ordering
		flight_computer->telemetry_frame[11] = read_data[0];
		flight_computer->telemetry_frame[12] = read_data[1];
		flight_computer->telemetry_frame[13] = read_data[4];
		flight_computer->telemetry_frame[14] = read_data[5];
		flight_computer->telemetry_frame[15] = read_data[2];
		flight_computer->telemetry_frame[16] = read_data[3];
	}

	FlightComputer_scaleMagnetometer(flight_computer);
	// BMP280 @ 0x1E reg 0xF7 (6 bytes)
	HAL_I2C_Mem_Read(flight_computer->hi2c, 0x76 << 1, 0xF7, 1, read_data, 6, 100);
	int32_t pressure_raw;
	pressure_raw = (int32_t)((read_data[0] << 12) | (read_data[1] << 4) | (read_data[2] >> 4));
	int32_t temperature_raw;
	temperature_raw = (int32_t)((read_data[3] << 12) | (read_data[4] << 4) | (read_data[5] >> 4));
	// konwersja ciśnienia potrzebuje temperatury
	int32_t t_fine = BaroThermo_calculate_t_fine(flight_computer, temperature_raw);
	flight_computer->barothermo.temperature = t_fine / 5120.0f;
	flight_computer->barothermo.pressure = BaroThermo_convertPressure(flight_computer, pressure_raw, t_fine);
	BaroThermo_calculateAltitude(flight_computer);

	FlightComputer_calculateVectorLengths(flight_computer);

}

void Sensors_bypass(FlightComputer* flight_computer){ //niedokonczone

//  WARTOSCI SUROWE
//	// Magnetometer
//	flight_computer->imu.magnetometer.x = (int16_t)(uart_rx_buffer[0] << 8 | uart_rx_buffer[1]);
//	flight_computer->imu.magnetometer.y = (int16_t)(uart_rx_buffer[2] << 8 | uart_rx_buffer[3]);
//	flight_computer->imu.magnetometer.z = (int16_t)(uart_rx_buffer[4] << 8 | uart_rx_buffer[5]);
//
//	// ADXL345 accelerometer
//	flight_computer->imu.accelerometer.x = (int16_t)(uart_rx_buffer[6] << 8 | uart_rx_buffer[7]);
//	flight_computer->imu.accelerometer.y = (int16_t)(uart_rx_buffer[8] << 8 | uart_rx_buffer[9]);
//	flight_computer->imu.accelerometer.z = (int16_t)(uart_rx_buffer[10] << 8 | uart_rx_buffer[11]);
//
//	// MPU/gyro
//	flight_computer->imu.gyroscope.x = (int16_t)(uart_rx_buffer[12] << 8 | uart_rx_buffer[13]);
//	flight_computer->imu.gyroscope.y = (int16_t)(uart_rx_buffer[14] << 8 | uart_rx_buffer[15]);
//	flight_computer->imu.gyroscope.z = (int16_t)(uart_rx_buffer[16] << 8 | uart_rx_buffer[17]);
//
//	for(int i=0; i<18; ++i){
//		flight_computer->telemetry_frame[i + 11] = uart_rx_buffer[i];
//	}

	if (__HAL_UART_GET_FLAG((flight_computer->huart), UART_FLAG_ORE) != RESET) {
		    __HAL_UART_CLEAR_OREFLAG((flight_computer->huart)); // Wyczyść błąd Overrun
		}


	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
	while(new_data_flag == 0){
		HAL_Delay(10);
	}
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

	// Magnetometr
	memcpy(&(flight_computer->imu.magnetometer.x_scaled), &uart_rx_buffer[0],  4);
	memcpy(&(flight_computer->imu.magnetometer.y_scaled), &uart_rx_buffer[4],  4);
	memcpy(&(flight_computer->imu.magnetometer.z_scaled), &uart_rx_buffer[8],  4);

	// Akcelerometr
	memcpy(&(flight_computer->imu.accelerometer.x_scaled), &uart_rx_buffer[12], 4);
	memcpy(&(flight_computer->imu.accelerometer.y_scaled), &uart_rx_buffer[16], 4);
	memcpy(&(flight_computer->imu.accelerometer.z_scaled), &uart_rx_buffer[20], 4);

	// Żyroskop
	memcpy(&(flight_computer->imu.gyroscope.x_scaled),     &uart_rx_buffer[24], 4);
	memcpy(&(flight_computer->imu.gyroscope.y_scaled),     &uart_rx_buffer[28], 4);
	memcpy(&(flight_computer->imu.gyroscope.z_scaled),     &uart_rx_buffer[32], 4);

	// Ciśnienie
	memcpy(&(flight_computer->barothermo.pressure),     &uart_rx_buffer[36], 4);

	// Obliczenie długości wektorów wypadkowych z otrzymanych wartości rzeczywistych
	FlightComputer_calculateVectorLengths(flight_computer);

	new_data_flag = 0;

	/*uint8_t buffer[40];

	HAL_UART_Receive(flight_computer->huart, buffer, 40, 2000);

	// Magnetometr
	memcpy(&(flight_computer->imu.magnetometer.x_scaled), &buffer[0],  4);
	memcpy(&(flight_computer->imu.magnetometer.y_scaled), &buffer[4],  4);
	memcpy(&(flight_computer->imu.magnetometer.z_scaled), &buffer[8],  4);

	// Akcelerometr
	memcpy(&(flight_computer->imu.accelerometer.x_scaled), &buffer[12], 4);
	memcpy(&(flight_computer->imu.accelerometer.y_scaled), &buffer[16], 4);
	memcpy(&(flight_computer->imu.accelerometer.z_scaled), &buffer[20], 4);

	// Żyroskop
	memcpy(&(flight_computer->imu.gyroscope.x_scaled),     &buffer[24], 4);
	memcpy(&(flight_computer->imu.gyroscope.y_scaled),     &buffer[28], 4);
	memcpy(&(flight_computer->imu.gyroscope.z_scaled),     &buffer[32], 4);

	// Ciśnienie
	memcpy(&(flight_computer->barothermo.pressure),     &buffer[36], 4);

	// Obliczenie długości wektorów wypadkowych z otrzymanych wartości rzeczywistych
	FlightComputer_calculateVectorLengths(flight_computer);

	new_data_flag = 0;*/
}

float BaroThermo_convertPressure(FlightComputer* flight_computer, int32_t pressure_raw, int32_t t_fine){
	float var1;
	float var2;

	float p;

	var1 = ((float)t_fine / 2.0f) - 64000.0f;
	var2 = var1 * var1 * ((float)flight_computer->barothermo.P6) / 32768.0f;
	var2 = var2 + var1 * ((float)flight_computer->barothermo.P5) * 2.0f;
	var2 = (var2 / 4.0f) + (((float)flight_computer->barothermo.P4) * 65536.0f);
	var1 = (((float)flight_computer->barothermo.P3) * var1 * var1 / 524288.0f +
	       ((float)flight_computer->barothermo.P2) * var1) / 524288.0f;
	var1 = (1.0f + var1 / 32768.0f) * ((float)flight_computer->barothermo.P1);

	if(var1 != 0.0f){
	    p = 1048576.0f - (float)pressure_raw;
	    p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
	    var1 = ((float)flight_computer->barothermo.P9) * p * p / 2147483648.0f;
	    var2 = p * ((float)flight_computer->barothermo.P8) / 32768.0f;
	    p = p + (var1 + var2 + ((float)flight_computer->barothermo.P7)) / 16.0f;
	}else{
	    p = 0;
	}

	return p;
}

int32_t BaroThermo_calculate_t_fine(FlightComputer* flight_computer, int32_t temperature_raw){
	int32_t t_fine;

	float var1;
	float var2;

	var1 = (((float)temperature_raw) / 16384.0f -
	        ((float)flight_computer->barothermo.T1) / 1024.0f) *
	        ((float)flight_computer->barothermo.T2);

	var2 = ((((float)temperature_raw) / 131072.0f -
	         ((float)flight_computer->barothermo.T1) / 8192.0f) *
	        (((float)temperature_raw) / 131072.0f -
	         ((float)flight_computer->barothermo.T1) / 8192.0f)) *
	        ((float)flight_computer->barothermo.T3);

	t_fine = (int32_t)(var1 + var2);

	return t_fine;
}

void BaroThermo_calculateAltitude(FlightComputer* flight_computer) {
	float p = flight_computer->barothermo.pressure;

	if (p > 0.0f) {
		flight_computer->barothermo.altitude = 44330.77f * (1.0f - powf((p / flight_computer->barothermo.pressure_reference), 0.190263f)); // Uproszczony wzór barometryczny
	} else {
		flight_computer->barothermo.altitude = 0.0f;
	}
}

void LoRa_send_telemetry(FlightComputer* flight_computer){

}

void FlightComputer_init(FlightComputer* flight_computer, SPI_HandleTypeDef* lora_hspi,
		GPIO_TypeDef *lora_port, uint16_t lora_pin, I2C_HandleTypeDef* hi2c, ADC_HandleTypeDef* hadc, UART_HandleTypeDef* huart){


	// LORA
	flight_computer->LoRa = newLoRa();

	flight_computer->LoRa.hSPIx                 = lora_hspi;
	flight_computer->LoRa.CS_port               = lora_port;
	flight_computer->LoRa.CS_pin                = lora_pin;

	flight_computer->LoRa.frequency             = 433;							  // default = 433 MHz
	flight_computer->LoRa.spredingFactor        = SF_7;							// default = SF_7
	flight_computer->LoRa.bandWidth			       = BW_500KHz;				  // default = BW_125KHz
	flight_computer->LoRa.crcRate				       = CR_4_5;						// default = CR_4_5
	flight_computer->LoRa.power					       = POWER_20db;				// default = 20db
	flight_computer->LoRa.overCurrentProtection = 120; 							// default = 100 mA
	flight_computer->LoRa.preamble				 = 10;		  					// default = 8;

	LoRa_reset(&(flight_computer->LoRa));
	LoRa_init(&(flight_computer->LoRa));
	LoRa_startReceiving(&(flight_computer->LoRa));

	// TELEMETRIA
	flight_computer->telemetry_frame[0] = 0x24; // $
	flight_computer->telemetry_frame[47] = 0x0A; // \n
	flight_computer->telemetry_frame[48] = 0x0D; // \r
	flight_computer->telemetry_frame[49] = 0x00; // \0

	//Inicjalizacja BMP280
	uint8_t settings = 0x08;
	HAL_I2C_Mem_Write(hi2c, 0x76 << 1, 0xE0, 1, &settings, 1, 100);
	HAL_Delay(100);
	uint8_t calibration[24];
	HAL_I2C_Mem_Read(hi2c, 0x76 << 1, 0x88, 1, calibration, 24, 100);
	flight_computer->barothermo.T1 = (uint16_t)((calibration[1] << 8) | calibration[0]);
	flight_computer->barothermo.T2 = (int16_t)((calibration[3] << 8) | calibration[2]);
	flight_computer->barothermo.T3 = (int16_t)((calibration[5] << 8) | calibration[4]);
	flight_computer->barothermo.P1 = (uint16_t)((calibration[7] << 8) | calibration[6]);
	flight_computer->barothermo.P2 = (int16_t)((calibration[9] << 8) | calibration[8]);
	flight_computer->barothermo.P3 = (int16_t)((calibration[11] << 8) | calibration[10]);
	flight_computer->barothermo.P4 = (int16_t)((calibration[13] << 8) | calibration[12]);
	flight_computer->barothermo.P5 = (int16_t)((calibration[15] << 8) | calibration[14]);
	flight_computer->barothermo.P6 = (int16_t)((calibration[17] << 8) | calibration[16]);
	flight_computer->barothermo.P7 = (int16_t)((calibration[19] << 8) | calibration[18]);
	flight_computer->barothermo.P8 = (int16_t)((calibration[21] << 8) | calibration[20]);
	flight_computer->barothermo.P9 = (int16_t)((calibration[23] << 8) | calibration[22]);
	settings = 0x2B;
	HAL_I2C_Mem_Write(hi2c, 0x76 << 1, 0xF4, 1, &settings, 1, 100);
	settings = 0x00;
	HAL_I2C_Mem_Write(hi2c, 0x76 << 1, 0xF5, 1, &settings, 1, 100);


	// Inicjalizacja IMU
	settings = 0x08;
	HAL_I2C_Mem_Write(hi2c, 0x53 << 1, 0x2D, 1, &settings, 1, 100);
	settings = 0x00;
	HAL_I2C_Mem_Write(hi2c, 0x68 << 1, 0x3E, 1, &settings, 1, 100);
	settings = 0x03;
	HAL_I2C_Mem_Write(hi2c, 0x68 << 1, 0x16, 1, &settings, 1, 100);
	settings = 0x00;
	HAL_I2C_Mem_Write(hi2c, 0x1E << 1,0x02, 1, &settings, 1, 100);
	settings = 0x00;
	HAL_I2C_Mem_Write(hi2c, 0x1E << 1,0x01, 1, &settings, 1, 100);

    // store hi2c and huart handles for later sensor reads
    flight_computer->hi2c = hi2c;
    flight_computer->huart = huart;

    //state init
    flight_computer->armed = 0;
    flight_computer->camera = 0;

    // parachute logic init
    flight_computer->parachute_fired = 0;
    flight_computer->parachuteCnt = 0;

	// Zmiana zakresow IMU
	uint8_t config_to_write = 0x0B; // Zakres +/- 16g
	HAL_I2C_Mem_Write(flight_computer->hi2c, 0x53 << 1, 0x31, 1, &config_to_write, 1, 100);
	uint8_t gyro_config = 0x18; // Zakres +/- 2000 deg/s
	HAL_I2C_Mem_Write(flight_computer->hi2c, 0x68 << 1, 0x1B, 1, &gyro_config, 1, 100);
	uint8_t mag_config_b = 0x20; // Domyślny zakres +/- 1.3 Gauss
	HAL_I2C_Mem_Write(flight_computer->hi2c, 0x1E << 1, 0x01, 1, &mag_config_b, 1, 100);
	uint8_t mag_mode = 0x00; // Ustawienie trybu ciągłego pomiaru (rejestr Mode 0x02 -> wartość 0x00)
	HAL_I2C_Mem_Write(flight_computer->hi2c, 0x1E << 1, 0x02, 1, &mag_mode, 1, 100);


    //ADC
    flight_computer->hadc = hadc;
    HAL_ADC_Start(flight_computer->hadc);

	// Initialize servos and PID controllers (conservative defaults)
//	Servos_Init();
	PID_Init(&pidX, &angleX);
	PID_Init(&pidZ, &angleZ);

	// start in WAITING state
	flight_computer->state = STATE_WAITING;
	flight_computer->state_change_timestamp = HAL_GetTick();
	flight_computer->barothermo.prev_pressure = 0;
	flight_computer->barothermo.apogee_pressure_window_index = 0;
	flight_computer->barothermo.apogee_pressure_window_count = 0;
	flight_computer->barothermo.apogee_pressure_window_sum = 0;
	for (int i = 0; i < APOGEE_PRESSURE_WINDOW_SIZE; i = i+1) {
		flight_computer->barothermo.apogee_pressure_window[i] = 0;
	}

	// calculate acceleration bias
	calculatePressureReference(flight_computer);
}

void StateMachine_idle(FlightComputer* flight_computer){
	// handlers should not change the state; they perform per-state actions
	(void)flight_computer;

}

void StateMachine_launch(FlightComputer* flight_computer){
	// TVC routine: compute PID corrections and write servo angles
	const double GEAR_RATIO = 4.0;
	const double SERVO_X_HOME = 81.0;
	const double SERVO_Z_HOME = 84.0;

	double cx = PID_Compute(&pidX);
	double cz = PID_Compute(&pidZ);
//	Servos_SetAngleX(SERVO_X_HOME - (cx * GEAR_RATIO));
//	Servos_SetAngleZ(SERVO_Z_HOME + (cz * GEAR_RATIO));
}

void StateMachine_ascent(FlightComputer* flight_computer){
	// Unpowered coast: disable servos to save power
//	Servos_Detach();
	(void)flight_computer;
}

/*void StateMachine_apogee(FlightComputer* flight_computer){
	// Placeholder - treat as immediate transition to descent
	flight_computer->state = STATE_DESCENT;
}*/

void StateMachine_descent(FlightComputer* flight_computer){
	// per-state actions (none for now)
	(void)flight_computer;
}

void StateMachine_landing(FlightComputer* flight_computer){
	// Finalize: stop recording/telemetry if implemented and power down
//	Servos_Detach();
	// Optionally power down the system
	(void)flight_computer; // leave power decision to main
}

void FlightComputer_setState(FlightComputer* flight_computer, int8_t new_state) {
	flight_computer->state = new_state;
	flight_computer->state_change_timestamp = HAL_GetTick();
}

void FlightComputer_handleState(FlightComputer* flight_computer, int8_t state) {
	switch (state) {
		case STATE_WAITING: StateMachine_idle(flight_computer); break;
		case STATE_POWERED_ASCENT: StateMachine_launch(flight_computer); break;
		case STATE_UNPOWERED_ASCENT: StateMachine_ascent(flight_computer); break;
		case STATE_DESCENT: StateMachine_descent(flight_computer); break;
		case STATE_LANDED: StateMachine_landing(flight_computer); break;
		case STATE_ABORT: /* no-op */ break;
		default: break;
	}
}

int8_t FlightComputer_evaluateTransitions(FlightComputer* flight_computer) {
	int8_t current = flight_computer->state;
	uint32_t now = HAL_GetTick();
	// tunable
	const uint32_t MOTOR_BURN_TIME_MS = 5000;
	const uint32_t MAX_DESCENT_MS = 300000;

	switch (current) {
		case STATE_WAITING:
			if (flight_computer->last_cmd_rx == 8) return STATE_POWERED_ASCENT;
			if (flight_computer->imu.accelerometer.acc_total > 4*GRAVITY_EARTH) return STATE_POWERED_ASCENT;
			break;
		case STATE_POWERED_ASCENT:
			//if (now - flight_computer->state_change_timestamp > MOTOR_BURN_TIME_MS) return STATE_UNPOWERED_ASCENT;
			if (flight_computer->imu.accelerometer.acc_total < 2*GRAVITY_EARTH) return STATE_UNPOWERED_ASCENT;
			break;
		case STATE_UNPOWERED_ASCENT: {
			// pressure stored in barothermo.pressure
			float pressure = flight_computer->barothermo.pressure;
			float pressure_average = FlightComputer_updateApogeePressureAverage(flight_computer, pressure);
			if (flight_computer->barothermo.apogee_pressure_window_index == 0 && flight_computer->barothermo.prev_pressure != 0 && pressure_average > flight_computer->barothermo.prev_pressure) {
				//flight_computer->parachuteCnt = 20;
				return STATE_DESCENT;
			}
			if(flight_computer->barothermo.apogee_pressure_window_index == 0){
				flight_computer->barothermo.prev_pressure = pressure_average;
			}
			break;
		}
		case STATE_DESCENT:
			if (now - flight_computer->state_change_timestamp > MAX_DESCENT_MS) return STATE_LANDED;
			break;
		case STATE_ABORT:
			// remain until manual reset
			break;
		default:
			break;
	}
	return current;
}

uint8_t* FlightComputer_getTelemetry(FlightComputer* flight_computer) {
	return &(flight_computer->telemetry_frame[0]);
}

void FlightComputer_handleCommand(FlightComputer* flight_computer){
	uint8_t received_data[1];
	uint8_t bytesRecv = LoRa_receive(&(flight_computer->LoRa), received_data, 1);
	if (bytesRecv > 0) {
		flight_computer->telemetry_frame[6] = received_data[0];
		flight_computer->last_cmd_rx = received_data[0];
		switch (received_data[0]) {
			case 0:
				break;
			case 1:
				flight_computer->armed = 1;
				break;
			case 2:
				flight_computer->armed = 0;
				break;
			case 3:
				break;
			case 4:
				break;
			case 5:
				flight_computer->camera = 1;
				break;
			case 6:
				flight_computer->camera = 0;
				break;
			case 7: // odpalenie spadochronu
				if(flight_computer->parachute_fired == 0){
					flight_computer->parachute_fired = 1;
					flight_computer->parachuteCnt = 30;
				}
				break;
			case 8:
				break;
			case 9:
				break;
			default:
				break;
		}
	}
}

void FlightComputer_loop(FlightComputer* flight_computer){

	uint32_t time_buff = HAL_GetTick();

	// Handle LoRa receive (commands)
	FlightComputer_handleCommand(flight_computer);

	// Transmit telemetry and restart RX
	int mode = LoRa_transmit_send(&(flight_computer->LoRa), &(flight_computer->telemetry_frame[0]), 62, 500);

	flight_computer->telemetry_frame[1] = (uint8_t)(time_buff >> 24);
	flight_computer->telemetry_frame[2] = (uint8_t)(time_buff >> 16);
	flight_computer->telemetry_frame[3] = (uint8_t)(time_buff >> 8);
	flight_computer->telemetry_frame[4] = (uint8_t)(time_buff);

	// debug - frames counting
	//flight_computer->telemetry_frame[45]++;

	// Read sensors and update telemetry bytes
	if(MODE == 0){
		Sensors_read(flight_computer);
	}else{
		Sensors_bypass(flight_computer);
	}

	// odczyt napiecia
	uint16_t adcValue;
	adcValue = (uint16_t)HAL_ADC_GetValue(flight_computer->hadc);
	HAL_ADC_Start(flight_computer->hadc);
	//dodać funkcję przeliczającą
	flight_computer->telemetry_frame[44] = (uint8_t)(adcValue >> 8);
	flight_computer->telemetry_frame[45] = (uint8_t)adcValue;

	// Parachute output handling (legacy behaviour)
	if (flight_computer->parachuteCnt > 0) {
		flight_computer->parachuteCnt -= 1;
		HAL_GPIO_WritePin(led_parachute_GPIO_Port, led_parachute_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
		//flight_computer->telemetry_frame[53] = 1;
	} else {
		HAL_GPIO_WritePin(led_parachute_GPIO_Port, led_parachute_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
		//flight_computer->telemetry_frame[53] = 0;
	}

	// State machine step
	FlightComputer_handleState(flight_computer, flight_computer->state);

	// battery placeholder
//	int16_t batt = (int16_t)(read_battery_voltage_adc() * 100.0f);
//	if (batt < 0) batt = 0;
//	flight_computer->telemetry_frame[54] = (uint8_t)((batt >> 8) & 0xFF);
//	flight_computer->telemetry_frame[55] = (uint8_t)(batt & 0xFF);

	// RSSI
	flight_computer->telemetry_frame[46] = (uint8_t)LoRa_getRSSI(&(flight_computer->LoRa));

	//state actualization
	flight_computer->state = FlightComputer_evaluateTransitions(flight_computer);
	flight_computer->telemetry_frame[5] = flight_computer->state;

	// check if the telemetry is send
	LoRa_transmit_check(&(flight_computer->LoRa), 500, mode);
	LoRa_startReceiving(&(flight_computer->LoRa));
	//HAL_UART_Transmit(flight_computer->huart, &(flight_computer->telemetry_frame[6]), 1, 100);

	// a window for uplink communication - entire loop should last 50 ms
	uint32_t time_diff = FRAME_TIME - 1 - (HAL_GetTick() - time_buff);
	if(time_diff < 1){
		time_diff = 1;
	}

	if(MODE == 0){
		HAL_Delay(time_diff);
	}else{
		HAL_Delay(10);
	}
}
