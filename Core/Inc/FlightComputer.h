/*
 * FlightComputer.h
 *
 *  Created on: May 11, 2026
 *      Authors: Julia Brąglewicz
 *
 */

#ifndef INC_FLIGHTCOMPUTER_H_
#define INC_FLIGHTCOMPUTER_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "LoRa.h"
#include "bmp280.h"

extern uint8_t uart_rx_buffer[18];
extern volatile uint8_t new_data_flag;

// Magnetometr, część IMU
typedef struct{

	int16_t x;
	int16_t y;
	int16_t z;

} Magnetometer;

// Akcelerometr, część IMU
typedef struct{

	int16_t x;
	int16_t y;
	int16_t z;

} Accelerometer;

// Żyroskop, część IMU
typedef struct{

	int16_t x;
	int16_t y;
	int16_t z;

} Gyroscope;

// GPS
typedef struct{

	float latitude;
	float longitude;
	float altitude;

} GPS;

// IMU
typedef struct{

	Magnetometer magnetometer;
	Accelerometer accelerometer;
	Gyroscope gyroscope;
	int16_t pressure;
	int16_t temperature;

} IMU;

// BMP280 (ciśnienie + temperatura + wilgotność)
typedef struct{

	float pressure;
	float temperature;

	uint16_t T1;
	int16_t  T2;
	int16_t  T3;

	uint16_t P1;
	int16_t  P2;
	int16_t  P3;
	int16_t  P4;
	int16_t  P5;
	int16_t  P6;
	int16_t  P7;
	int16_t  P8;
	int16_t  P9;

} BaroThermo;

// Struktura danych przechowująca aktualny stan awioniki, dane czujników itp.
typedef struct{

	//Status
	int32_t start_time;
	int8_t state;
	int8_t last_cmd_rx;
	uint32_t state_change_timestamp;
	int32_t prev_pressure;
	int32_t apogee_pressure_window[10];
	uint8_t apogee_pressure_window_index;
	uint8_t apogee_pressure_window_count;
	int32_t apogee_pressure_window_sum;

	// Moduły
	LoRa LoRa;
	IMU imu;
	GPS gps;
	BaroThermo barothermo;
	// HAL handles
	I2C_HandleTypeDef* hi2c;
	ADC_HandleTypeDef* hadc;
	UART_HandleTypeDef* huart;

	// parachute counter (loops remaining to fire output)
	int parachuteCnt;

	// Telemetria
	int32_t time;
	int8_t gpio_state;
	int16_t battery_voltage;
	int8_t RSSI;

	uint8_t telemetry_frame[62];
	/*
		0 - preambuła
		1-4 - timestamp
		5 - stan
		6 - ostatnia komenda
		7-8 - wysokosc [decymetry]
		9-10 - temperatura
		11-16 - magnetometr x,y,z
		17-22 - akcelerometr x,y,z
		23-28 - żyroskop x,y,z
		29-30 - ciśnienie imu (TO DELETE)
		31-32 - temperatura imu (TO DELETE)
		33-52 - GPS (TBD)
		53 - GPIO state
		54-55 - napięcie na baterii
		56 - RSSI
		57-58 - CRC (upewnić się czy potrzebne)
		59-61 - zakończenie (\n\r\0)
	*/

} FlightComputer;

// Sensor access
void Sensors_read(FlightComputer* flight_computer);
void Sensors_bypass(FlightComputer* flight_computer);

// Sensor conversions
void BaroThermo_convertPressure(FlightComputer* flight_computer, int32_t pressure_raw, int32_t t_fine);
int32_t BaroThermo_convertTemperature(FlightComputer* flight_computer, int32_t temperature_raw);

// Higher-level APIs used by application (main)
void FlightComputer_setState(FlightComputer* flight_computer, int8_t new_state);
void FlightComputer_handleState(FlightComputer* flight_computer, int8_t state);
int8_t FlightComputer_evaluateTransitions(FlightComputer* flight_computer);
uint8_t* FlightComputer_getTelemetry(FlightComputer* flight_computer);

void FlightComputer_init(FlightComputer* flight_computer, SPI_HandleTypeDef* lora_hspi,
		GPIO_TypeDef *lora_port, uint16_t lora_pin, I2C_HandleTypeDef* hi2c, ADC_HandleTypeDef* hadc, UART_HandleTypeDef* huart);
void FlightComputer_loop(FlightComputer* flight_computer);

void StateMachine_idle(FlightComputer* flight_computer);
void StateMachine_launch(FlightComputer* flight_computer);
void StateMachine_ascent(FlightComputer* flight_computer);
void StateMachine_apogee(FlightComputer* flight_computer);
void StateMachine_descent(FlightComputer* flight_computer);
void StateMachine_landing(FlightComputer* flight_computer);

#endif /* INC_FLIGHTCOMPUTER_H_ */
