/*
 * FlightComputer.h
 *
 *  Created on: May 11, 2026
 *      Authors: Julia Brąglewicz
 *      		 Franciszek Ślusarczyk
 */

#ifndef INC_FLIGHTCOMPUTER_H_
#define INC_FLIGHTCOMPUTER_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "LoRa.h"
#include "bmp280.h"

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

	BMP280_HandleTypedef handle;
	float pressure;
	float temperature;

} BaroThermo;

// Struktura danych przechowująca aktualny stan awioniki, dane czujników itp.
typedef struct{

	//Status
	int32_t start_time;
	int8_t state;
	int8_t last_cmd_rx;

	// Moduły
	LoRa LoRa;
	IMU imu;
	GPS gps;
	BaroThermo barothermo;

	// Telemetria
	uint8_t preamble;
	int32_t time;
	int8_t gpio_state
	int16_t battery_voltage;
	int8_t RSSI;

} FlightComputer;

void Sensors_read(FlightComputer* flight_computer);
void Sensors_bypass(FlightComputer* flight_computer);

void FlightComputer_init(FlightComputer* flight_computer);
void FlightComputer_loop(FlightComputer* flight_computer);

void StateMachine_idle(FlightComputer* flight_computer);
void StateMachine_launch(FlightComputer* flight_computer);
void StateMachine_ascent(FlightComputer* flight_computer);
void StateMachine_apogee(FlightComputer* flight_computer);
void StateMachine_descent(FlightComputer* flight_computer);
void StateMachine_landing(FlightComputer* flight_computer);

#endif /* INC_FLIGHTCOMPUTER_H_ */
