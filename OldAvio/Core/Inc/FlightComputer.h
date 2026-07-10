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


extern uint8_t uart_rx_buffer[40];
extern volatile uint8_t new_data_flag;

// Magnetometr, część IMU
typedef struct{

	int16_t x;
	int16_t y;
	int16_t z;

	float x_scaled;
	float y_scaled;
	float z_scaled;

	float mag_total; // Całkowite pole magnetyczne w uT

} Magnetometer;

// Akcelerometr, część IMU
typedef struct{

	int16_t x;
	int16_t y;
	int16_t z;

	float x_scaled;
	float y_scaled;
	float z_scaled;

	float acc_total; // Całkowite przyspieszenie w m/s^2

} Accelerometer;

// Żyroskop, część IMU
typedef struct{

	int16_t x;
	int16_t y;
	int16_t z;

	float x_scaled;
	float y_scaled;
	float z_scaled;

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

} IMU;

// BMP280 (ciśnienie + temperatura + wilgotność)
typedef struct{

	float pressure;
	float pressure_reference;
	float temperature;
	float altitude; // wysokosc w metrach

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

	float prev_pressure;
	float apogee_pressure_window[10];
	uint8_t apogee_pressure_window_index;
	uint8_t apogee_pressure_window_count;
	float apogee_pressure_window_sum;

} BaroThermo;

// Struktura danych przechowująca aktualny stan awioniki, dane czujników itp.
typedef struct{

	//Status
	uint32_t start_time;
	int8_t state;
	int8_t last_cmd_rx;
	uint32_t state_change_timestamp;
	int8_t armed;
	int8_t camera;

	// liczniki potwierdzające (debounce) przejść w maszynie stanów
	uint8_t launch_detect_counter;
	uint8_t burnout_detect_counter;

	float prev_altitude_for_velocity;
	uint32_t prev_altitude_timestamp;
	float vertical_velocity;        // m/s, dodatnia = wznoszenie, ujemna = opadanie
	uint8_t landed_detect_counter;

	uint8_t freefall_detect_counter;
	uint8_t apogee_pressure_rise_counter;

	// Moduły
	LoRa LoRa;
	IMU imu;
	GPS gps;
	BaroThermo barothermo;
	// HAL handles
	I2C_HandleTypeDef* hi2c;
	ADC_HandleTypeDef* hadc;
	UART_HandleTypeDef* huart;
	UART_HandleTypeDef* huart_gps;

	// parachute counter (loops remaining to fire output)
	int parachuteCnt;
	// flag to avoid second launch
	uint8_t parachute_fired;
	//breakaway wire flag
	uint8_t breakaway_wire_detached;
	uint8_t breakaway_detect_counter;
	// czas (HAL_GetTick) zerwania breakwire - do odliczania 9s do startu
	uint32_t breakaway_wire_detach_time;

	// Telemetria
	int32_t time;
	int8_t gpio_state;
	int16_t battery_voltage;
	int8_t RSSI;

	uint8_t telemetry_frame[50];
	/*
		0 - preambuła ($) [uint8]
		1-4 - timestamp [uint32]
		5 - stan [uint8]
		6 - ostatnia komenda [uint8]
		7-8 - wysokosc [int16, decymetry]
		9-10 - temperatura [int16, 1/100 st.C]
		11-16 - magnetometr (x,y,z) [3x int16]
		17-22 - akcelerometr (x,y,z) [3x int16]
		23-28 - żyroskop (x,y,z) [3x int16]
		29-42 - GPS (fix quality, liczba satelitow, szerokosc, długosc, wysokosc) [2x uint8, 3x float]
		43 - GPIO state (pyro1, pyro2, led_r, led_g, led_b, led_y, buzzer, camera) [uint8]
		44-45 - napięcie na baterii [uint16, 1/100 V]
		46 - RSSI [uint8]
		47-49 - zakończenie (\n\r\0) [3x uint8]
	*/

} FlightComputer;

// Sensor access
void FlightComputer_scaleAccelerometer(FlightComputer* flight_computer);
void Sensors_read(FlightComputer* flight_computer);
void Sensors_bypass(FlightComputer* flight_computer);

// Sensor conversions
float BaroThermo_convertPressure(FlightComputer* flight_computer, int32_t pressure_raw, int32_t t_fine);
int32_t BaroThermo_calculate_t_fine(FlightComputer* flight_computer, int32_t temperature_raw);
void BaroThermo_calculateAltitude(FlightComputer* flight_computer);

void FlightComputer_calculateVectorLengths(FlightComputer* flight_computer);

// Higher-level APIs used by application (main)
void FlightComputer_setState(FlightComputer* flight_computer, int8_t new_state);
void FlightComputer_handleState(FlightComputer* flight_computer, int8_t state);
int8_t FlightComputer_evaluateTransitions(FlightComputer* flight_computer);
uint8_t* FlightComputer_getTelemetry(FlightComputer* flight_computer);

void FlightComputer_init(FlightComputer* flight_computer, SPI_HandleTypeDef* lora_hspi,
		GPIO_TypeDef *lora_port, uint16_t lora_pin, I2C_HandleTypeDef* hi2c, ADC_HandleTypeDef* hadc, UART_HandleTypeDef* huart, UART_HandleTypeDef* huart_gps);
void FlightComputer_loop(FlightComputer* flight_computer);

void StateMachine_idle(FlightComputer* flight_computer);
void StateMachine_launch(FlightComputer* flight_computer);
void StateMachine_ascent(FlightComputer* flight_computer);
void StateMachine_apogee(FlightComputer* flight_computer);
void StateMachine_descent(FlightComputer* flight_computer);
void StateMachine_landing(FlightComputer* flight_computer);

#endif /* INC_FLIGHTCOMPUTER_H_ */
