/*
 * FlightComputer.c
 *
 *  Created on: May 11, 2026
 *      Authors: Julia Brąglewicz
 *
 */

#include "FlightComputer.h"
#include "pid.h"
#include "servos.h"
#include "power.h"

#define bmp280 flight_computer->barothermo.handle

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

#define APOGEE_PRESSURE_WINDOW_SIZE 10

static int32_t FlightComputer_updateApogeePressureAverage(FlightComputer* flight_computer, int32_t pressure) {
	uint8_t index = flight_computer->apogee_pressure_window_index;

	if (flight_computer->apogee_pressure_window_count < APOGEE_PRESSURE_WINDOW_SIZE) {
		flight_computer->apogee_pressure_window_count++;
	} else {
		flight_computer->apogee_pressure_window_sum -= flight_computer->apogee_pressure_window[index];
	}

	flight_computer->apogee_pressure_window[index] = pressure;
	flight_computer->apogee_pressure_window_sum += pressure;
	flight_computer->apogee_pressure_window_index = (uint8_t)((index + 1) % APOGEE_PRESSURE_WINDOW_SIZE);

	return flight_computer->apogee_pressure_window_sum / flight_computer->apogee_pressure_window_count;
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

	// Magnetometer @ 0x1E reg 0x03 (6 bytes)
	if (HAL_I2C_Mem_Read(flight_computer->hi2c, 0x1E << 1, 0x03, 1, read_data, 6, 100) == HAL_OK) {
		flight_computer->imu.magnetometer.x = (int16_t)(read_data[0] << 8 | read_data[1]);
		flight_computer->imu.magnetometer.y = (int16_t)(read_data[2] << 8 | read_data[3]);
		flight_computer->imu.magnetometer.z = (int16_t)(read_data[4] << 8 | read_data[5]);
		// legacy telemetry ordering
		flight_computer->telemetry_frame[11] = read_data[0];
		flight_computer->telemetry_frame[12] = read_data[1];
		flight_computer->telemetry_frame[13] = read_data[4];
		flight_computer->telemetry_frame[14] = read_data[5];
		flight_computer->telemetry_frame[15] = read_data[2];
		flight_computer->telemetry_frame[16] = read_data[3];
	}
}

void Sensors_bypass(FlightComputer* flight_computer){

}

void LoRa_send_telemetry(FlightComputer* flight_computer){

}

void FlightComputer_init(FlightComputer* flight_computer, SPI_HandleTypeDef* lora_hspi,
		GPIO_TypeDef *lora_port, uint16_t lora_pin, I2C_HandleTypeDef* hi2c){

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
	flight_computer->telemetry_frame[59] = 0x0A; // \n
	flight_computer->telemetry_frame[60] = 0x0D; // \r
	flight_computer->telemetry_frame[61] = 0x00; // \0

	/*flight_computer->telemetry_frame[1] = 0x31; // $
	flight_computer->telemetry_frame[2] = 0x31; // \n
	flight_computer->telemetry_frame[3] = 0x31; // \r
	flight_computer->telemetry_frame[4] = 0x31; // \0*/

	/*bmp280_init_default_params(&(bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;*/

	// Inicjalizacja IMU

	/*uint8_t v;
	v = 0x70;
	HAL_I2C_Mem_Write(hi2c, 0x1E<<1, 0x00, 1, &v, 1, 100);
	v = 0x20;
	HAL_I2C_Mem_Write(hi2c, 0x1E<<1, 0x01, 1, &v, 1, 100);
	v = 0x00;
	HAL_I2C_Mem_Write(hi2c, 0x1E<<1, 0x02, 1, &v, 1, 100);*/

	uint8_t settings = 0x08;
	HAL_I2C_Mem_Write(hi2c, 0x53 << 1, 0x2D, 1, &settings, 1, 100);
	settings = 0x00;
	HAL_I2C_Mem_Write(hi2c, 0x68 << 1, 0x3E, 1, &settings, 1, 100);
	settings = 0x03;
	HAL_I2C_Mem_Write(hi2c, 0x68 << 1, 0x16, 1, &settings, 1, 100);
	settings = 0x00;
	HAL_I2C_Mem_Write(hi2c, 0x1E << 1,0x02, 1, &settings, 1, 100);
	settings = 0x00;
	HAL_I2C_Mem_Write(hi2c, 0x1E << 1,0x01, 1, &settings, 1, 100);

    // store hi2c handle for later sensor reads
    flight_computer->hi2c = hi2c;
    flight_computer->parachuteCnt = 0;

	// Initialize servos and PID controllers (conservative defaults)
	Servos_Init();
	PID_Init(&pidX, &angleX);
	PID_Init(&pidZ, &angleZ);

	// start in WAITING state
	flight_computer->state = STATE_WAITING;
	flight_computer->state_change_timestamp = HAL_GetTick();
	flight_computer->prev_pressure = 0;
	flight_computer->apogee_pressure_window_index = 0;
	flight_computer->apogee_pressure_window_count = 0;
	flight_computer->apogee_pressure_window_sum = 0;
	for (int i = 0; i < APOGEE_PRESSURE_WINDOW_SIZE; ++i) {
		flight_computer->apogee_pressure_window[i] = 0;
	}
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
	Servos_SetAngleX(SERVO_X_HOME - (cx * GEAR_RATIO));
	Servos_SetAngleZ(SERVO_Z_HOME + (cz * GEAR_RATIO));
}

void StateMachine_ascent(FlightComputer* flight_computer){
	// Unpowered coast: disable servos to save power
	Servos_Detach();
	(void)flight_computer;
}

void StateMachine_apogee(FlightComputer* flight_computer){
	// Placeholder - treat as immediate transition to descent
	flight_computer->state = STATE_DESCENT;
}

void StateMachine_descent(FlightComputer* flight_computer){
	// per-state actions (none for now)
	(void)flight_computer;
}

void StateMachine_landing(FlightComputer* flight_computer){
	// Finalize: stop recording/telemetry if implemented and power down
	Servos_Detach();
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
			break;
		case STATE_POWERED_ASCENT:
			if (now - flight_computer->state_change_timestamp > MOTOR_BURN_TIME_MS) return STATE_UNPOWERED_ASCENT;
			break;
		case STATE_UNPOWERED_ASCENT: {
			// pressure stored in telemetry_frame[7..8]
			int32_t pressure = ((int32_t)(uint8_t)flight_computer->telemetry_frame[7] << 8) |
							   (int32_t)(uint8_t)flight_computer->telemetry_frame[8];
			int32_t pressure_average = FlightComputer_updateApogeePressureAverage(flight_computer, pressure);
			if (flight_computer->prev_pressure != 0 && pressure_average > flight_computer->prev_pressure) {
				return STATE_DESCENT;
			}
			flight_computer->prev_pressure = pressure_average;
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

void FlightComputer_loop(FlightComputer* flight_computer){
	uint32_t time_buff = HAL_GetTick();
	flight_computer->telemetry_frame[1] = (uint8_t)(time_buff >> 24);
	flight_computer->telemetry_frame[2] = (uint8_t)(time_buff >> 16);
	flight_computer->telemetry_frame[3] = (uint8_t)(time_buff >> 8);
	flight_computer->telemetry_frame[4] = (uint8_t)(time_buff);

	// Read sensors and update telemetry bytes
	Sensors_read(flight_computer);

	// Handle LoRa receive (commands)
	uint8_t received_data[1];
	uint8_t bytesRecv = LoRa_receive(&(flight_computer->LoRa), received_data, 1);
	if (bytesRecv > 0) {
		flight_computer->telemetry_frame[6] = received_data[0];
		flight_computer->last_cmd_rx = received_data[0];
		if (received_data[0] == 8) {
			flight_computer->parachuteCnt = 20;
		}
	}

	// Parachute output handling (legacy behaviour)
	if (flight_computer->parachuteCnt > 0) {
		flight_computer->parachuteCnt -= 1;
		HAL_GPIO_WritePin(led_parachute_GPIO_Port, led_parachute_Pin, GPIO_PIN_SET);
		flight_computer->telemetry_frame[53] = 1;
	} else {
		HAL_GPIO_WritePin(led_parachute_GPIO_Port, led_parachute_Pin, GPIO_PIN_RESET);
		flight_computer->telemetry_frame[53] = 0;
	}

	// State machine step
	switch (flight_computer->state) {
		case STATE_WAITING:
			StateMachine_idle(flight_computer);
			break;
		case STATE_POWERED_ASCENT:
			StateMachine_launch(flight_computer);
			break;
		case STATE_UNPOWERED_ASCENT:
			StateMachine_ascent(flight_computer);
			break;
		case STATE_DESCENT:
			StateMachine_descent(flight_computer);
			break;
		case STATE_LANDED:
			StateMachine_landing(flight_computer);
			break;
		case STATE_ABORT:
			// keep in abort or implement abort routine
			break;
		default:
			break;
	}

	// update telemetry state and last command
	flight_computer->telemetry_frame[5] = (uint8_t)flight_computer->state;
	flight_computer->telemetry_frame[6] = (uint8_t)flight_computer->last_cmd_rx;

	// battery placeholder
	int16_t batt = (int16_t)(read_battery_voltage_adc() * 100.0f);
	if (batt < 0) batt = 0;
	flight_computer->telemetry_frame[54] = (uint8_t)((batt >> 8) & 0xFF);
	flight_computer->telemetry_frame[55] = (uint8_t)(batt & 0xFF);

	// RSSI
	flight_computer->telemetry_frame[56] = (uint8_t)LoRa_getRSSI(&(flight_computer->LoRa));

	// Transmit telemetry and restart RX
	LoRa_transmit(&(flight_computer->LoRa), &(flight_computer->telemetry_frame[0]), 62, 500);
	LoRa_startReceiving(&(flight_computer->LoRa));

	HAL_Delay(1000);
}
