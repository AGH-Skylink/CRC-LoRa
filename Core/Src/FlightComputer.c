/*
 * FlightComputer.c
 *
 *  Created on: May 11, 2026
 *      Authors: Julia Brąglewicz
 *      		 Franciszek Ślusarczyk
 */

#include "FlightComputer.h"
#define bmp280 flight_computer->barothermo.handle

void Sensors_read(FlightComputer* flight_computer){

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
	uint8_t settings = 0x08;
	HAL_I2C_Mem_Write(hi2c, 0x53 << 1, 0x2D, 1, &settings, 1, 100);
	settings = 0x00;
	HAL_I2C_Mem_Write(hi2c, 0x68 << 1, 0x3E, 1, &settings, 1, 100);
	HAL_I2C_Mem_Write(hi2c, 0x1E << 1,0x02, 1, &settings, 1, 100);
}

void StateMachine_idle(FlightComputer* flight_computer){

}

void StateMachine_launch(FlightComputer* flight_computer){

}

void StateMachine_ascent(FlightComputer* flight_computer){

}

void StateMachine_apogee(FlightComputer* flight_computer){

}

void StateMachine_descent(FlightComputer* flight_computer){

}

void StateMachine_landing(FlightComputer* flight_computer){

}

void FlightComputer_loop(FlightComputer* flight_computer){
	/*switch (FlightComputer->state) {
	    case 1:
	        // IDLE
	    	StateMachine_idle(flight_computer);
	        break;
	    case 2:
	        // LAUNCH
	    	StateMachine_launch(flight_computer);
	        break;
	    case 3:
	    	// ASCENT
	    	StateMachine_ascent(flight_computer);
	    	break;
	    case 1:
	    	// APOGEE
	    	StateMachine_apogee(flight_computer);
	    	break;
	    case 2:
	    	// DESCENT
	    	StateMachine_descent(flight_computer);
	    	break;
	    case 2:
	    	// LANDING
	    	StateMachine_landing(flight_computer);
	    	break;
	    default:
	        // ERROR
	        break;
	}*/

	HAL_Delay(1000);
}
