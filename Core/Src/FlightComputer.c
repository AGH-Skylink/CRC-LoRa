/*
 * FlightComputer.c
 *
 *  Created on: May 11, 2026
 *      Authors: Julia Brąglewicz
 *      		 Franciszek Ślusarczyk
 */

#include "FlightComputer.h"

void Sensors_read(FlightComputer* flight_computer){

}

void Sensors_bypass(FlightComputer* flight_computer){

}

void LoRa_send_telemetry(FlightComputer* flight_computer){

}

void FlightComputer_init(FlightComputer* flight_computer){

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
	switch (FlightComputer->state) {
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
	}
}
