/*
 * gps.h
 *
 *  Created on: May 25, 2026
 *      Author: julia
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "main.h"
#include "minmea.h"

typedef struct {
    int     fix_quality;
    int     satellites_tracked;
    float   lat;
    float   lon;
    float   alt;
    float   speed;
    float   course;
} GPS_Data_t;

void GPS_Init_debug(UART_HandleTypeDef *huart_gps, UART_HandleTypeDef *huart_debug);
void GPS_Init(UART_HandleTypeDef *huart_gps);
void GPS_Task_debug(void);
void GPS_Task(void);
GPS_Data_t GPS_GetData(void);

#endif /* INC_GPS_H_ */
