/*
 * gps.c
 *
 *  Created on: May 25, 2026
 *      Author: julia
 */

#include "gps.h"
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef *_huart_gps;
static UART_HandleTypeDef *_huart_debug;

static char    gps_line[MINMEA_MAX_SENTENCE_LENGTH];
static uint8_t gps_char;
static uint8_t gps_idx = 0;
static GPS_Data_t gps_data = {0};

void GPS_Init(UART_HandleTypeDef *huart_gps, UART_HandleTypeDef *huart_debug)
{
    _huart_gps   = huart_gps;
    _huart_debug = huart_debug;
}

void GPS_Task(void)
{
    if (HAL_UART_Receive(_huart_gps, &gps_char, 1, 100) != HAL_OK)
        return;

    if (gps_char == '\n' || gps_idx >= MINMEA_MAX_SENTENCE_LENGTH - 1)
    {
        gps_line[gps_idx] = '\0';
        gps_idx = 0;

        char buf[128];

        switch (minmea_sentence_id(gps_line, false))
        {
            case MINMEA_SENTENCE_GGA: {
                struct minmea_sentence_gga frame;
                if (minmea_parse_gga(&frame, gps_line)) {
                    gps_data.fix_quality        = frame.fix_quality;
                    gps_data.satellites_tracked = frame.satellites_tracked;
                    gps_data.lat = minmea_tocoord(&frame.latitude);
                    gps_data.lon = minmea_tocoord(&frame.longitude);
                    gps_data.alt = minmea_tofloat(&frame.altitude);

                    snprintf(buf, sizeof(buf),
                        "FIX:%d SATS:%d LAT:%f LON:%f ALT:%fm\r\n",
                        gps_data.fix_quality,
                        gps_data.satellites_tracked,
                        gps_data.lat,
                        gps_data.lon,
                        gps_data.alt);
                    HAL_UART_Transmit(_huart_debug, (uint8_t*)buf, strlen(buf), 100);
                }
            } break;

            case MINMEA_SENTENCE_RMC: {
                struct minmea_sentence_rmc frame;
                if (minmea_parse_rmc(&frame, gps_line)) {
                    gps_data.speed  = minmea_tofloat(&frame.speed);
                    gps_data.course = minmea_tofloat(&frame.course);

                    snprintf(buf, sizeof(buf),
                        "SPD:%f kt HDG:%f\r\n",
                        gps_data.speed,
                        gps_data.course);
                    HAL_UART_Transmit(_huart_debug, (uint8_t*)buf, strlen(buf), 100);
                }
            } break;

            default: {
                snprintf(buf, sizeof(buf), "RAW: %s\r\n", gps_line);
                HAL_UART_Transmit(_huart_debug, (uint8_t*)buf, strlen(buf), 100);
            } break;
        }
    }
    else
    {
        gps_line[gps_idx++] = gps_char;
    }
}

GPS_Data_t GPS_GetData(void)
{
    return gps_data;
}
