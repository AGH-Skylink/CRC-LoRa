/*
 * gps.c
 *
 *  Created on: May 25, 2026
 *      Author: julia
 */

#include "gps.h"
#include <string.h>
#include <stdio.h>

#define GPS_DMA_BUF_SIZE 256

static UART_HandleTypeDef *_huart_gps;
static UART_HandleTypeDef *_huart_debug;

static char    gps_line[MINMEA_MAX_SENTENCE_LENGTH];
static uint8_t gps_char;
static uint8_t gps_idx = 0;
static GPS_Data_t gps_data = {0};

static uint8_t dma_rx_buf[GPS_DMA_BUF_SIZE];
static uint16_t dma_read_idx = 0;

void GPS_Init_debug(UART_HandleTypeDef *huart_gps, UART_HandleTypeDef *huart_debug)
{
    _huart_gps   = huart_gps;
    _huart_debug = huart_debug;
}

void GPS_Task_debug(void)
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

//GPS_Data_t GPS_GetData(void)
//{
//    return gps_data;
//}
//
//void GPS_Init(UART_HandleTypeDef *huart_gps)
//{
//    _huart_gps = huart_gps;
////    HAL_UART_Receive_IT(_huart_gps, &gps_char, 1);
//}
//
//void GPS_Task(void)
//{
//	if (__HAL_UART_GET_FLAG(_huart_gps, UART_FLAG_ORE))
//	    {
//	        __HAL_UART_CLEAR_OREFLAG(_huart_gps);
//	        // jeśli to się często zdarza - masz potwierdzenie utraty bajtów
//	    }
//
//    if (HAL_UART_Receive(_huart_gps, &gps_char, 1, 2) != HAL_OK)
//        return;
//
//    if (gps_char == '\n' || gps_idx >= MINMEA_MAX_SENTENCE_LENGTH - 1)
//    {
//        gps_line[gps_idx] = '\0';
//        gps_idx = 0;
//
//        switch (minmea_sentence_id(gps_line, false))
//        {
//            case MINMEA_SENTENCE_GGA: {
//                struct minmea_sentence_gga frame;
//                if (minmea_parse_gga(&frame, gps_line)) {
//                    gps_data.fix_quality        = frame.fix_quality;
//                    gps_data.satellites_tracked = frame.satellites_tracked;
//                    gps_data.lat = minmea_tocoord(&frame.latitude);
//                    gps_data.lon = minmea_tocoord(&frame.longitude);
//                    gps_data.alt = minmea_tofloat(&frame.altitude);
//                }
//            } break;
//
//            case MINMEA_SENTENCE_RMC: {
//                struct minmea_sentence_rmc frame;
//                if (minmea_parse_rmc(&frame, gps_line)) {
//                    gps_data.speed  = minmea_tofloat(&frame.speed);
//                    gps_data.course = minmea_tofloat(&frame.course);
//                }
//            } break;
//
//            default: break;
//        }
//    }
//    else
//    {
//        gps_line[gps_idx++] = gps_char;
//    }
//}

static void GPS_ParseLine(void)
{
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
            }
        } break;

        case MINMEA_SENTENCE_RMC: {
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, gps_line)) {
                gps_data.speed  = minmea_tofloat(&frame.speed);
                gps_data.course = minmea_tofloat(&frame.course);
            }
        } break;

        default: break;
    }
}

void GPS_Init(UART_HandleTypeDef *huart_gps)
{
    _huart_gps = huart_gps;

    // Start transferu cyklicznego - DMA sam, sprzętowo, kopiuje bajty do dma_rx_buf w kółko
    HAL_UART_Receive_DMA(_huart_gps, dma_rx_buf, GPS_DMA_BUF_SIZE);

    // Wyłącz przerwania DMA na tym kanale - działamy wyłącznie przez polling licznika
    __HAL_DMA_DISABLE_IT(_huart_gps->hdmarx, DMA_IT_TC);
    __HAL_DMA_DISABLE_IT(_huart_gps->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(_huart_gps->hdmarx, DMA_IT_TE);

    // Opcjonalnie: wyłącz też przerwania błędów UART (PE/ERR),
    // jeśli chcesz zero jakiejkolwiek aktywności NVIC związanej z tym peryferalem
    __HAL_UART_DISABLE_IT(_huart_gps, UART_IT_ERR);
    __HAL_UART_DISABLE_IT(_huart_gps, UART_IT_PE);
}

void GPS_Task(void)
{
    // Aktualna pozycja zapisu DMA w buforze cyklicznym
    uint16_t write_idx = GPS_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(_huart_gps->hdmarx);

    while (dma_read_idx != write_idx)
    {
        uint8_t c = dma_rx_buf[dma_read_idx];
        dma_read_idx = (dma_read_idx + 1) % GPS_DMA_BUF_SIZE;

        if (c == '\n' || gps_idx >= MINMEA_MAX_SENTENCE_LENGTH - 1)
        {
            gps_line[gps_idx] = '\0';
            gps_idx = 0;
            GPS_ParseLine();
        }
        else
        {
            gps_line[gps_idx++] = c;
        }
    }
}

GPS_Data_t GPS_GetData(void)
{
    return gps_data;
}
