//
// Created by lukasz on 21.03.2026.
//
#pragma once
#include <sys/_stdint.h>
// #include "ff.h"

#ifndef SS_SD_CARD_IO_H
#define SS_SD_CARD_IO_H

#define SS_SD_IDX 0U
#define SS_SD_FATFS fs0 // Work area (filesystem object) for logical drive
#define SS_SD_FDST fdst // Destination file

int SS_sd_card_init(void);

int SS_sd_card_mount(void);
int SS_sd_card_unmount(void);

int SS_sd_card_create_next_logfile(void);
int SS_sd_card_open_last_logfile(void);
void SS_sd_card_close_logfile(void);

int SS_sd_card_write_data(const char* buffer);
int SS_sd_card_read_data(const char* filename, void* buffer, const UINT btr);
void SS_sd_card_log_task(void* pvParameters);


#endif
