//
// Created by lukasz on 21.03.2026.
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// #include "FreeRTOS.h"
// #include "fatfs.h"
// #include "task.h"

#define SS_LOG_PREFIX "Agatka_log_"
#define SS_LOGFILE_PATH_MAX 64U
#define SS_LOG_TASK_PERIOD_MS 100U

FIL SS_SD_FDST;
UINT CURRENT_IDX = 0U;

static const char *SS_sd_card_path(void) {
    return (SDPath[0] != '\0') ? SDPath : "0:";
}


int SS_sd_card_init(void) {
    MX_FATFS_Init();

    // The actual SDIO SD card init is handled by MX_SDIO_SD_Init().
    DSTATUS status = disk_status(SS_SD_IDX/* [IN] Physical drive number, always 0 for single drive */);
    FRESULT fr;
    fr = SS_sd_card_mount();
    if (fr != FR_OK) {
        SS_println("Failed to mount SD card: status %d", (int)fr);
        return (int)fr;
    }

    fr = SS_sd_card_create_next_logfile();
    if (fr != FR_OK) {
        SS_println("Failed to create logfile: status %d", (int)fr);
        return (int)fr;
    }
    fr = SS_sd_card_open_last_logfile();
    if (fr != FR_OK) {
        SS_println("Failed to open logfile: status %d", (int)fr);
        return (int)fr;
    }
    return 0;
}

static int SS_sd_card_build_log_filename(char* path, const UINT idx) {
    if (path == NULL) {
        return -1;
    }
    uint8_t written = snprintf(path, SS_LOGFILE_PATH_MAX, "%s" SS_LOG_PREFIX "%u", SS_sd_card_path(), (unsigned int)idx);
    /*
    if (written != sizeof(path)) {
        return -1;
    }
    */
    return 0;
}

int SS_sd_card_mount(void) {
    return (int)f_mount(&SDFatFS, SS_sd_card_path(), SS_SD_IDX);
}

int SS_sd_card_find_last_idx(void) {
    DIR dir;
    FILINFO fno;
    UINT max_idx = 0U;
    int found_any = 0;
    const size_t prefix_len = strlen(SS_LOG_PREFIX);

    const char *path = SS_sd_card_path();
    FRESULT res = f_opendir(&dir, path); /* Open the directory */
    if (res != FR_OK) {
        SS_print("Failed to open \"%s\". (%u)\n", path, res);
        return (int)res;
    }

    for (;;) {
        res = f_readdir(&dir, &fno); /* Read a directory item */
        if (res != FR_OK || fno.fname[0] == 0) {
            break; /* Error or end of dir */
        }

        if (fno.fattrib & AM_DIR) {
            continue;
        }
        if (strncmp(fno.fname, SS_LOG_PREFIX, prefix_len) != 0) {
            continue;
        }

        char* endptr = NULL;
        const unsigned long idx = strtoul(&fno.fname[prefix_len], &endptr, 10);
        if (endptr == &fno.fname[prefix_len] || *endptr != '\0') {
            continue;
        }

        if (!found_any || idx > max_idx) {
            max_idx = (UINT)idx;
            found_any = 1;
        }
    }

    (void)f_closedir(&dir);

    if (res != FR_OK) {
        return (int)res;
    }

    if (!found_any) {
        CURRENT_IDX = 0U;
        return 0;
    }

    CURRENT_IDX = max_idx;
    return (int)CURRENT_IDX;
}

int SS_sd_card_create_next_logfile(void) {
    const int last_idx = SS_sd_card_find_last_idx();
    if (last_idx < 0) {
        return last_idx;
    }
    CURRENT_IDX = (UINT)last_idx + 1U;
    char path[SS_LOGFILE_PATH_MAX];
    if (SS_sd_card_build_log_filename(path, CURRENT_IDX) != 0) {
        return -1;
    }
    SS_println("%s%s", "Log path: ", path);
    FRESULT fr = f_open(&SS_SD_FDST, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        return (int)fr;
    }

    fr = f_close(&SS_SD_FDST);
    return (int)fr;
}

int SS_sd_card_open_last_logfile(void) {
    if (CURRENT_IDX == 0U) {
        const int last_idx = SS_sd_card_find_last_idx();
        if (last_idx < 0) {
            return last_idx;
        }
        if (last_idx == 0) {
            return (int)FR_NO_FILE;
        }
        CURRENT_IDX = (UINT)last_idx;
    }

    char path[SS_LOGFILE_PATH_MAX];
    if (SS_sd_card_build_log_filename(path, CURRENT_IDX) != 0) {
        return -1;
    }

    FRESULT fr = f_open(&SS_SD_FDST, path, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
    return (int)fr;
}

int SS_sd_card_write_data(const char* buffer) {
    if (buffer == NULL) {
        return (int)FR_INVALID_PARAMETER;
    }

    const UINT btw = (UINT)strlen(buffer);
    if (btw == 0U) {
        return (int)FR_OK;
    }

    UINT bw = 0U; // bytes written
    FRESULT fr = f_write(&SS_SD_FDST, buffer, btw /* bytes to write */, &bw);
    if (fr != FR_OK) {
        return (int)fr;
    }
    if (bw != btw) {
        return (int)FR_DISK_ERR;
    }

    fr = f_sync(&SS_SD_FDST); // flush cache
    if (fr != FR_OK) {
        return (int)fr;
    }
    return (int)FR_OK;
}

int SS_sd_card_read_data(const char* filename, void* buffer, const UINT btr /* bytes to read */) {
    if (filename == NULL || buffer == NULL) {
        return (int)FR_INVALID_PARAMETER;
    }

    char path[SS_LOGFILE_PATH_MAX];
    const char* open_path = filename;
    if (strchr(filename, ':') == NULL) {
        const int written = snprintf(path, sizeof(path), "%s%s", SS_sd_card_path(), filename);
        if (written < 0 || (size_t)written >= sizeof(path)) {
            return -1;
        }
        open_path = path;
    }

    FIL fdst;
    FRESULT fr = f_open(&fdst, open_path, FA_READ);
    if (fr != FR_OK) {
        return (int)fr;
    }

    UINT br = 0U;
    fr = f_read(&fdst, buffer, btr, &br);

    const FRESULT close_res = f_close(&fdst);
    if (fr != FR_OK) {
        return (int)fr;
    }
    if (close_res != FR_OK) {
        return (int)close_res;
    }
    if (br != btr) {
        return (int)FR_INT_ERR;
    }

    return (int)FR_OK;
}

void SS_sd_card_close_logfile(void) {
    (void)f_close(&SS_SD_FDST);
}

void SS_sd_card_log_task(void* pvParameters) {
    (void)pvParameters;
    uint32_t writes = 0U;
    uint32_t write_errors = 0U;
    FRESULT fr;

    for (;;) {
        char msg[64];
        const unsigned long fat_time = (unsigned long)get_fattime();
        const int len = snprintf(msg, sizeof(msg), "%lu still testing...\n", fat_time); // TODO: log actual data

        if (len > 0) {
            fr = SS_sd_card_write_data(msg);
            if (fr != FR_OK) {
                SS_println("Log error code: %d", fr);
                ++write_errors;
            } else {
                ++writes;
            }
        } else {
            ++write_errors;
        }
        /*
        if (((writes + write_errors) % 100U) == 0U) {
            SS_print("SD logger: writes=%lu errors=%lu\n", (unsigned long)writes, (unsigned long)write_errors);
        }
        */
        vTaskDelay(pdMS_TO_TICKS(SS_LOG_TASK_PERIOD_MS));
    }
}

int SS_sd_card_unmount(void) {
    const FRESULT close_res = f_close(&SS_SD_FDST);
    const FRESULT unmount_res = f_mount(NULL, SS_sd_card_path(), SS_SD_IDX);

    if (unmount_res != FR_OK) {
        return (int)unmount_res;
    }
    if (close_res != FR_OK && close_res != FR_INVALID_OBJECT) {
        return (int)close_res;
    }

    return (int)FR_OK;
}
