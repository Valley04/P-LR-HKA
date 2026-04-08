#ifndef OTA_TASK_H
#define OTA_TASK_H

#include "printer_config.h"
#include "printer_task.h"
#include "printer_mqtt.h"

#define OTA_MAX_URL_LEN      256
#define OTA_MAX_TARGET_LEN   16
#define OTA_MAX_VERSION_LEN  16

typedef struct {
    char url[OTA_MAX_URL_LEN];
    char objetivo[OTA_MAX_TARGET_LEN];  // "ismart" o "printer"
    char version[OTA_MAX_VERSION_LEN];
} ota_config_t;

void procesar_comando_ota(const char* payload, int len);

#endif // OTA_TASK_H
