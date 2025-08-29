#pragma once

#include <string>
#include <unordered_set>

#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "portmacro.h"

#include "HeaderFiles/Aggregator.h"
#include "HeaderFiles/EnvironmentalSensorData.h"
#include "HeaderFiles/settings.h"
#include "HeaderFiles/Ui.h"

#include "NimBLEDevice.h"
#include "esp_log.h"
#include "esp_log_buffer.h"
#include "esp_log_level.h"

#include <cstdint>
#include <cmath>
#include <sys/time.h>
#include <time.h>

#define DEVICE_CONNECT_TIMEOUT_MS     5000    //5 Sek
#define DEVICE_ADV_UPDATE_PERIOD_MS   30000   
#define BLE_CONNECT_ATTEMPT_COUNT     3       
#define DEVICE_HISTORY_QUERY_SIZE     72      
#define DEVICE_CHAR_UPDATE_PERIOD_MS  10000   
#define DEVICE_NUM                    10      
#define NIMBLE_MAX_CONNECTIONS        3      

#define BLE_EVENT_SCANNER_STOPPED_MSK BIT0
#define BLE_EVENT_CLIENT_READY_MSK BIT1
#define BLE_EVENT_CLIENT_START_MSK BIT2
#define BLE_EVENT_CLIENT_STOPPED_MSK BIT3

#define BLE_ADV_SERVICE_UUID "FCD2"
#define BLE_ESS_SERVICE_UUID "181A"
#define BLE_TIME_SERVICE_UUID "1805"
#define BLE_TIME_CHAR_UUID "2A2B"
#define BLE_VOC_CHAR_UUID "2BE7"
#define BLE_IAQ_CHAR_UUID "E2890598-1286-43D6-82BA-121248BDA7DA"
#define BLE_HIST_READ_UUID "E3890598-1286-43D6-82BA-121248BDA7DA"
#define BLE_HIST_CONF_UUID "E4890598-1286-43D6-82BA-121248BDA7DA"

#define BLE_CYCLE_ENTRIES 24 // 480 byte per cycle

static const char* TAG = "ble";

class BLE
{
    std::unordered_set<std::string> known_devices;
    EventGroupHandle_t              ble_event_bits;
    QueueHandle_t                   ble_adv_dev_queue;
    static void                     ble_scan_task(void* pvParameters);
    static void                     ble_connect_task(void* pvParameters);
    SemaphoreHandle_t*              radioMutex;

public:
    static BLE& instance()
    {
        static BLE instance;
        return instance;
    }
    void init(SemaphoreHandle_t* radioMutex_);
};
