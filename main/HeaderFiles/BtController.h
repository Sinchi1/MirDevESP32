#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include <string>
#include <unordered_set>

// BLE Configuration constants
#define DEVICE_CONNECT_TIMEOUT_MS     5000    // Таймаут подключения к устройству (5 сек)
#define DEVICE_ADV_UPDATE_PERIOD_MS   30000   // Период сканирования устройств (30 сек)
#define BLE_CONNECT_ATTEMPT_COUNT     3       // Количество попыток подключения
#define DEVICE_HISTORY_QUERY_SIZE     72      // Размер запроса истории (72 записи = 3 дня по 24 записи)
#define DEVICE_CHAR_UPDATE_PERIOD_MS  10000   // Период обновления характеристик (10 сек)
#define DEVICE_NUM                    10      // Максимальное количество устройств в очереди
#define NIMBLE_MAX_CONNECTIONS        3       // Максимум одновременных BLE соединений


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
