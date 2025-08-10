#pragma once

#include <cstddef>
#include <cstdint>

// Время сканирования рекламы (мс)
#ifndef DEVICE_ADV_UPDATE_PERIOD_MS
#define DEVICE_ADV_UPDATE_PERIOD_MS 4000
#endif

// Пауза между циклами чтения характеристик (мс)
#ifndef DEVICE_CHAR_UPDATE_PERIOD_MS
#define DEVICE_CHAR_UPDATE_PERIOD_MS 2000
#endif

// Кол-во попыток подключения
#ifndef BLE_CONNECT_ATTEMPT_COUNT
#define BLE_CONNECT_ATTEMPT_COUNT 3
#endif

// Максимум BLE клиентов (должно совпадать с конфигурацией NimBLE)
#ifndef NIMBLE_MAX_CONNECTIONS
#define NIMBLE_MAX_CONNECTIONS 3
#endif

// Таймаут соединения (мс)
#ifndef DEVICE_CONNECT_TIMEOUT_MS
#define DEVICE_CONNECT_TIMEOUT_MS 2000
#endif

// Размер запроса истории (в записях)
#ifndef DEVICE_HISTORY_QUERY_SIZE
#define DEVICE_HISTORY_QUERY_SIZE 96
#endif

// Максимум одновременно найденных устройств в одном цикле (для очереди)
#ifndef DEVICE_NUM
#define DEVICE_NUM 8
#endif
