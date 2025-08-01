#pragma once

#include "esp_mac.h"
#include <esp_gap_ble_api.h>
#include <esp_gattc_api.h>
#include <esp_log.h>
#include <esp_bt_main.h>
#include <string>
#include "string.h"

static uint16_t connectionId = 0;
static uint16_t tempCharHandle = 0;
static uint16_t humidityCharHandle = 0;

static esp_bt_uuid_t tempCharUuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = 0x2A6E}, 
};
static esp_bt_uuid_t humidityCharUuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = 0x2A6F}, 
};


struct CharacteristicOfAirFromDevice{
    std::string device;
    float humidity;
    float temperature;
};

static esp_ble_scan_params_t params = {
            .scan_type = BLE_SCAN_TYPE_ACTIVE,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
            .scan_interval = 0x50,
            .scan_window = 0x30,
            .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};

class BtController{    
    public: 
        void runController();
        uint16_t getLatestTemperature();
        uint16_t getLatestHumidity();

    private: 
      static void gapScan(esp_gap_ble_cb_event_t, esp_ble_gap_cb_param_t*);
      static void gattcCallback(esp_gattc_cb_event_t, esp_gatt_if_t, esp_ble_gattc_cb_param_t*);
};