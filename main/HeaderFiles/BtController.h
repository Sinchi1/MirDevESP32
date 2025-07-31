#pragma once

#include "esp_mac.h"
#include <esp_gap_ble_api.h>
#include <esp_gattc_api.h>
#include <esp_bt_main.h>
#include <string>

struct CharacteristicOfAirFromDevice{
    std::string device;
    _Float16 humidity;
    _Float16 temperature;
};

class BtController{    
    public: 
        void runController();
    
    private: 
        void gapScan(esp_gap_ble_cb_event_t, esp_ble_gap_cb_param_t*);
        void gattcCallback(esp_gattc_cb_event_t, esp_gatt_if_t, esp_ble_gattc_cb_param_t*);
};