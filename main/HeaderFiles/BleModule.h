#pragma once 

#include <esp_log.h>
#include <esp_err.h>
#include <esp_bt.h>
#include <esp_gap_ble_api.h>
#include <esp_gattc_api.h>
#include <esp_bt_main.h>

class BleModule{

public:
    void connect();
    void getData();
private:
    char* readData();
};