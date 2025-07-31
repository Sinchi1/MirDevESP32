#pragma once 

#include "esp_mac.h"
#include <iostream>
#include <esp_log.h>
#include <esp_err.h>
#include <nvs_flash.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_bt.h>

class BleModuleInit{
public:
    void initBT();
    void deinitBT();
};