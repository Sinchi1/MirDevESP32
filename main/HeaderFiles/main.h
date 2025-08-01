#pragma once 

#include "HeaderFiles/BleModuleInit.h"
#include "HeaderFiles/BtController.h"

#include "esp_mac.h"
#include <iostream>
#include "freertos/FreeRTOS.h"
#include <nvs_flash.h>


class ProgrammRunner{

public:
    void run();
};