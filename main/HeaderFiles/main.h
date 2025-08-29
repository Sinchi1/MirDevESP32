#pragma once 


#define I2C_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO GPIO_NUM_15 /*!< gpio number for I2C master clock */
#define I2C_MASTER_SCL_IO GPIO_NUM_16 /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ 100000     /*!< I2C master clock frequency */

#include "esp_display_panel.hpp"
#include "esp_log.h"

#include "adapt/lvgl_port_v8.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_log_level.h"
#include "lvgl.h"
#include "i2c_bus.h"
#include "nvs_flash.h"

#include "Ui.h"
#include "BtController.h"

static const char* TAG = "main";

static SemaphoreHandle_t radioMutex = nullptr;
