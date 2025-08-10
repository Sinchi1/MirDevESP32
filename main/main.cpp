// #include "HeaderFiles/main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "HeaderFiles/BtController.h"  

static const char* TAG = "main";

static SemaphoreHandle_t radioMutex = nullptr;

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    radioMutex = xSemaphoreCreateMutex();
    if (radioMutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create radio mutex");
        return;
    }

    ESP_LOGI(TAG, "Starting BLE application");

    BLE& bleInstance = BLE::instance();
    bleInstance.init(&radioMutex);

    ESP_LOGI(TAG, "BLE initialized, tasks started");

    while (1) {
        ESP_LOGI(TAG, "Main loop running...");
        vTaskDelay(pdMS_TO_TICKS(10000)); 
        
    }
}
