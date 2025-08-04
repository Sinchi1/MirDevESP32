#include "HeaderFiles/main.h"

extern "C" void app_main(void){
    ProgrammRunner runner = ProgrammRunner();
    runner.run();
} 

void ProgrammRunner::run(){
    nvs_flash_init();
    BleModuleInit test = BleModuleInit();
    BtController test2 = BtController();
    test.initBT();
    test2.runController();
    std::cout << "Hello, ESP!\n";
        while (1) {
        esp_ble_gap_start_scanning(5);

        float temp = test2.getLatestTemperature();
        float hum = test2.getLatestHumidity();

        ESP_LOGI("MAIN", "Temp: %.2f, Humidity: %.2f", temp, hum);
        vTaskDelay(pdMS_TO_TICKS(5000)); 
    }
}