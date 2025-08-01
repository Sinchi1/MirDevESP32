#include "HeaderFiles/main.h"
#include "HeaderFiles/BleModuleInit.h"
#include "HeaderFiles/BtController.h"



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
        float temp = test2.getLatestTemperature();
        float hum = test2.getLatestHumidity();

        ESP_LOGI("MAIN", "Temp: %.2f, Humidity: %.2f", temp, hum);
        vTaskDelay(pdMS_TO_TICKS(5000)); // ждать 5 секунд
    }
}