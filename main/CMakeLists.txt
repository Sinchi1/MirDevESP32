idf_component_register(
    SRCS "main.cpp" "BLE/BtController.cpp" "entities/Aggregator.cpp" "UI/Ui.cpp" "../managed_components/espressif__esp_lvgl_port/include/esp_lvgl_port.h"
    INCLUDE_DIRS "." "BLE" "entities" "UI" "HeaderFiles" "../managed_components/espressif__esp32_display_panel/template_files" "../managed_components/espressif__esp_lvgl_port/include"
    REQUIRES 
        nvs_flash
        bt
        lvgl
        freertos
)