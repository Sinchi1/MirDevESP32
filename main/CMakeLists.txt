idf_component_register(
    SRCS "main.cpp" "BLE/BtController.cpp" "BLE/BleModuleInit.cpp"
    INCLUDE_DIRS "." "BLE"
    REQUIRES 
        nvs_flash
        bt
        lvgl
        freertos
)
