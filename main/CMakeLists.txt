idf_component_register(
    SRCS "main.cpp" "BLE/BtController.cpp" "entities/Aggregator.cpp"
    INCLUDE_DIRS "." "BLE" "entities"
    REQUIRES 
        nvs_flash
        bt
        lvgl
        freertos
)
