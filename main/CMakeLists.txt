idf_component_register(
    SRCS "main.cpp" "BLE/BleModuleInit.cpp" "BLE/BtController.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES lvgl bt freertos
)