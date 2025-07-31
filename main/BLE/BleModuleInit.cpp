#include "HeaderFiles/BleModuleInit.h"


    void BleModuleInit::initBT(){
        esp_err_t ret;

        nvs_flash_init();
                
        ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
        if (ret != ESP_OK){
            ESP_LOGE("ESP BT","Error accured while esp_bt_controller_mem_release %s ", esp_err_to_name(ret));
        }

        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

        ret = esp_bt_controller_init(&bt_cfg);
        if (ret != ESP_OK){
            ESP_LOGE("ESP BT","Error accured while esp_bt_controller_init %s ", esp_err_to_name(ret));
        }

        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret != ESP_OK){
            ESP_LOGE("ESP BT","Error accured while esp_bt_controller_init %s ", esp_err_to_name(ret));
        }


        ret = esp_bluedroid_init();
        if (ret != ESP_OK){
            ESP_LOGE("ESP BT","Error accured while esp_bt_controller_init %s ", esp_err_to_name(ret));
        }

        ret = esp_bluedroid_enable();
        if (ret != ESP_OK){
            ESP_LOGE("ESP BT","Error accured while esp_bluedroid_enable %s ", esp_err_to_name(ret));
        }

        ESP_LOGI("ESP BT", "Succesfull bt init");
    }

    void BleModuleInit::deinitBT(){

        esp_err_t ret;
        ret = esp_bluedroid_deinit();
        if (ret != ESP_OK){
            ESP_LOGE("ESP BT","Error accured while esp_bluedroid_deinit %s ", esp_err_to_name(ret));
        }

        ret = esp_bt_controller_deinit();
        if(ret != ESP_OK){
            ESP_LOGE("ESP BT","Error accured while esp_bt_controller_deinit %s ", esp_err_to_name(ret));
        }

        ESP_LOGI("ESP BT", "Регистрация блютуза успешна");

    }