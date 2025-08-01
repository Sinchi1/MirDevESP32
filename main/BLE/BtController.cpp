#include "../HeaderFiles/BtController.h"


CharacteristicOfAirFromDevice characteristicOfAirFromDevice;

static esp_gatt_if_t gatt;

void BtController::gapScan(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* query){
    if (event == ESP_GAP_BLE_SCAN_RESULT_EVT){
        auto* result = &query->scan_rst;
        std::string advAunt;
        uint8_t lenght; 
        const uint8_t* name = esp_ble_resolve_adv_data(result->ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &lenght);
        if (name && lenght > 0){
            advAunt = std::string((char*)name, lenght);
            if (advAunt.find("esp") != std::string::npos){
            esp_ble_gap_stop_scanning();
            esp_ble_gattc_open(gatt, result->bda, BLE_ADDR_TYPE_PUBLIC, true);                
            }
        }
        
    }
}

uint16_t BtController::getLatestTemperature(){
    return tempCharHandle;
}

uint16_t BtController::getLatestHumidity(){
    return humidityCharHandle;
}

void BtController::gattcCallback(esp_gattc_cb_event_t event, esp_gatt_if_t if_t, esp_ble_gattc_cb_param_t* query){
    switch (event){
    case ESP_GATTC_REG_EVT:{
        gatt = if_t;
        esp_ble_gap_set_scan_params(&params);
        break;
    }

    case ESP_GATTC_CONNECT_EVT:{
        connectionId = query->connect.conn_id;
        esp_ble_gattc_search_service(gatt, connectionId, NULL);
        break;    
    }

    case ESP_GATTC_SEARCH_CMPL_EVT:{
        uint16_t count = 0;

        esp_gatt_status_t status = esp_ble_gattc_get_attr_count(
            gatt,
            query->search_cmpl.conn_id,
            ESP_GATT_DB_CHARACTERISTIC,
            query->search_res.start_handle,
            query->search_res.end_handle,
            0,
            &count);

        if (status != ESP_GATT_OK || count == 0) {
            ESP_LOGE("Esp", "None char. found inside module");
            break;
        }

        esp_gattc_char_elem_t* charElement = (esp_gattc_char_elem_t*)malloc(count * sizeof(esp_gattc_char_elem_t));
        if (!charElement) {
            ESP_LOGE("esp", "Malloc for describtion of charElement failed.");
            break;
        }

        status = esp_ble_gattc_get_char_by_uuid(
            gatt,
            query->search_res.conn_id,
            query->search_res.start_handle,
            query->search_res.end_handle,
            tempCharUuid,
            charElement,
            &count
        );
        if (status == ESP_GATT_OK && count > 0) {
            tempCharHandle = charElement[0].char_handle;
            esp_ble_gattc_read_char(gatt, connectionId, tempCharHandle, ESP_GATT_AUTH_REQ_NONE);
        }

        count = 1;
        status = esp_ble_gattc_get_char_by_uuid(
            gatt,
            query->search_res.conn_id,
            query->search_res.start_handle,
            query->search_res.end_handle,
            tempCharUuid,
            charElement,
            &count
        );
        if (status == ESP_GATT_OK && count > 0) {
            humidityCharHandle = charElement[0].char_handle;
            esp_ble_gattc_read_char(gatt, connectionId, humidityCharHandle, ESP_GATT_AUTH_REQ_NONE);
        }
        free(charElement);
        break;
    }
    

    case ESP_GATTC_READ_CHAR_EVT:{
        if (query->read.status != ESP_GATT_OK) {
            ESP_LOGE("esp", "unable to read char: %d", query->read.status);
            break;
        }

        ESP_LOGI("esp", "Readed char handle=%d, len=%d", query->read.handle, query->read.value_len);

        /*
        *
        *   ....
        * 
        */
    }
    

    default:{
        break;
    }
    
    }
}

void BtController::runController(){
    esp_ble_gattc_register_callback(gattcCallback);
    esp_ble_gap_register_callback(gapScan);
    esp_ble_gattc_app_register(1);
}
