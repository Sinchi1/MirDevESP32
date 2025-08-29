/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "main.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

extern "C" void app_main(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master =
            {
                .clk_speed = I2C_MASTER_FREQ_HZ,
            },
    };

    i2c_bus_handle_t i2c_bus = i2c_bus_create(I2C_NUM, &conf);

    Board*      board     = new Board();
    static bool connected = false;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGE(TAG, "mktime failed");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    assert(board);

    ESP_UTILS_CHECK_FALSE_EXIT(board->init(), "Board init failed");
    ESP_UTILS_CHECK_FALSE_EXIT(board->begin(), "Board begin failed");
    ESP_UTILS_CHECK_FALSE_EXIT(lvgl_port_init(board->getLCD(), board->getTouch()),
                               "LVGL init failed");
    lv_disp_t*  dispp = lv_disp_get_default();
    lv_theme_t* theme =
    lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE),
                              lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
    // lv_disp_set_theme(dispp, theme);

    UI_ESP& ui_esp = UI_ESP::instance();

    g_sensor_queue = xQueueCreate(10, sizeof(uint8_t*));
    assert(g_sensor_queue != NULL);

        ESP_LOGI(TAG, "Starting BLE application");

    BLE& bleInstance = BLE::instance();

    radioMutex = xSemaphoreCreateMutex();

    if (radioMutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create radio mutex");
        return;
    }

    bleInstance.init(&radioMutex);

    ui_esp.lvgl_init(dispp, theme);

}