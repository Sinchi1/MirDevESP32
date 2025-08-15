#include "HeaderFiles/Ui.h"

static TaskHandle_t lvgl_task_handle = NULL;

QueueHandle_t g_sensor_queue = nullptr;

void UI_ESP::lvgl_init(){

    esp_panel::board::Board* board = new esp_panel::board::Board();
    lvgl_port_cfg_t cfg =  ESP_LVGL_PORT_INIT_CONFIG();
    cfg.task_affinity = -1;
    
    assert(board);
    ESP_UTILS_CHECK_FALSE_EXIT(board->init(), "board init failed");
    ESP_UTILS_CHECK_FALSE_EXIT(board->begin(), "board begin failed");
    // ESP_UTILS_CHECK_FALSE_EXIT(lvgl_port_init(board->getLCD(),board->getTouch()), "lgvl_port init failed");
    lvgl_port_init(&cfg);

    lv_disp_t* disp = lv_disp_get_default();
    
    lv_theme_t* theme = lv_theme_default_init(
        disp,
        lv_palette_main(LV_PALETTE_BLUE),
        lv_palette_main(LV_PALETTE_CYAN),
        true,
        LV_FONT_DEFAULT
    );

    lv_disp_set_theme(disp, theme);

    lv_obj_t* screen = lv_disp_get_scr_act(disp);
    lv_obj_t* label = lv_label_create(screen);
    lv_label_set_text(label, "Hello esp!");
    lv_obj_align(label, LV_ALIGN_CENTER,0,0);

    BaseType_t ret = xTaskCreate(
        UI_ESP::lvgl_print_results,
        "lvgl_task",
        4096,
        NULL,
        5,
        &lvgl_task_handle
    );

    ESP_UTILS_CHECK_FALSE_EXIT(ret == pdPASS, "Failed to create lvgl task");

}

void UI_ESP::create_text_panel()
{
    lv_obj_t *scr = lv_disp_get_scr_act(nullptr);

    lbl_temp = lv_label_create(scr);
    lv_label_set_text(lbl_temp, "Temp: --.- °C");
    lv_obj_align(lbl_temp, LV_ALIGN_TOP_MID, 0, 20);

    lbl_hum = lv_label_create(scr);
    lv_label_set_text(lbl_hum, "Hum : --.- %");
    lv_obj_align(lbl_hum, LV_ALIGN_TOP_MID, 0, 40);

    lbl_bat = lv_label_create(scr);
    lv_label_set_text(lbl_bat, "Battery : --.- %");
    lv_obj_align(lbl_bat, LV_ALIGN_TOP_MID, 0, 60);

    lbl_pres = lv_label_create(scr);
    lv_label_set_text(lbl_pres, "Pressure : --.- %");
    lv_obj_align(lbl_pres, LV_ALIGN_TOP_MID, 0, 80);
}

void UI_ESP::lvgl_task(void *arg)
{
    const uint8_t *data;
    while(true)
    {
        if (xQueueReceive(g_sensor_queue, &data, pdMS_TO_TICKS(1000)) == pdTRUE) {
            lvgl_port_lock(pdMS_TO_TICKS(10));  
            update_text_panel(data);
            lvgl_port_unlock();
        }

        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void UI_ESP::update_text_panel(const uint8_t *data){
    uint8_t battery = data[IDX_BATT];
    int16_t temperature = (int16_t(data[IDX_TEMPH]) << 8) | data[IDX_TEMPL];
    uint16_t humidity = (uint16_t(data[IDX_HUMH]) << 8) | data[IDX_HUML];
    uint32_t pressure =
        (uint32_t(data[IDX_PRESSUREH]) << 16) |
        (uint32_t(data[IDX_PRESSUREL+1]) << 8) |
        data[IDX_PRESSUREL];
    uint16_t co2 = (uint16_t(data[IDX_CO2H]) << 8) | data[IDX_CO2L];

    lv_label_set_text_fmt(lbl_temp, "Temp: %.1f °C", temperature);
    lv_label_set_text_fmt(lbl_hum , "Hum : %.1f %%", humidity);
    lv_label_set_text_fmt(lbl_bat, "Battery: %.1f °C", battery);
    lv_label_set_text_fmt(lbl_pres , "Pressure : %.1f ", pressure);
}