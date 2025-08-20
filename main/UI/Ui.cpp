#include "HeaderFiles/Ui.h"

static TaskHandle_t lvgl_task_handle = NULL;

QueueHandle_t g_sensor_queue = nullptr;

lv_obj_t* UI_ESP::lbl_temp = nullptr; 
lv_obj_t* UI_ESP::lbl_hum = nullptr;
lv_obj_t* UI_ESP::lbl_bat = nullptr;
lv_obj_t* UI_ESP::lbl_pres = nullptr;

void UI_ESP::lvgl_init(){

    esp_panel::board::Board* board = new esp_panel::board::Board();
    
    assert(board);
    ESP_UTILS_CHECK_FALSE_EXIT(board->init(), "board init failed");
    ESP_UTILS_CHECK_FALSE_EXIT(board->begin(), "board begin failed");
    ESP_UTILS_CHECK_FALSE_EXIT(lvgl_port_init(board->getLCD(),board->getTouch()), "lgvl_port init failed");

    lvgl_port_lock(pdMS_TO_TICKS(100));

    lv_disp_t* disp = lv_disp_get_default();

    if (!disp) {
        lvgl_port_unlock();
        ESP_LOGE("ui", "lv_disp_get_default() returned NULL");
        return;
    }
    
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

    // lv_obj_t* scr = lv_disp_get_scr_act(NULL);
    // lv_obj_set_style_bg_color(scr,lv_color_white(), 0);


    create_text_panel();
    lvgl_port_unlock();

    const uint8_t* data = nullptr;
    for (;;) {
        if (xQueueReceive(g_sensor_queue, &data, portMAX_DELAY) == pdTRUE && data) {
            lvgl_port_lock(pdMS_TO_TICKS(50));
            update_text_panel(data);
            lvgl_port_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }


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


void UI_ESP::update_text_panel(const uint8_t *data){
    uint8_t battery = data[IDX_BATT];
    int16_t temperature = (int16_t(data[IDX_TEMPH]) << 8) | data[IDX_TEMPL];
    uint16_t humidity = (uint16_t(data[IDX_HUMH]) << 8) | data[IDX_HUML];
    uint32_t pressure =
        (uint32_t(data[IDX_PRESSUREH]) << 16) |
        (uint32_t(data[IDX_PRESSUREL+1]) << 8) |
        data[IDX_PRESSUREL];
    uint16_t co2 = (uint16_t(data[IDX_CO2H]) << 8) | data[IDX_CO2L];

    lv_label_set_text_fmt(lbl_temp, "Temp: %.2f °C", float(temperature)/100);
    lv_label_set_text_fmt(lbl_hum , "Hum : %.2f%%", float(humidity)/100);
    lv_label_set_text_fmt(lbl_bat, "Battery: %u °C", battery);
    lv_label_set_text_fmt(lbl_pres , "Pressure : %lu ", pressure);
}