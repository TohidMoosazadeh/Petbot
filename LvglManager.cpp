#include "LvglManager.hpp"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace dogrobot {

static const char* TAG = "LvglManager";

LvglManager::LvglManager(LcdDisplay& lcd) : lcd_(lcd) {}

LvglManager::~LvglManager() {
    if (buf1_) heap_caps_free(buf1_);
    if (buf2_) heap_caps_free(buf2_);
}

void LvglManager::flushCallback(lv_disp_drv_t* disp_drv, 
                                 const lv_area_t* area, 
                                 lv_color_t* color_p) {
    LvglManager* self = static_cast<LvglManager*>(disp_drv->user_data);
    esp_lcd_panel_handle_t panel = self->lcd_.getPanelHandle();
    
    esp_lcd_panel_draw_bitmap(panel, 
                              area->x1 + self->lcd_.getOffsetX(), 
                              area->y1 + self->lcd_.getOffsetY(), 
                              area->x2 + 1 + self->lcd_.getOffsetX(), 
                              area->y2 + 1 + self->lcd_.getOffsetY(), 
                              color_p);
    lv_disp_flush_ready(disp_drv);
}

void LvglManager::taskEntry(void* arg) {
    LvglManager* self = static_cast<LvglManager*>(arg);
    self->taskLoop();
}

void LvglManager::taskLoop() {
    while (true) {
        if (update_callback_) {
            update_callback_();
        }
        
        lv_tick_inc(FRAME_TIME_MS);
        lv_timer_handler();
        
        vTaskDelay(pdMS_TO_TICKS(FRAME_TIME_MS));
    }
}

bool LvglManager::start() {
    ESP_LOGI(TAG, "Starting LVGL...");
    
    // Allocate buffers
    size_t buf_size = 320 * BUFFER_LINES * sizeof(lv_color_t);
    
    buf1_ = static_cast<lv_color_t*>(heap_caps_malloc(buf_size, MALLOC_CAP_DMA));
    buf2_ = static_cast<lv_color_t*>(heap_caps_malloc(buf_size, MALLOC_CAP_DMA));
    
    if (!buf1_ || !buf2_) {
        ESP_LOGE(TAG, "Failed to allocate LVGL buffers");
        return false;
    }

    // Initialize LVGL
    lv_init();
    lv_disp_draw_buf_init(&draw_buf_, buf1_, buf2_, 320 * BUFFER_LINES);

    // ⭐ Use member variable disp_drv_ instead of local variable
    lv_disp_drv_init(&disp_drv_);
    disp_drv_.hor_res = 320;
    disp_drv_.ver_res = 240;
    disp_drv_.flush_cb = flushCallback;
    disp_drv_.draw_buf = &draw_buf_;
    disp_drv_.user_data = this;  // ⭐ Store 'this' pointer
    lv_disp_drv_register(&disp_drv_);  // ⭐ Register the persistent driver

    // Start render task
    if (xTaskCreate(taskEntry, "lvgl_task", 8192, this, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LVGL task");
        return false;
    }

    started_ = true;
    ESP_LOGI(TAG, "LVGL started successfully");
    return true;
}

void LvglManager::setUpdateCallback(UpdateCallback callback) {
    update_callback_ = std::move(callback);
}

void LvglManager::requestUpdate() {
    update_requested_ = true;
}

} // namespace dogrobot