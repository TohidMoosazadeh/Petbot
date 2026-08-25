#include <esp_log.h>
#include "RobotController.hpp"

static const char* TAG = "main";

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== Starting Dog Robot ===");
    
    dogrobot::RobotController controller;
    
    if (!controller.initialize()) {
        ESP_LOGE(TAG, "Failed to initialize robot controller");
        return;
    }
    
    controller.start();
    
    // Main thread idles
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}