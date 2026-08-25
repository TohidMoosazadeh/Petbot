#include "RobotController.hpp"
#include <esp_log.h>
#include <cstring>
#include <cstdio>

namespace dogrobot {

static const char* TAG = "RobotController";

RobotController::RobotController() {}
RobotController::~RobotController() {}

Emotion RobotController::mapStateToEmotion(const CommandState& state) {
    if (state.funcMode == 1) return Emotion::SLEEPY;
    if (state.funcMode == 2) return Emotion::ANGRY;
    if (state.funcMode == 3) return Emotion::SURPRISED;
    if (state.moveFB == 1 && state.moveLR == 0) return Emotion::HAPPY;
    if (state.moveFB == -1 && state.moveLR == 0) return Emotion::SAD;
    if (state.sideWalkLR != 0) return Emotion::SURPRISED;
    if (state.moveLR != 0) return Emotion::SLEEPY;
	return Emotion::NEUTRAL;
}

// Changed to non-const reference to allow flag resetting
void RobotController::onCommandReceived(CommandState& state) {
    // 1. Handle Calibration Commands First
    if (state.applyTrim) {
        if (motion_) {
            motion_->applyServoTrim(state.trimMotorId, state.trimOffset);
        }
        state.applyTrim = false; // Reset flag after processing
        ESP_LOGI(TAG, "Servo trim applied and flag reset");
    }

    if (state.imuZero) {
        if (motion_) {
            motion_->resetImuReference();
        }
        state.imuZero = false; // Reset flag after processing
        ESP_LOGI(TAG, "IMU zeroed and flag reset");
    }

    // 2. Update command text for LCD
    if (state.funcMode > 0) {
        std::snprintf(last_command_text_, sizeof(last_command_text_),
                      "Action: %d", state.funcMode);
    } else if (state.moveFB == 0 && state.moveLR == 0 && state.sideWalkLR == 0) {
        std::snprintf(last_command_text_, sizeof(last_command_text_), "Stopped");
    } else {
        std::snprintf(last_command_text_, sizeof(last_command_text_),
                      "FB:%d LR:%d SW:%d Spd:%d H:%d",
                      state.moveFB, state.moveLR, state.sideWalkLR, 
                      state.speed, state.bodyHeight);
    }

    // 3. Forward movement parameters to motion system
    if (motion_) {
        if (state.funcMode > 0) {
            motion_->setAction(state.funcMode);
        } else {
            motion_->setMovement(state.moveFB, state.moveLR,
                                 state.sideWalkLR, state.speed, state.bodyHeight);
        }
    }

    // 4. Update emotion
    if (emotion_) {
        Emotion target = mapStateToEmotion(state);
        if (target != emotion_->getCurrentEmotion()) {
            emotion_->showEmotion(target);
        }
    }

    // 5. Update status label
    if (status_label_) {
        lv_label_set_text(status_label_, last_command_text_);
    }
}

void RobotController::updateUI() {
    if (emotion_) {
        emotion_->updateAnimation();
    }
}

bool RobotController::initialize() {
    ESP_LOGI(TAG, "=== Initializing Robot Controller ===");

    // 1. LCD
    LcdConfig lcd_config = {
        .mosi_pin = GPIO_NUM_11, .clk_pin = GPIO_NUM_12,
        .cs_pin = GPIO_NUM_10, .dc_pin = GPIO_NUM_5,
        .rst_pin = GPIO_NUM_4, .bk_pin = GPIO_NUM_14,
        .width = 320, .height = 240,
        .swap_xy = true, .mirror_x = true, .mirror_y = false,
        .offset_x = 0, .offset_y = 0,
        .spi_freq_hz = 20 * 1000 * 1000,
    };
    lcd_ = std::make_unique<LcdDisplay>(lcd_config);
    if (!lcd_->initialize()) return false;

    // 2. Wi-Fi
    WifiConfig wifi_config = {
        .ssid = "WAVEGO", .password = "12345678",
        .max_connections = 4, .tx_power = 40,
    };
    wifi_ = std::make_unique<WifiBridge>(wifi_config);
    if (!wifi_->start()) return false;

    // 3. HTTP
    http_ = std::make_unique<HttpApiServer>(82);
    if (!http_->start()) return false;
    
    // Note: The lambda captures 'this' and passes 'state' by reference automatically
    http_->setCommandCallback([this](CommandState& state) {
        onCommandReceived(state);
    });

    // 4. Emotion Engine
    emotion_ = std::make_unique<EmotionEngine>();

    // 5. LVGL
    lvgl_ = std::make_unique<LvglManager>(*lcd_);
    lvgl_->setUpdateCallback([this]() {
        updateUI();
        http_->updateDeadman(DEADMAN_TIMEOUT_MS);
    });
    if (!lvgl_->start()) return false;

    // 6. Initialize emotion UI
    emotion_->initialize();
    emotion_->showEmotion(Emotion::NEUTRAL);

    status_label_ = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_font(status_label_, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(status_label_, lv_color_white(), 0);
    lv_obj_align(status_label_, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_label_set_text(status_label_, last_command_text_);
    lv_obj_move_foreground(status_label_);

    // 7. Robot Motion
    motion_ = std::make_unique<RobotMotion>();
    if (!motion_->initialize()) {
        ESP_LOGW(TAG, "Motion init failed - running in display-only mode");
    }

    initialized_ = true;
    ESP_LOGI(TAG, "=== Robot Controller initialized ===");
    return true;
}

void RobotController::start() {
    if (motion_) {
        motion_->start();
    }
    ESP_LOGI(TAG, "Robot started. Connect to Wi-Fi 'WAVEGO'");
}

} // namespace dogrobot