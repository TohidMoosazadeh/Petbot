#pragma once

#include <cstdint>
#include <lvgl.h>

namespace dogrobot {

enum class Emotion : uint8_t {
    NEUTRAL,
    HAPPY,
    SAD,
    ANGRY,
    SURPRISED,
    SLEEPY,
    COUNT
};

class EmotionEngine {
public:
    EmotionEngine();
    
    // Initialize the eyes container (call once after LVGL is ready)
    void initialize();
    
    // Show a specific emotion
    void showEmotion(Emotion emotion);
    
    // Update animation (call every frame)
    void updateAnimation();
    
    // Get current emotion
    Emotion getCurrentEmotion() const { return current_emotion_; }

private:
    void makeNormalEyes(lv_obj_t* parent, bool sad);
    void makeHappyEyes(lv_obj_t* parent);
    void makeAngryEyes(lv_obj_t* parent);
    void makeSurprisedEyes(lv_obj_t* parent);
    void makeSleepyEyes(lv_obj_t* parent);
    
    lv_obj_t* makeShape(lv_obj_t* parent, int x, int y, int w, int h, 
                        lv_color_t color, int radius);
    void makeLine(lv_obj_t* parent, const lv_point_t* points, 
                  uint16_t point_count, lv_color_t color, uint16_t width);
    
    // ⭐ NEW: Container for eyes (isolated from status label)
    lv_obj_t* eyes_container_ = nullptr;
    
    Emotion current_emotion_ = Emotion::NEUTRAL;
    uint32_t emotion_started_at_ = 0;
    lv_obj_t* left_eye_ = nullptr;
    lv_obj_t* right_eye_ = nullptr;
    lv_obj_t* left_pupil_ = nullptr;
    lv_obj_t* right_pupil_ = nullptr;
    
    static constexpr uint32_t EYE_BLUE = 0x00DCFF;
    static constexpr uint32_t GLOW_BLUE = 0x005078;
    static constexpr uint32_t TEAR_BLUE = 0x00A0FF;
    static constexpr float PI_F = 3.14159265358979323846f;
};

} // namespace dogrobot