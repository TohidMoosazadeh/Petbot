#include "EmotionEngine.hpp"
#include <cmath>
#include <esp_log.h>

namespace dogrobot {

static const char* TAG = "EmotionEngine";

EmotionEngine::EmotionEngine() {}

void EmotionEngine::initialize() {
    lv_obj_t* screen = lv_scr_act();
    
    // Set black background on screen
    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    
    // ⭐ Create a dedicated container for eyes
    eyes_container_ = lv_obj_create(screen);
    lv_obj_remove_style_all(eyes_container_);
    lv_obj_set_size(eyes_container_, LV_PCT(100), LV_PCT(100));
    lv_obj_set_pos(eyes_container_, 0, 0);
    lv_obj_set_style_bg_opa(eyes_container_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(eyes_container_, 0, 0);
    lv_obj_clear_flag(eyes_container_, LV_OBJ_FLAG_SCROLLABLE);
    
    ESP_LOGI(TAG, "EmotionEngine initialized with container");
}

lv_obj_t* EmotionEngine::makeShape(lv_obj_t* parent, int x, int y, int w, int h, 
                                    lv_color_t color, int radius) {
    lv_obj_t* shape = lv_obj_create(parent);
    lv_obj_remove_style_all(shape);
    lv_obj_set_pos(shape, x, y);
    lv_obj_set_size(shape, w, h);
    lv_obj_set_style_bg_color(shape, color, 0);
    lv_obj_set_style_bg_opa(shape, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(shape, radius, 0);
    lv_obj_clear_flag(shape, LV_OBJ_FLAG_SCROLLABLE);
    return shape;
}

void EmotionEngine::makeLine(lv_obj_t* parent, const lv_point_t* points, 
                              uint16_t point_count, lv_color_t color, uint16_t width) {
    lv_obj_t* line = lv_line_create(parent);
    lv_line_set_points(line, points, point_count);
    lv_obj_set_style_line_color(line, color, 0);
    lv_obj_set_style_line_width(line, width, 0);
    lv_obj_set_style_line_rounded(line, true, 0);
}

void EmotionEngine::makeNormalEyes(lv_obj_t* parent, bool sad) {
    const int y = sad ? 89 : 81;
    left_eye_ = makeShape(parent, 49, y, 86, 62, lv_color_hex(EYE_BLUE), 26);
    right_eye_ = makeShape(parent, 185, y, 86, 62, lv_color_hex(EYE_BLUE), 26);
    left_pupil_ = makeShape(parent, 80, y + 19, 24, 24, lv_color_black(), LV_RADIUS_CIRCLE);
    right_pupil_ = makeShape(parent, 216, y + 19, 24, 24, lv_color_black(), LV_RADIUS_CIRCLE);

    if (sad) {
        static const lv_point_t left_brow[] = {{55, 62}, {130, 92}};
        static const lv_point_t right_brow[] = {{190, 92}, {265, 62}};
        makeLine(parent, left_brow, 2, lv_color_hex(EYE_BLUE), 10);
        makeLine(parent, right_brow, 2, lv_color_hex(EYE_BLUE), 10);
        makeShape(parent, 72, 155, 16, 23, lv_color_hex(TEAR_BLUE), LV_RADIUS_CIRCLE);
    }
}

void EmotionEngine::makeHappyEyes(lv_obj_t* parent) {
    for (int x = 45; x <= 175; x += 130) {
        lv_obj_t* arc = lv_arc_create(parent);
        lv_obj_remove_style_all(arc);
        lv_obj_set_pos(arc, x, 55);
        lv_obj_set_size(arc, 100, 90);
        lv_arc_set_range(arc, 0, 180);
        lv_arc_set_value(arc, 140);
        lv_obj_set_style_arc_color(arc, lv_color_hex(EYE_BLUE), LV_PART_INDICATOR);
        lv_obj_set_style_arc_width(arc, 18, LV_PART_INDICATOR);
        lv_obj_set_style_arc_rounded(arc, true, LV_PART_INDICATOR);
        lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    }
    makeShape(parent, 70, 150, 30, 25, lv_color_hex(GLOW_BLUE), LV_RADIUS_CIRCLE);
    makeShape(parent, 220, 150, 30, 25, lv_color_hex(GLOW_BLUE), LV_RADIUS_CIRCLE);
}

void EmotionEngine::makeAngryEyes(lv_obj_t* parent) {
    static const lv_point_t left_eye_points[] = {{48, 88}, {142, 66}, {132, 132}, {55, 132}, {48, 88}};
    static const lv_point_t right_eye_points[] = {{178, 66}, {272, 88}, {265, 132}, {188, 132}, {178, 66}};
    static const lv_point_t left_brow[] = {{45, 55}, {145, 85}};
    static const lv_point_t right_brow[] = {{175, 85}, {275, 55}};

    makeLine(parent, left_eye_points, 5, lv_color_hex(EYE_BLUE), 18);
    makeLine(parent, right_eye_points, 5, lv_color_hex(EYE_BLUE), 18);
    makeShape(parent, 85, 98, 23, 23, lv_color_black(), LV_RADIUS_CIRCLE);
    makeShape(parent, 212, 98, 23, 23, lv_color_black(), LV_RADIUS_CIRCLE);
    makeLine(parent, left_brow, 2, lv_color_hex(EYE_BLUE), 12);
    makeLine(parent, right_brow, 2, lv_color_hex(EYE_BLUE), 12);
}

void EmotionEngine::makeSurprisedEyes(lv_obj_t* parent) {
    for (int x = 53; x <= 183; x += 130) {
        makeShape(parent, x, 70, 84, 84, lv_color_hex(EYE_BLUE), LV_RADIUS_CIRCLE);
        makeShape(parent, x + 26, 100, 32, 32, lv_color_black(), LV_RADIUS_CIRCLE);
    }
    lv_obj_t* mouth = makeShape(parent, 145, 168, 30, 37, lv_color_hex(EYE_BLUE), LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_color(mouth, lv_color_black(), 0);
    lv_obj_set_style_border_color(mouth, lv_color_hex(EYE_BLUE), 0);
    lv_obj_set_style_border_width(mouth, 8, 0);
}

void EmotionEngine::makeSleepyEyes(lv_obj_t* parent) {
    static const lv_point_t left_lid[] = {{50, 115}, {140, 115}};
    static const lv_point_t right_lid[] = {{180, 115}, {270, 115}};
    makeLine(parent, left_lid, 2, lv_color_hex(EYE_BLUE), 18);
    makeLine(parent, right_lid, 2, lv_color_hex(EYE_BLUE), 18);

    lv_obj_t* z1 = lv_label_create(parent);
    lv_label_set_text(z1, "Z");
    lv_obj_set_style_text_color(z1, lv_color_white(), 0);
    lv_obj_set_style_text_font(z1, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(z1, 235, 45);
    
    lv_obj_t* z2 = lv_label_create(parent);
    lv_label_set_text(z2, "z");
    lv_obj_set_style_text_color(z2, lv_color_white(), 0);
    lv_obj_set_style_text_font(z2, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(z2, 260, 25);
}

void EmotionEngine::showEmotion(Emotion emotion) {
    if (!eyes_container_) {
        ESP_LOGE(TAG, "EmotionEngine not initialized!");
        return;
    }
    
    // ⭐ CRITICAL: Only clean the eyes container, NOT the whole screen
    // This preserves the status_label and other UI elements
    lv_obj_clean(eyes_container_);
    
    left_eye_ = right_eye_ = left_pupil_ = right_pupil_ = nullptr;

    switch (emotion) {
        case Emotion::HAPPY:     makeHappyEyes(eyes_container_); break;
        case Emotion::SAD:       makeNormalEyes(eyes_container_, true); break;
        case Emotion::ANGRY:     makeAngryEyes(eyes_container_); break;
        case Emotion::SURPRISED: makeSurprisedEyes(eyes_container_); break;
        case Emotion::SLEEPY:    makeSleepyEyes(eyes_container_); break;
        case Emotion::NEUTRAL:
        default:                 makeNormalEyes(eyes_container_, false); break;
    }

    current_emotion_ = emotion;
    emotion_started_at_ = lv_tick_get();
    ESP_LOGI(TAG, "Emotion changed to: %d", static_cast<int>(emotion));
}

void EmotionEngine::updateAnimation() {
    if (left_eye_ == nullptr) return;

    uint32_t elapsed = lv_tick_elaps(emotion_started_at_);
    float t = elapsed / 1000.0f;
    int look_x = static_cast<int>(sinf(t * 1.25f) * 9.0f);
    int look_y = static_cast<int>(cosf(t * 1.65f) * 4.0f);
    int eye_y = (current_emotion_ == Emotion::SAD) ? 89 : 81;

    float blink_phase = fmodf(t, 3.2f);
    float blink = blink_phase < 0.18f ? sinf((blink_phase / 0.18f) * PI_F) : 0.0f;
    int eye_h = 62 - static_cast<int>(56.0f * blink);
    if (eye_h < 6) eye_h = 6;

    lv_obj_set_height(left_eye_, eye_h);
    lv_obj_set_height(right_eye_, eye_h);
    lv_obj_set_y(left_eye_, eye_y + (62 - eye_h) / 2);
    lv_obj_set_y(right_eye_, eye_y + (62 - eye_h) / 2);
    
    if (eye_h <= 15) {
        lv_obj_add_flag(left_pupil_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(right_pupil_, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_clear_flag(left_pupil_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(right_pupil_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(left_pupil_, 80 + look_x, eye_y + 19 + look_y);
        lv_obj_set_pos(right_pupil_, 216 + look_x, eye_y + 19 + look_y);
    }
}

} // namespace dogrobot