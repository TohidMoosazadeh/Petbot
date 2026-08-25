#pragma once

#include <lvgl.h>
#include <functional>
#include "LcdDisplay.hpp"

namespace dogrobot {

class LvglManager {
public:
    using UpdateCallback = std::function<void()>;
    
    explicit LvglManager(LcdDisplay& lcd);
    ~LvglManager();
    
    // Non-copyable
    LvglManager(const LvglManager&) = delete;
    LvglManager& operator=(const LvglManager&) = delete;
    
    // Initialize LVGL and start the render task
    bool start();
    
    // Set the update callback (called every frame)
    void setUpdateCallback(UpdateCallback callback);
    
    // Request UI update (thread-safe)
    void requestUpdate();

private:
    static void flushCallback(lv_disp_drv_t* disp_drv, 
                              const lv_area_t* area, 
                              lv_color_t* color_p);
    static void taskEntry(void* arg);
    void taskLoop();
    
    LcdDisplay& lcd_;
    lv_disp_draw_buf_t draw_buf_;
    lv_disp_drv_t disp_drv_;  // ⭐ ADDED: Must persist after start()
    lv_color_t* buf1_ = nullptr;
    lv_color_t* buf2_ = nullptr;
    UpdateCallback update_callback_;
    volatile bool update_requested_ = false;
    bool started_ = false;
    
    static constexpr uint16_t BUFFER_LINES = 30;
    static constexpr uint32_t FRAME_TIME_MS = 33;
};

} // namespace dogrobot