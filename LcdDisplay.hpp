#pragma once

#include <cstdint>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>

namespace dogrobot {

struct LcdConfig {
    // Hardware pins
    gpio_num_t mosi_pin;
    gpio_num_t clk_pin;
    gpio_num_t cs_pin;
    gpio_num_t dc_pin;
    gpio_num_t rst_pin;
    gpio_num_t bk_pin;
    
    // Display dimensions
    uint16_t width;
    uint16_t height;
    
    // Orientation settings
    bool swap_xy;
    bool mirror_x;
    bool mirror_y;
    uint16_t offset_x;
    uint16_t offset_y;
    
    // SPI settings
    uint32_t spi_freq_hz;
};

class LcdDisplay {
public:
    explicit LcdDisplay(const LcdConfig& config);
    ~LcdDisplay();
    
    // Non-copyable
    LcdDisplay(const LcdDisplay&) = delete;
    LcdDisplay& operator=(const LcdDisplay&) = delete;
    
    // Initialize the LCD hardware
    bool initialize();
    
    // Get the panel handle for LVGL
    esp_lcd_panel_handle_t getPanelHandle() const { return panel_handle_; }
    
    // Get offset values for LVGL flush callback
    uint16_t getOffsetX() const { return config_.offset_x; }
    uint16_t getOffsetY() const { return config_.offset_y; }
    
    // Control backlight
    void setBacklight(bool on);

private:
    LcdConfig config_;
    esp_lcd_panel_io_handle_t io_handle_ = nullptr;
    esp_lcd_panel_handle_t panel_handle_ = nullptr;
    bool initialized_ = false;
};

} // namespace dogrobot