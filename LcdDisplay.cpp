#include "LcdDisplay.hpp"
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>

namespace dogrobot {

static const char* TAG = "LcdDisplay";

LcdDisplay::LcdDisplay(const LcdConfig& config) : config_(config) {}

LcdDisplay::~LcdDisplay() {
    if (panel_handle_) {
        esp_lcd_panel_del(panel_handle_);
    }
    if (io_handle_) {
        esp_lcd_panel_io_del(io_handle_);
    }
    spi_bus_free(SPI2_HOST);
}

bool LcdDisplay::initialize() {
    ESP_LOGI(TAG, "Initializing LCD %dx%d...", config_.width, config_.height);

    // Configure backlight GPIO
    gpio_set_direction(config_.bk_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(config_.bk_pin, 1);

    // Initialize SPI bus
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = config_.mosi_pin;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = config_.clk_pin;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = static_cast<int>(config_.width * config_.height * sizeof(uint16_t));
    
    if (spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init SPI bus");
        return false;
    }

    // Initialize LCD IO
    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.cs_gpio_num = config_.cs_pin;
    io_config.dc_gpio_num = config_.dc_pin;
    io_config.pclk_hz = config_.spi_freq_hz;
    io_config.trans_queue_depth = 10;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    
    if (esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, 
                                  &io_config, &io_handle_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init LCD IO");
        return false;
    }

    // Initialize LCD panel (ST7789)
    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = config_.rst_pin;
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_config.bits_per_pixel = 16;
    
    if (esp_lcd_new_panel_st7789(io_handle_, &panel_config, &panel_handle_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LCD panel");
        return false;
    }

    // Reset and initialize panel
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle_));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle_));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle_, config_.swap_xy));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle_, config_.mirror_x, config_.mirror_y));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle_, true));

    initialized_ = true;
    ESP_LOGI(TAG, "LCD initialized successfully");
    return true;
}

void LcdDisplay::setBacklight(bool on) {
    gpio_set_level(config_.bk_pin, on ? 1 : 0);
}

} // namespace dogrobot