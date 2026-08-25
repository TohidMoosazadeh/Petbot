#include "WifiBridge.hpp"
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <cstring>

namespace dogrobot {

static const char* TAG = "WifiBridge";

WifiBridge::WifiBridge(const WifiConfig& config) : config_(config) {}

WifiBridge::~WifiBridge() {
    if (started_) {
        esp_wifi_stop();
        esp_wifi_deinit();
    }
}

bool WifiBridge::start() {
    ESP_LOGI(TAG, "Starting Wi-Fi AP...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    // Initialize Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Configure AP
    wifi_config_t wifi_config = {};
    std::strncpy(reinterpret_cast<char*>(wifi_config.ap.ssid), config_.ssid.c_str(), 
            sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid_len = config_.ssid.length();
    std::strncpy(reinterpret_cast<char*>(wifi_config.ap.password), config_.password.c_str(), 
            sizeof(wifi_config.ap.password) - 1);
    wifi_config.ap.max_connection = config_.max_connections;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Reduce TX power to prevent brownout
    esp_wifi_set_max_tx_power(config_.tx_power);
    
    started_ = true;
    ESP_LOGI(TAG, "Wi-Fi AP started. SSID: %s", config_.ssid.c_str());
    return true;
}

} // namespace dogrobot