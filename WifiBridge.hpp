#pragma once

#include <cstdint>
#include <string>

namespace dogrobot {

struct WifiConfig {
    std::string ssid;
    std::string password;
    uint8_t max_connections;
    uint8_t tx_power;  // Reduced power to prevent brownout
};

class WifiBridge {
public:
    explicit WifiBridge(const WifiConfig& config);
    ~WifiBridge();
    
    // Non-copyable
    WifiBridge(const WifiBridge&) = delete;
    WifiBridge& operator=(const WifiBridge&) = delete;
    
    // Start the Wi-Fi AP
    bool start();

private:
    WifiConfig config_;
    bool started_ = false;
};

} // namespace dogrobot