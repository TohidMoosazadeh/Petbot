#pragma once
#include <cstdint>
#include <functional>
#include <esp_http_server.h>

namespace dogrobot {

struct CommandState {
    int moveFB = 0;
    int moveLR = 0;
    int sideWalkLR = 0;      // side-walk direction (-1, 0, +1)
    int speed = 13;          // gait speed (1..25)
    int bodyHeight = 95;     // body height in mm (75..115), default 95
    int funcMode = 0;
    uint64_t lastMotionMs = 0;
    
    // Calibration fields
    int trimMotorId = 0;
    int trimOffset = 0;
    bool applyTrim = false;
    bool imuZero = false;
};

class HttpApiServer {
public:
    // Changed to non-const reference to allow flag resetting in the callback
    using CommandCallback = std::function<void(CommandState&)>;

    explicit HttpApiServer(uint16_t port = 82);
    ~HttpApiServer();

    HttpApiServer(const HttpApiServer&) = delete;
    HttpApiServer& operator=(const HttpApiServer&) = delete;

    bool start();
    void setCommandCallback(CommandCallback callback);
    void updateDeadman(uint64_t timeout_ms);

private:
    static void addCors(httpd_req_t* req);
    static esp_err_t sendJson(httpd_req_t* req, const char* json, const char* status = "200 OK");
    static bool readInt(httpd_req_t* req, const char* name, int* value);
    static int readOptionalInt(httpd_req_t* req, const char* name, int fallback);

    static esp_err_t pingHandler(httpd_req_t* req);
    static esp_err_t moveHandler(httpd_req_t* req);
    static esp_err_t stopHandler(httpd_req_t* req);
    static esp_err_t actionHandler(httpd_req_t* req);
    static esp_err_t bodyHandler(httpd_req_t* req);
    static esp_err_t servoTrimHandler(httpd_req_t* req);
    static esp_err_t imuZeroHandler(httpd_req_t* req);
    static esp_err_t statusHandler(httpd_req_t* req);

    uint16_t port_;
    httpd_handle_t server_ = nullptr;
    CommandCallback command_callback_;
    static CommandState state_;
};

} // namespace dogrobot