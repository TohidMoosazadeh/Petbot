#include "HttpApiServer.hpp"
#include <esp_log.h>
#include <esp_timer.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>

namespace dogrobot {

static const char* TAG = "HttpApiServer";
CommandState HttpApiServer::state_ = {};

HttpApiServer::HttpApiServer(uint16_t port) : port_(port) {}

HttpApiServer::~HttpApiServer() {
    if (server_) {
        httpd_stop(server_);
    }
}

void HttpApiServer::addCors(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, OPTIONS");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
}

esp_err_t HttpApiServer::sendJson(httpd_req_t* req, const char* json, const char* status) {
    addCors(req);
    httpd_resp_set_status(req, status ? status : "200 OK");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, std::strlen(json));
}

bool HttpApiServer::readInt(httpd_req_t* req, const char* name, int* value) {
    char query[128] = {0};
    char rawValue[16] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;
    if (httpd_query_key_value(query, name, rawValue, sizeof(rawValue)) != ESP_OK) return false;
    *value = std::atoi(rawValue);
    return true;
}

int HttpApiServer::readOptionalInt(httpd_req_t* req, const char* name, int fallback) {
    int value = fallback;
    return readInt(req, name, &value) ? value : fallback;
}

esp_err_t HttpApiServer::pingHandler(httpd_req_t* req) {
    // API version bumped to 4 (calibration support)
    return sendJson(req, "{\"ok\":true,\"device\":\"wavego-esp32\",\"api\":4}");
}

esp_err_t HttpApiServer::moveHandler(httpd_req_t* req) {
    int fb = 0, lr = 0;
    const int swLr = readOptionalInt(req, "sw_lr", 0);
    const int speed = readOptionalInt(req, "speed", 13);
    const int height = readOptionalInt(req, "bodyHeight", 95);

    if (!readInt(req, "fb", &fb) || !readInt(req, "lr", &lr)) {
        return sendJson(req, "{\"ok\":false,\"error\":\"fb and lr are required\"}", "400 Bad Request");
    }
    if (fb < -1 || fb > 1 || lr < -1 || lr > 1 || swLr < -1 || swLr > 1) {
        return sendJson(req, "{\"ok\":false,\"error\":\"fb, lr, sw_lr range is -1..1\"}", "400 Bad Request");
    }
    if (speed < 1 || speed > 25) {
        return sendJson(req, "{\"ok\":false,\"error\":\"speed range is 1..25\"}", "400 Bad Request");
    }
    if (height < 75 || height > 115) {
        return sendJson(req, "{\"ok\":false,\"error\":\"bodyHeight range is 75..115\"}", "400 Bad Request");
    }

    state_.funcMode = 0;
    state_.moveFB = fb;
    state_.moveLR = lr;
    state_.sideWalkLR = swLr;
    state_.speed = speed;
    state_.bodyHeight = height;
    state_.lastMotionMs = esp_timer_get_time() / 1000;

    char resp[160];
    std::snprintf(resp, sizeof(resp),
        "{\"ok\":true,\"fb\":%d,\"lr\":%d,\"sw_lr\":%d,\"speed\":%d,\"bodyHeight\":%d}",
        state_.moveFB, state_.moveLR, state_.sideWalkLR, state_.speed, state_.bodyHeight);
    return sendJson(req, resp);
}

esp_err_t HttpApiServer::stopHandler(httpd_req_t* req) {
    state_.moveFB = 0;
    state_.moveLR = 0;
    state_.sideWalkLR = 0;
    state_.lastMotionMs = 0;
    return sendJson(req, "{\"ok\":true,\"stopped\":true}");
}

esp_err_t HttpApiServer::actionHandler(httpd_req_t* req) {
    int id = 0;
    if (!readInt(req, "id", &id) || id < 0 || id > 25) {
        return sendJson(req, "{\"ok\":false,\"error\":\"id must be 0..25\"}", "400 Bad Request");
    }
    state_.moveFB = 0;
    state_.moveLR = 0;
    state_.sideWalkLR = 0;
    state_.funcMode = id;
    state_.lastMotionMs = 0;

    char resp[64];
    std::snprintf(resp, sizeof(resp), "{\"ok\":true,\"funcMode\":%d}", id);
    return sendJson(req, resp);
}

esp_err_t HttpApiServer::bodyHandler(httpd_req_t* req) {
    int height = 0;
    if (!readInt(req, "height", &height)) {
        return sendJson(req, "{\"ok\":false,\"error\":\"height parameter required\"}", "400 Bad Request");
    }
    if (height < 75 || height > 115) {
        return sendJson(req, "{\"ok\":false,\"error\":\"height must be 75..115\"}", "400 Bad Request");
    }

    state_.moveFB = 0;
    state_.moveLR = 0;
    state_.sideWalkLR = 0;
    state_.funcMode = 0;
    state_.bodyHeight = height;
    state_.lastMotionMs = 0;

    char resp[64];
    std::snprintf(resp, sizeof(resp), "{\"ok\":true,\"height\":%d}", height);
    return sendJson(req, resp);
}

esp_err_t HttpApiServer::servoTrimHandler(httpd_req_t* req) {
    int motorId = 0;
    int offset = 0;
    if (!readInt(req, "motorId", &motorId) || !readInt(req, "offset", &offset)) {
        return sendJson(req, "{\"ok\":false,\"error\":\"motorId and offset are required\"}", "400 Bad Request");
    }
    if (offset < -15 || offset > 15) {
        return sendJson(req, "{\"ok\":false,\"error\":\"offset must be between -15 and 15\"}", "400 Bad Request");
    }

    state_.trimMotorId = motorId;
    state_.trimOffset = offset;
    state_.applyTrim = true;
    state_.lastMotionMs = esp_timer_get_time() / 1000; // Reset deadman to ensure processing

    char resp[128];
    std::snprintf(resp, sizeof(resp), "{\"ok\":true,\"motorId\":%d,\"offset\":%d}", motorId, offset);
    return sendJson(req, resp);
}

esp_err_t HttpApiServer::imuZeroHandler(httpd_req_t* req) {
    state_.imuZero = true;
    state_.lastMotionMs = esp_timer_get_time() / 1000;
    return sendJson(req, "{\"ok\":true,\"imuZero\":true}");
}

esp_err_t HttpApiServer::statusHandler(httpd_req_t* req) {
    char resp[256];
    std::snprintf(resp, sizeof(resp),
        "{\"ok\":true,\"uptime\":%llu,\"fb\":%d,\"lr\":%d,"
        "\"sw_lr\":%d,\"speed\":%d,\"bodyHeight\":%d,\"funcMode\":%d}",
        static_cast<unsigned long long>(esp_timer_get_time() / 1000),
        state_.moveFB, state_.moveLR,
        state_.sideWalkLR, state_.speed, state_.bodyHeight, state_.funcMode);
    return sendJson(req, resp);
}

bool HttpApiServer::start() {
    ESP_LOGI(TAG, "Starting HTTP server on port %d...", port_);

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = port_;
    config.ctrl_port = 32770;
    config.max_uri_handlers = 10;   // Increased to accommodate new calibration endpoints
    config.stack_size = 6144;
    config.lru_purge_enable = true;

    if (httpd_start(&server_, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return false;
    }

    httpd_uri_t uri_ping     = {"/api/ping",       HTTP_GET, pingHandler,     nullptr};
    httpd_uri_t uri_move     = {"/api/move",       HTTP_GET, moveHandler,     nullptr};
    httpd_uri_t uri_stop     = {"/api/stop",       HTTP_GET, stopHandler,     nullptr};
    httpd_uri_t uri_action   = {"/api/action",     HTTP_GET, actionHandler,   nullptr};
    httpd_uri_t uri_body     = {"/api/body",       HTTP_GET, bodyHandler,     nullptr};
    httpd_uri_t uri_servo    = {"/api/servo_trim", HTTP_GET, servoTrimHandler, nullptr};
    httpd_uri_t uri_imu      = {"/api/imu_zero",   HTTP_GET, imuZeroHandler,   nullptr};
    httpd_uri_t uri_status   = {"/api/status",     HTTP_GET, statusHandler,   nullptr};

    httpd_register_uri_handler(server_, &uri_ping);
    httpd_register_uri_handler(server_, &uri_move);
    httpd_register_uri_handler(server_, &uri_stop);
    httpd_register_uri_handler(server_, &uri_action);
    httpd_register_uri_handler(server_, &uri_body);
    httpd_register_uri_handler(server_, &uri_servo);
    httpd_register_uri_handler(server_, &uri_imu);
    httpd_register_uri_handler(server_, &uri_status);

    ESP_LOGI(TAG, "HTTP server started successfully");
    return true;
}

void HttpApiServer::setCommandCallback(CommandCallback callback) {
    command_callback_ = std::move(callback);
}

void HttpApiServer::updateDeadman(uint64_t timeout_ms) {
    if (state_.lastMotionMs > 0) {
        uint64_t now = esp_timer_get_time() / 1000;
        if (now - state_.lastMotionMs > timeout_ms) {
            state_.moveFB = 0;
            state_.moveLR = 0;
            state_.sideWalkLR = 0;
            state_.lastMotionMs = 0;
        }
    }

    if (command_callback_) {
        // Pass by non-const reference to allow the callback to reset flags
        command_callback_(state_); 
    }
}

} // namespace dogrobot