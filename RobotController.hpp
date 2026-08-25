#pragma once
#include <memory>
#include "LcdDisplay.hpp"
#include "LvglManager.hpp"
#include "EmotionEngine.hpp"
#include "WifiBridge.hpp"
#include "HttpApiServer.hpp"
#include "RobotMotion.hpp"

namespace dogrobot {

class RobotController {
public:
    RobotController();
    ~RobotController();

    RobotController(const RobotController&) = delete;
    RobotController& operator=(const RobotController&) = delete;

    bool initialize();
    void start();

private:
    // Changed to non-const reference to allow flag resetting
    void onCommandReceived(CommandState& state);
    void updateUI();
    Emotion mapStateToEmotion(const CommandState& state);

    std::unique_ptr<LcdDisplay> lcd_;
    std::unique_ptr<LvglManager> lvgl_;
    std::unique_ptr<EmotionEngine> emotion_;
    std::unique_ptr<WifiBridge> wifi_;
    std::unique_ptr<HttpApiServer> http_;
    std::unique_ptr<RobotMotion> motion_;

    lv_obj_t* status_label_ = nullptr;
    char last_command_text_[64] = "Waiting...";
    bool initialized_ = false;

    static constexpr uint64_t DEADMAN_TIMEOUT_MS = 550;
};

} // namespace dogrobot