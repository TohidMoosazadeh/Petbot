#pragma once

#include <cstdint>
extern "C" {
    #include "i2c_bus.h"
    #include "ina226.h"
    #include "mpu6050.h"
    #include "pca9685.h"
    #include "leg_trajectory.h"
    #include "leg_ik.h"
}
namespace dogrobot {

// Motion state shared with HTTP API
struct MotionState {
    volatile float roll = 0.0f;
    volatile float pitch = 0.0f;
    volatile float bus_voltage = 0.0f;
    volatile float current = 0.0f;
    volatile bool is_moving = false;
    volatile bool is_calibrated = false;
};

class RobotMotion {
public:
    RobotMotion();
    ~RobotMotion();

    // Non-copyable
    RobotMotion(const RobotMotion&) = delete;
    RobotMotion& operator=(const RobotMotion&) = delete;

    // Initialize all hardware (I2C, sensors, servos)
    bool initialize();

    // Start motion tasks (IMU, gait, power monitor)
    bool start();

    // Stop all motion
    void stop();

    // Get current motion state (thread-safe)
    MotionState getState() const;

    // ⭐ UPDATED: Now accepts bodyHeight parameter
    void setMovement(int fb, int lr, int sw_lr, int speed, int body_height);
    void setBodyHeight(int height_mm);   // ⭐ NEW: Dedicated setter
    int  getBodyHeight() const;          // ⭐ NEW: Getter
    void setAction(int actionId);
    void emergencyStop();

    // Calibration methods
    void applyServoTrim(int motorId, int offset);
    void resetImuReference();

private:
    // FreeRTOS task wrappers (must be static for C callbacks)
    static void imuTaskEntry(void* arg);
    static void gaitTaskEntry(void* arg);
    static void powerTaskEntry(void* arg);

    void imuTaskLoop();
    void gaitTaskLoop();
    void powerTaskLoop();

    // Hardware initialization helpers
    bool initI2C();
    bool initSensors();
    bool initServos();
    void homePose();
    void applyServoAngles(const leg_angles_t& A, const leg_angles_t& B, const leg_angles_t& C, const leg_angles_t& D);

    MotionState state_;
    volatile int cmd_fb_ = 0;
    volatile int cmd_lr_ = 0;
    volatile int cmd_sw_lr_ = 0;
    volatile int cmd_speed_ = 13;
    volatile int cmd_body_height_ = 95;   // ⭐ NEW: default 95mm
    volatile int cmd_action_ = 0;
    volatile bool running_ = false;

    // Servo trim offsets array for 16 channels
    int servo_trim_offsets_[16] = {0};
    static constexpr float LEG_HOME_Y_MM = -110.0f;
    static constexpr int GAIT_TRAJECTORY_POINTS = 30;
    static constexpr int GAIT_PERIOD_MS = 300;
    static constexpr int BODY_HEIGHT_DEFAULT_MM = 95;   // ⭐ NEW
    static constexpr int BODY_HEIGHT_MIN_MM = 80;       // ⭐ NEW
    static constexpr int BODY_HEIGHT_MAX_MM = 110;      // ⭐ NEW
};

} // namespace dogrobot
