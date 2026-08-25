#include "RobotMotion.hpp"

extern "C" {
    #include "i2c_bus.h"
    #include "ina226.h"
    #include "mpu6050.h"
    #include "pca9685.h"
    #include "leg_trajectory.h"
    #include "leg_ik.h"
}
#define DEG2RAD(d)  ((d) * (float)M_PI / 180.0f)

#include <cmath>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace dogrobot {

static const char* TAG = "RobotMotion";

static constexpr float DEG2RAD = 3.14159265358979323846f / 180.0f;

static const leg_ik_geometry_t ik_geom = {
    .l0 = 25.3f,
    .l1 = 40.0f,
    .l2 = 40.0f,
    .l3 = 50.0f,
    .l4 = 50.0f,
    .l5 = 50.0f,
};

// Helper function to apply trim offset
static inline float apply_trim(int channel, float angle_deg, const int* trim_offsets) {
    return angle_deg + static_cast<float>(trim_offsets[channel]);
}

RobotMotion::RobotMotion() {
    // Initialize trim offsets to 0
    for (int i = 0; i < 16; ++i) {
        servo_trim_offsets_[i] = 0;
    }
}

RobotMotion::~RobotMotion() { stop(); }


void RobotMotion::applyServoAngles(const leg_angles_t& A, const leg_angles_t& B, const leg_angles_t& C, const leg_angles_t& D) {
    // Leg A
    pca9685_servo_write(0,  A.theta1_deg - 2.0f);
    pca9685_servo_write(1,  A.theta2_deg - 0.0f);
    pca9685_servo_write(3,  A.theta3_deg + 90.0f);
    // Leg B
    pca9685_servo_write(5,  B.theta1_deg);
    pca9685_servo_write(4,  B.theta2_deg - 4.0f);
    pca9685_servo_write(7, -B.theta3_deg + 87.0f);
    // Leg C
    pca9685_servo_write(8,  180.0f - C.theta1_deg);
    pca9685_servo_write(9,  180.0f - C.theta2_deg);
    pca9685_servo_write(11, -C.theta3_deg + 83.0f);
    // Leg D
    pca9685_servo_write(13, 180.0f - D.theta1_deg + 5.0f);
    pca9685_servo_write(12, 180.0f - D.theta2_deg - 5.0f);
    pca9685_servo_write(15,  D.theta3_deg + 90.0f);
}

bool RobotMotion::initI2C() {
    ESP_LOGI(TAG, "Initializing I2C bus...");
    esp_err_t err = i2c_bus_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

bool RobotMotion::initSensors() {
    ESP_LOGI(TAG, "Initializing sensors...");

    esp_err_t err = ina226_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "INA226 init failed: %s (non-fatal)", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "INA226 initialized");
    }

    err = mpu6050_setup();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 setup failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "MPU6050 initialized");
    return true;
}

bool RobotMotion::initServos() {
    ESP_LOGI(TAG, "Initializing PCA9685...");
    esp_err_t err = pca9685_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 init failed: %s", esp_err_to_name(err));
        return false;
    }
    err = pca9685_set_pwm_freq(50.0f);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 set freq failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "PCA9685 initialized");
    return true;
}

void RobotMotion::applyServoTrim(int motorId, int offset) {
    if (motorId >= 0 && motorId < 16) {
        servo_trim_offsets_[motorId] = offset;
        ESP_LOGI(TAG, "Servo trim applied: motor %d, offset %d", motorId, offset);
    }
}

void RobotMotion::resetImuReference() {
    ESP_LOGI(TAG, "Resetting IMU reference (calibrating gyro)...");
    esp_err_t err = mpu6050_calibrate_gyro(500);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "IMU reference reset successfully");
    } else {
        ESP_LOGE(TAG, "Failed to reset IMU reference: %s", esp_err_to_name(err));
    }
}

void RobotMotion::homePose() {
    ESP_LOGI(TAG, "Moving to home pose...");
    
	float y_target = LEG_HOME_Y_MM;
    
    leg_angles_t home = leg_ik_compute(0.0f, y_target, 0.0f, &ik_geom);
    if (!home.valid) {
        ESP_LOGW(TAG, "Home pose IK invalid");
        return;
    }
    // Apply home angles to all 4 legs using the unified helper
    applyServoAngles(home, home, home, home);

}

bool RobotMotion::initialize() {
    ESP_LOGI(TAG, "=== Initializing Robot Motion ===");

    if (!initI2C()) return false;
    if (!initSensors()) return false;
    if (!initServos()) return false;

    // Move to home pose and wait for settling
    homePose();
    ESP_LOGI(TAG, "Waiting 5s for legs to settle...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Calibrate IMU while robot is still
    ESP_LOGI(TAG, "Calibrating IMU (keep robot still)...");
    esp_err_t err = mpu6050_calibrate_gyro(500);
    if (err != ESP_OK) ESP_LOGW(TAG, "Gyro cal failed: %s", esp_err_to_name(err));
    err = mpu6050_calibrate_accel(500);
    if (err != ESP_OK) ESP_LOGW(TAG, "Accel cal failed: %s", esp_err_to_name(err));

    state_.is_calibrated = true;
    ESP_LOGI(TAG, "=== Motion init complete ===");
    return true;
}

void RobotMotion::imuTaskEntry(void* arg) { static_cast<RobotMotion*>(arg)->imuTaskLoop(); }
void RobotMotion::gaitTaskEntry(void* arg) { static_cast<RobotMotion*>(arg)->gaitTaskLoop(); }
void RobotMotion::powerTaskEntry(void* arg) { static_cast<RobotMotion*>(arg)->powerTaskLoop(); }

// ================= IMU Task (100 Hz) =================

void RobotMotion::imuTaskLoop() {
    ESP_LOGI(TAG, "IMU task started (100 Hz)");
    while (running_) {
        if (mpu6050_update() == ESP_OK) {
            attitude_t att = mpu6050_get_attitude();
            state_.roll = att.roll;
            state_.pitch = att.pitch;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(nullptr);
}

// ================= Power Monitor Task (2 Hz) =================

void RobotMotion::powerTaskLoop() {
    ESP_LOGI(TAG, "Power monitor task started (2 Hz)");
    while (running_) {
        ina226_data_t data;
        if (ina226_read_all(&data) == ESP_OK) {
            state_.bus_voltage = data.bus_voltage;
            state_.current = data.current;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(nullptr);
}

// ⭐ UPDATED: Now applies body height offset to gait
void RobotMotion::gaitTaskLoop() {
    ESP_LOGI(TAG, "Gait task started (50 Hz)");

    int step = 0;
    static const leg_trajectory_config_t traj_cfg = {
        .step_length_mm   = LEG_TRAJ_DEFAULT_STEP_LENGTH_MM,
        .lift_height_mm   = LEG_TRAJ_DEFAULT_LIFT_HEIGHT_MM,
        .duty_factor      = LEG_TRAJ_DEFAULT_DUTY_FACTOR,
        .ground_offset_mm = LEG_TRAJ_DEFAULT_GROUND_OFFSET_MM,
    };

    // Pitch offset drift compensation variables
    static float pitch_offset = 0.0f;

    while (running_) {
        // Body rotation compensation
        float roll_deg = state_.roll;
        float pitch_deg = state_.pitch;

        if (pitch_deg > 1.5f) pitch_offset += 0.08f;
        else if (pitch_deg < -1.5f) pitch_offset -= 0.08f;

		float y_b_r = 0;
		//tan(roll_deg * DEG2RAD) * 79.0f;
		float y_b_p = 0;
		 //tan(pitch_deg * DEG2RAD) * 68.5f +
			//pitch_offset * (1.0f / cosf(pitch_deg * DEG2RAD));
        
        ESP_LOGI(TAG, "servo_trim_offsets_ = %d", servo_trim_offsets_[1]);
            
		// ⭐ NEW: Calculate body height offset
        float h_offset = static_cast<float>(cmd_body_height_ - BODY_HEIGHT_DEFAULT_MM);

        // Clamp offset to safe range
        if (h_offset > (BODY_HEIGHT_MAX_MM - BODY_HEIGHT_DEFAULT_MM)) {
            h_offset = BODY_HEIGHT_MAX_MM - BODY_HEIGHT_DEFAULT_MM;
        }
        if (h_offset < (BODY_HEIGHT_MIN_MM - BODY_HEIGHT_DEFAULT_MM)) {
            h_offset = BODY_HEIGHT_MIN_MM - BODY_HEIGHT_DEFAULT_MM;
        }
		//h_offset = 0.0;
		// Check if robot should be moving
        bool should_move = (cmd_fb_ != 0 || cmd_lr_ != 0 || cmd_sw_lr_ != 0 || cmd_action_ != 0);
        state_.is_moving = should_move;

        int step_increment = (cmd_speed_ > 13) ? 2 : 1;

        if (should_move) {
            // ⭐ Speed scaling: speed 1..25 maps to gait period
            // Higher speed = shorter period = faster gait
            // Default speed 13 → GAIT_PERIOD_MS (1000ms)
            // Speed 1 → 2000ms (slow), Speed 25 → 400ms (fast)
            int current_speed = cmd_speed_;
            if (current_speed < 1) current_speed = 1;
            if (current_speed > 25) current_speed = 25;
            ESP_LOGI(TAG, "cmd_speed_ = %d", cmd_speed_);

            float speed_factor = static_cast<float>(current_speed) / 13.0f;
            float scaled_period = static_cast<float>(GAIT_PERIOD_MS) / speed_factor;
            int tick_ms = static_cast<int>(scaled_period / GAIT_TRAJECTORY_POINTS);
            if (tick_ms < 5) tick_ms = 5;  // Minimum tick to avoid overload


            // Trot gait phases: A & C together, B & D together (0.5 offset)
            float phaseA = static_cast<float>(step) / GAIT_TRAJECTORY_POINTS;
            float phaseB = phaseA + 0.5f;
            float phaseC = phaseA + 0.0f;
            float phaseD = phaseA + 0.5f;

            if (phaseB >= 1.0f) phaseB -= 1.0f;
            if (phaseC >= 1.0f) phaseC -= 1.0f;
            if (phaseD >= 1.0f) phaseD -= 1.0f;

            float x, y;
            
            ESP_LOGI(TAG, "h_offset = %f", h_offset);

            leg_trajectory_compute(phaseA, &traj_cfg, &x, &y);
            float xA = x, yA = y, zA = 0.0;
			leg_trajectory_compute(phaseB, &traj_cfg, &x, &y);
			float xB = x, yB = y, zB = 0.0;
            leg_trajectory_compute(phaseC, &traj_cfg, &x, &y);
			float xC = x, yC = y, zC = 0.0;
			leg_trajectory_compute(phaseD, &traj_cfg, &x, &y);
			float xD = x, yD = y, zD = 0.0;

			// Movements**************************************************************************************************************
            if (cmd_fb_ != 0){  // Forward or Backward movement
			   // x direction of legs
				xA *= static_cast<float>(cmd_fb_);
    			xB *= static_cast<float>(cmd_fb_);
				xC *= static_cast<float>(cmd_fb_);
			    xD *= static_cast<float>(cmd_fb_);

                // y direction of legs
			    yA = yA - 0.0 - h_offset;
			    yB = yB - 14  - h_offset;
			    yC = yC - 10  - h_offset;
			    yD = yD - 2   - h_offset;

                // z direction of legs
			    zA =  0.0;
			    zB =  0.0;
			    zC =  0.0;
			    zD =  0.0;
			}
            else if (cmd_fb_ == 0 && cmd_lr_ != 0){   // Left or Rigth turn movement
			   // z direction of legs
				zA = -xA * static_cast<float>(cmd_lr_);
    			zB =  xB * static_cast<float>(cmd_lr_);
				zC =  xC * static_cast<float>(cmd_lr_);
			    zD = -xD * static_cast<float>(cmd_lr_);

                // y direction of legs
			    yA = yA - 0.0 - h_offset;
			    yB = yB - 14  - h_offset;
			    yC = yC - 10  - h_offset;
			    yD = yD - 2   - h_offset;

                // x direction of legs
				xA = 0.0;
    			xB = 0.0;
				xC = 0.0;
			    xD = 0.0;
            }                
            else if(cmd_fb_ == 0 && cmd_lr_ == 0 &&  cmd_sw_lr_!= 0){   // Left or Right sidewalk movement
               // z direction of legs
                zA = -xA * static_cast<float>(cmd_sw_lr_);
    			zB = -xB * static_cast<float>(cmd_sw_lr_);
				zC = -xC * static_cast<float>(cmd_sw_lr_);
			    zD = -xD * static_cast<float>(cmd_sw_lr_);

                // y direction of legs
			    yA = yA - 0.0 - h_offset;
			    yB = yB - 14  - h_offset;
			    yC = yC - 10  - h_offset;
			    yD = yD - 2   - h_offset;

                // x direction of legs
				xA = 0.0;
    			xB = 0.0;
				xC = 0.0;
			    xD = 0.0;
			}
            else{
               // z direction of legs
                zA = 0.0;
    			zB = 0.0;
				zC = 0.0;
			    zD = 0.0;

                // y direction of legs
			    yA = LEG_HOME_Y_MM - 0.0 - h_offset;
			    yB = LEG_HOME_Y_MM - 14  - h_offset;
			    yC = LEG_HOME_Y_MM - 10  - h_offset;
			    yD = LEG_HOME_Y_MM - 2   - h_offset;

                // x direction of legs
				xA = 0.0;
    			xB = 0.0;
				xC = 0.0;
			    xD = 0.0;
            }


            // Actions**************************************************************************************************************
            static float dY = 25.0f;
            if (cmd_action_ == 2)  // Stand Action
            {
			   // x direction of legs
               xA = 0.0;
			   xB = 0.0;
			   xC = 0.0;
			   xD = 0.0;

               // y direction of legs
			   yA = LEG_HOME_Y_MM - 0.0 - dY;
			   yB = LEG_HOME_Y_MM - 14  - dY;
			   yC = LEG_HOME_Y_MM - 10  - dY;
			   yD = LEG_HOME_Y_MM - 2   - dY;

               // z direction of legs
			   zA =  0.0;
			   zB =  0.0;
			   zC =  0.0;
			   zD =  0.0;
            }

            if (cmd_action_ == 12)  //Sit Action
            {
			   // x direction of legs
               xA = 0.0;
			   xB = 0.0;
			   xC = 0.0;
			   xD = 0.0;

               // y direction of legs
			   yA = LEG_HOME_Y_MM - 0.0 + dY;
			   yB = LEG_HOME_Y_MM - 14  + dY;
			   yC = LEG_HOME_Y_MM - 10  + dY;
			   yD = LEG_HOME_Y_MM - 2   + dY;

               // z direction of legs
			   zA =  0.0;
			   zB =  0.0;
			   zC =  0.0;
			   zD =  0.0;
            }
           if (cmd_action_ == 13)  //Hand shake Action
            {
			   // x direction of legs
               xA = 80.0;
			   xB = 0.0;
			   xC = 0.0;
			   xD = 0.0;

               // y direction of legs
			   yA = LEG_HOME_Y_MM - 20 + 70.0;
			   yB = LEG_HOME_Y_MM - 14 + 50.0;
			   yC = LEG_HOME_Y_MM - 10 + 50.0;
			   yD = LEG_HOME_Y_MM - 2  - 20.0;

               // z direction of legs
			   zA =  0.0;
			   zB = -70.0;
			   zC =  70.0;
			   zD =  0.0;
            }

           if (cmd_action_ == 14)  //Crouch Action
            {
			   // x direction of legs
               xA = 0.0;
			   xB = 0.0;
			   xC = 0.0;
			   xD = 0.0;

               // y direction of legs
			   yA = LEG_HOME_Y_MM - 0.0 - 20.0;
			   yB = LEG_HOME_Y_MM - 14  + 45.0;
			   yC = LEG_HOME_Y_MM - 10  + 45.0;
			   yD = LEG_HOME_Y_MM - 2   - 20.0;

               // z direction of legs
			   zA =  0.0;
			   zB = -70.0;
			   zC =  70.0;
			   zD =  0.0;
            }

            if (cmd_action_ == 17)  //Simple Dance Action
            {
			   // x direction of legs
               xA = 0.0;
			   xB = 0.0;
			   xC = 0.0;
			   xD = 0.0;

               // y direction of legs
               int64_t now_time1 = esp_timer_get_time();
               float rol1 = 20 * (sinf(DEG2RAD(now_time1 / (6000.0f * 1.0f))));
               float rol2 = 20 * (cosf(DEG2RAD(now_time1 / (6000.0f * 1.0f))));

			   yA = -100 - 0.0 + rol2;
			   yB = -100 - 14  + rol1;
			   yC = -100 - 10  + rol1;
			   yD = -100 - 2   + rol2;

               // z direction of legs
			   zA = 0.0;
			   zB = 0.0;
			   zC = 0.0;
			   zD = 0.0;
            }

            if (cmd_action_ == 19)  //Wave Dance Action
            {
			   // x direction of legs
               xA = 0.0;
			   xB = 0.0;
			   xC = 0.0;
			   xD = 0.0;

               // y direction of legs
                int64_t now_time1 = esp_timer_get_time();
                float rol1 = 15 * (sinf(DEG2RAD(now_time1 / (4000.0f * 1.0f))));
                float rol2 = 15 * (sinf(DEG2RAD((now_time1 / (4000.0f * 1.0f))+90)));
                float rol3 = 15 * (sinf(DEG2RAD((now_time1 / (4000.0f * 1.0f))+180)));
                float rol4 = 15 * (sinf(DEG2RAD((now_time1 / (4000.0f * 1.0f))+270)));
        

			   yA = -100 - 0.0 + rol1;
			   yB = -100 - 14  + rol2;
			   yC = -100 - 10  + rol3;
			   yD = -100 - 2   + rol4;

               // z direction of legs
			   zA = 0.0;
			   zB = 0.0;
			   zC = 0.0;
			   zD = 0.0;
            }

			// NOTE: Removed `+ LEG_HOME_Y_MM` from Y coordinates here. 
            // The trajectory generator outputs absolute coordinates based on the baseline. 
            // Adding it again would double the offset.
            // Specific fine-tuning offsets (+10, -4, 0, +8) restored from original C code.
            
     		leg_angles_t anglesA = leg_ik_compute(xA, yA ,zA, &ik_geom);
            leg_angles_t anglesB = leg_ik_compute(xB, yB ,zB, &ik_geom);
            leg_angles_t anglesC = leg_ik_compute(xC, yC ,zC, &ik_geom);
            leg_angles_t anglesD = leg_ik_compute(xD, yD ,zD, &ik_geom);
          
      
            if (anglesA.valid && anglesB.valid && anglesC.valid && anglesD.valid) {
                applyServoAngles(anglesA, anglesB, anglesC, anglesD);
            }
            step = (step + step_increment) % GAIT_TRAJECTORY_POINTS;
            vTaskDelay(pdMS_TO_TICKS(tick_ms));  // ⭐ Speed-adjusted delay

        } 
        else {
            // Standing: balance compensation + body height
            float y_target = LEG_HOME_Y_MM + y_b_r + y_b_p - h_offset;   // ⭐ Apply height
            leg_angles_t home = leg_ik_compute(0.0f, y_target, 0.0f, &ik_geom);
            
            if (home.valid) {
                applyServoAngles(home, home, home, home);
            }
            step = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz gait loop
        ESP_LOGI(TAG, "ActionEnd: %d", cmd_action_);
    }
    vTaskDelete(nullptr);
}

// ================= Public Control Methods =================

bool RobotMotion::start() {
    if (running_) return true;
    running_ = true;

    ESP_LOGI(TAG, "Starting motion tasks...");

    // ⭐ IMU task: 4096 bytes (100 Hz, خواندن MPU6050 + محاسبات attitude)
    if (xTaskCreatePinnedToCore(imuTaskEntry, "imu", 4096, this, 6, nullptr, 1) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task");
        return false;
    }

    // ⭐ Gait task: 6144 bytes (50 Hz, محاسبات سنگین trigonometry + IK)
    if (xTaskCreatePinnedToCore(gaitTaskEntry, "gait", 6144, this, 5, nullptr, 1) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create gait task");
        return false;
    }

    // ⭐ Power monitor task: 4096 bytes (INA226 + ESP_LOG)
    if (xTaskCreate(powerTaskEntry, "power", 4096, this, 3, nullptr) != pdPASS) {
        ESP_LOGW(TAG, "Failed to create power task (non-fatal)");
    }

    ESP_LOGI(TAG, "Motion tasks started");
    return true;
}

void RobotMotion::stop() {
    running_ = false;
    cmd_fb_ = 0;
    cmd_lr_ = 0;
    cmd_sw_lr_ = 0;
    cmd_action_ = 0;
    state_.is_moving = false;
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for tasks to exit
}

MotionState RobotMotion::getState() const { return state_; }

// ⭐ UPDATED: Now accepts bodyHeight
void RobotMotion::setMovement(int fb, int lr, int sw_lr, int speed, int body_height) {
    cmd_fb_ = fb;
    cmd_lr_ = lr;
    cmd_sw_lr_ = sw_lr;
    cmd_speed_ = speed;
    cmd_body_height_ = body_height;   // ⭐ NEW
    cmd_action_ = 0;
}

// ⭐ NEW: Dedicated setter for body height
void RobotMotion::setBodyHeight(int height_mm) {
    if (height_mm < BODY_HEIGHT_MIN_MM) height_mm = BODY_HEIGHT_MIN_MM;
    if (height_mm > BODY_HEIGHT_MAX_MM) height_mm = BODY_HEIGHT_MAX_MM;
    
    // A height change is a posture command, stop movement first
    cmd_fb_ = 0;
    cmd_lr_ = 0;
    cmd_sw_lr_ = 0;
    cmd_action_ = 0;
    cmd_body_height_ = height_mm;
    
    ESP_LOGI(TAG, "Body height set to %d mm", height_mm);
}

// ⭐ NEW: Getter for body height
int RobotMotion::getBodyHeight() const {
    return cmd_body_height_;
}

void RobotMotion::setAction(int actionId) {
    cmd_fb_ = 0;
    cmd_lr_ = 0;
    cmd_sw_lr_ = 0;
    cmd_action_ = actionId;
}

void RobotMotion::emergencyStop() {
    cmd_fb_ = 0;
    cmd_lr_ = 0;
    cmd_sw_lr_ = 0;
    cmd_action_ = 0;
    state_.is_moving = false;
    homePose();
    ESP_LOGW(TAG, "EMERGENCY STOP");
}

} // namespace dogrobot