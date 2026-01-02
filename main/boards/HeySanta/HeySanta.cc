#include "wifi_board.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
#include "esp32_camera.h"
#include "mcp_server.h"
#include "audio/codecs/santa_audio_codec.h"
#include "assets/lang_config.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include "esp_lcd_st7735.h"
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <wifi_station.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "emoji_display.h"
#include <string.h>
#include <cstdlib>
#include <cmath>

#define TAG "HeySanta"

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);

// ═══════════════════════════════════════════════════════
// SERVO PROTOCOL
// ═══════════════════════════════════════════════════════
#define SERVO_UART_NUM UART_NUM_1
#define SERVO_TX_PIN GPIO_NUM_10
#define SERVO_RX_PIN GPIO_NUM_11
#define SERVO_TXEN_PIN GPIO_NUM_3
#define SERVO_BAUD_RATE 1000000

#define STS_FRAME_HEADER 0xFF
#define STS_BROADCAST_ID 0xFE
#define STS_PING 0x01
#define STS_WRITE 0x03
#define STS_SYNC_WRITE 0x83
#define STS_TORQUE_ENABLE 0x28
#define STS_GOAL_POSITION_L 0x2A

#define SERVO_FL 2
#define SERVO_FR 1
#define SERVO_BL 4
#define SERVO_BR 3

// Neutral positions - ORIGINAL VALUES
#define NEUTRAL_FL 80
#define NEUTRAL_FR 80
#define NEUTRAL_BL 260
#define NEUTRAL_BR 260

// ═══════════════════════════════════════════════════════
// QMI8658A IMU CONFIG
// ═══════════════════════════════════════════════════════
#define IMU_SDA_PIN         GPIO_NUM_1
#define IMU_SCL_PIN         GPIO_NUM_2
#define IMU_I2C_FREQ_HZ     400000

#define IMU_I2C_ADDR_PRIMARY   0x6A
#define IMU_I2C_ADDR_SECONDARY 0x6B

#define IMU_CHIP_ID         0x05

// QMI8658A Registers
#define QMI_REG_CHIP_ID     0x00
#define QMI_REG_REVISION    0x01
#define QMI_REG_CTRL1       0x02
#define QMI_REG_CTRL2       0x03
#define QMI_REG_CTRL3       0x04
#define QMI_REG_CTRL4       0x05
#define QMI_REG_CTRL5       0x06
#define QMI_REG_CTRL6       0x07
#define QMI_REG_CTRL7       0x08
#define QMI_REG_CTRL8       0x09
#define QMI_REG_CTRL9       0x0A
#define QMI_REG_STATUS0     0x2E
#define QMI_REG_AX_L        0x35

// Gyro Balance Config
#define GYRO_BALANCE_ENABLED_DEFAULT    false
#define GYRO_BALANCE_MAX_CORRECTION     90.0f
#define GYRO_BALANCE_DEADZONE           0.5f
#define GYRO_BALANCE_GAIN               1.6f
#define GYRO_BALANCE_SMOOTHING          0.3f
#define GYRO_BALANCE_DECAY              0.98f
#define GYRO_BALANCE_UPDATE_INTERVAL_MS 50
#define IMU_UPDATE_INTERVAL_MS          50
#define GYRO_BALANCE_SPEED_MIN          150
#define GYRO_BALANCE_SPEED_MAX          2000
#define GYRO_BALANCE_SPEED_THRESHOLD    10.0f
#define GYRO_BALANCE_SPEED_CURVE        1.2f
#define GYRO_BALANCE_TOGGLE_THRESHOLD   150.0f
#define GYRO_BALANCE_TOGGLE_WINDOW_MS   1000
#define GYRO_BALANCE_TOGGLE_COOLDOWN_MS 1500

struct ImuData {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float accel_magnitude;
};

// ═══════════════════════════════════════════════════════
// GLOBAL CONVERSATION LOCK
// ═══════════════════════════════════════════════════════
static volatile bool g_conversation_active = false;

void SetConversationActive(bool active) {
    g_conversation_active = active;
    if (active) {
        ESP_LOGI(TAG, "🔒 CONVERSATION ACTIVE - IMU balance paused");
    } else {
        ESP_LOGI(TAG, "🔓 CONVERSATION ENDED - IMU balance resumed");
    }
}

bool IsConversationActive() {
    return g_conversation_active;
}

// ═══════════════════════════════════════════════════════
// IMU CONTROLLER
// ═══════════════════════════════════════════════════════
class ImuController {
private:
    bool initialized_ = false;
    uint8_t detected_address_ = 0;
    i2c_master_dev_handle_t imu_device_ = nullptr;
    
    esp_err_t WriteReg(uint8_t reg, uint8_t value) {
        if (!imu_device_) return ESP_ERR_INVALID_STATE;
        uint8_t buf[2] = {reg, value};
        return i2c_master_transmit(imu_device_, buf, 2, 1000);
    }
    
    esp_err_t ReadReg(uint8_t reg, uint8_t* value) {
        if (!imu_device_) return ESP_ERR_INVALID_STATE;
        return i2c_master_transmit_receive(imu_device_, &reg, 1, value, 1, 1000);
    }
    
    esp_err_t ReadRegs(uint8_t reg, uint8_t* buf, size_t len) {
        if (!imu_device_) return ESP_ERR_INVALID_STATE;
        return i2c_master_transmit_receive(imu_device_, &reg, 1, buf, len, 1000);
    }
    
    float AccelToMs2(int16_t raw) {
        return (raw / 4096.0f) * 9.81f;
    }
    
    float GyroToDps(int16_t raw) {
        return raw / 64.0f;
    }

public:
    bool Initialize(i2c_master_bus_handle_t i2c_bus) {
        ESP_LOGI(TAG, "=== QMI8658A IMU Initialization ===");
        
        if (!i2c_bus) {
            ESP_LOGE(TAG, "I2C bus handle is null");
            return false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
        
        uint8_t addresses[] = {IMU_I2C_ADDR_PRIMARY, IMU_I2C_ADDR_SECONDARY};
        bool found = false;
        
        for (int i = 0; i < 2; i++) {
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addresses[i],
                .scl_speed_hz = IMU_I2C_FREQ_HZ,
            };
            
            i2c_master_dev_handle_t temp_dev;
            if (i2c_master_bus_add_device(i2c_bus, &dev_cfg, &temp_dev) == ESP_OK) {
                uint8_t chip_id;
                uint8_t reg = QMI_REG_CHIP_ID;
                if (i2c_master_transmit_receive(temp_dev, &reg, 1, &chip_id, 1, 1000) == ESP_OK) {
                    if (chip_id == IMU_CHIP_ID) {
                        ESP_LOGI(TAG, "✓ IMU found at address 0x%02X (chip ID: 0x%02X)", addresses[i], chip_id);
                        imu_device_ = temp_dev;
                        detected_address_ = addresses[i];
                        found = true;
                        break;
                    }
                }
                i2c_master_bus_rm_device(temp_dev);
            }
        }
        
        if (!found) {
            ESP_LOGE(TAG, "✗ IMU not found at 0x6A or 0x6B");
            return false;
        }
        
        WriteReg(QMI_REG_CTRL1, 0x80);
        vTaskDelay(pdMS_TO_TICKS(50));
        
        WriteReg(QMI_REG_CTRL1, 0x40);
        WriteReg(QMI_REG_CTRL2, 0x24);
        WriteReg(QMI_REG_CTRL3, 0x54);
        WriteReg(QMI_REG_CTRL7, 0x03);
        WriteReg(QMI_REG_CTRL9, 0x00);
        
        vTaskDelay(pdMS_TO_TICKS(100));
        
        uint8_t status;
        if (ReadReg(QMI_REG_STATUS0, &status) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read status register");
            return false;
        }
        
        ESP_LOGI(TAG, "✓ IMU initialized (status: 0x%02X)", status);
        ESP_LOGI(TAG, "  Config: ±8g accel, ±512dps gyro, 500Hz ODR");
        
        initialized_ = true;
        return true;
    }
    
    bool Read(ImuData* data) {
        if (!initialized_) return false;
        
        uint8_t buf[12];
        if (ReadRegs(QMI_REG_AX_L, buf, 12) != ESP_OK) {
            return false;
        }
        
        int16_t raw_ax = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t raw_ay = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t raw_az = (int16_t)((buf[5] << 8) | buf[4]);
        int16_t raw_gx = (int16_t)((buf[7] << 8) | buf[6]);
        int16_t raw_gy = (int16_t)((buf[9] << 8) | buf[8]);
        int16_t raw_gz = (int16_t)((buf[11] << 8) | buf[10]);
        
        data->accel_x = AccelToMs2(raw_ax);
        data->accel_y = AccelToMs2(raw_ay);
        data->accel_z = AccelToMs2(raw_az);
        data->gyro_x = GyroToDps(raw_gx);
        data->gyro_y = GyroToDps(raw_gy);
        data->gyro_z = GyroToDps(raw_gz);
        data->accel_magnitude = sqrtf(data->accel_x * data->accel_x + 
                                       data->accel_y * data->accel_y + 
                                       data->accel_z * data->accel_z);
        return true;
    }
    
    bool IsInitialized() const { return initialized_; }
    uint8_t GetAddress() const { return detected_address_; }
};

// ═══════════════════════════════════════════════════════
// GYRO BALANCE CONTROLLER
// ═══════════════════════════════════════════════════════
class GyroBalanceController {
private:
    bool enabled_ = false;
    float accumulated_angle_ = 0.0f;
    float last_correction_ = 0.0f;
    uint32_t last_update_time_ = 0;
    
    enum ToggleState { IDLE, FIRST_ROTATION, WAITING_REVERSE };
    ToggleState toggle_state_ = IDLE;
    bool first_rotation_positive_ = false;
    uint32_t first_rotation_time_ = 0;
    uint32_t last_toggle_time_ = 0;
    
    uint16_t CalculateDynamicSpeed(float angle_delta) {
        float abs_delta = fabsf(angle_delta);
        if (abs_delta < 0.5f) return GYRO_BALANCE_SPEED_MIN;
        
        float normalized = abs_delta / GYRO_BALANCE_SPEED_THRESHOLD;
        if (normalized > 1.0f) normalized = 1.0f;
        
        float curved = powf(normalized, GYRO_BALANCE_SPEED_CURVE);
        uint16_t speed = GYRO_BALANCE_SPEED_MIN + 
                        (uint16_t)(curved * (GYRO_BALANCE_SPEED_MAX - GYRO_BALANCE_SPEED_MIN));
        
        if (speed > GYRO_BALANCE_SPEED_MAX) speed = GYRO_BALANCE_SPEED_MAX;
        return speed;
    }
    
public:
    bool ProcessToggle(float gyro_x) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        if ((now - last_toggle_time_) < GYRO_BALANCE_TOGGLE_COOLDOWN_MS) {
            return false;
        }
        
        bool toggled = false;
        
        switch (toggle_state_) {
            case IDLE:
                if (fabsf(gyro_x) > GYRO_BALANCE_TOGGLE_THRESHOLD) {
                    toggle_state_ = FIRST_ROTATION;
                    first_rotation_positive_ = (gyro_x > 0);
                    first_rotation_time_ = now;
                }
                break;
                
            case FIRST_ROTATION:
                if ((now - first_rotation_time_) > GYRO_BALANCE_TOGGLE_WINDOW_MS) {
                    toggle_state_ = IDLE;
                } else if (fabsf(gyro_x) < GYRO_BALANCE_TOGGLE_THRESHOLD * 0.5f) {
                    toggle_state_ = WAITING_REVERSE;
                }
                break;
                
            case WAITING_REVERSE:
                if ((now - first_rotation_time_) > GYRO_BALANCE_TOGGLE_WINDOW_MS) {
                    toggle_state_ = IDLE;
                } else if (fabsf(gyro_x) > GYRO_BALANCE_TOGGLE_THRESHOLD) {
                    bool current_positive = (gyro_x > 0);
                    if (current_positive != first_rotation_positive_) {
                        enabled_ = !enabled_;
                        toggled = true;
                        last_toggle_time_ = now;
                        toggle_state_ = IDLE;
                        
                        accumulated_angle_ = 0.0f;
                        last_correction_ = 0.0f;
                        
                        ESP_LOGI("GyroBalance", "TOGGLE! Balance mode: %s", 
                                 enabled_ ? "ENABLED" : "DISABLED");
                    } else {
                        toggle_state_ = IDLE;
                    }
                }
                break;
        }
        
        return toggled;
    }
    
    struct BalanceResult {
        float front_offset;
        float back_offset;
        uint16_t speed;
    };
    
    BalanceResult Calculate(float gyro_y, float dt_sec) {
        BalanceResult result = {0.0f, 0.0f, GYRO_BALANCE_SPEED_MIN};
        
        if (!enabled_) return result;
        
        float filtered_gyro = gyro_y;
        if (fabsf(filtered_gyro) < GYRO_BALANCE_DEADZONE) {
            filtered_gyro = 0.0f;
        }
        
        float raw_integration = filtered_gyro * dt_sec;
        accumulated_angle_ += raw_integration * GYRO_BALANCE_SMOOTHING;
        accumulated_angle_ *= GYRO_BALANCE_DECAY;
        
        float correction = accumulated_angle_ * GYRO_BALANCE_GAIN;
        
        if (correction > GYRO_BALANCE_MAX_CORRECTION) correction = GYRO_BALANCE_MAX_CORRECTION;
        if (correction < -GYRO_BALANCE_MAX_CORRECTION) correction = -GYRO_BALANCE_MAX_CORRECTION;
        
        float correction_delta = correction - last_correction_;
        result.speed = CalculateDynamicSpeed(correction_delta);
        last_correction_ = correction;
        
        result.front_offset = correction;
        result.back_offset = -correction;
        
        return result;
    }
    
    void Enable(bool enable) { 
        enabled_ = enable;
        if (!enable) {
            accumulated_angle_ = 0.0f;
            last_correction_ = 0.0f;
        }
    }
    bool IsEnabled() const { return enabled_; }
    void Reset() { 
        accumulated_angle_ = 0.0f; 
        last_correction_ = 0.0f;
    }
};

// ═══════════════════════════════════════════════════════
// SERVO CONTROLLER - ORIGINAL MOVEMENTS
// ═══════════════════════════════════════════════════════
class ServoController {
private:
    int global_servo_speed = 4095;
    bool is_flipped = false;
    
    struct ServoState {
        float last_unwrapped_hw;
    };
    
    ServoState servo_states[4] = {
        {NEUTRAL_FL},
        {360.0f - NEUTRAL_FR},
        {NEUTRAL_BL},
        {360.0f - NEUTRAL_BR}
    };
    
    struct {
        float fl, fr, bl, br;
    } continuous_pos = {NEUTRAL_FL, NEUTRAL_FR, NEUTRAL_BL, NEUTRAL_BR};

    static inline float wrap_0_360(float a) {
        float r = fmodf(a, 360.0f);
        if (r < 0) r += 360.0f;
        return r;
    }

    static inline uint16_t angle_to_position(float angle_0_360) {
        if (angle_0_360 < 0) angle_0_360 = wrap_0_360(angle_0_360);
        if (angle_0_360 >= 360.0f) angle_0_360 = wrap_0_360(angle_0_360);
        return (uint16_t)lroundf(angle_0_360 * 4095.0f / 360.0f);
    }

    static inline float unwrap_to_nearest(float current, float target) {
        float k = roundf((current - target) / 360.0f);
        return target + 360.0f * k;
    }

    float map_target_to_hw_unwrapped(float target_continuous, bool reverse, float last_unwrapped_hw) {
        float t = target_continuous;
        if (reverse) t = 360.0f - t;
        return unwrap_to_nearest(last_unwrapped_hw, t);
    }

    static uint8_t sts_checksum(uint8_t *buf, int len) {
        uint8_t sum = 0;
        for (int i = 2; i < len - 1; i++) sum += buf[i];
        return (uint8_t)(~sum);
    }

    void sts_send_packet(uint8_t id, uint8_t cmd, uint8_t *params, int param_len) {
        uint8_t packet[128];
        packet[0] = STS_FRAME_HEADER;
        packet[1] = STS_FRAME_HEADER;
        packet[2] = id;
        packet[3] = param_len + 2;
        packet[4] = cmd;
        if (params && param_len > 0) memcpy(&packet[5], params, param_len);
        
        int total_len = 5 + param_len + 1;
        packet[total_len - 1] = sts_checksum(packet, total_len);

        gpio_set_level(SERVO_TXEN_PIN, 1);
        uart_flush(SERVO_UART_NUM);
        uart_write_bytes(SERVO_UART_NUM, packet, total_len);
        uart_wait_tx_done(SERVO_UART_NUM, pdMS_TO_TICKS(50));
        gpio_set_level(SERVO_TXEN_PIN, 0);
    }

    void sts_enable_torque(uint8_t id, bool enable) {
        uint8_t params[2];
        params[0] = STS_TORQUE_ENABLE;
        params[1] = enable ? 1 : 0;
        sts_send_packet(id, STS_WRITE, params, 2);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    void sts_sync_write_positions(uint8_t *ids, float *angles_0_360, uint8_t num_servos, uint16_t speed) {
        uint8_t params[128];
        params[0] = STS_GOAL_POSITION_L;
        params[1] = 6;

        int idx = 2;
        for (int i = 0; i < num_servos; i++) {
            float a = wrap_0_360(angles_0_360[i]);
            uint16_t position = angle_to_position(a);

            params[idx++] = ids[i];
            params[idx++] = position & 0xFF;
            params[idx++] = (position >> 8) & 0xFF;
            params[idx++] = 0x00;
            params[idx++] = 0x00;
            params[idx++] = speed & 0xFF;
            params[idx++] = (speed >> 8) & 0xFF;
        }

        sts_send_packet(STS_BROADCAST_ID, STS_SYNC_WRITE, params, idx);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    void servo_write_all(float fl, float fr, float bl, float br, uint16_t speed) {
        uint8_t ids[] = {SERVO_FL, SERVO_FR, SERVO_BL, SERVO_BR};

        continuous_pos.fl = fl;
        continuous_pos.fr = fr;
        continuous_pos.bl = bl;
        continuous_pos.br = br;

        float unwrapped_hw[4];
        unwrapped_hw[0] = map_target_to_hw_unwrapped(fl, false, servo_states[0].last_unwrapped_hw);
        unwrapped_hw[1] = map_target_to_hw_unwrapped(fr, true,  servo_states[1].last_unwrapped_hw);
        unwrapped_hw[2] = map_target_to_hw_unwrapped(bl, false, servo_states[2].last_unwrapped_hw);
        unwrapped_hw[3] = map_target_to_hw_unwrapped(br, true,  servo_states[3].last_unwrapped_hw);

        for (int i = 0; i < 4; i++) servo_states[i].last_unwrapped_hw = unwrapped_hw[i];

        float hw_wrapped[4] = {
            wrap_0_360(unwrapped_hw[0]),
            wrap_0_360(unwrapped_hw[1]),
            wrap_0_360(unwrapped_hw[2]),
            wrap_0_360(unwrapped_hw[3]),
        };

        sts_sync_write_positions(ids, hw_wrapped, 4, speed);
    }

public:
    void Initialize() {
        ESP_LOGI(TAG, "Initializing servo controller...");
        
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << SERVO_TXEN_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(SERVO_TXEN_PIN, 0);

        uart_config_t uart_config = {
            .baud_rate = SERVO_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };

        ESP_ERROR_CHECK(uart_driver_install(SERVO_UART_NUM, 2048, 2048, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(SERVO_UART_NUM, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(SERVO_UART_NUM, SERVO_TX_PIN, SERVO_RX_PIN,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_LOGI(TAG, "Enabling servo torque...");
        for (uint8_t id = 1; id <= 4; id++) {
            sts_enable_torque(id, true);
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        MoveInit();
    }

    void MoveInit() {
        ESP_LOGI(TAG, "Dog: Moving to neutral position");
        continuous_pos.fl = NEUTRAL_FL;
        continuous_pos.fr = NEUTRAL_FR;
        continuous_pos.bl = NEUTRAL_BL;
        continuous_pos.br = NEUTRAL_BR;

        servo_states[0].last_unwrapped_hw = NEUTRAL_FL;
        servo_states[1].last_unwrapped_hw = 360.0f - NEUTRAL_FR;
        servo_states[2].last_unwrapped_hw = NEUTRAL_BL;
        servo_states[3].last_unwrapped_hw = 360.0f - NEUTRAL_BR;
        
        servo_write_all(NEUTRAL_FL, NEUTRAL_FR, NEUTRAL_BL, NEUTRAL_BR, (uint16_t)global_servo_speed);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    void MoveReset() {
        ESP_LOGI(TAG, "Dog: Resetting to neutral");
        SetConversationActive(true);  // **BLOCK IMU**
        servo_write_all(NEUTRAL_FL, NEUTRAL_FR, NEUTRAL_BL, NEUTRAL_BR, (uint16_t)global_servo_speed);
        is_flipped = false;
        vTaskDelay(pdMS_TO_TICKS(1000));
        SetConversationActive(false);  // **UNBLOCK IMU**
    }

    void ApplyBalance(float front_offset, float back_offset, uint16_t speed) {
        if (IsConversationActive()) return;  // **CHECK CONVERSATION LOCK**
        
        float fl = NEUTRAL_FL + front_offset;
        float fr = NEUTRAL_FR + front_offset;
        float bl = NEUTRAL_BL - back_offset;
        float br = NEUTRAL_BR - back_offset;

        servo_write_all(fl, fr, bl, br, speed);
    }

    // ═══════════════════════════════════════════════════════
    // ORIGINAL WORKING MOVEMENTS
    // ═══════════════════════════════════════════════════════
    void WalkForward(int loops = 6) {
        ESP_LOGI(TAG, "Dog: >>> WALKING FORWARD <<<");
        SetConversationActive(true);  // **BLOCK IMU**
        
        int FL_angles[] = {65, 35, 100, 65};
        int FR_angles[] = {65, 95, 65, 40};
        int BL_angles[] = {210, 245, 210, 175};
        int BR_angles[] = {210, 180, 240, 210};
        uint16_t delays_ms[] = {175, 135, 135, 135};

        for (int loop = 0; loop < loops; loop++) {
            for (int i = 0; i < 4; i++) {
                servo_write_all(FL_angles[i], FR_angles[i], BL_angles[i], BR_angles[i], (uint16_t)global_servo_speed);
                vTaskDelay(delays_ms[i] / portTICK_PERIOD_MS);
            }
        }
        MoveReset();
        SetConversationActive(false);  // **UNBLOCK IMU**
    }

    void Dance() {
        ESP_LOGI(TAG, "Dog: >>> DANCING <<<");
        SetConversationActive(true);  // **BLOCK IMU**
        
        int FL_dance[] = {65,50,35,20,35,50,65,45,25,5,25,45,65,40,65,30,65,40,65,30,15,30,65};
        int FR_dance[] = {65,80,95,110,95,80,65,85,105,125,105,85,65,90,65,100,65,90,65,100,115,100,65};
        int BL_dance[] = {210,225,240,255,240,225,210,230,250,265,250,230,210,235,210,245,210,235,210,245,260,245,210};
        int BR_dance[] = {210,195,180,165,180,195,210,190,170,155,170,190,210,185,210,175,210,185,210,175,160,175,210};
        int num_steps = 23;

        for (int loop = 0; loop < 4; loop++) {
            for (int i = 0; i < num_steps; i++) {
                servo_write_all(FL_dance[i], FR_dance[i], BL_dance[i], BR_dance[i], (uint16_t)global_servo_speed);
                vTaskDelay(40 / portTICK_PERIOD_MS);
            }
        }
        MoveReset();
        SetConversationActive(false);  // **UNBLOCK IMU**
    }

    void Flip() {
        ESP_LOGI(TAG, "Dog: >>> FLIP <<<");
        SetConversationActive(true);  // **BLOCK IMU**
        
        if (!is_flipped) {
            int FL_angles[] = {210, 185, 210, 355};
            int FR_angles[] = {210, 185, 210, 355};
            int BL_angles[] = {165, 185, 205, 20};
            int BR_angles[] = {165, 185, 205, 20};
            uint16_t delays_ms[] = {50, 200, 150, 200};

            for (int i = 0; i < 4; i++) {
                servo_write_all(FL_angles[i], FR_angles[i], BL_angles[i], BR_angles[i], (uint16_t)global_servo_speed);
                vTaskDelay(delays_ms[i] / portTICK_PERIOD_MS);
            }
            is_flipped = true;
        } else {
            int FL_angles[] = {355, 210, 185, NEUTRAL_FL};
            int FR_angles[] = {355, 210, 185, NEUTRAL_FR};
            int BL_angles[] = {20, 205, 185, NEUTRAL_BL};
            int BR_angles[] = {20, 205, 185, NEUTRAL_BR};
            uint16_t delays_ms[] = {200, 150, 200, 50};

            for (int i = 0; i < 4; i++) {
                servo_write_all(FL_angles[i], FR_angles[i], BL_angles[i], BR_angles[i], (uint16_t)global_servo_speed);
                vTaskDelay(delays_ms[i] / portTICK_PERIOD_MS);
            }
            is_flipped = false;
        }
        
        SetConversationActive(false);  // **UNBLOCK IMU**
    }

    void ShakeHand() {
        ESP_LOGI(TAG, "Dog: >>> SHAKE HAND <<<");
        SetConversationActive(true);  // **BLOCK IMU**
        
        int FL_angles[] = {NEUTRAL_FL, 260, 300, 260, 300, 260, 300, 260, 300, 260, 300, 260, NEUTRAL_FL};
        int FR_angles[] = {NEUTRAL_FR, NEUTRAL_FR, NEUTRAL_FR, NEUTRAL_FR, NEUTRAL_FR, NEUTRAL_FR, NEUTRAL_FR, NEUTRAL_FR, NEUTRAL_FR, NEUTRAL_FR, NEUTRAL_FR, NEUTRAL_FR, NEUTRAL_FR};
        int BL_angles[] = {NEUTRAL_BL, NEUTRAL_BL, NEUTRAL_BL, NEUTRAL_BL, NEUTRAL_BL, NEUTRAL_BL, NEUTRAL_BL, NEUTRAL_BL, NEUTRAL_BL, NEUTRAL_BL, NEUTRAL_BL, NEUTRAL_BL, NEUTRAL_BL};
        int BR_angles[] = {NEUTRAL_BR, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, NEUTRAL_BR};
        uint16_t delays_ms[] = {50, 100, 80, 80, 80, 80, 80, 80, 80, 80, 80, 100, 100};

        for (int i = 0; i < 13; i++) {
            servo_write_all(FL_angles[i], FR_angles[i], BL_angles[i], BR_angles[i], (uint16_t)global_servo_speed);
            vTaskDelay(delays_ms[i] / portTICK_PERIOD_MS);
        }
        MoveReset();
        SetConversationActive(false);  // **UNBLOCK IMU**
    }

    void Pounce() {
        ESP_LOGI(TAG, "Dog: >>> POUNCE <<<");
        SetConversationActive(true);  // **BLOCK IMU**
        
        int FL_angles[] = {210, 210, 170, NEUTRAL_FL};
        int FR_angles[] = {165, 75, 170, NEUTRAL_FR};
        int BL_angles[] = {210, 210, 230, NEUTRAL_BL};
        int BR_angles[] = {165, 235, 230, NEUTRAL_BR};
        uint16_t delays_ms[] = {300, 120, 150, 200};

        for (int i = 0; i < 4; i++) {
            servo_write_all(FL_angles[i], FR_angles[i], BL_angles[i], BR_angles[i], (uint16_t)global_servo_speed);
            vTaskDelay(delays_ms[i] / portTICK_PERIOD_MS);
        }
        MoveReset();
        SetConversationActive(false);  // **UNBLOCK IMU**
    }

    void SideFlip() {
        ESP_LOGI(TAG, "Dog: >>> SIDE FLIP <<<");
        SetConversationActive(true);  // **BLOCK IMU**
        
        if (!is_flipped) {
            int FL_angles[] = {65, 210, 220, 165, 210};
            int FR_angles[] = {65, 210, 90, 210, 210};
            int BL_angles[] = {210, 65, 225, 115, 65};
            int BR_angles[] = {210, 65, 225, 65, 65};
            uint16_t delays_ms[] = {404, 343, 670, 230, 230};

            for (int i = 0; i < 5; i++) {
                servo_write_all(FL_angles[i], FR_angles[i], BL_angles[i], BR_angles[i], (uint16_t)global_servo_speed);
                vTaskDelay(delays_ms[i] / portTICK_PERIOD_MS);
            }
            is_flipped = true;
        } else {
            int FL_angles[] = {210, 65, 90, 210, NEUTRAL_FL};
            int FR_angles[] = {210, 65, 220, 165, NEUTRAL_FR};
            int BL_angles[] = {65, 210, 225, 65, NEUTRAL_BL};
            int BR_angles[] = {65, 210, 225, 115, NEUTRAL_BR};
            uint16_t delays_ms[] = {404, 343, 670, 230, 230};

            for (int i = 0; i < 5; i++) {
                servo_write_all(FL_angles[i], FR_angles[i], BL_angles[i], BR_angles[i], (uint16_t)global_servo_speed);
                vTaskDelay(delays_ms[i] / portTICK_PERIOD_MS);
            }
            is_flipped = false;
        }
        
        SetConversationActive(false);  // **UNBLOCK IMU**
    }
};

// ═══════════════════════════════════════════════════════
// AUDIO CODEC
// ═══════════════════════════════════════════════════════
class HeySantaCodec : public SantaAudioCodec  {
public:
    HeySantaCodec(i2c_master_bus_handle_t i2c_bus, int input_sample_rate, int output_sample_rate,
    gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout, gpio_num_t din, uint8_t es7210_addr, bool input_reference)
        : SantaAudioCodec(i2c_bus, input_sample_rate, output_sample_rate,
                             mclk,  bclk,  ws,  dout,  din, es7210_addr, input_reference) {}

    virtual void EnableOutput(bool enable) override {
        SantaAudioCodec::EnableOutput(enable);
        SetConversationActive(enable);  // **BLOCK IMU WHEN SPEAKING**
    }
};

// ═══════════════════════════════════════════════════════
// HEYSANTABOARD
// ═══════════════════════════════════════════════════════
class HeySantaBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    Button wake_button_;
    anim::EmojiWidget* display_ = nullptr;
    Esp32Camera* camera_;
    ServoController servo_controller_;
    ImuController imu_controller_;
    GyroBalanceController gyro_balance_;
    TaskHandle_t imu_task_handle_ = nullptr;
    bool imu_task_running_ = false;
    uint32_t last_imu_time_ = 0;

    static void ImuTaskWrapper(void* param) {
        static_cast<HeySantaBoard*>(param)->ImuTask();
    }

    void ImuTask() {
        ESP_LOGI(TAG, "IMU monitoring task started");
        const TickType_t interval = pdMS_TO_TICKS(IMU_UPDATE_INTERVAL_MS);
        ImuData imu_data;
        
        while (imu_task_running_) {
            if (imu_controller_.Read(&imu_data)) {
                uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
                float dt = (now - last_imu_time_) / 1000.0f;
                if (dt <= 0.0f || dt > 1.0f) dt = 0.05f;
                last_imu_time_ = now;
                
                gyro_balance_.ProcessToggle(imu_data.gyro_x);
                
                // **ONLY RUN IF NOT TALKING**
                if (gyro_balance_.IsEnabled() && !IsConversationActive()) {
                    auto result = gyro_balance_.Calculate(imu_data.gyro_y, dt);
                    servo_controller_.ApplyBalance(result.front_offset, result.back_offset, result.speed);
                }
            }
            vTaskDelay(interval);
        }
        
        ESP_LOGI(TAG, "IMU monitoring task stopped");
        vTaskDelete(NULL);
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        
        mcp_server.AddTool("dog.walk", "Make the robot dog walk forward", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog walk command received");
                servo_controller_.WalkForward(6);
                return "Dog walked forward";
            });
        
        mcp_server.AddTool("dog.dance", "Make the robot dog dance", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog dance command received");
                servo_controller_.Dance();
                return "Dog danced";
            });
        
        mcp_server.AddTool("dog.flip", "Make the robot dog do a flip", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog flip command received");
                servo_controller_.Flip();
                return "Dog flipped";
            });

        mcp_server.AddTool("dog.shake_hand", "Make the robot dog shake its paw", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog shake hand command received");
                servo_controller_.ShakeHand();
                return "Dog shook hand";
            });

        mcp_server.AddTool("dog.pounce", "Make the robot dog pounce", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog pounce command received");
                servo_controller_.Pounce();
                return "Dog pounced";
            });

        mcp_server.AddTool("dog.side_flip", "Make the robot dog do a side flip", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog side flip command received");
                servo_controller_.SideFlip();
                return "Dog did side flip";
            });

        mcp_server.AddTool("dog.reset", "Reset the robot dog to neutral position", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Dog reset command received");
                servo_controller_.MoveReset();
                return "Dog reset to neutral";
            });

        mcp_server.AddTool("dog.balance_enable", "Enable gyro balance mode", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Balance enable command received");
                gyro_balance_.Enable(true);
                return "Gyro balance mode enabled";
            });

        mcp_server.AddTool("dog.balance_disable", "Disable gyro balance mode", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Balance disable command received");
                gyro_balance_.Enable(false);
                servo_controller_.MoveReset();
                return "Gyro balance mode disabled";
            });

        mcp_server.AddTool("dog.balance_status", "Get current gyro balance mode status", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                bool enabled = gyro_balance_.IsEnabled();
                bool imu_ok = imu_controller_.IsInitialized();
                char status[128];
                snprintf(status, sizeof(status), "Balance: %s, IMU: %s (addr: 0x%02X)", 
                         enabled ? "ENABLED" : "DISABLED",
                         imu_ok ? "OK" : "NOT AVAILABLE",
                         imu_controller_.GetAddress());
                return std::string(status);
            });

        mcp_server.AddTool("self.audio.be_quiet", "Make Santa speak more quietly (50% volume)", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "BeQuiet command - setting volume to 50%%");
                auto& board = Board::GetInstance();
                auto codec = board.GetAudioCodec();
                if (codec) {
                    codec->SetOutputVolume(50);
                    return "Santa will now speak more quietly (50% volume)";
                } else {
                    return "Audio codec not available";
                }
            });

        mcp_server.AddTool("self.system.quit", "Quit the application and restart Santa", PropertyList(), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "Quit command - sending exit to server");
                auto& app = Application::GetInstance();
                app.SendSystemCommand("exit");
                return "Ho ho ho! Santa is ending this session. See you soon!";
            });
    }

    void InitializeI2c() {
        ESP_LOGI(TAG, "=== Initializing SINGLE I2C bus on GPIO1/GPIO2 ===");
        
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = IMU_SDA_PIN,
            .scl_io_num = IMU_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        
        esp_err_t ret = i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "✓ I2C bus created on GPIO1/GPIO2 (shared by IMU + codec + camera)");
        }
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_40;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_41;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });

        wake_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.PlaySound(Lang::Sounds::P3_ACTIVE);  
            } else if (app.GetDeviceState() == kDeviceStateListening) {
                app.PlaySound(Lang::Sounds::P3_DEACTIVATE); 
            }
            
            app.ToggleChatState();
        });
    }

    void InitializeSt7735Display() {
        ESP_LOGI(TAG, "Initializing ST7735 display...");
        
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_NC;
        io_config.dc_gpio_num = GPIO_NUM_39;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 20 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        panel_config.vendor_config = nullptr;
        
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7735(panel_io, &panel_config, &panel));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
        
        ESP_LOGI(TAG, "✓ Display initialized");
        display_ = new anim::EmojiWidget(panel, panel_io);
    }

    void InitializeCamera() {
        camera_config_t config = {};
        config.ledc_channel = LEDC_CHANNEL_5;
        config.ledc_timer = LEDC_TIMER_1;
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = -1;
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 0;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

        camera_ = new Esp32Camera(config);
    }

public:
    HeySantaBoard() : boot_button_(BOOT_BUTTON_GPIO), wake_button_(WAKE_BUTTON_GPIO) {
        InitializeI2c();
        InitializeSpi();
        InitializeSt7735Display();
        InitializeButtons();
        // InitializeCamera();
        servo_controller_.Initialize();

         vTaskDelay(pdMS_TO_TICKS(500));
        
        if (imu_controller_.Initialize(i2c_bus_)) {
            imu_task_running_ = true;
            last_imu_time_ = xTaskGetTickCount() * portTICK_PERIOD_MS;
            xTaskCreate(ImuTaskWrapper, "imu_task", 4096, this, 5, &imu_task_handle_);
            ESP_LOGI(TAG, "✓ IMU task started");
        } else {
            ESP_LOGW(TAG, "✗ IMU initialization failed - balance mode disabled");
        }
        
        InitializeTools();
        GetBacklight()->RestoreBrightness();
        
        ESP_LOGI(TAG, "=== HeySanta board initialized! ===");
    }
    
    ~HeySantaBoard() {
        if (imu_task_running_) {
            imu_task_running_ = false;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    virtual AudioCodec* GetAudioCodec() override {
        static HeySantaCodec audio_codec(i2c_bus_, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN, 
            AUDIO_CODEC_ES7210_ADDR, AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }
};

DECLARE_BOARD(HeySantaBoard);