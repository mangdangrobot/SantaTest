#include "wifi_board.h"
// #include "audio_codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
// #include "iot/thing_manager.h"
#include "esp32_camera.h"
#include "mcp_server.h"
#include "audio/codecs/santa_audio_codec.h"
// #include "audio_codecs/no_audio_codec.h"
#include "assets/lang_config.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <wifi_station.h>
#include <esp_lcd_touch_ft5x06.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "emoji_display.h"
#include <string.h>
#include <cstdlib>

#define TAG "HeySanta"

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);

float m1_coefficient = 1.0;
float m2_coefficient = 1.0;

class HeySantaCodec : public SantaAudioCodec  {
private:    

public:
    HeySantaCodec(i2c_master_bus_handle_t i2c_bus, int input_sample_rate, int output_sample_rate,
    gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout, gpio_num_t din, uint8_t es7210_addr, bool input_reference)
        : SantaAudioCodec(i2c_bus, input_sample_rate, output_sample_rate,
                             mclk,  bclk,  ws,  dout,  din, es7210_addr, input_reference) {}

    virtual void EnableOutput(bool enable) override {
        SantaAudioCodec::EnableOutput(enable);
    }
};

class HeySantaBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    i2c_master_dev_handle_t pca9557_handle_;
    Button boot_button_;
    Button wake_button_;
    anim::EmojiWidget* display_ = nullptr;
    Esp32Camera* camera_;

    volatile bool head_shake_active = false;

    
    void Initialize_Motor(void)
    {
        // Prepare and then apply the LEDC PWM timer configuration
        ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_MODE,
            .duty_resolution  = LEDC_DUTY_RES,
            .timer_num        = LEDC_TIMER,
            .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
            .clk_cfg          = LEDC_AUTO_CLK
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

        // Array of channel configurations for easy iteration
        const uint8_t motor_ledc_channel[LEDC_CHANNEL_COUNT] = {LEDC_M1_CHANNEL_A, LEDC_M1_CHANNEL_B, LEDC_M2_CHANNEL_A, LEDC_M2_CHANNEL_B};
        const int32_t ledc_channel_pins[LEDC_CHANNEL_COUNT] = {LEDC_M1_CHANNEL_A_IO, LEDC_M1_CHANNEL_B_IO, LEDC_M2_CHANNEL_A_IO, LEDC_M2_CHANNEL_B_IO};
        for (int i = 0; i < LEDC_CHANNEL_COUNT; i++) {
            ledc_channel_config_t ledc_channel = {
                .gpio_num       = ledc_channel_pins[i],
                .speed_mode     = LEDC_MODE,
                .channel        = (ledc_channel_t)motor_ledc_channel[i],
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LEDC_TIMER,
                .duty           = 0, // Set duty to 0%
                .hpoint         = 0
            };
            ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
        }
    }

    static void set_motor_A_speed(int speed)
    {
        if (speed >= 0) {
            uint32_t m1a_duty = (uint32_t)((speed * m1_coefficient * 8192) / 100);
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_M1_CHANNEL_A, m1a_duty));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_M1_CHANNEL_A));
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_M1_CHANNEL_B, 0));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_M1_CHANNEL_B));
        } else {
            uint32_t m1b_duty = (uint32_t)((-speed * m1_coefficient * 8192) / 100);
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_M1_CHANNEL_A, 0));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_M1_CHANNEL_A));
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_M1_CHANNEL_B, m1b_duty));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_M1_CHANNEL_B));
        }
    }

    static void set_motor_B_speed(int speed)
    {
        if (speed >= 0) {
            uint32_t m2a_duty = (uint32_t)((speed * m2_coefficient * 8192) / 100);
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_M2_CHANNEL_A, m2a_duty));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_M2_CHANNEL_A));
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_M2_CHANNEL_B, 0));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_M2_CHANNEL_B));
        } else {
            uint32_t m2b_duty = (uint32_t)((-speed * m2_coefficient * 8192) / 100);
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_M2_CHANNEL_A, 0));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_M2_CHANNEL_A));
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_M2_CHANNEL_B, m2b_duty));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_M2_CHANNEL_B));
        }
    }
    void movement_type(int motor, uint32_t mode, int dir) 
    //motor: 1 for head, 2 for hip 
    //mode: 0 for slow, 1 for mid, 2 for stop 
    //dir: 1 for forward, -1 for backward
    {
        int speed_head[3] = {87, 93, 100}; 
        int speed_hip[3] = {90, 95, 100};
        int speed;
        if (motor == 1)  
        {
            if (dir == 1) speed = speed_head[mode];
            else speed = -speed_head[mode];
            ESP_LOGI(TAG, "Setting head speed to %d", speed);
            set_motor_A_speed(speed);
        }
        else 
        {
            if (dir == 1) speed = speed_hip[mode];
            else speed = -speed_hip[mode];
            ESP_LOGI(TAG, "Setting hip speed to %d", speed);
            if (speed < 0) {
                speed = abs(speed);
                int step = speed / 4;
                for (int i = 1 ; i <= 4 ; i++)
                {
                    set_motor_B_speed(speed); // Gradually increase speed to avoid sudden jerk
                    speed -= step; 
                    vTaskDelay(25 / portTICK_PERIOD_MS); // Wait for 0.1 second    
                }
                set_motor_B_speed(0); // Set final speed
            }
            else 
            {
                set_motor_B_speed(speed);
            }
        }
    }

    void SetHeadSpeed(int speed) {
        ESP_LOGI(TAG, "Setting head speed to %d", speed);
        set_motor_A_speed(speed);
    }

    void SetHipSpeed(int speed) {
        ESP_LOGI(TAG, "Setting hip speed to %d", speed);
        set_motor_B_speed(speed);    
    }

    uint32_t unbiasedRandom3() {
        uint32_t r;
        const uint32_t upper_bound = 0xFFFFFFFF - (0xFFFFFFFF % 3);
        
        do {
            r = esp_random();
        } while (r >= upper_bound);
        
        return r % 3;
    }

    uint32_t unbiasedRandomRange(int min, int max) {
        // Calculate the range size
        uint32_t range = max - min + 1;
        
        // Determine the upper bound to reject values that cause bias
        uint32_t upper_bound = 0xFFFFFFFF - (0xFFFFFFFF % range);
        uint32_t r;
        
        do {
            r = esp_random(); // Get 32-bit random number from hardware
        } while (r >= upper_bound);
        
        return min + (r % range);
    }
    void dance()
    
    {
        head_shake_active = false;
        for (int i = 1 ; i <= 3; i++) // dance for 3 times
        {
            uint32_t head_mode = unbiasedRandom3(); // Randomly choose head mode 
            uint32_t hip_mode = unbiasedRandom3(); // Randomly choose hip mode
            movement_type(1, head_mode, 1); // Head forward
            vTaskDelay(unbiasedRandomRange(1500, 5000) / portTICK_PERIOD_MS); // Wait for 5 second
            set_motor_A_speed(0); 
            vTaskDelay(unbiasedRandomRange(150, 1000) / portTICK_PERIOD_MS); // Wait for 1 second
            for (int j = 0; j < 3; j++) // Shake head 3 times
            {
                movement_type(2, hip_mode, 1); // Hip forward
                vTaskDelay(150 / portTICK_PERIOD_MS); // Wait for 0.5 second

                movement_type(2, hip_mode, -1); // Hip forward
                vTaskDelay(150 / portTICK_PERIOD_MS); // Wait for 0.5 second
            }
            set_motor_B_speed(0);
        }
    }

    void HeadShakeOnly() {
        ESP_LOGI(TAG, "Head shake (on/off mode)!");
        
        for (int i = 0; i < 50; i++) {
            SetHeadSpeed(100);   // Full speed forward
            vTaskDelay(80 / portTICK_PERIOD_MS);
            
            SetHeadSpeed(-100);  // Full speed backward
            vTaskDelay(80 / portTICK_PERIOD_MS);
        }
        SetHeadSpeed(0);
        ESP_LOGI(TAG, "Head shake complete!");
    }
        
    void HeadShake_start() {
        ESP_LOGI(TAG, "Head shake starting - will shake until manually stopped or timeout");
        head_shake_active = true;
        
        auto& app = Application::GetInstance();
        
        for (int i = 0; head_shake_active; i++) {
            ESP_LOGI(TAG, "Head shake cycle %d", i + 1);
            
            // Auto-stop conditions:
            // 1. Device goes to idle (conversation ended)
            // 2. Timeout after 60 seconds
            if (app.GetDeviceState() == kDeviceStateIdle || i >= 150) {
                ESP_LOGI(TAG, "Auto-stopping shake: state=%d, cycles=%d", 
                        app.GetDeviceState(), i);
                break;
            }
            
            // ON phase - 1 second
            SetHeadSpeed(100);
            for (int j = 0; j < 10 && head_shake_active; j++) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            
            if (!head_shake_active) break;
            
            // OFF phase - 1 second  
            SetHeadSpeed(0);
            for (int j = 0; j < 10 && head_shake_active; j++) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
        }
        
        SetHeadSpeed(0);
        head_shake_active = false;
        ESP_LOGI(TAG, "Head shake completed");
    }

    void HeadShake_stop() {
        ESP_LOGI(TAG, "stop Head shake (on/off mode)!");
        head_shake_active = false;
        for (int i = 0; i < 1; i++) {
            SetHeadSpeed(100);   // Full speed forward
            vTaskDelay(80 / portTICK_PERIOD_MS);
            
            SetHeadSpeed(-100);  // Full speed backward
            vTaskDelay(80 / portTICK_PERIOD_MS);
        }
        SetHeadSpeed(0);
    }

    void HipShakeOnly() {
        head_shake_active = false;
        ESP_LOGI(TAG, "Hip shake (on/off mode)!");
        SetHeadSpeed(0);
        for (int i = 0; i < 12; i++) {
            SetHipSpeed(72);    // Forward
            vTaskDelay(150 / portTICK_PERIOD_MS);
            SetHipSpeed(0);      // Stop - adds gentleness
            vTaskDelay(50 / portTICK_PERIOD_MS);
            SetHipSpeed(-72);   // Backward
            vTaskDelay(150 / portTICK_PERIOD_MS);
            SetHipSpeed(0);      // Stop - adds gentleness
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        SetHipSpeed(0);
        ESP_LOGI(TAG, "Hip shake complete!");
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        
        mcp_server.AddTool("self.chassis.dance", "跳舞", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            ESP_LOGI(TAG, "Dance command received");
            dance();
            return true;
        });
        
        mcp_server.AddTool("self_chassis_shake_body", "摇头", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            ESP_LOGI(TAG, "Head shake command received");
            HeadShakeOnly();
            return true;
        });
        
        mcp_server.AddTool("self_chassis_shake_hip", "摇屁股", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            ESP_LOGI(TAG, "Hip shake command received");
            HipShakeOnly();
            return true;
        });

        mcp_server.AddTool("self_chassis_shake_body_start", "摇头1", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            HeadShake_start();
            return true;
        });

        mcp_server.AddTool("self_chassis_shake_body_stop", "摇头2", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            HeadShake_stop();
            return true;
        });

        // BeQuiet/Silent tool - sets volume to 50%
        mcp_server.AddTool("self.audio.be_quiet", "Make Santa speak more quietly by setting volume to 50%%. Use when user asks Santa to be quiet, silent, or speak softer.", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            ESP_LOGI(TAG, "BeQuiet command received - setting volume to 50%%");
            auto& board = Board::GetInstance();
            auto codec = board.GetAudioCodec();
            if (codec) {
                codec->SetOutputVolume(50);
                ESP_LOGI(TAG, "Volume set to 50%%");
                return "Santa will now speak more quietly (volume set to 50%)";
            } else {
                ESP_LOGW(TAG, "Audio codec not available");
                return "Audio codec not available";
            }
        });

        // Enough/Quit tool - sends exit command to server
        mcp_server.AddTool("self.system.quit", "Quit the application and restart Santa. Use when user says goodbye, wants to end the conversation, or asks Santa to go away.", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            ESP_LOGI(TAG, " Quit command received - sending exit command to server");
            auto& app = Application::GetInstance();
            
            // Send a system command to the server to exit/quit
            app.SendSystemCommand("exit");
            
            return " Ho ho ho! Santa is telling the server to end this session. See you soon! 🎄";
        });
    }

    void InitializeI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
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
            
            // ADD SOUND BASED ON CURRENT STATE BEFORE TOGGLING
            if (app.GetDeviceState() == kDeviceStateIdle) {
                // Going from idle to listening - play "entering listening" sound
                app.PlaySound(Lang::Sounds::P3_ACTIVE);  
            } else if (app.GetDeviceState() == kDeviceStateListening) {
                // Going from listening to idle - play "entering idle" sound  
                app.PlaySound(Lang::Sounds::P3_DEACTIVATE); 
            }
            
            app.ToggleChatState();
        });

    #if CONFIG_USE_DEVICE_AEC
        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.SetAecMode(app.GetAecMode() == kAecOff ? kAecOnDeviceSide : kAecOff);
            }
        });

        wake_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.SetAecMode(app.GetAecMode() == kAecOff ? kAecOnDeviceSide : kAecOff);
            }
        });
    #endif
    }

    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_NC;
        io_config.dc_gpio_num = GPIO_NUM_39;
        io_config.spi_mode = 2;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
        
        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        esp_lcd_panel_disp_on_off(panel, true);
        
        ESP_LOGI(TAG, "Panel handle: %p, Panel IO handle: %p", panel, panel_io);
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
        config.sccb_i2c_port = 1;
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
        InitializeSt7789Display();
        InitializeButtons();
        InitializeCamera();
        Initialize_Motor();
        InitializeTools();

#if CONFIG_IOT_PROTOCOL_XIAOZHI
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Screen"));
#endif
        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static HeySantaCodec audio_codec(i2c_bus_, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_ES7210_ADDR, AUDIO_INPUT_REFERENCE);
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