#include "wifi_board.h"
#include "codecs/dummy_audio_codec.h"
#include "display/oled_display.h"
#include "application.h"
#include "button.h"
#include "config.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#ifdef SH1106
#include <esp_lcd_panel_sh1106.h>
#endif

// Labplus mPython V2 custom board support
#define TAG "LabplusMpythonV2"

class LabplusMpythonV2 : public WifiBoard {
private:
    i2c_master_bus_handle_t display_i2c_bus_ = nullptr;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;

    Display* display_ = nullptr;

    Button boot_button_;
    Button button_b_;

    void InitializeDisplayI2c() {
        i2c_master_bus_config_t bus_config = {};

        bus_config.i2c_port = I2C_NUM_0;
        bus_config.sda_io_num = DISPLAY_SDA_PIN;
        bus_config.scl_io_num = DISPLAY_SCL_PIN;
        bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_config.glitch_ignore_cnt = 7;
        bus_config.intr_priority = 0;
        bus_config.trans_queue_depth = 0;
        bus_config.flags.enable_internal_pullup = 1;

        ESP_ERROR_CHECK(
            i2c_new_master_bus(&bus_config, &display_i2c_bus_)
        );
    }

    void InitializeDisplay() {
        esp_lcd_panel_io_i2c_config_t io_config = {};

        io_config.dev_addr = 0x3C;
        io_config.scl_speed_hz = 400 * 1000;
        io_config.control_phase_bytes = 1;
        io_config.dc_bit_offset = 6;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        io_config.on_color_trans_done = nullptr;
        io_config.user_ctx = nullptr;
        io_config.flags.dc_low_on_data = 0;
        io_config.flags.disable_control_phase = 0;

        ESP_ERROR_CHECK(
            esp_lcd_new_panel_io_i2c(
                display_i2c_bus_,
                &io_config,
                &panel_io_
            )
        );

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.bits_per_pixel = 1;

        esp_lcd_panel_ssd1306_config_t oled_config = {};
        oled_config.height = static_cast<uint8_t>(DISPLAY_HEIGHT);

        panel_config.vendor_config = &oled_config;

#ifdef SH1106
        ESP_LOGI(TAG, "Initializing SH1106 OLED");
        ESP_ERROR_CHECK(
            esp_lcd_new_panel_sh1106(
                panel_io_,
                &panel_config,
                &panel_
            )
        );
#else
        ESP_LOGI(TAG, "Initializing SSD1306 OLED");
        ESP_ERROR_CHECK(
            esp_lcd_new_panel_ssd1306(
                panel_io_,
                &panel_config,
                &panel_
            )
        );
#endif

        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));

        if (esp_lcd_panel_init(panel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize OLED");
            display_ = new NoDisplay();
            return;
        }

        ESP_ERROR_CHECK(
            esp_lcd_panel_invert_color(panel_, false)
        );

        ESP_ERROR_CHECK(
            esp_lcd_panel_disp_on_off(panel_, true)
        );

        display_ = new OledDisplay(
            panel_io_,
            panel_,
            DISPLAY_WIDTH,
            DISPLAY_HEIGHT,
            DISPLAY_MIRROR_X,
            DISPLAY_MIRROR_Y
        );
    }

    void InitializeButtons() {
        // A 键：进入配网 / 切换聊天状态
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();

            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }

            app.ToggleChatState();
        });

        // B 键：按住说话
        button_b_.OnPressDown([this]() {
            Application::GetInstance().StartListening();
        });

        button_b_.OnPressUp([this]() {
            Application::GetInstance().StopListening();
        });
    }

public:
    LabplusMpythonV2()
        : boot_button_(BOOT_BUTTON_GPIO),
          button_b_(BUTTON_B_GPIO) {

        InitializeDisplayI2c();
        InitializeDisplay();
        InitializeButtons();

        ESP_LOGI(TAG, "Labplus mPython V2 initialized");
    }

    virtual AudioCodec* GetAudioCodec() override {
        // 第一阶段暂时不启用模拟麦克风和 DAC 音频
        static DummyAudioCodec audio_codec(
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE
        );

        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
};

DECLARE_BOARD(LabplusMpythonV2);
