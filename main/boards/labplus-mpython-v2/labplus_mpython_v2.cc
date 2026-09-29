#include "wifi_board.h"
#include "audio_codec.h"
#include "display/oled_display.h"
#include "application.h"
#include "button.h"
#include "config.h"

#include <esp_log.h>
#include <esp_err.h>

#include <driver/i2c_master.h>
#include <driver/dac_continuous.h>

#include <esp_adc/adc_continuous.h>

#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#include <soc/soc_caps.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <array>
#include <vector>
#include <mutex>
#include <algorithm>
#include <cstdint>

#ifdef SH1106
#include <esp_lcd_panel_sh1106.h>
#endif

#define TAG "LabplusMpythonV2"


// ============================================================
// mPython V2 Audio Codec
//
// MIC:
// GPIO38 -> ADC Continuous DMA -> 24 kHz
// Xiaozhi 自动重采样到 16 kHz
//
// Speaker:
// Xiaozhi 24 kHz PCM
// -> ESP32 DAC Continuous DMA
// -> GPIO25 + GPIO26
// -> 扩展板功放 / 扬声器
//
// ESP32 ADC DMA 与 DAC DMA 都使用 I2S0，
// 因此采用半双工自动切换。
// ============================================================

class MpythonV2AudioCodec : public AudioCodec {
private:
    static constexpr int MIC_GPIO = 38;

    static constexpr int MIC_SAMPLE_RATE = 24000;
    static constexpr int SPEAKER_SAMPLE_RATE = 24000;

    // 先用 4 倍增益
    // 后面只需要根据实际 peak 调这一项即可
    static constexpr int MIC_GAIN = 4;

    static constexpr size_t ADC_BATCH_SIZE = 256;

    adc_continuous_handle_t adc_handle_ = nullptr;
    dac_continuous_handle_t dac_handle_ = nullptr;

    adc_unit_t mic_adc_unit_ = ADC_UNIT_1;
    adc_channel_t mic_adc_channel_ = ADC_CHANNEL_2;

    bool adc_started_ = false;
    bool dac_started_ = false;

    std::array<adc_continuous_data_t, ADC_BATCH_SIZE> adc_samples_{};
    std::vector<uint8_t> dac_buffer_;

    int32_t mic_dc_q16_ = 0;
    bool mic_dc_initialized_ = false;

    uint32_t mic_log_counter_ = 0;

    // 防止输出任务还在 Write() 时关闭 DAC
    std::mutex dac_mutex_;


    // ========================================================
    // 打开麦克风 ADC Continuous DMA
    // ========================================================

    bool OpenMicrophone() {
        if (adc_handle_ != nullptr) {
            return true;
        }

        ESP_LOGI(TAG, "Opening MIC DMA on GPIO%d", MIC_GPIO);

        esp_err_t ret = adc_continuous_io_to_channel(
            MIC_GPIO,
            &mic_adc_unit_,
            &mic_adc_channel_
        );

        if (ret != ESP_OK) {
            ESP_LOGE(
                TAG,
                "GPIO%d ADC mapping failed: %s",
                MIC_GPIO,
                esp_err_to_name(ret)
            );
            return false;
        }

        ESP_LOGI(
            TAG,
            "MIC GPIO%d -> ADC%d channel %d",
            MIC_GPIO,
            static_cast<int>(mic_adc_unit_) + 1,
            static_cast<int>(mic_adc_channel_)
        );


        adc_continuous_handle_cfg_t handle_cfg = {};

        handle_cfg.max_store_buf_size = 4096;
        handle_cfg.conv_frame_size = 512;

        ret = adc_continuous_new_handle(
            &handle_cfg,
            &adc_handle_
        );

        if (ret != ESP_OK) {
            ESP_LOGE(
                TAG,
                "adc_continuous_new_handle failed: %s",
                esp_err_to_name(ret)
            );

            adc_handle_ = nullptr;
            return false;
        }


        adc_digi_pattern_config_t pattern = {};

        pattern.atten = ADC_ATTEN_DB_12;
        pattern.channel =
            static_cast<uint8_t>(mic_adc_channel_);

        pattern.unit =
            static_cast<uint8_t>(mic_adc_unit_);

        pattern.bit_width =
            SOC_ADC_DIGI_MAX_BITWIDTH;


        adc_continuous_config_t adc_cfg = {};

        adc_cfg.pattern_num = 1;
        adc_cfg.adc_pattern = &pattern;

        adc_cfg.sample_freq_hz =
            MIC_SAMPLE_RATE;

        adc_cfg.conv_mode =
            ADC_CONV_SINGLE_UNIT_1;

        // 经典 ESP32 单 ADC 模式
        adc_cfg.format =
            ADC_DIGI_OUTPUT_FORMAT_TYPE1;


        ret = adc_continuous_config(
            adc_handle_,
            &adc_cfg
        );

        if (ret != ESP_OK) {
            ESP_LOGE(
                TAG,
                "adc_continuous_config failed: %s",
                esp_err_to_name(ret)
            );

            adc_continuous_deinit(adc_handle_);
            adc_handle_ = nullptr;

            return false;
        }


        ret = adc_continuous_start(
            adc_handle_
        );

        if (ret != ESP_OK) {
            ESP_LOGE(
                TAG,
                "adc_continuous_start failed: %s",
                esp_err_to_name(ret)
            );

            adc_continuous_deinit(adc_handle_);
            adc_handle_ = nullptr;

            return false;
        }


        adc_started_ = true;
        mic_dc_initialized_ = false;

        ESP_LOGI(
            TAG,
            "MIC DMA started @ %d Hz",
            MIC_SAMPLE_RATE
        );

        return true;
    }


    // ========================================================
    // 关闭并彻底释放 ADC / I2S0
    // ========================================================

    void CloseMicrophone() {
        if (adc_handle_ == nullptr) {
            return;
        }

        ESP_LOGI(TAG, "Closing MIC DMA");

        if (adc_started_) {
            esp_err_t ret =
                adc_continuous_stop(adc_handle_);

            if (ret != ESP_OK &&
                ret != ESP_ERR_INVALID_STATE) {

                ESP_LOGW(
                    TAG,
                    "ADC stop: %s",
                    esp_err_to_name(ret)
                );
            }

            adc_started_ = false;
        }


        esp_err_t ret =
            adc_continuous_deinit(adc_handle_);

        if (ret != ESP_OK) {
            ESP_LOGW(
                TAG,
                "ADC deinit: %s",
                esp_err_to_name(ret)
            );
        }

        adc_handle_ = nullptr;
        mic_dc_initialized_ = false;

        ESP_LOGI(
            TAG,
            "MIC released I2S0"
        );
    }


    // ========================================================
    // 打开 DAC DMA
    // ========================================================

    bool OpenSpeaker() {
        std::lock_guard<std::mutex> lock(dac_mutex_);

        if (dac_handle_ != nullptr) {
            return true;
        }


        ESP_LOGI(
            TAG,
            "Opening DAC DMA on GPIO25+GPIO26"
        );


        dac_continuous_config_t cfg = {};

        // GPIO25 + GPIO26 两路一起开
        cfg.chan_mask =
            DAC_CHANNEL_MASK_ALL;

        cfg.desc_num = 6;
        cfg.buf_size = 1024;

        cfg.freq_hz =
            SPEAKER_SAMPLE_RATE;

        cfg.offset = 0;

        // 24kHz 在 ESP32 默认 DAC 时钟范围内，
        // 不使用 APLL，减少资源冲突
        cfg.clk_src =
            DAC_DIGI_CLK_SRC_DEFAULT;

        // 同一份单声道 PCM 同时送到 25、26
        cfg.chan_mode =
            DAC_CHANNEL_MODE_SIMUL;


        esp_err_t ret =
            dac_continuous_new_channels(
                &cfg,
                &dac_handle_
            );


        if (ret != ESP_OK) {
            ESP_LOGE(
                TAG,
                "dac_continuous_new_channels failed: %s",
                esp_err_to_name(ret)
            );

            dac_handle_ = nullptr;
            return false;
        }


        ret =
            dac_continuous_enable(
                dac_handle_
            );


        if (ret != ESP_OK) {
            ESP_LOGE(
                TAG,
                "dac_continuous_enable failed: %s",
                esp_err_to_name(ret)
            );

            dac_continuous_del_channels(
                dac_handle_
            );

            dac_handle_ = nullptr;
            return false;
        }


        dac_started_ = true;

        ESP_LOGI(
            TAG,
            "Speaker DAC started @ %d Hz",
            SPEAKER_SAMPLE_RATE
        );

        return true;
    }


    // ========================================================
    // 关闭并释放 DAC / I2S0
    // ========================================================

    void CloseSpeaker() {
        std::lock_guard<std::mutex> lock(dac_mutex_);

        if (dac_handle_ == nullptr) {
            return;
        }


        ESP_LOGI(TAG, "Closing speaker DAC");


        if (dac_started_) {
            // 先回到 DAC 中点，减少爆音
            std::array<uint8_t, 64> silence{};
            silence.fill(128);

            size_t written = 0;

            dac_continuous_write(
                dac_handle_,
                silence.data(),
                silence.size(),
                &written,
                100
            );


            esp_err_t ret =
                dac_continuous_disable(
                    dac_handle_
                );


            if (ret != ESP_OK &&
                ret != ESP_ERR_INVALID_STATE) {

                ESP_LOGW(
                    TAG,
                    "DAC disable: %s",
                    esp_err_to_name(ret)
                );
            }

            dac_started_ = false;
        }


        esp_err_t ret =
            dac_continuous_del_channels(
                dac_handle_
            );


        if (ret != ESP_OK) {
            ESP_LOGW(
                TAG,
                "DAC delete: %s",
                esp_err_to_name(ret)
            );
        }


        dac_handle_ = nullptr;

        ESP_LOGI(
            TAG,
            "Speaker released I2S0"
        );
    }


protected:

    // ========================================================
    // ADC DMA -> 16-bit PCM
    // ========================================================

    int Read(
        int16_t* dest,
        int samples
    ) override {

        if (!input_enabled_ ||
            adc_handle_ == nullptr ||
            dest == nullptr ||
            samples <= 0) {

            return 0;
        }


        int filled = 0;
        int32_t peak = 0;


        while (filled < samples) {
            uint32_t wanted =
                static_cast<uint32_t>(
                    samples - filled
                );

            if (wanted > ADC_BATCH_SIZE) {
                wanted = ADC_BATCH_SIZE;
            }


            uint32_t got = 0;

            esp_err_t ret =
                adc_continuous_read_parse(
                    adc_handle_,
                    adc_samples_.data(),
                    wanted,
                    &got,
                    50
                );


            if (ret == ESP_ERR_TIMEOUT) {
                // 避免因为偶发 timeout 直接结束音频任务
                std::fill(
                    dest + filled,
                    dest + samples,
                    0
                );

                return samples;
            }


            if (ret != ESP_OK) {
                ESP_LOGE(
                    TAG,
                    "ADC read failed: %s",
                    esp_err_to_name(ret)
                );

                return 0;
            }


            for (
                uint32_t i = 0;
                i < got && filled < samples;
                ++i
            ) {

                const auto& s =
                    adc_samples_[i];


                if (!s.valid) {
                    continue;
                }


                if (s.unit != mic_adc_unit_ ||
                    s.channel != mic_adc_channel_) {

                    continue;
                }


                int32_t raw =
                    static_cast<int32_t>(
                        s.raw_data
                    );


                // ------------------------------
                // 自动消除模拟麦克风 DC 偏置
                // ------------------------------

                if (!mic_dc_initialized_) {
                    mic_dc_q16_ =
                        raw << 16;

                    mic_dc_initialized_ =
                        true;

                } else {
                    int32_t target =
                        raw << 16;

                    // 慢速跟踪直流中心
                    mic_dc_q16_ +=
                        (target - mic_dc_q16_) >> 10;
                }


                int32_t dc =
                    mic_dc_q16_ >> 16;


                int32_t centered =
                    raw - dc;


                // 12-bit ADC -> 16-bit PCM
                int32_t pcm =
                    centered *
                    16 *
                    MIC_GAIN;


                if (pcm > 32767) {
                    pcm = 32767;
                }

                if (pcm < -32768) {
                    pcm = -32768;
                }


                dest[filled++] =
                    static_cast<int16_t>(pcm);


                int32_t abs_pcm =
                    pcm >= 0 ? pcm : -pcm;

                if (abs_pcm > peak) {
                    peak = abs_pcm;
                }
            }
        }


        // 串口观察声音幅度
        if (++mic_log_counter_ >= 50) {
            ESP_LOGI(
                TAG,
                "MIC DMA: peak=%ld dc=%ld samples=%d",
                static_cast<long>(peak),
                static_cast<long>(
                    mic_dc_q16_ >> 16
                ),
                samples
            );

            mic_log_counter_ = 0;
        }


        return samples;
    }


    // ========================================================
    // 16-bit PCM -> 8-bit ESP32 DAC
    // ========================================================

    int Write(
        const int16_t* data,
        int samples
    ) override {

        if (!output_enabled_ ||
            data == nullptr ||
            samples <= 0) {

            return samples;
        }


        std::lock_guard<std::mutex> lock(
            dac_mutex_
        );


        if (dac_handle_ == nullptr) {
            return samples;
        }


        dac_buffer_.resize(
            static_cast<size_t>(samples)
        );


        int volume = output_volume_;

        if (volume < 0) {
            volume = 0;
        }

        if (volume > 100) {
            volume = 100;
        }


        for (int i = 0; i < samples; ++i) {
            int32_t pcm =
                static_cast<int32_t>(
                    data[i]
                );


            pcm =
                pcm * volume / 100;


            if (pcm > 32767) {
                pcm = 32767;
            }

            if (pcm < -32768) {
                pcm = -32768;
            }


            // int16 PCM -> unsigned DAC 0~255
            int32_t value =
                (pcm + 32768) >> 8;


            if (value < 0) {
                value = 0;
            }

            if (value > 255) {
                value = 255;
            }


            dac_buffer_[i] =
                static_cast<uint8_t>(
                    value
                );
        }


        size_t bytes_written = 0;

        esp_err_t ret =
            dac_continuous_write(
                dac_handle_,
                dac_buffer_.data(),
                dac_buffer_.size(),
                &bytes_written,
                1000
            );


        if (ret != ESP_OK) {
            ESP_LOGE(
                TAG,
                "DAC write failed: %s",
                esp_err_to_name(ret)
            );
        }


        return samples;
    }


public:

    MpythonV2AudioCodec() {
        // 半双工
        duplex_ = false;

        input_reference_ = false;

        input_channels_ = 1;
        output_channels_ = 1;

        input_sample_rate_ =
            MIC_SAMPLE_RATE;

        output_sample_rate_ =
            SPEAKER_SAMPLE_RATE;

        dac_buffer_.reserve(2048);

        ESP_LOGI(
            TAG,
            "mPython V2 DMA audio codec ready"
        );
    }


    ~MpythonV2AudioCodec() override {
        CloseMicrophone();
        CloseSpeaker();
    }


    // ========================================================
    // MIC 开关
    // ========================================================

    void EnableInput(bool enable) override {
        if (enable == input_enabled_) {
            return;
        }


        if (enable) {
            ESP_LOGI(TAG, "MIC requested ON");


            // 如果 DAC 还开着，先关闭扬声器释放 I2S0
            if (output_enabled_) {
                EnableOutput(false);
            }


            if (!OpenMicrophone()) {
                ESP_LOGE(
                    TAG,
                    "Failed to start microphone"
                );

                return;
            }


            AudioCodec::EnableInput(true);

        } else {
            ESP_LOGI(TAG, "MIC requested OFF");

            CloseMicrophone();

            AudioCodec::EnableInput(false);
        }
    }


    // ========================================================
    // Speaker 开关
    // ========================================================

    void EnableOutput(bool enable) override {
        if (enable == output_enabled_) {
            return;
        }


        if (enable) {
            ESP_LOGI(
                TAG,
                "Speaker requested ON"
            );


            // 等 audio_input task 自己关闭 ADC。
            // 正常情况下几十毫秒内完成。
            for (
                int i = 0;
                i < 200 && input_enabled_;
                ++i
            ) {
                vTaskDelay(
                    pdMS_TO_TICKS(5)
                );
            }


            if (input_enabled_) {
                ESP_LOGE(
                    TAG,
                    "MIC still owns I2S0"
                );

                return;
            }


            if (!OpenSpeaker()) {
                ESP_LOGE(
                    TAG,
                    "Failed to start speaker"
                );

                return;
            }


            AudioCodec::EnableOutput(true);

        } else {
            ESP_LOGI(
                TAG,
                "Speaker requested OFF"
            );

            CloseSpeaker();

            AudioCodec::EnableOutput(false);
        }
    }
};


// ============================================================
// Board
// ============================================================

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
            i2c_new_master_bus(
                &bus_config,
                &display_i2c_bus_
            )
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

        oled_config.height =
            static_cast<uint8_t>(
                DISPLAY_HEIGHT
            );

        panel_config.vendor_config =
            &oled_config;


#ifdef SH1106
        ESP_LOGI(
            TAG,
            "Initializing SH1106 OLED"
        );

        ESP_ERROR_CHECK(
            esp_lcd_new_panel_sh1106(
                panel_io_,
                &panel_config,
                &panel_
            )
        );
#else
        ESP_LOGI(
            TAG,
            "Initializing SSD1306 OLED"
        );

        ESP_ERROR_CHECK(
            esp_lcd_new_panel_ssd1306(
                panel_io_,
                &panel_config,
                &panel_
            )
        );
#endif


        ESP_ERROR_CHECK(
            esp_lcd_panel_reset(panel_)
        );


        if (
            esp_lcd_panel_init(panel_)
            != ESP_OK
        ) {
            ESP_LOGE(
                TAG,
                "Failed to initialize OLED"
            );

            display_ = new NoDisplay();
            return;
        }


        ESP_ERROR_CHECK(
            esp_lcd_panel_invert_color(
                panel_,
                false
            )
        );

        ESP_ERROR_CHECK(
            esp_lcd_panel_disp_on_off(
                panel_,
                true
            )
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
        // A 键
        boot_button_.OnClick([this]() {
            auto& app =
                Application::GetInstance();

            if (
                app.GetDeviceState()
                == kDeviceStateStarting
            ) {
                EnterWifiConfigMode();
                return;
            }

            app.ToggleChatState();
        });


        // B 键按下
        button_b_.OnPressDown([this]() {
            ESP_LOGI(
                TAG,
                "B DOWN -> StartListening"
            );

            Application::GetInstance()
                .StartListening();
        });


        // B 键松开
        button_b_.OnPressUp([this]() {
            ESP_LOGI(
                TAG,
                "B UP -> StopListening"
            );

            Application::GetInstance()
                .StopListening();
        });
    }


public:
    LabplusMpythonV2()
        : boot_button_(BOOT_BUTTON_GPIO),
          button_b_(BUTTON_B_GPIO) {

        InitializeDisplayI2c();
        InitializeDisplay();
        InitializeButtons();

        ESP_LOGI(
            TAG,
            "Labplus mPython V2 initialized"
        );
    }


    AudioCodec* GetAudioCodec() override {
        static MpythonV2AudioCodec
            audio_codec;

        return &audio_codec;
    }


    Display* GetDisplay() override {
        return display_;
    }
};


DECLARE_BOARD(LabplusMpythonV2);
