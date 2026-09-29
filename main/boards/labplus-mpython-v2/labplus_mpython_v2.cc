#include "wifi_board.h"
#include "audio_codec.h"
#include "display/oled_display.h"
#include "application.h"
#include "button.h"
#include "config.h"

#include <esp_log.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>

#include <driver/i2c_master.h>
#include <driver/dac_continuous.h>

#include <esp_adc/adc_oneshot.h>

#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#include <vector>
#include <array>
#include <cstdint>
#include <algorithm>

#ifdef SH1106
#include <esp_lcd_panel_sh1106.h>
#endif


#define TAG "LabplusMpythonV2"


// ============================================================
// Labplus mPython V2 Audio Codec
//
// MIC:
//   onboard microphone
//       -> GPIO38
//       -> ADC1 CH2
//       -> 16 kHz / 16 bit PCM
//
// SPEAKER:
//   Xiaozhi PCM
//       -> ESP32 internal DAC
//       -> GPIO25 + GPIO26
//       -> P9 + P8
//       -> expansion-board amplifier / speaker
//
// 注意：
// ESP32 的 ADC continuous 与 DAC continuous 都会占 I2S0。
// 因此 MIC 使用 ADC oneshot 定时采样，DAC 使用 DMA continuous。
// 这样输入输出可以存在于同一个固件中，不争抢 I2S0。
// ============================================================

class MpythonV2AudioCodec : public AudioCodec {
private:

    static constexpr adc_unit_t MIC_ADC_UNIT = ADC_UNIT_1;
    static constexpr adc_channel_t MIC_ADC_CHANNEL = ADC_CHANNEL_2;

    // GPIO38 = ADC1_CH2
    static constexpr int MIC_SAMPLE_RATE = 16000;

    // ESP32 DAC PLL 默认时钟要求较高频率，
    // 24 kHz 很适合 Xiaozhi 语音播放
    static constexpr int SPEAKER_SAMPLE_RATE = 24000;

    // 软件麦克风增益
    static constexpr int MIC_GAIN = 4;

    adc_oneshot_unit_handle_t adc_handle_ = nullptr;

    dac_continuous_handle_t dac_handle_ = nullptr;

    std::vector<uint8_t> dac_buffer_;

    int32_t mic_dc_q16_ = 0;
    bool mic_dc_initialized_ = false;

    uint32_t mic_log_counter_ = 0;


    // --------------------------------------------------------
    // MIC ADC
    // --------------------------------------------------------

    void InitializeMicrophone() {

        ESP_LOGI(
            TAG,
            "Initializing microphone: GPIO38 / ADC1_CH2"
        );

        adc_oneshot_unit_init_cfg_t adc_init = {};

        adc_init.unit_id = MIC_ADC_UNIT;

        ESP_ERROR_CHECK(
            adc_oneshot_new_unit(
                &adc_init,
                &adc_handle_
            )
        );


        adc_oneshot_chan_cfg_t mic_cfg = {};

        mic_cfg.atten = ADC_ATTEN_DB_12;
        mic_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;


        ESP_ERROR_CHECK(
            adc_oneshot_config_channel(
                adc_handle_,
                MIC_ADC_CHANNEL,
                &mic_cfg
            )
        );


        ESP_LOGI(
            TAG,
            "Microphone ready: 16000 Hz"
        );
    }


    // --------------------------------------------------------
    // Internal DAC
    //
    // GPIO25 = DAC1 / P9
    // GPIO26 = DAC2 / P8
    //
    // SAME PCM is sent to both channels.
    // --------------------------------------------------------

    void InitializeSpeaker() {

        ESP_LOGI(
            TAG,
            "Initializing speaker DAC on GPIO25 + GPIO26"
        );


        dac_continuous_config_t dac_config = {};

        // 两路都开
        dac_config.chan_mask =
            DAC_CHANNEL_MASK_ALL;

        // DMA 描述符
        dac_config.desc_num =
            4;

        dac_config.buf_size =
            1024;

        // 24 kHz
        dac_config.freq_hz =
            SPEAKER_SAMPLE_RATE;

        dac_config.offset =
            0;

        dac_config.clk_src =
            DAC_DIGI_CLK_SRC_DEFAULT;

        // 两个 DAC 同时输出同一个采样值
        // 适合小智单声道语音
        dac_config.chan_mode =
            DAC_CHANNEL_MODE_SIMUL;


        ESP_ERROR_CHECK(
            dac_continuous_new_channels(
                &dac_config,
                &dac_handle_
            )
        );


        // 提前留好缓存，避免播放时反复申请内存
        dac_buffer_.reserve(2048);


        ESP_LOGI(
            TAG,
            "Speaker DAC ready: 24000 Hz, GPIO25+26"
        );
    }


protected:

    // ========================================================
    // MIC -> Xiaozhi
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


        // 16000 Hz = 每个采样约 62.5 us
        int64_t next_sample_us =
            esp_timer_get_time();

        uint32_t timing_accumulator = 0;

        int32_t peak = 0;


        for (int i = 0; i < samples; ++i) {

            // 精确控制平均采样率为 16 kHz
            while (true) {

                int64_t now =
                    esp_timer_get_time();

                int64_t wait =
                    next_sample_us - now;

                if (wait <= 0) {
                    break;
                }

                if (wait > 2) {
                    esp_rom_delay_us(
                        static_cast<uint32_t>(wait)
                    );
                }
            }


            int raw = 0;

            esp_err_t ret =
                adc_oneshot_read(
                    adc_handle_,
                    MIC_ADC_CHANNEL,
                    &raw
                );


            if (ret != ESP_OK) {

                ESP_LOGW(
                    TAG,
                    "MIC ADC read failed: %s",
                    esp_err_to_name(ret)
                );

                dest[i] = 0;

            } else {

                // ------------------------------
                // 自动追踪麦克风 DC 偏置
                // ------------------------------

                if (!mic_dc_initialized_) {

                    mic_dc_q16_ =
                        raw << 16;

                    mic_dc_initialized_ =
                        true;

                } else {

                    const int32_t target =
                        raw << 16;

                    // 慢速 DC 跟踪
                    mic_dc_q16_ +=
                        (target - mic_dc_q16_) >> 10;
                }


                const int32_t dc =
                    mic_dc_q16_ >> 16;


                int32_t centered =
                    raw - dc;


                // 12-bit ADC -> 16-bit PCM
                int32_t pcm =
                    centered * 16 * MIC_GAIN;


                // 限幅
                if (pcm > 32767) {
                    pcm = 32767;
                }

                if (pcm < -32768) {
                    pcm = -32768;
                }


                dest[i] =
                    static_cast<int16_t>(pcm);


                int32_t abs_pcm =
                    pcm >= 0
                        ? pcm
                        : -pcm;

                if (abs_pcm > peak) {
                    peak = abs_pcm;
                }
            }


            // 1,000,000 / 16,000 = 62.5 us
            //
            // 用整数累加方式交替 62 / 63 us，
            // 避免长期采样率漂移
            timing_accumulator +=
                1000000;

            uint32_t step =
                timing_accumulator /
                MIC_SAMPLE_RATE;

            timing_accumulator %=
                MIC_SAMPLE_RATE;

            next_sample_us +=
                step;
        }


        // 大约每 1 秒打印一次 MIC 峰值
        if (++mic_log_counter_ >= 100) {

            ESP_LOGI(
                TAG,
                "MIC: samples=%d peak=%ld dc=%ld",
                samples,
                static_cast<long>(peak),
                static_cast<long>(
                    mic_dc_q16_ >> 16
                )
            );

            mic_log_counter_ = 0;
        }


        return samples;
    }


    // ========================================================
    // Xiaozhi -> DAC -> expansion-board speaker
    // ========================================================

    int Write(
        const int16_t* data,
        int samples
    ) override {

        if (!output_enabled_ ||
            dac_handle_ == nullptr ||
            data == nullptr ||
            samples <= 0) {

            return samples;
        }


        dac_buffer_.resize(samples);


        int volume =
            output_volume_;

        if (volume < 0) {
            volume = 0;
        }

        // 允许 Xiaozhi 音量超过 100 时仍安全限幅
        if (volume > 150) {
            volume = 150;
        }


        for (int i = 0; i < samples; ++i) {

            // 音量处理
            int32_t pcm =
                static_cast<int32_t>(
                    data[i]
                ) * volume / 100;


            if (pcm > 32767) {
                pcm = 32767;
            }

            if (pcm < -32768) {
                pcm = -32768;
            }


            // signed 16bit
            // -32768 ... +32767
            //
            // -> unsigned 8bit
            // 0 ... 255
            //
            // 128 是 DAC 的静音中心点
            int32_t dac_sample =
                (pcm + 32768) >> 8;


            if (dac_sample < 0) {
                dac_sample = 0;
            }

            if (dac_sample > 255) {
                dac_sample = 255;
            }


            dac_buffer_[i] =
                static_cast<uint8_t>(
                    dac_sample
                );
        }


        size_t bytes_written = 0;


        esp_err_t ret =
            dac_continuous_write(
                dac_handle_,
                dac_buffer_.data(),
                dac_buffer_.size(),
                &bytes_written,
                -1
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

        // 小智这里按单声道使用
        input_channels_ = 1;
        output_channels_ = 1;

        input_reference_ = false;

        // 没有硬件 AEC，声明为非硬件全双工 Codec
        duplex_ = false;


        input_sample_rate_ =
            MIC_SAMPLE_RATE;

        output_sample_rate_ =
            SPEAKER_SAMPLE_RATE;


        InitializeMicrophone();

        InitializeSpeaker();


        ESP_LOGI(
            TAG,
            "mPython V2 AudioCodec initialized"
        );
    }


    ~MpythonV2AudioCodec() override {

        if (dac_handle_ != nullptr) {

            if (output_enabled_) {
                dac_continuous_disable(
                    dac_handle_
                );
            }

            dac_continuous_del_channels(
                dac_handle_
            );

            dac_handle_ = nullptr;
        }


        if (adc_handle_ != nullptr) {

            adc_oneshot_del_unit(
                adc_handle_
            );

            adc_handle_ = nullptr;
        }
    }


    // ========================================================
    // MIC power state
    // ========================================================

    void EnableInput(
        bool enable
    ) override {

        if (enable == input_enabled_) {
            return;
        }


        if (enable) {

            ESP_LOGI(
                TAG,
                "Microphone ON"
            );

            // 每次重新聆听都重新建立 DC 基线
            mic_dc_initialized_ = false;

        } else {

            ESP_LOGI(
                TAG,
                "Microphone OFF"
            );
        }


        AudioCodec::EnableInput(
            enable
        );
    }


    // ========================================================
    // Speaker power state
    // ========================================================

    void EnableOutput(
        bool enable
    ) override {

        if (enable == output_enabled_) {
            return;
        }


        if (enable) {

            ESP_LOGI(
                TAG,
                "Speaker ON"
            );


            esp_err_t ret =
                dac_continuous_enable(
                    dac_handle_
                );


            if (ret != ESP_OK) {

                ESP_LOGE(
                    TAG,
                    "DAC enable failed: %s",
                    esp_err_to_name(ret)
                );

                return;
            }

        } else {

            ESP_LOGI(
                TAG,
                "Speaker OFF"
            );


            // 关闭之前将 DAC 拉回中点，
            // 避免喇叭出现“啪”的直流跳变声
            std::array<uint8_t, 64> silence {};

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


            if (ret != ESP_OK) {

                ESP_LOGW(
                    TAG,
                    "DAC disable warning: %s",
                    esp_err_to_name(ret)
                );
            }
        }


        AudioCodec::EnableOutput(
            enable
        );
    }
};


// ============================================================
// Labplus mPython V2 Board
// ============================================================

class LabplusMpythonV2 : public WifiBoard {
private:

    i2c_master_bus_handle_t
        display_i2c_bus_ = nullptr;

    esp_lcd_panel_io_handle_t
        panel_io_ = nullptr;

    esp_lcd_panel_handle_t
        panel_ = nullptr;

    Display*
        display_ = nullptr;

    Button
        boot_button_;

    Button
        button_b_;


    // --------------------------------------------------------
    // OLED I2C
    // --------------------------------------------------------

    void InitializeDisplayI2c() {

        i2c_master_bus_config_t
            bus_config = {};


        bus_config.i2c_port =
            I2C_NUM_0;

        bus_config.sda_io_num =
            DISPLAY_SDA_PIN;

        bus_config.scl_io_num =
            DISPLAY_SCL_PIN;

        bus_config.clk_source =
            I2C_CLK_SRC_DEFAULT;

        bus_config.glitch_ignore_cnt =
            7;

        bus_config.intr_priority =
            0;

        bus_config.trans_queue_depth =
            0;

        bus_config.flags.enable_internal_pullup =
            1;


        ESP_ERROR_CHECK(
            i2c_new_master_bus(
                &bus_config,
                &display_i2c_bus_
            )
        );
    }


    // --------------------------------------------------------
    // OLED
    // --------------------------------------------------------

    void InitializeDisplay() {

        esp_lcd_panel_io_i2c_config_t
            io_config = {};


        io_config.dev_addr =
            0x3C;

        io_config.scl_speed_hz =
            400 * 1000;

        io_config.control_phase_bytes =
            1;

        io_config.dc_bit_offset =
            6;

        io_config.lcd_cmd_bits =
            8;

        io_config.lcd_param_bits =
            8;

        io_config.on_color_trans_done =
            nullptr;

        io_config.user_ctx =
            nullptr;

        io_config.flags.dc_low_on_data =
            0;

        io_config.flags.disable_control_phase =
            0;


        ESP_ERROR_CHECK(
            esp_lcd_new_panel_io_i2c(
                display_i2c_bus_,
                &io_config,
                &panel_io_
            )
        );


        esp_lcd_panel_dev_config_t
            panel_config = {};


        panel_config.reset_gpio_num =
            GPIO_NUM_NC;

        panel_config.bits_per_pixel =
            1;


        esp_lcd_panel_ssd1306_config_t
            oled_config = {};


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
            esp_lcd_panel_reset(
                panel_
            )
        );


        if (
            esp_lcd_panel_init(panel_)
            != ESP_OK
        ) {

            ESP_LOGE(
                TAG,
                "Failed to initialize OLED"
            );


            display_ =
                new NoDisplay();

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


        display_ =
            new OledDisplay(
                panel_io_,
                panel_,
                DISPLAY_WIDTH,
                DISPLAY_HEIGHT,
                DISPLAY_MIRROR_X,
                DISPLAY_MIRROR_Y
            );
    }


    // --------------------------------------------------------
    // Buttons
    // --------------------------------------------------------

    void InitializeButtons() {

        // A 键：
        // 配网 / 切换聊天状态
        boot_button_.OnClick(
            [this]() {

                auto& app =
                    Application::GetInstance();


                if (
                    app.GetDeviceState() ==
                    kDeviceStateStarting
                ) {

                    EnterWifiConfigMode();

                    return;
                }


                app.ToggleChatState();
            }
        );


        // B 键按下：
        // 开始讲话
        button_b_.OnPressDown(
            [this]() {

                ESP_LOGI(
                    TAG,
                    "B DOWN -> StartListening"
                );


                Application::GetInstance()
                    .StartListening();
            }
        );


        // B 键松开：
        // 发送并等待小智回答
        button_b_.OnPressUp(
            [this]() {

                ESP_LOGI(
                    TAG,
                    "B UP -> StopListening"
                );


                Application::GetInstance()
                    .StopListening();
            }
        );
    }


public:

    LabplusMpythonV2()
        :
        boot_button_(
            BOOT_BUTTON_GPIO
        ),
        button_b_(
            BUTTON_B_GPIO
        ) {


        InitializeDisplayI2c();

        InitializeDisplay();

        InitializeButtons();


        ESP_LOGI(
            TAG,
            "Labplus mPython V2 initialized"
        );
    }


    // --------------------------------------------------------
    // 完整 AudioCodec：
    // MIC + SPEAKER
    // --------------------------------------------------------

    AudioCodec*
    GetAudioCodec() override {

        static MpythonV2AudioCodec
            audio_codec;

        return &audio_codec;
    }


    Display*
    GetDisplay() override {

        return display_;
    }
};


DECLARE_BOARD(LabplusMpythonV2);
