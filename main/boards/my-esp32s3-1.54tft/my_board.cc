#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "power_save_timer.h"
#include "led/single_led.h"
#include "assets/lang_config.h"
#include "power_manager.h"
#include "mcp_server.h"

#include <esp_log.h>
#include <esp_wifi.h>
#include <wifi_manager.h>
#include <esp_lcd_panel_vendor.h>

#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#define TAG "MyBoard"

class MyBoard : public WifiBoard {
private:
    Button boot_button_;
    Button volume_up_button_;
    Button volume_down_button_;
    SpiLcdDisplay* display_;
    PowerSaveTimer* power_save_timer_;
    PowerManager* power_manager_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    EventGroupHandle_t scan_event_group_ = nullptr;
    static constexpr int SCAN_DONE_BIT = BIT0;

    static void OnScanDone(void* arg, esp_event_base_t base, int32_t id, void* data) {
        auto* group = static_cast<EventGroupHandle_t*>(arg);
        if (group && *group) {
            xEventGroupSetBits(*group, SCAN_DONE_BIT);
        }
        ESP_LOGI(TAG, "WIFI_EVENT_SCAN_DONE received");
    }

    void WaitForStationScanDone() {
        xEventGroupClearBits(scan_event_group_, SCAN_DONE_BIT);
        ESP_LOGI(TAG, "Waiting for station initial scan to complete...");
        EventBits_t bits = xEventGroupWaitBits(scan_event_group_, SCAN_DONE_BIT,
            pdTRUE, pdFALSE, pdMS_TO_TICKS(8000));
        if (bits & SCAN_DONE_BIT) {
            ESP_LOGI(TAG, "Station scan done, safe to scan now");
        } else {
            ESP_LOGW(TAG, "Station scan timeout, proceeding anyway");
        }
    }

    void InitializePowerManager() {
        power_manager_ = new PowerManager(GPIO_NUM_38);
        power_manager_->OnChargingStatusChanged([this](bool is_charging) {
            if (is_charging) {
                power_save_timer_->SetEnabled(false);
            } else {
                power_save_timer_->SetEnabled(true);
            }
        });
    }

    void InitializePowerSaveTimer() {
        rtc_gpio_init(GPIO_NUM_21);
        rtc_gpio_set_direction(GPIO_NUM_21, RTC_GPIO_MODE_OUTPUT_ONLY);
        rtc_gpio_set_level(GPIO_NUM_21, 1);

        power_save_timer_ = new PowerSaveTimer(-1, 60, 300);
        power_save_timer_->OnEnterSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(1);
        });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->RestoreBrightness();
        });
        power_save_timer_->OnShutdownRequest([this]() {
            ESP_LOGI(TAG, "Shutting down");
            rtc_gpio_set_level(GPIO_NUM_21, 0);
            // 启用保持功能，确保睡眠期间电平不变
            rtc_gpio_hold_en(GPIO_NUM_21);
            esp_lcd_panel_disp_on_off(panel_, false); //关闭显示
            esp_deep_sleep_start();
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_SDA;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_SCL;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

        volume_up_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            power_save_timer_->WakeUp();
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            power_save_timer_->WakeUp();
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });
    }

    void InitializeSt7789Display() {
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS;
        io_config.dc_gpio_num = DISPLAY_DC;
        io_config.spi_mode = 3;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io_));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RES;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io_, &panel_config, &panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, true));

        display_ = new SpiLcdDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, 
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

public:
    MyBoard() :
        boot_button_(BOOT_BUTTON_GPIO),
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        InitializePowerManager();
        InitializePowerSaveTimer();
        InitializeSpi();
        InitializeButtons();
        InitializeSt7789Display();
        GetBacklight()->RestoreBrightness();
        InitializeTools();
    }

    void InitializeTools() {
        if (!scan_event_group_) {
            scan_event_group_ = xEventGroupCreate();
        }
        auto &mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.wifi.scan_networks",
            "Scan all available WiFi networks nearby and return the list.\n"
            "Returns: A list of WiFi networks with SSID, signal strength (RSSI), and encryption type.\n"
            "Use this tool when the user wants to see available WiFi networks or check WiFi signal.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                auto display = GetDisplay();
                char debug_buf[256];

                auto& wifi_mgr = WifiManager::GetInstance();
                bool initialized = wifi_mgr.IsInitialized();
                bool config_mode = wifi_mgr.IsConfigMode();
                std::string current_ssid = wifi_mgr.GetSsid();

                snprintf(debug_buf, sizeof(debug_buf),
                    "WiFi init:%d cfg:%d ssid:%s",
                    initialized, config_mode,
                    current_ssid.empty() ? "none" : current_ssid.c_str());
                ESP_LOGI(TAG, "WiFi state: %s", debug_buf);

                if (!initialized) {
                    display->SetChatMessage("system", "WiFi not initialized");
                    return std::string("{\"error\":\"WiFi not initialized\"}");
                }
                if (config_mode) {
                    display->SetChatMessage("system", "WiFi in config mode, can't scan");
                    return std::string("{\"error\":\"WiFi in config AP mode\"}");
                }

                esp_wifi_set_ps(WIFI_PS_NONE);
                vTaskDelay(pdMS_TO_TICKS(200));

                ESP_LOGI(TAG, "Stopping WifiStation to avoid scan conflict");
                wifi_mgr.StopStation();
                vTaskDelay(pdMS_TO_TICKS(500));

                ESP_LOGI(TAG, "Restarting WiFi driver for scanning");
                ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
                ESP_ERROR_CHECK(esp_wifi_start());

                esp_event_handler_instance_t scan_handler = nullptr;
                esp_event_handler_instance_register(
                    WIFI_EVENT, WIFI_EVENT_SCAN_DONE,
                    &OnScanDone, &scan_event_group_, &scan_handler);

                ESP_LOGI(TAG, "Starting initial scan to let station settle");
                xEventGroupClearBits(scan_event_group_, SCAN_DONE_BIT);
                esp_wifi_scan_start(nullptr, false);
                WaitForStationScanDone();

                int max_retries = 3;
                uint16_t ap_count = 0;
                wifi_ap_record_t *ap_records = nullptr;

                for (int attempt = 1; attempt <= max_retries; attempt++) {
                    xEventGroupClearBits(scan_event_group_, SCAN_DONE_BIT);

                    snprintf(debug_buf, sizeof(debug_buf), "Scan attempt %d/%d...", attempt, max_retries);
                    display->SetChatMessage("system", debug_buf);
                    ESP_LOGI(TAG, "Scan attempt %d/%d", attempt, max_retries);

                    wifi_scan_config_t scan_config = {};
                    scan_config.show_hidden = false;
                    scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
                    scan_config.scan_time.active.min = 200;
                    scan_config.scan_time.active.max = 400;

                    esp_err_t err = esp_wifi_scan_start(&scan_config, false);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "scan_start failed: %s", esp_err_to_name(err));
                        continue;
                    }

                    TickType_t timeout = pdMS_TO_TICKS(6000);
                    EventBits_t bits = xEventGroupWaitBits(scan_event_group_, SCAN_DONE_BIT,
                        pdTRUE, pdFALSE, timeout);

                    if (!(bits & SCAN_DONE_BIT)) {
                        ESP_LOGW(TAG, "Scan timed out on attempt %d", attempt);
                        esp_wifi_scan_stop();
                        continue;
                    }

                    ap_count = 0;
                    esp_wifi_scan_get_ap_num(&ap_count);
                    ESP_LOGI(TAG, "Attempt %d: found %d APs", attempt, ap_count);

                    if (ap_count > 0) {
                        ap_records = new wifi_ap_record_t[ap_count];
                        esp_wifi_scan_get_ap_records(&ap_count, ap_records);
                        break;
                    }

                    if (attempt < max_retries) {
                        display->SetChatMessage("system", "Retrying scan...");
                        vTaskDelay(pdMS_TO_TICKS(1500));
                    }
                }

                if (scan_handler) {
                    esp_event_handler_instance_unregister(
                        WIFI_EVENT, WIFI_EVENT_SCAN_DONE, scan_handler);
                }

                ESP_LOGI(TAG, "Stopping direct WiFi and restarting station");
                esp_wifi_scan_stop();
                esp_wifi_disconnect();
                esp_wifi_stop();
                vTaskDelay(pdMS_TO_TICKS(500));
                wifi_mgr.StartStation();

                cJSON *root = cJSON_CreateArray();
                std::string summary;
                int displayed = 0;

                if (ap_count > 0 && ap_records) {
                    for (int i = 0; i < ap_count; i++) {
                        std::string ssid = reinterpret_cast<char*>(ap_records[i].ssid);
                        if (ssid.empty()) continue;

                        ESP_LOGI(TAG, "  [%d] SSID=%s RSSI=%d AUTH=%d",
                                 i, ssid.c_str(), ap_records[i].rssi, ap_records[i].authmode);

                        cJSON *net = cJSON_CreateObject();
                        cJSON_AddStringToObject(net, "ssid", ssid.c_str());
                        cJSON_AddNumberToObject(net, "rssi", ap_records[i].rssi);

                        const char *auth = "open";
                        if (ap_records[i].authmode == WIFI_AUTH_WEP) auth = "wep";
                        else if (ap_records[i].authmode == WIFI_AUTH_WPA_PSK) auth = "wpa";
                        else if (ap_records[i].authmode == WIFI_AUTH_WPA2_PSK) auth = "wpa2";
                        else if (ap_records[i].authmode == WIFI_AUTH_WPA3_PSK) auth = "wpa3";
                        else if (ap_records[i].authmode == WIFI_AUTH_WPA_WPA2_PSK) auth = "wpa/wpa2";
                        cJSON_AddStringToObject(net, "auth", auth);
                        cJSON_AddItemToArray(root, net);

                        if (displayed < 8) {
                            if (!summary.empty()) summary += ", ";
                            summary += ssid;
                            summary += " (";
                            summary += std::to_string(ap_records[i].rssi);
                            summary += "dBm)";
                        }
                        displayed++;
                    }
                    delete[] ap_records;

                    if (displayed > 8) {
                        summary += " + " + std::to_string(displayed - 8) + " more";
                    }
                }

                esp_wifi_set_ps(WIFI_PS_MAX_MODEM);

                char msg[128];
                snprintf(msg, sizeof(msg), "Found %d networks", ap_count);
                display->SetChatMessage("system", msg);
                ESP_LOGI(TAG, "Scan complete: %s", msg);

                if (summary.empty()) summary = "No WiFi networks found";

                char *json_str = cJSON_PrintUnformatted(root);
                std::string result_str(json_str);
                cJSON_free(json_str);
                cJSON_Delete(root);

                return result_str;
            });
    }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        static bool last_discharging = false;
        charging = power_manager_->IsCharging();
        discharging = power_manager_->IsDischarging();
        if (discharging != last_discharging) {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }
        level = power_manager_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }
};

DECLARE_BOARD(MyBoard);
