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
#include <esp_netif.h>
#include <wifi_manager.h>
#include <ssid_manager.h>
#include <esp_lcd_panel_vendor.h>
#include <nvs_flash.h>
#include <esp_heap_caps.h>
#include <nvs_flash.h>
#include <esp_partition.h>
#include <lwip/dns.h>

#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include <atomic>
#include <cstring>
#include <map>
#include <mutex>

#define TAG "MyBoard"
#define NVS_SNIFFER_NAMESPACE "sniffer"
#define MAX_SNIFFER_LOG 100

static const wifi_promiscuous_filter_t filter = {
    .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA
};

class MyBoard : public WifiBoard {
private:
    struct WifiScanTaskParam {
        MyBoard* self;
        int reply_id;
    };

    struct WifiConnectTaskParam {
        int reply_id;
        std::string ssid;
        std::string password;
    };

    struct SnifferPacket {
        std::string description;
        std::string short_info;
    };

    static bool sniffer_active_;
    static bool sniffer_disconnected_;
    static TaskHandle_t sniffer_task_handle_;
    static QueueHandle_t sniffer_queue_;
    static std::vector<SnifferPacket> sniffer_log_;
    static int sniffer_duration_seconds_;

    // Answer of the ESP8266 light board for the command that is being executed.
    // It is stored in RAM and in NVS right after the request so that the result
    // survives the Wi-Fi network switch and even a reboot.
    struct LightReport {
        bool valid = false;           // true once at least one command was executed
        std::string command;          // turn_on / turn_off / status / restore_a0 / reboot
        bool success = false;         // ESP8266 answered the HTTP request
        std::string error;            // human readable failure reason
        std::string response;         // raw answer body of the ESP8266
        std::string relay;            // "on" / "off"
        std::string mode;             // "auto" (A0 control) / "manual"
        int adc = -1;                 // current A0 (ADC) value, 0..1023
        int threshold = -1;           // A0 threshold used by the auto mode
        std::string home_ssid;        // network of the xiaozhi before the switch
        bool wifi_restored = false;   // xiaozhi reconnected to that network
        uint32_t uptime_sec = 0;      // seconds since boot when the answer arrived
    };

    struct LightCommandTaskParam {
        MyBoard* self;
        int reply_id;
        std::string action;
    };

    static std::atomic<bool> light_command_running_;
    static std::mutex light_report_mutex_;
    static LightReport last_light_report_;

public:
    struct DnsQuery {
        uint32_t timestamp_sec;
        std::string hostname;
    };
    static std::vector<DnsQuery> dns_log_;
    static constexpr size_t MAX_DNS_LOG = 50;

    static void SaveSnifferLogToNvs() {
        nvs_handle_t handle;
        if (nvs_open(NVS_SNIFFER_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to open NVS for sniffer log");
            return;
        }

        // Clear old entries first
        nvs_erase_all(handle);

        // Save count
        uint32_t count = sniffer_log_.size();
        nvs_set_u32(handle, "count", count);

        // Save each entry
        for (int i = 0; i < (int)count; i++) {
            std::string key = "s" + std::to_string(i);
            std::string value = sniffer_log_[i].short_info + "|" + sniffer_log_[i].description;
            nvs_set_str(handle, key.c_str(), value.c_str());
        }

        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "Saved %d sniffer entries to NVS", count);
    }

    static void LoadSnifferLogFromNvs() {
        nvs_handle_t handle;
        if (nvs_open(NVS_SNIFFER_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
            ESP_LOGI(TAG, "No saved sniffer log found");
            return;
        }

        uint32_t count = 0;
        if (nvs_get_u32(handle, "count", &count) != ESP_OK || count == 0) {
            nvs_close(handle);
            return;
        }

        sniffer_log_.clear();
        for (int i = 0; i < (int)count && i < MAX_SNIFFER_LOG; i++) {
            std::string key = "s" + std::to_string(i);
            char buf[512];
            size_t len = sizeof(buf);
            if (nvs_get_str(handle, key.c_str(), buf, &len) == ESP_OK) {
                std::string val(buf);
                size_t sep = val.find('|');
                if (sep != std::string::npos) {
                    sniffer_log_.push_back({val.substr(sep + 1), val.substr(0, sep)});
                }
            }
        }

        nvs_close(handle);
        ESP_LOGI(TAG, "Loaded %d sniffer entries from NVS", (int)sniffer_log_.size());
    }

    static void ClearSnifferLogFromNvs() {
        nvs_handle_t handle;
        if (nvs_open(NVS_SNIFFER_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
            nvs_erase_all(handle);
            nvs_commit(handle);
            nvs_close(handle);
        }
        sniffer_log_.clear();
        ESP_LOGI(TAG, "Sniffer log cleared");
    }

    Button boot_button_;
    Button volume_up_button_;
    Button volume_down_button_;
    SpiLcdDisplay* display_;
    PowerSaveTimer* power_save_timer_;
    PowerManager* power_manager_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;

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

        // Load persisted sniffer log from NVS
        LoadSnifferLogFromNvs();

        // Load the remembered answer of the last light command from NVS
        LoadLightReportFromNvs();
    }

    static void WifiScanTask(void* param) {
        auto* p = static_cast<WifiScanTaskParam*>(param);
        auto* self = p->self;
        int captured_id = p->reply_id;
        delete p;

        auto display = self->GetDisplay();
        auto& wifi_mgr = WifiManager::GetInstance();

        display->SetChatMessage("system", "Scanning WiFi...");
        ESP_LOGI(TAG, "Background scan task started on core %d, priority %d", xPortGetCoreID(), uxTaskPriorityGet(nullptr));

        wifi_mgr.SetExternalScanMode(true);
        esp_wifi_set_ps(WIFI_PS_NONE);
        vTaskDelay(pdMS_TO_TICKS(500));

        uint16_t ap_count = 0;
        wifi_ap_record_t* ap_records = nullptr;

        for (int attempt = 1; attempt <= 3; attempt++) {
            wifi_scan_config_t scan_config = {};
            scan_config.show_hidden = false;
            scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
            scan_config.scan_time.active.min = 100;
            scan_config.scan_time.active.max = 300;

            esp_err_t err = esp_wifi_scan_start(&scan_config, true);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "scan_start failed: %s", esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }

            esp_wifi_scan_get_ap_num(&ap_count);
            ESP_LOGI(TAG, "Attempt %d: found %d APs", attempt, ap_count);

            if (ap_count > 0) {
                ap_records = new wifi_ap_record_t[ap_count];
                esp_wifi_scan_get_ap_records(&ap_count, ap_records);
                break;
            }

            if (attempt < 3) vTaskDelay(pdMS_TO_TICKS(1500));
        }

        wifi_mgr.SetExternalScanMode(false);
        esp_wifi_set_ps(WIFI_PS_MAX_MODEM);

        cJSON* root = cJSON_CreateArray();
        std::string summary;
        int displayed = 0;

        if (ap_records) {
            for (int i = 0; i < ap_count; i++) {
                std::string ssid = reinterpret_cast<char*>(ap_records[i].ssid);
                if (ssid.empty()) continue;

                ESP_LOGI(TAG, "  [%d] SSID=%s RSSI=%d AUTH=%d",
                         i, ssid.c_str(), ap_records[i].rssi, ap_records[i].authmode);

                cJSON* net = cJSON_CreateObject();
                cJSON_AddStringToObject(net, "ssid", ssid.c_str());
                cJSON_AddNumberToObject(net, "rssi", ap_records[i].rssi);

                const char* auth = "open";
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

        char msg[128];
        snprintf(msg, sizeof(msg), "Found %d networks", ap_count);
        display->SetChatMessage("system", msg);
        ESP_LOGI(TAG, "Scan complete: %s", msg);

        if (summary.empty()) summary = "No WiFi networks found";

        char* json_str = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);

        cJSON* wrapper = cJSON_CreateObject();
        cJSON* content = cJSON_CreateArray();
        cJSON* text = cJSON_CreateObject();
        cJSON_AddStringToObject(text, "type", "text");
        cJSON_AddStringToObject(text, "text", json_str);
        cJSON_free(json_str);
        cJSON_AddItemToArray(content, text);
        cJSON_AddItemToObject(wrapper, "content", content);
        cJSON_AddBoolToObject(wrapper, "isError", false);

        char* wrapper_str = cJSON_PrintUnformatted(wrapper);
        std::string result_str(wrapper_str);
        cJSON_free(wrapper_str);
        cJSON_Delete(wrapper);

        ESP_LOGI(TAG, "MCP reply payload: %s", result_str.c_str());
        McpServer::GetInstance().SendReply(captured_id, result_str);
        ESP_LOGI(TAG, "MCP reply sent for id=%d", captured_id);

        vTaskDelete(nullptr);
    }

    static void WifiConnectTask(void* arg) {
        auto* param = static_cast<WifiConnectTaskParam*>(arg);
        int captured_id = param->reply_id;
        std::string ssid = std::move(param->ssid);
        std::string password = std::move(param->password);
        delete param;

        ESP_LOGI(TAG, "WiFi connect task started for SSID: %s", ssid.c_str());

        auto& wifi_mgr = WifiManager::GetInstance();

        // Save credentials to NVS
        auto& ssid_manager = SsidManager::GetInstance();
        ssid_manager.AddSsid(ssid, password);
        ESP_LOGI(TAG, "Saved SSID to NVS: %s", ssid.c_str());

        // Stop current connection and reconnect with new credentials
        wifi_mgr.StopStation();
        vTaskDelay(pdMS_TO_TICKS(500));
        wifi_mgr.StartStation();

        // Wait for connection with 15s timeout
        auto start_time = xTaskGetTickCount();
        const TickType_t timeout_ticks = pdMS_TO_TICKS(15000);
        bool connected = false;

        while ((xTaskGetTickCount() - start_time) < timeout_ticks) {
            if (wifi_mgr.IsConnected()) {
                connected = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        // Build MCP reply
        cJSON* wrapper = cJSON_CreateObject();
        cJSON* content = cJSON_CreateArray();
        cJSON* text = cJSON_CreateObject();
        cJSON_AddStringToObject(text, "type", "text");

        if (connected) {
            std::string msg = "Successfully connected to " + ssid + ".";
            cJSON_AddStringToObject(text, "text", msg.c_str());
            ESP_LOGI(TAG, "Connected to %s", ssid.c_str());
        } else {
            std::string msg = "Failed to connect to " + ssid + " within 15 seconds. "
                              "Please check the password and try again, or use captive portal at http://192.168.4.1.";
            cJSON_AddStringToObject(text, "text", msg.c_str());
            ESP_LOGW(TAG, "Connection to %s timed out", ssid.c_str());
        }

        cJSON_AddItemToArray(content, text);
        cJSON_AddItemToObject(wrapper, "content", content);
        cJSON_AddBoolToObject(wrapper, "isError", !connected);

        char* wrapper_str = cJSON_PrintUnformatted(wrapper);
        std::string result_str(wrapper_str);
        cJSON_free(wrapper_str);
        cJSON_Delete(wrapper);

        ESP_LOGI(TAG, "MCP reply payload: %s", result_str.c_str());
        McpServer::GetInstance().SendReply(captured_id, result_str);
        ESP_LOGI(TAG, "MCP reply sent for id=%d", captured_id);

        vTaskDelete(nullptr);
    }

    static void SnifferCallback(void* recv_buf, wifi_promiscuous_pkt_type_t type) {
        if (!sniffer_active_) return;
        if (type == WIFI_PKT_MISC) return;

        auto* pkt = (wifi_promiscuous_pkt_t*)recv_buf;
        const uint8_t* frame = pkt->payload;
        uint16_t len = pkt->rx_ctrl.sig_len;
        int8_t rssi = pkt->rx_ctrl.rssi;
        uint8_t channel = pkt->rx_ctrl.channel;

        if (len < 2) return;

        uint16_t frame_ctrl = frame[0] | (frame[1] << 8);
        uint8_t frame_subtype = (frame_ctrl >> 4) & 0xf;

        char sa[18] = "??:??:??:??:??:??";
        char da[18] = "??:??:??:??:??:??";
        char bssid[18] = "??:??:??:??:??:??";
        if (len >= 24) {
            snprintf(sa, sizeof(sa), "%02x:%02x:%02x:%02x:%02x:%02x", frame[10], frame[11], frame[12], frame[13], frame[14], frame[15]);
            snprintf(da, sizeof(da), "%02x:%02x:%02x:%02x:%02x:%02x", frame[4], frame[5], frame[6], frame[7], frame[8], frame[9]);
            snprintf(bssid, sizeof(bssid), "%02x:%02x:%02x:%02x:%02x:%02x", frame[16], frame[17], frame[18], frame[19], frame[20], frame[21]);
        }

        char desc[1024] = {0};
        char info[128] = {0};

        if (type == WIFI_PKT_MGMT) {
            if (frame_subtype == 0x08 && len > 24) {
                // Beacon
                char ssid[33] = {0};
                int offset = 24;
                while (offset + 1 < len) {
                    uint8_t eid = frame[offset];
                    uint8_t elen = frame[offset + 1];
                    if (eid == 0 && elen > 0 && elen < 33) {
                        memcpy(ssid, frame + offset + 2, elen);
                        break;
                    }
                    offset += 2 + elen;
                }
                snprintf(info, sizeof(info), "BEACON ch%d", channel);
                snprintf(desc, sizeof(desc), "BEACON %s \"%s\" %ddBm", sa, ssid, rssi);
            } else if (frame_subtype == 0x04 && len > 24) {
                // Probe Request
                char ssid[33] = {0};
                int offset = 24;
                while (offset + 1 < len) {
                    uint8_t eid = frame[offset];
                    uint8_t elen = frame[offset + 1];
                    if (eid == 0 && elen > 0 && elen < 33) {
                        memcpy(ssid, frame + offset + 2, elen);
                        break;
                    }
                    offset += 2 + elen;
                }
                snprintf(info, sizeof(info), "PROBE_REQ");
                snprintf(desc, sizeof(desc), "PROBE_REQ %s \"%s\" %ddBm", sa, ssid, rssi);
            } else if (frame_subtype == 0x0C) {
                snprintf(info, sizeof(info), "DEAUTH");
                snprintf(desc, sizeof(desc), "DEAUTH %s -> %s %ddBm", sa, da, rssi);
            } else if (frame_subtype == 0x00) {
                snprintf(info, sizeof(info), "ASSOC_REQ");
                snprintf(desc, sizeof(desc), "ASSOC_REQ %s -> %s %ddBm", sa, bssid, rssi);
            } else if (frame_subtype == 0x01) {
                snprintf(info, sizeof(info), "ASSOC_RESP");
                snprintf(desc, sizeof(desc), "ASSOC_RESP %s -> %s %ddBm", bssid, sa, rssi);
            } else if (frame_subtype == 0x0B) {
                snprintf(info, sizeof(info), "AUTH");
                snprintf(desc, sizeof(desc), "AUTH %s -> %s %ddBm", sa, bssid, rssi);
            } else if (frame_subtype == 0x0D) {
                snprintf(info, sizeof(info), "DISASSOC");
                snprintf(desc, sizeof(desc), "DISASSOC %s -> %s %ddBm", sa, da, rssi);
            } else {
                snprintf(info, sizeof(info), "MGMT_%02x", frame_subtype);
                snprintf(desc, sizeof(desc), "MGMT sub=%d from=%s %ddBm", frame_subtype, sa, rssi);
            }
        }
        else if (type == WIFI_PKT_DATA) {
            if (len < 24) return;

            // Try to parse LLC/SNAP + IPv4 for extra detail
            int snap_offset = -1;
            if (len > 24) {
                // Find LLC/SNAP header
                for (int i = 24; i < (int)len - 8; i++) {
                    if (frame[i] == 0xaa && frame[i+1] == 0xaa && frame[i+2] == 0x03) {
                        snap_offset = i;
                        break;
                    }
                }
            }

            if (snap_offset >= 0) {
                uint16_t ethertype = (frame[snap_offset + 6] << 8) | frame[snap_offset + 7];
                int ip_offset = snap_offset + 8;

                if (ethertype == 0x0800 && ip_offset + 20 <= (int)len) {
                    // IPv4
                    uint8_t protocol = frame[ip_offset + 9];
                    char src_ip[16], dst_ip[16];
                    snprintf(src_ip, sizeof(src_ip), "%d.%d.%d.%d", frame[ip_offset+12], frame[ip_offset+13], frame[ip_offset+14], frame[ip_offset+15]);
                    snprintf(dst_ip, sizeof(dst_ip), "%d.%d.%d.%d", frame[ip_offset+16], frame[ip_offset+17], frame[ip_offset+18], frame[ip_offset+19]);

                    char extra[256] = {0};
                    std::string extra_str;
                    int transport_offset = ip_offset + (frame[ip_offset] & 0x0f) * 4;

                    if (protocol == 6 && transport_offset + 20 <= (int)len) {
                        uint16_t sport = (frame[transport_offset] << 8) | frame[transport_offset+1];
                        uint16_t dport = (frame[transport_offset+2] << 8) | frame[transport_offset+3];
                        uint8_t flags = frame[transport_offset + 13];
                        char flag_str[8] = {0};
                        int f = 0;
                        if (flags & 0x02) flag_str[f++] = 'S';
                        if (flags & 0x10) flag_str[f++] = 'A';
                        if (flags & 0x01) flag_str[f++] = 'F';
                        if (flags & 0x04) flag_str[f++] = 'R';
                        if (flags & 0x08) flag_str[f++] = 'P';
                        snprintf(extra, sizeof(extra), " %s:%d->%s:%d [%s]", src_ip, sport, dst_ip, dport, flag_str);
                        snprintf(info, sizeof(info), "TCP [%s]", flag_str);
                    } else if (protocol == 17 && transport_offset + 8 <= (int)len) {
                        uint16_t sport = (frame[transport_offset] << 8) | frame[transport_offset+1];
                        uint16_t dport = (frame[transport_offset+2] << 8) | frame[transport_offset+3];
                        snprintf(extra, sizeof(extra), " %s:%d->%s:%d", src_ip, sport, dst_ip, dport);

                        int udp_payload = transport_offset + 8;

                        if ((dport == 53 || sport == 53) && udp_payload + 12 <= (int)len) {
                            // DNS packet
                            uint16_t dns_flags = (frame[udp_payload+2] << 8) | frame[udp_payload+3];
                            uint16_t ancount = (frame[udp_payload+6] << 8) | frame[udp_payload+7];
                            bool is_response = (dns_flags >> 15) & 1;

                            char dns_name[256] = {0};
                            int pos = udp_payload + 12;

                            // Parse query name
                            for (int label = 0; label < 128 && pos < (int)len; label++) {
                                uint8_t label_len = frame[pos];
                                if (label_len == 0) { pos++; break; }
                                if (label_len > 63 || pos + 1 + label_len > (int)len) break;
                                if (label > 0 && strlen(dns_name) < sizeof(dns_name) - 1) {
                                    strcat(dns_name, ".");
                                }
                                int copy_len = label_len;
                                if (strlen(dns_name) + label_len >= sizeof(dns_name) - 1) {
                                    copy_len = sizeof(dns_name) - 2 - strlen(dns_name);
                                }
                                if (copy_len > 0) {
                                    strncat(dns_name, (const char*)(frame + pos + 1), copy_len);
                                }
                                pos += 1 + label_len;
                            }

                            // Skip query type and class
                            if (is_response) {
                                // Parse answers to get resolved IPs
                                pos += 4; // skip type + class in question
                                char resolved_ips[256] = {0};
                                for (int a = 0; a < ancount && a < 10 && pos + 10 <= (int)len; a++) {
                                    // Skip name (could be pointer)
                                    if ((frame[pos] & 0xC0) == 0xC0) {
                                        pos += 2;
                                    } else {
                                        while (pos < (int)len && frame[pos] != 0) pos++;
                                        pos++;
                                    }
                                    if (pos + 10 > (int)len) break;
                                    uint16_t rr_type = (frame[pos] << 8) | frame[pos+1];
                                    // uint16_t rr_len = (frame[pos+8] << 8) | frame[pos+9];
                                    pos += 10; // type(2) + class(2) + ttl(4) + rdlen(2)

                                    if (rr_type == 1 && pos + 4 <= (int)len) {
                                        // A record
                                        char a_ip[16];
                                        snprintf(a_ip, sizeof(a_ip), "%d.%d.%d.%d", frame[pos], frame[pos+1], frame[pos+2], frame[pos+3]);
                                        if (resolved_ips[0]) strcat(resolved_ips, ", ");
                                        strcat(resolved_ips, a_ip);
                                        pos += 4;
                                    } else if (rr_type == 28 && pos + 16 <= (int)len) {
                                        // AAAA record (skip)
                                        pos += 16;
                                    } else {
                                        uint16_t rdlen = (frame[pos-2] << 8) | frame[pos-1];
                                        pos += rdlen;
                                    }
                                }

                                snprintf(info, sizeof(info), "DNS_RESP");
                                if (dns_name[0] && resolved_ips[0]) {
                                    extra_str = std::string(" ") + src_ip + " -> " + dns_name + " [" + resolved_ips + "]";
                                } else if (dns_name[0]) {
                                    extra_str = std::string(" ") + src_ip + " -> " + dns_name;
                                }
                            } else {
                                // DNS query
                                snprintf(info, sizeof(info), "DNS_QUERY");
                                if (dns_name[0]) {
                                    uint16_t qtype = (frame[pos] << 8) | frame[pos+1];
                                    const char* type_str = "A";
                                    if (qtype == 28) type_str = "AAAA";
                                    else if (qtype == 12) type_str = "PTR";
                                    else if (qtype == 5) type_str = "CNAME";
                                    else if (qtype == 15) type_str = "MX";
                                    else if (qtype == 16) type_str = "TXT";
                                    extra_str = std::string(" ") + src_ip + " -> " + dns_name + " (" + type_str + ")";
                                }
                            }
                        } else if (dport == 5353 || sport == 5353) {
                            snprintf(info, sizeof(info), "mDNS");
                        } else {
                            snprintf(info, sizeof(info), "UDP");
                        }
                    } else {
                        snprintf(extra, sizeof(extra), " %s->%s proto=%d", src_ip, dst_ip, protocol);
                        snprintf(info, sizeof(info), "IP_%d", protocol);
                    }

                    // Build desc with string to avoid format-truncation
                    {
                        std::string extra_part = extra_str.empty() ? extra : extra_str;
                        std::string d = std::string("DATA ") + sa + "->" + da + " [" + info + "]" + extra_part + " " + std::to_string(rssi) + "dBm ch" + std::to_string(channel);
                        snprintf(desc, sizeof(desc), "%s", d.c_str());
                    }
                } else if (ethertype == 0x0806) {
                    snprintf(info, sizeof(info), "ARP");
                    snprintf(desc, sizeof(desc), "ARP %s %ddBm ch%d", sa, rssi, channel);
                } else {
                    snprintf(info, sizeof(info), "DATA_0x%04x", ethertype);
                    snprintf(desc, sizeof(desc), "DATA %s->%s eth=0x%04x %ddBm ch%d", sa, da, ethertype, rssi, channel);
                }
            } else {
                snprintf(info, sizeof(info), "DATA");
                snprintf(desc, sizeof(desc), "DATA %s->%s len=%d %ddBm ch%d", sa, da, len, rssi, channel);
            }
        }
        else {
            return;
        }

        if (desc[0] == 0) return;

        auto* entry = new SnifferPacket();
        entry->description = desc;
        entry->short_info = info;

        if (xQueueSend(sniffer_queue_, &entry, 0) != pdTRUE) {
            delete entry;
        }
    }

    static void SnifferTask(void* arg) {
        int duration_sec = (int)(uintptr_t)arg;
        ESP_LOGI(TAG, "Sniffer task started, will run for %d seconds", duration_sec);

        auto& app = Application::GetInstance();
        char msg[64];
        snprintf(msg, sizeof(msg), "Sniffer: %ds remaining...", duration_sec);
        app.Alert("Sniffer", msg, "radar", Lang::Sounds::OGG_POPUP);

        int elapsed = 0;
        while (sniffer_active_ && elapsed < duration_sec) {
            // Wait for packets with 1 second timeout
            SnifferPacket* pkt = nullptr;
            if (sniffer_queue_ && xQueueReceive(sniffer_queue_, &pkt, pdMS_TO_TICKS(1000)) == pdTRUE && pkt) {
                app.Alert("Network", pkt->short_info.c_str(), "wifi");

                if (sniffer_log_.size() >= MAX_SNIFFER_LOG) {
                    sniffer_log_.erase(sniffer_log_.begin());
                }
                sniffer_log_.push_back(*pkt);

                ESP_LOGI(TAG, "[SNIFFER] %s", pkt->description.c_str());
                delete pkt;
            }
            elapsed++;

            // Update countdown on screen every 5 seconds
            if (elapsed % 5 == 0 && elapsed < duration_sec) {
                snprintf(msg, sizeof(msg), "Sniffer: %ds remaining...", duration_sec - elapsed);
                app.Alert("Sniffer", msg, "radar");
            }
        }

        // Auto-stop: disable promiscuous mode
        sniffer_active_ = false;
        esp_wifi_set_promiscuous(false);

        // Re-enable WifiStation auto behavior
        auto& wifi_mgr = WifiManager::GetInstance();
        wifi_mgr.SetExternalScanMode(false);

        // Reconnect if we were in disconnected mode
        if (sniffer_disconnected_) {
            sniffer_disconnected_ = false;
            wifi_mgr.Reconnect();
        }

        if (sniffer_queue_) {
            SnifferPacket* pkt;
            while (xQueueReceive(sniffer_queue_, &pkt, 0) == pdTRUE) {
                delete pkt;
            }
            vQueueDelete(sniffer_queue_);
            sniffer_queue_ = nullptr;
        }

        ESP_LOGI(TAG, "Sniffer auto-stopped after %ds, %d packets captured",
                 duration_sec, (int)sniffer_log_.size());

        // Persist log to NVS
        SaveSnifferLogToNvs();

        char result_msg[128];
        snprintf(result_msg, sizeof(result_msg), "Sniffer done: %d packets in %ds.",
                 (int)sniffer_log_.size(), duration_sec);
        app.Alert("Sniffer", result_msg, "radar");

        sniffer_task_handle_ = nullptr;
        vTaskDelete(nullptr);
    }

    // ====================================================================
    // ESP8266 light control
    // Firmware: https://github.com/spartacus1988/esp8266-light/
    //
    // The ESP8266 light board does not use a router: it creates its own Wi-Fi
    // access point (LIGHT_AP_SSID). Every voice command therefore:
    //   1. leaves the current network and joins LIGHT_AP_SSID,
    //   2. sends an HTTP request to the ESP8266 web server,
    //   3. remembers the answer of the ESP8266 (RAM + NVS),
    //   4. reconnects to the network it was using before,
    //   5. reports the remembered answer to the LLM once the connection to the
    //      server is restored (deferred MCP reply of the original tool call).
    // ====================================================================

    static std::string LightPassword() {
        // A saved network with the same name wins (custom password)
        for (const auto& item : SsidManager::GetInstance().GetSsidList()) {
            if (item.ssid == LIGHT_AP_SSID && !item.password.empty()) {
                return item.password;
            }
        }
        return LIGHT_AP_PASSWORD;
    }

    static bool StaIpInLightSubnet(std::string& ip_out) {
        auto* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif == nullptr) {
            return false;
        }
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK || ip_info.ip.addr == 0) {
            return false;
        }
        char ip[16] = {};
        esp_ip4addr_ntoa(&ip_info.ip, ip, sizeof(ip));
        ip_out = ip;
        return strncmp(ip, LIGHT_AP_IP_PREFIX, strlen(LIGHT_AP_IP_PREFIX)) == 0;
    }

    static bool ApplyStaticIp() {
        auto* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif == nullptr) {
            return false;
        }
        // Fallback for an ESP8266 AP that does not run a DHCP server
        esp_netif_dhcpc_stop(netif);
        esp_netif_ip_info_t ip_info = {};
        ip_info.ip.addr = esp_ip4addr_aton(LIGHT_STATIC_FALLBACK_IP);
        ip_info.gw.addr = esp_ip4addr_aton(LIGHT_DEVICE_HOST);
        ip_info.netmask.addr = esp_ip4addr_aton("255.255.255.0");
        esp_err_t err = esp_netif_set_ip_info(netif, &ip_info);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "[LIGHT] cannot set static IP: %s", esp_err_to_name(err));
            return false;
        }
        return true;
    }

    static bool JoinLightAp(std::string& error) {
        auto& wifi_mgr = WifiManager::GetInstance();

        // Take the radio away from WifiStation so it neither scans nor reconnects
        wifi_mgr.SetExternalScanMode(true);
        esp_wifi_set_ps(WIFI_PS_NONE);

        esp_wifi_disconnect();
        for (int i = 0; i < 30 && wifi_mgr.IsConnected(); i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        std::string password = LightPassword();
        wifi_config_t config = {};
        strncpy(reinterpret_cast<char*>(config.sta.ssid), LIGHT_AP_SSID, sizeof(config.sta.ssid) - 1);
        strncpy(reinterpret_cast<char*>(config.sta.password), password.c_str(), sizeof(config.sta.password) - 1);
        config.sta.scan_method = WIFI_FAST_SCAN;
        config.sta.threshold.authmode = WIFI_AUTH_OPEN;
        esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &config);
        if (err != ESP_OK) {
            error = std::string("Cannot configure WiFi: ") + esp_err_to_name(err);
            return false;
        }

        ESP_LOGI(TAG, "[LIGHT] joining %s", LIGHT_AP_SSID);
        err = esp_wifi_connect();
        if (err != ESP_OK) {
            error = std::string("Cannot start WiFi connection: ") + esp_err_to_name(err);
            return false;
        }

        std::string ip;
        bool static_ip_applied = false;
        for (int waited = 0; waited < LIGHT_JOIN_TIMEOUT_MS; waited += 200) {
            if (StaIpInLightSubnet(ip)) {
                ESP_LOGI(TAG, "[LIGHT] joined %s with IP %s", LIGHT_AP_SSID, ip.c_str());
                return true;
            }
            if (!static_ip_applied && waited >= LIGHT_DHCP_TIMEOUT_MS) {
                static_ip_applied = true;
                ESP_LOGW(TAG, "[LIGHT] no DHCP lease from %s, trying static IP %s",
                         LIGHT_AP_SSID, LIGHT_STATIC_FALLBACK_IP);
                ApplyStaticIp();
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        error = std::string("Cannot join WiFi network ") + LIGHT_AP_SSID +
                " (is the esp8266 light board powered on?)";
        return false;
    }

    static bool RestoreHomeNetwork(const std::string& original_ssid, std::string& error) {
        auto& wifi_mgr = WifiManager::GetInstance();

        // Give the radio back to WifiStation: it rescans the saved networks
        wifi_mgr.SetExternalScanMode(false);
        esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
        wifi_mgr.Reconnect();

        for (int waited = 0; waited < LIGHT_RESTORE_TIMEOUT_MS; waited += 500) {
            if (wifi_mgr.IsConnected()) {
                std::string ssid = wifi_mgr.GetSsid();
                if (original_ssid.empty() || ssid == original_ssid || waited >= 8000) {
                    ESP_LOGI(TAG, "[LIGHT] back on '%s' (%s)", ssid.c_str(), wifi_mgr.GetIpAddress().c_str());
                    return true;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        if (wifi_mgr.IsConnected()) {
            ESP_LOGW(TAG, "[LIGHT] connected to '%s' instead of '%s'",
                     wifi_mgr.GetSsid().c_str(), original_ssid.c_str());
            return true;
        }

        error = "Cannot reconnect to the original network '" + original_ssid + "'";
        return false;
    }

    static void SendLightCommand(const std::string& action, LightReport& report) {
        std::string url = std::string("http://") + LIGHT_DEVICE_HOST;
        if (action == "turn_on") {
            url += "/api?relay=on";
        } else if (action == "turn_off") {
            url += "/api?relay=off";
        } else if (action == "status") {
            url += "/api";
        } else if (action == "restore_a0") {
            url += "/api?mode=auto";
        } else if (action == "reboot") {
            url += "/?reset=1";   // the board answers with a redirect and resets itself
        } else {
            report.error = "Unknown light action: " + action;
            return;
        }

        ESP_LOGI(TAG, "[LIGHT] GET %s", url.c_str());
        auto http = Board::GetInstance().GetNetwork()->CreateHttp(0);
        http->SetTimeout(5000);
        if (!http->Open("GET", url)) {
            report.error = "No HTTP answer from the esp8266 light board";
            http->Close();
            return;
        }

        int status = http->GetStatusCode();
        std::string body = http->ReadAll();
        http->Close();

        if (action == "reboot") {
            if (status == 200 || status == 302 || status == 303) {
                report.success = true;
                report.response = "reset requested (HTTP " + std::to_string(status) + ")";
            } else {
                report.error = "Unexpected HTTP status " + std::to_string(status) + " on reboot request";
            }
            return;
        }

        if (status != 200) {
            report.error = "Unexpected HTTP status " + std::to_string(status);
            report.response = body;
            return;
        }

        // {"relay":"on","mode":"manual","adc":512,"threshold":512,"ip":"192.168.4.1"}
        cJSON* root = cJSON_Parse(body.c_str());
        if (root == nullptr) {
            report.error = "Cannot parse the answer of the esp8266";
            report.response = body;
            return;
        }
        cJSON* relay = cJSON_GetObjectItem(root, "relay");
        cJSON* mode = cJSON_GetObjectItem(root, "mode");
        cJSON* adc = cJSON_GetObjectItem(root, "adc");
        cJSON* threshold = cJSON_GetObjectItem(root, "threshold");
        if (cJSON_IsString(relay)) report.relay = relay->valuestring;
        if (cJSON_IsString(mode)) report.mode = mode->valuestring;
        if (cJSON_IsNumber(adc)) report.adc = adc->valueint;
        if (cJSON_IsNumber(threshold)) report.threshold = threshold->valueint;
        cJSON_Delete(root);

        report.response = body;
        if (report.relay.empty()) {
            report.error = "The esp8266 answer does not contain the relay state";
            return;
        }
        report.success = true;
    }

    static std::string LightReportToJson(const LightReport& report) {
        cJSON* root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "command", report.command.c_str());
        cJSON_AddBoolToObject(root, "success", report.success);
        if (!report.error.empty()) cJSON_AddStringToObject(root, "error", report.error.c_str());
        if (!report.response.empty()) cJSON_AddStringToObject(root, "esp8266_response", report.response.c_str());
        if (!report.relay.empty()) cJSON_AddStringToObject(root, "relay", report.relay.c_str());
        if (!report.mode.empty()) cJSON_AddStringToObject(root, "mode", report.mode.c_str());
        if (report.adc >= 0) cJSON_AddNumberToObject(root, "a0_adc", report.adc);
        if (report.threshold >= 0) cJSON_AddNumberToObject(root, "a0_threshold", report.threshold);
        cJSON_AddStringToObject(root, "wifi_ssid", report.home_ssid.c_str());
        cJSON_AddBoolToObject(root, "wifi_restored", report.wifi_restored);
        cJSON_AddNumberToObject(root, "seconds_since_boot", report.uptime_sec);

        char* printed = cJSON_PrintUnformatted(root);
        std::string result(printed);
        cJSON_free(printed);
        cJSON_Delete(root);
        return result;
    }

    static std::string LightSummary(const LightReport& report) {
        std::string summary;
        if (report.success) {
            summary = "Light command '" + report.command + "' executed on the esp8266 light board.";
            if (!report.response.empty()) summary += " Board replied: " + report.response + ".";
            if (!report.relay.empty()) summary += " Relay is " + report.relay + ".";
            if (!report.mode.empty()) summary += " Control mode: " + report.mode +
                                                 (report.mode == "auto" ? " (relay follows the A0 input)." : ".");
            if (report.adc >= 0) summary += " A0 (ADC) reading: " + std::to_string(report.adc) +
                                            " (threshold " + std::to_string(report.threshold) + ").";
        } else {
            summary = "Light command '" + report.command + "' failed: " +
                      (report.error.empty() ? std::string("unknown error") : report.error) + ".";
        }
        if (report.wifi_restored) {
            summary += " The device reconnected to WiFi '" + report.home_ssid + "'.";
        } else {
            summary += " WARNING: the device could not reconnect to WiFi '" + report.home_ssid + "'.";
        }
        return summary;
    }

    static std::string LightMcpResult(const LightReport& report, const std::string& summary) {
        cJSON* wrapper = cJSON_CreateObject();
        cJSON* content = cJSON_CreateArray();
        cJSON* text = cJSON_CreateObject();
        cJSON_AddStringToObject(text, "type", "text");
        cJSON_AddStringToObject(text, "text", summary.c_str());
        cJSON_AddItemToArray(content, text);
        cJSON_AddItemToObject(wrapper, "content", content);
        cJSON_AddBoolToObject(wrapper, "isError", !report.success);

        char* printed = cJSON_PrintUnformatted(wrapper);
        std::string result(printed);
        cJSON_free(printed);
        cJSON_Delete(wrapper);
        return result;
    }

    static void SaveLightReportToNvs(const LightReport& report) {
        nvs_handle_t handle;
        if (nvs_open("light", NVS_READWRITE, &handle) != ESP_OK) {
            ESP_LOGE(TAG, "[LIGHT] cannot store the answer in NVS");
            return;
        }
        nvs_set_u8(handle, "valid", 1);
        nvs_set_str(handle, "command", report.command.c_str());
        nvs_set_u8(handle, "ok", report.success ? 1 : 0);
        nvs_set_str(handle, "error", report.error.c_str());
        nvs_set_str(handle, "response", report.response.c_str());
        nvs_set_str(handle, "relay", report.relay.c_str());
        nvs_set_str(handle, "mode", report.mode.c_str());
        nvs_set_i32(handle, "adc", report.adc);
        nvs_set_i32(handle, "threshold", report.threshold);
        nvs_set_str(handle, "ssid", report.home_ssid.c_str());
        nvs_set_u8(handle, "restored", report.wifi_restored ? 1 : 0);
        nvs_set_u32(handle, "uptime", report.uptime_sec);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "[LIGHT] answer stored in NVS");
    }

    static void LoadLightReportFromNvs() {
        nvs_handle_t handle;
        if (nvs_open("light", NVS_READONLY, &handle) != ESP_OK) {
            return;
        }
        uint8_t valid = 0;
        if (nvs_get_u8(handle, "valid", &valid) != ESP_OK || valid == 0) {
            nvs_close(handle);
            return;
        }

        LightReport report;
        report.valid = true;
        char buf[768];
        size_t len;

        len = sizeof(buf); if (nvs_get_str(handle, "command", buf, &len) == ESP_OK) report.command = buf;
        uint8_t ok = 0; nvs_get_u8(handle, "ok", &ok); report.success = ok != 0;
        len = sizeof(buf); if (nvs_get_str(handle, "error", buf, &len) == ESP_OK) report.error = buf;
        len = sizeof(buf); if (nvs_get_str(handle, "response", buf, &len) == ESP_OK) report.response = buf;
        len = sizeof(buf); if (nvs_get_str(handle, "relay", buf, &len) == ESP_OK) report.relay = buf;
        len = sizeof(buf); if (nvs_get_str(handle, "mode", buf, &len) == ESP_OK) report.mode = buf;
        int32_t adc = -1, threshold = -1;
        nvs_get_i32(handle, "adc", &adc);
        nvs_get_i32(handle, "threshold", &threshold);
        report.adc = adc;
        report.threshold = threshold;
        len = sizeof(buf); if (nvs_get_str(handle, "ssid", buf, &len) == ESP_OK) report.home_ssid = buf;
        uint8_t restored = 0; nvs_get_u8(handle, "restored", &restored); report.wifi_restored = restored != 0;
        nvs_get_u32(handle, "uptime", &report.uptime_sec);
        nvs_close(handle);

        {
            std::lock_guard<std::mutex> lock(light_report_mutex_);
            last_light_report_ = report;
        }
        ESP_LOGI(TAG, "[LIGHT] remembered answer restored from NVS: %s", report.command.c_str());
    }

#if LIGHT_AUTO_REPORT
    // Bring the audio channel to the LLM back up. The old channel died together
    // with the previous network, so the conversation is restarted from idle.
    static bool ReopenAudioChannel() {
        auto& app = Application::GetInstance();

        auto state = app.GetDeviceState();
        if (state == kDeviceStateConnecting || state == kDeviceStateListening || state == kDeviceStateSpeaking) {
            if (!app.SetDeviceState(kDeviceStateIdle)) {
                ESP_LOGW(TAG, "[LIGHT] cannot leave device state %d", static_cast<int>(state));
                return false;
            }
        }

        for (int waited = 0; waited < LIGHT_REPORT_TIMEOUT_MS; waited += 500) {
            state = app.GetDeviceState();
            if (state == kDeviceStateListening || state == kDeviceStateSpeaking) {
                return true;
            }
            if (state == kDeviceStateIdle) {
                // Opens the audio channel when it is closed (HandleToggleChatEvent)
                app.ToggleChatState();
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        ESP_LOGW(TAG, "[LIGHT] audio channel did not open within %d ms", LIGHT_REPORT_TIMEOUT_MS);
        if (app.GetDeviceState() == kDeviceStateConnecting) {
            app.SetDeviceState(kDeviceStateIdle);   // do not stay in "connecting"
        }
        return false;
    }

    // Do not leave the device in the listening state we opened ourselves
    static void SettleAfterReport() {
        auto& app = Application::GetInstance();

        int total = 0;
        int quiet = 0;
        while (total < 45000) {
            auto state = app.GetDeviceState();
            if (state == kDeviceStateIdle) {
                return;
            }
            if (state == kDeviceStateListening) {
                quiet += 500;
                if (quiet >= 6000) {
                    app.StopListening();
                    return;
                }
            } else {
                quiet = 0;   // the model is speaking - keep waiting
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            total += 500;
        }

        // The model took longer than the budget: wait for it to finish anyway
        for (int i = 0; i < 60 && app.GetDeviceState() == kDeviceStateSpeaking; i++) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        if (app.GetDeviceState() == kDeviceStateListening) {
            app.StopListening();
        }
    }
#endif // LIGHT_AUTO_REPORT

    // Report the remembered answer of the ESP8266 to the LLM
    static void ReportToLlm(MyBoard* self, int reply_id, const LightReport& report) {
        std::string summary = LightSummary(report);
        std::string mcp_result = LightMcpResult(report, summary);
        ESP_LOGI(TAG, "[LIGHT] %s", summary.c_str());

        if (reply_id >= 0) {
#if LIGHT_AUTO_REPORT
            // The connection to the LLM died with the previous network: open the
            // audio channel again so the remembered answer can reach the model.
            if (!ReopenAudioChannel()) {
                ESP_LOGW(TAG, "[LIGHT] LLM channel unavailable, answer kept for self.light.get_last_report");
            }
#endif
            // Deferred MCP reply of the original tool call
            McpServer::GetInstance().SendReply(reply_id, mcp_result);
            ESP_LOGI(TAG, "[LIGHT] deferred MCP reply sent for id=%d", reply_id);
#if LIGHT_AUTO_REPORT
            // Give the model time to speak the result, then leave the
            // conversation that was opened only for this report.
            SettleAfterReport();
#endif
        }

        // Show the remembered answer on the device
        auto display = self->GetDisplay();
        display->ShowNotification(report.success ? "Light command done" : "Light command failed", 4000);
        display->SetChatMessage("system", summary.c_str());
    }

    static void LightCommandTask(void* param) {
        auto* p = static_cast<LightCommandTaskParam*>(param);
        MyBoard* self = p->self;
        int reply_id = p->reply_id;
        std::string action = std::move(p->action);
        delete p;

        auto& wifi_mgr = WifiManager::GetInstance();
        auto display = self->GetDisplay();

        // Remember where we are before switching networks
        LightReport report;
        report.valid = true;
        report.command = action;
        report.home_ssid = wifi_mgr.GetSsid();
        report.uptime_sec = static_cast<uint32_t>(esp_timer_get_time() / 1000000LL);
        ESP_LOGI(TAG, "[LIGHT] command '%s' started, home network '%s'",
                 action.c_str(), report.home_ssid.c_str());

        display->SetChatMessage("system", "Light: connecting to " LIGHT_AP_SSID "...");

        // 1+2. Join the ESP8266 access point and send the HTTP command
        std::string error;
        if (JoinLightAp(error)) {
            SendLightCommand(action, report);
        } else {
            report.success = false;
            report.error = error;
        }

        // 3. Remember the answer of the ESP8266 right away (RAM + NVS)
        report.uptime_sec = static_cast<uint32_t>(esp_timer_get_time() / 1000000LL);
        {
            std::lock_guard<std::mutex> lock(light_report_mutex_);
            last_light_report_ = report;
        }
        ESP_LOGI(TAG, "[LIGHT] esp8266 answer: %s",
                 report.response.empty() ? report.error.c_str() : report.response.c_str());

        // 4. Switch back to the network we were using before
        display->SetChatMessage("system", "Light: restoring WiFi ...");
        std::string restore_error;
        report.wifi_restored = RestoreHomeNetwork(report.home_ssid, restore_error);
        if (!report.wifi_restored) {
            report.error = report.error.empty() ? restore_error : report.error + "; " + restore_error;
            ESP_LOGE(TAG, "[LIGHT] %s", restore_error.c_str());
        }
        {
            std::lock_guard<std::mutex> lock(light_report_mutex_);
            last_light_report_ = report;
        }
        SaveLightReportToNvs(report);

        // 5. Report the remembered answer once the connection is back
        if (report.wifi_restored) {
            ReportToLlm(self, reply_id, report);
        } else {
            ESP_LOGW(TAG, "[LIGHT] report kept for self.light.get_last_report (id=%d not delivered)", reply_id);
            display->ShowNotification("Light: WiFi not restored", 5000);
            display->SetChatMessage("system", LightSummary(report).c_str());
        }

        light_command_running_ = false;
        ESP_LOGI(TAG, "[LIGHT] command task finished");
        vTaskDelete(nullptr);
    }

    // Starts a light command in the background; the MCP reply is deferred and
    // sent only after the ESP8266 answered and the network is restored.
    ReturnValue StartLightCommand(const std::string& action) {
        auto& wifi_mgr = WifiManager::GetInstance();

        if (!wifi_mgr.IsInitialized()) {
            return std::string("{\"error\":\"WiFi not initialized\"}");
        }
        if (wifi_mgr.IsConfigMode()) {
            return std::string("{\"error\":\"WiFi is in configuration mode\"}");
        }
        if (sniffer_active_) {
            return std::string("{\"error\":\"The WiFi sniffer is running, cannot switch networks now\"}");
        }

        bool expected = false;
        if (!light_command_running_.compare_exchange_strong(expected, true)) {
            return std::string("{\"error\":\"Another light command is already in progress\"}");
        }

        int reply_id = McpServer::GetInstance().GetPendingToolCallId();
        ESP_LOGI(TAG, "[LIGHT] '%s' requested, deferred reply id=%d", action.c_str(), reply_id);

        McpServer::GetInstance().SetDeferredReply(true);
        auto* param = new LightCommandTaskParam{this, reply_id, action};
        if (xTaskCreatePinnedToCore(LightCommandTask, "light_cmd", 8192, param, 18, nullptr, 1) != pdPASS) {
            delete param;
            light_command_running_ = false;
            McpServer::GetInstance().SetDeferredReply(false);
            return std::string("{\"error\":\"Failed to start the light command task\"}");
        }

        return std::string("Executing the light command on the " LIGHT_AP_SSID " network. "
                           "The device will report the result after it reconnects.");
    }

    void InitializeTools() {
        auto &mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.wifi.scan_networks",
            "Scan all available WiFi networks nearby and return the list.\n"
            "Returns: A list of WiFi networks with SSID, signal strength (RSSI), and encryption type.\n"
            "Use this tool when the user wants to see available WiFi networks or check WiFi signal.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                auto& wifi_mgr = WifiManager::GetInstance();
                int reply_id = McpServer::GetInstance().GetPendingToolCallId();

                ESP_LOGI(TAG, "WiFi scan requested, reply_id=%d", reply_id);

                if (!wifi_mgr.IsInitialized()) {
                    return std::string("{\"error\":\"WiFi not initialized\"}");
                }
                if (wifi_mgr.IsConfigMode()) {
                    return std::string("{\"error\":\"WiFi in config AP mode\"}");
                }

                auto* self = this;
                int captured_id = reply_id;
                McpServer::GetInstance().SetDeferredReply(true);
                auto* param = new WifiScanTaskParam{self, captured_id};
                xTaskCreatePinnedToCore(WifiScanTask, "wifi_scan", 8192, param, 24, nullptr, 0);

                return std::string("WiFi scan in progress. Results will be announced shortly.");
            });

        mcp_server.AddTool("self.wifi.connect_network",
            "Connect to a WiFi network.\n"
            "Args:\n"
            "  `ssid`: The name of the WiFi network to connect to.\n"
            "  `password`: The password for the WiFi network (optional if already saved).\n"
            "If the network was previously connected, the device already has the password saved and will connect automatically.\n"
            "Only ask the user for the password if the network has never been connected before.",
            PropertyList({
                Property("ssid", kPropertyTypeString),
                Property("password", kPropertyTypeString, std::string(""))
            }),
            [](const PropertyList& properties) -> ReturnValue {
                auto& wifi_mgr = WifiManager::GetInstance();
                int reply_id = McpServer::GetInstance().GetPendingToolCallId();
                std::string ssid = properties["ssid"].value<std::string>();
                std::string password = properties["password"].value<std::string>();

                ESP_LOGI(TAG, "WiFi connect requested: ssid=%s, reply_id=%d", ssid.c_str(), reply_id);

                if (!wifi_mgr.IsInitialized()) {
                    return std::string("{\"error\":\"WiFi not initialized\"}");
                }
                if (ssid.empty()) {
                    return std::string("{\"error\":\"SSID is required\"}");
                }

                bool used_saved = false;
                // If no password provided, check if we have saved credentials for this SSID
                if (password.empty()) {
                    auto& ssid_manager = SsidManager::GetInstance();
                    auto& ssid_list = ssid_manager.GetSsidList();
                    for (int i = 0; i < (int)ssid_list.size(); i++) {
                        if (ssid_list[i].ssid == ssid) {
                            password = ssid_list[i].password;
                            used_saved = true;
                            ESP_LOGI(TAG, "Using saved credentials for SSID: %s", ssid.c_str());
                            break;
                        }
                    }
                    if (password.empty()) {
                        return std::string("{\"error\":\"No saved password for " + ssid + ". Please provide the password.\"}");
                    }
                }

                // Save and connect
                {
                    auto& ssid_manager = SsidManager::GetInstance();
                    ssid_manager.AddSsid(ssid, password);
                }
                McpServer::GetInstance().SetDeferredReply(true);
                auto* param = new WifiConnectTaskParam{reply_id, ssid, password};
                xTaskCreatePinnedToCore(WifiConnectTask, "wifi_connect", 8192, param, 24, nullptr, 0);
                if (used_saved) {
                    return std::string("Using saved password to connect to " + ssid + "...");
                }
                return std::string("Connecting to " + ssid + "...");
            });

        mcp_server.AddTool("self.wifi.get_saved_networks",
            "Get the list of WiFi networks saved on the device.\n"
            "Returns: A list of saved WiFi network names (SSIDs).\n"
            "Use this tool to check which networks the device remembers.",
            PropertyList(), [](const PropertyList& properties) -> ReturnValue {
                auto& ssid_manager = SsidManager::GetInstance();
                auto& ssid_list = ssid_manager.GetSsidList();

                if (ssid_list.empty()) {
                    return std::string("{\"networks\":[]}");
                }

                cJSON* root = cJSON_CreateObject();
                cJSON* networks = cJSON_CreateArray();
                for (const auto& item : ssid_list) {
                    cJSON* net = cJSON_CreateObject();
                    cJSON_AddStringToObject(net, "ssid", item.ssid.c_str());
                    cJSON_AddBoolToObject(net, "has_password", !item.password.empty());
                    cJSON_AddItemToArray(networks, net);
                }
                cJSON_AddItemToObject(root, "networks", networks);

                char* json_str = cJSON_PrintUnformatted(root);
                std::string result(json_str);
                cJSON_free(json_str);
                cJSON_Delete(root);
                return result;
            });

        mcp_server.AddTool("self.wifi.forget_network",
            "Remove a saved WiFi network from the device memory.\n"
            "Args:\n"
            "  `ssid`: The name of the WiFi network to forget.\n"
            "Use this tool when a saved network has the wrong password or the user no longer needs it.",
            PropertyList({
                Property("ssid", kPropertyTypeString)
            }), [](const PropertyList& properties) -> ReturnValue {
                std::string ssid = properties["ssid"].value<std::string>();
                if (ssid.empty()) {
                    return std::string("{\"error\":\"SSID is required\"}");
                }

                auto& ssid_manager = SsidManager::GetInstance();
                auto& ssid_list = ssid_manager.GetSsidList();
                bool found = false;
                for (int i = 0; i < (int)ssid_list.size(); i++) {
                    if (ssid_list[i].ssid == ssid) {
                        ssid_manager.RemoveSsid(i);
                        found = true;
                        ESP_LOGI(TAG, "Forgot network: %s", ssid.c_str());
                        break;
                    }
                }
                if (!found) {
                    return std::string("{\"error\":\"Network " + ssid + " not found in saved networks.\"}");
                }
                return std::string("{\"success\":true,\"message\":\"Network " + ssid + " has been removed from saved networks.\"}");
            });

        mcp_server.AddTool("self.wifi.start_sniffer",
            "Start WiFi packet sniffer for a fixed duration.\n"
            "Default: captures WiFi management frames while staying connected.\n"
            "With disconnected=true: disconnects WiFi to capture ALL traffic including DNS queries from other devices.\n"
            "After the duration, the sniffer auto-stops and WiFi reconnects.\n"
            "After it finishes, use self.wifi.get_sniffer_log to see captured packets.",
            PropertyList({
                Property("duration_seconds", kPropertyTypeInteger, 10),
                Property("disconnected", kPropertyTypeBoolean, false)
            }), [](const PropertyList& properties) -> ReturnValue {
                if (sniffer_active_) {
                    return std::string("{\"error\":\"Sniffer is already running\"}");
                }

                auto& wifi_mgr = WifiManager::GetInstance();
                if (!wifi_mgr.IsInitialized() || wifi_mgr.IsConfigMode()) {
                    return std::string("{\"error\":\"WiFi not connected\"}");
                }

                int duration = properties["duration_seconds"].value<int>();
                if (duration < 3) duration = 3;
                if (duration > 60) duration = 60;
                bool disconnected = properties["disconnected"].value<bool>();

                // Get current channel
                uint8_t primary;
                wifi_second_chan_t second;
                esp_wifi_get_channel(&primary, &second);

                sniffer_active_ = true;
                sniffer_log_.clear();
                sniffer_queue_ = xQueueCreate(32, sizeof(SnifferPacket*));

                if (disconnected) {
                    // Disconnect mode: captures ALL traffic on channel
                    sniffer_disconnected_ = true;
                    wifi_mgr.SetExternalScanMode(true);
                    esp_wifi_disconnect();
                    vTaskDelay(pdMS_TO_TICKS(1500));

                    esp_wifi_set_channel(primary, second);
                    esp_wifi_set_promiscuous_rx_cb(SnifferCallback);
                    esp_wifi_set_promiscuous(true);

                    ESP_LOGI(TAG, "WiFi sniffer (disconnected) enabled on channel %d for %d seconds", primary, duration);
                    xTaskCreatePinnedToCore(SnifferTask, "sniffer", 4096, (void*)(uintptr_t)duration, 18, &sniffer_task_handle_, 1);

                    return std::string("{\"success\":true,\"message\":\"Sniffer started (disconnected mode) for " + std::to_string(duration) + " seconds on channel " + std::to_string(primary) + ". Capturing ALL traffic including DNS.\",\"duration\":" + std::to_string(duration) + ",\"mode\":\"disconnected\"}");
                } else {
                    // Connected mode: captures management frames + some data
                    sniffer_disconnected_ = false;
                    wifi_mgr.SetExternalScanMode(true);
                    esp_wifi_set_promiscuous_rx_cb(SnifferCallback);
                    esp_wifi_set_promiscuous(true);
                    esp_wifi_set_ps(WIFI_PS_NONE);

                    ESP_LOGI(TAG, "WiFi sniffer (connected) enabled on channel %d for %d seconds", primary, duration);
                    xTaskCreatePinnedToCore(SnifferTask, "sniffer", 4096, (void*)(uintptr_t)duration, 18, &sniffer_task_handle_, 1);

                    return std::string("{\"success\":true,\"message\":\"Sniffer started for " + std::to_string(duration) + " seconds on channel " + std::to_string(primary) + ". Capturing WiFi management frames.\",\"duration\":" + std::to_string(duration) + ",\"mode\":\"connected\"}");
                }
            });

        mcp_server.AddTool("self.wifi.get_sniffer_log",
            "Get the log of captured WiFi packets from the sniffer.\n"
            "Returns: A list of captured network packets with source/destination IPs, protocol, and details.\n"
            "Use this to analyze what traffic was captured.",
            PropertyList(), [](const PropertyList& properties) -> ReturnValue {
                if (sniffer_log_.empty()) {
                    return std::string("{\"packets\":[]}");
                }

                cJSON* root = cJSON_CreateObject();
                cJSON* packets = cJSON_CreateArray();

                // Return last 50 packets
                size_t start = sniffer_log_.size() > 50 ? sniffer_log_.size() - 50 : 0;
                for (size_t i = start; i < sniffer_log_.size(); i++) {
                    cJSON* pkt_json = cJSON_CreateObject();
                    cJSON_AddStringToObject(pkt_json, "info", sniffer_log_[i].short_info.c_str());
                    cJSON_AddStringToObject(pkt_json, "details", sniffer_log_[i].description.c_str());
                    cJSON_AddItemToArray(packets, pkt_json);
                }
                cJSON_AddItemToObject(root, "packets", packets);
                cJSON_AddNumberToObject(root, "total", sniffer_log_.size());

                char* json_str = cJSON_PrintUnformatted(root);
                std::string result(json_str);
                cJSON_free(json_str);
                cJSON_Delete(root);
                return result;
            });

        mcp_server.AddTool("self.wifi.clear_sniffer_log",
            "Clear all saved sniffer log entries from device memory.\n"
            "Use this when the user wants to delete the captured traffic log.",
            PropertyList(), [](const PropertyList& properties) -> ReturnValue {
                int count = sniffer_log_.size();
                ClearSnifferLogFromNvs();
                return std::string("{\"success\":true,\"message\":\"Cleared " + std::to_string(count) + " sniffer log entries.\"}");
            });

        mcp_server.AddTool("self.wifi.get_dns_log",
            "Get DNS query log captured by LwIP hook.\n"
            "Returns list of DNS queries made by this device (hostname + timestamp).\n"
            "Use this to see what hostnames the device is resolving.",
            PropertyList(), [](const PropertyList& properties) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                cJSON* queries = cJSON_CreateArray();
                uint32_t now = esp_timer_get_time() / 1000000;
                for (const auto& q : dns_log_) {
                    cJSON* item = cJSON_CreateObject();
                    cJSON_AddStringToObject(item, "hostname", q.hostname.c_str());
                    cJSON_AddNumberToObject(item, "seconds_ago", now - q.timestamp_sec);
                    cJSON_AddItemToArray(queries, item);
                }
                cJSON_AddItemToObject(root, "dns_queries", queries);
                cJSON_AddNumberToObject(root, "total", dns_log_.size());
                char* json = cJSON_PrintUnformatted(root);
                std::string result(json);
                cJSON_free(json);
                cJSON_Delete(root);
                return result;
            });

        // ---- ESP8266 light control (every command switches WiFi networks) ----
        mcp_server.AddTool("self.light.turn_on",
            "Turn ON the light (relay) of the esp8266 light board.\n"
            "The device temporarily joins the '" LIGHT_AP_SSID "' network, sends the command, "
            "remembers the answer of the board, reconnects to the previous network and reports "
            "the result (relay state + A0/ADC value) back to you.\n"
            "Use it when the user asks to turn the light on.",
            PropertyList(), [this](const PropertyList&) -> ReturnValue {
                return StartLightCommand("turn_on");
            });

        mcp_server.AddTool("self.light.turn_off",
            "Turn OFF the light (relay) of the esp8266 light board.\n"
            "The device temporarily joins the '" LIGHT_AP_SSID "' network, sends the command, "
            "remembers the answer of the board, reconnects to the previous network and reports "
            "the result (relay state + A0/ADC value) back to you.\n"
            "Use it when the user asks to turn the light off.",
            PropertyList(), [this](const PropertyList&) -> ReturnValue {
                return StartLightCommand("turn_off");
            });

        mcp_server.AddTool("self.light.get_status",
            "Query the esp8266 light board and return its current state.\n"
            "Returns: relay state (on/off), control mode (auto = the relay follows the A0 input), "
            "the current A0 (ADC) value and the auto threshold.\n"
            "The device switches to the '" LIGHT_AP_SSID "' network and back, so the answer is "
            "reported a bit later than usual.",
            PropertyList(), [this](const PropertyList&) -> ReturnValue {
                return StartLightCommand("status");
            });

        mcp_server.AddTool("self.light.restore_a0_control",
            "Give control of the light relay back to the A0 analog input of the esp8266 board "
            "(auto mode: A0 above the threshold keeps the light on, otherwise off).\n"
            "Returns: the new control mode and the current A0 (ADC) value.\n"
            "Use it when the user wants the automatic A0 control back after using voice commands.",
            PropertyList(), [this](const PropertyList&) -> ReturnValue {
                return StartLightCommand("restore_a0");
            });

        mcp_server.AddTool("self.light.reboot",
            "Reboot the esp8266 light board (it restarts in a few seconds, the Wi-Fi network "
            LIGHT_AP_SSID " disappears and comes back).\n"
            "Only use it when the user explicitly asks to reboot the light board.",
            PropertyList(), [this](const PropertyList&) -> ReturnValue {
                return StartLightCommand("reboot");
            });

        mcp_server.AddTool("self.light.get_last_report",
            "Get the remembered answer of the LAST light command without switching networks.\n"
            "Returns: command, success flag, raw answer of the esp8266, relay state, control "
            "mode, A0/ADC value, threshold and the WiFi network of the device.\n"
            "Use it when the result of a previous light command was not delivered yet or to "
            "check the light state without waiting for a network switch.",
            PropertyList(), [](const PropertyList&) -> ReturnValue {
                std::lock_guard<std::mutex> lock(light_report_mutex_);
                if (!last_light_report_.valid) {
                    return std::string("{\"report\":null,\"message\":\"No light command has been executed yet\"}");
                }
                return LightReportToJson(last_light_report_);
            });

        mcp_server.AddTool("self.get_memory_info",
            "Get detailed memory usage information by memory type.\n"
            "Returns: Internal RAM, PSRAM, DMA, and task stack usage.\n"
            "Use this to check how much memory is available on the device.",
            PropertyList(), [](const PropertyList& properties) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();

                // Internal DRAM (heap_caps_get_free_size with MALLOC_CAP_8BIT)
                size_t dram_total = heap_caps_get_total_size(MALLOC_CAP_8BIT);
                size_t dram_free = heap_caps_get_free_size(MALLOC_CAP_8BIT);
                size_t dram_min = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
                cJSON* dram = cJSON_CreateObject();
                cJSON_AddNumberToObject(dram, "total", dram_total);
                cJSON_AddNumberToObject(dram, "free", dram_free);
                cJSON_AddNumberToObject(dram, "min_free", dram_min);
                cJSON_AddNumberToObject(dram, "used", dram_total - dram_free);
                cJSON_AddNumberToObject(dram, "usage_pct", dram_total > 0 ? (int)((dram_total - dram_free) * 100 / dram_total) : 0);
                cJSON_AddItemToObject(root, "internal_dram", dram);

                // Internal IRAM
                size_t iram_total = heap_caps_get_total_size(MALLOC_CAP_32BIT);
                size_t iram_free = heap_caps_get_free_size(MALLOC_CAP_32BIT);
                size_t iram_min = heap_caps_get_minimum_free_size(MALLOC_CAP_32BIT);
                cJSON* iram = cJSON_CreateObject();
                cJSON_AddNumberToObject(iram, "total", iram_total);
                cJSON_AddNumberToObject(iram, "free", iram_free);
                cJSON_AddNumberToObject(iram, "min_free", iram_min);
                cJSON_AddNumberToObject(iram, "used", iram_total - iram_free);
                cJSON_AddNumberToObject(iram, "usage_pct", iram_total > 0 ? (int)((iram_total - iram_free) * 100 / iram_total) : 0);
                cJSON_AddItemToObject(root, "internal_iram", iram);

                // PSRAM
                size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
                size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
                size_t psram_min = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);
                cJSON* psram = cJSON_CreateObject();
                cJSON_AddNumberToObject(psram, "total", psram_total);
                cJSON_AddNumberToObject(psram, "free", psram_free);
                cJSON_AddNumberToObject(psram, "min_free", psram_min);
                cJSON_AddNumberToObject(psram, "used", psram_total - psram_free);
                cJSON_AddNumberToObject(psram, "usage_pct", psram_total > 0 ? (int)((psram_total - psram_free) * 100 / psram_total) : 0);
                cJSON_AddItemToObject(root, "psram", psram);

                // DMA-capable memory
                size_t dma_total = heap_caps_get_total_size(MALLOC_CAP_DMA);
                size_t dma_free = heap_caps_get_free_size(MALLOC_CAP_DMA);
                size_t dma_min = heap_caps_get_minimum_free_size(MALLOC_CAP_DMA);
                cJSON* dma = cJSON_CreateObject();
                cJSON_AddNumberToObject(dma, "total", dma_total);
                cJSON_AddNumberToObject(dma, "free", dma_free);
                cJSON_AddNumberToObject(dma, "min_free", dma_min);
                cJSON_AddNumberToObject(dma, "used", dma_total - dma_free);
                cJSON_AddNumberToObject(dma, "usage_pct", dma_total > 0 ? (int)((dma_total - dma_free) * 100 / dma_total) : 0);
                cJSON_AddItemToObject(root, "dma", dma);

                // Overall heap summary
                cJSON* summary = cJSON_CreateObject();
                cJSON_AddNumberToObject(summary, "total_heap", heap_caps_get_total_size(MALLOC_CAP_DEFAULT));
                cJSON_AddNumberToObject(summary, "free_heap", esp_get_free_heap_size());
                cJSON_AddNumberToObject(summary, "min_free_heap", esp_get_minimum_free_heap_size());
                cJSON_AddItemToObject(root, "heap_summary", summary);

                // NVS (Non-Volatile Storage)
                cJSON* nvs_json = cJSON_CreateObject();
                nvs_stats_t nvs_stats;
                if (nvs_get_stats(NULL, &nvs_stats) == ESP_OK) {
                    cJSON_AddNumberToObject(nvs_json, "total_entries", nvs_stats.total_entries);
                    cJSON_AddNumberToObject(nvs_json, "used_entries", nvs_stats.used_entries);
                    cJSON_AddNumberToObject(nvs_json, "free_entries", nvs_stats.free_entries);
                    cJSON_AddNumberToObject(nvs_json, "usage_pct", nvs_stats.total_entries > 0 ?
                        (int)(nvs_stats.used_entries * 100 / nvs_stats.total_entries) : 0);
                }
                const esp_partition_t* nvs_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
                if (nvs_part) {
                    cJSON_AddNumberToObject(nvs_json, "partition_size", nvs_part->size);
                    cJSON_AddStringToObject(nvs_json, "partition_label", nvs_part->label);
                }
                cJSON_AddItemToObject(root, "nvs", nvs_json);

                // Top task stacks
                cJSON* tasks = cJSON_CreateArray();
                UBaseType_t task_count = uxTaskGetNumberOfTasks();
                TaskStatus_t* task_status = (TaskStatus_t*)pvPortMalloc(task_count * sizeof(TaskStatus_t));
                if (task_status) {
                    uint32_t total_runtime;
                    task_count = uxTaskGetSystemState(task_status, task_count, &total_runtime);
                    for (UBaseType_t i = 0; i < task_count; i++) {
                        cJSON* t = cJSON_CreateObject();
                        cJSON_AddStringToObject(t, "name", task_status[i].pcTaskName);
                        cJSON_AddNumberToObject(t, "stack_high", task_status[i].usStackHighWaterMark * 4);
                        cJSON_AddNumberToObject(t, "stack_min_pct", task_status[i].usStackHighWaterMark > 0 ?
                            (int)((1 - (float)task_status[i].usStackHighWaterMark * 4 / (float)configMINIMAL_STACK_SIZE / 4) * 100) : 0);
                        cJSON_AddNumberToObject(t, "priority", task_status[i].uxCurrentPriority);
                        cJSON_AddNumberToObject(t, "runtime_pct", task_status[i].ulRunTimeCounter * 100 / total_runtime);
                        cJSON_AddItemToArray(tasks, t);
                    }
                    vPortFree(task_status);
                }
                cJSON_AddItemToObject(root, "tasks", tasks);

                char* json_str = cJSON_PrintUnformatted(root);
                std::string result(json_str);
                cJSON_free(json_str);
                cJSON_Delete(root);
                return result;
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
        // Block LOW_POWER during sniffer to keep radio active for promiscuous mode
        if (sniffer_active_ && level == PowerSaveLevel::LOW_POWER) {
            return;
        }
        WifiBoard::SetPowerSaveLevel(level);
    }
};

bool MyBoard::sniffer_active_ = false;
bool MyBoard::sniffer_disconnected_ = false;
TaskHandle_t MyBoard::sniffer_task_handle_ = nullptr;
QueueHandle_t MyBoard::sniffer_queue_ = nullptr;
std::vector<MyBoard::SnifferPacket> MyBoard::sniffer_log_;
int MyBoard::sniffer_duration_seconds_ = 10;
std::vector<MyBoard::DnsQuery> MyBoard::dns_log_;
std::atomic<bool> MyBoard::light_command_running_{false};
std::mutex MyBoard::light_report_mutex_;
MyBoard::LightReport MyBoard::last_light_report_;

extern "C" int lwip_hook_dns_external_resolve(const char* name, ip_addr_t* addr,
                                               dns_found_callback found, void* callback_arg,
                                               u8_t addrtype, err_t* err) {
    if (name && strlen(name) > 0) {
        uint32_t now = esp_timer_get_time() / 1000000;
        MyBoard::dns_log_.push_back({now, std::string(name)});
        if (MyBoard::dns_log_.size() > MyBoard::MAX_DNS_LOG) {
            MyBoard::dns_log_.erase(MyBoard::dns_log_.begin());
        }
        ESP_LOGI("DNS_HOOK", "Query: %s", name);
    }
    return 0;  // 0 = don't consume, let normal DNS resolution continue
}

DECLARE_BOARD(MyBoard);
