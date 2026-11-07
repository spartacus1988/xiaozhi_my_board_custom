
#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000
#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_4
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_5
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_6
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_7
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_15
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_16

#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_40
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_39

#define DISPLAY_SDA GPIO_NUM_10
#define DISPLAY_SCL GPIO_NUM_9
#define DISPLAY_DC GPIO_NUM_8
#define DISPLAY_CS GPIO_NUM_14
#define DISPLAY_RES GPIO_NUM_18
#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  240
#define DISPLAY_SWAP_XY  false
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define BACKLIGHT_INVERT false
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0
#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_13
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false

// ---------------------------------------------------------------------------
// Voice control of the ESP8266 light board
// Firmware: https://github.com/spartacus1988/esp8266-light/
//
// The ESP8266 creates its own Wi-Fi access point, so every light command makes
// this board:
//   1. leave the current network and join LIGHT_AP_SSID,
//   2. send an HTTP request to the ESP8266 web server (LIGHT_DEVICE_HOST),
//   3. remember the answer of the board (RAM + NVS),
//   4. reconnect to the network it was using before,
//   5. report the remembered answer to the LLM once the connection to the
//      server is restored (deferred MCP reply, see LIGHT_AUTO_REPORT).
//
// HTTP API of the esp8266-light firmware:
//   GET /api              -> {"relay":"on","mode":"auto","adc":512,"threshold":512,"ip":"..."}
//   GET /api?relay=on     -> switch the relay on and return the state
//   GET /api?relay=off    -> switch the relay off and return the state
//   GET /api?mode=auto    -> give control of the relay back to the A0 input
//   GET /?reset=1         -> reboot the ESP8266 (answers with a redirect)
// ---------------------------------------------------------------------------
#define LIGHT_AP_SSID              "esp-light"      // AP_SSID in config.py of the ESP8266
#define LIGHT_AP_PASSWORD          "light1234"      // AP_PASSWORD in config.py (a saved network with the same name wins)
#define LIGHT_DEVICE_HOST          "192.168.4.1"    // web server of the ESP8266 (AP gateway)
#define LIGHT_AP_IP_PREFIX         "192.168.4."     // subnet of the ESP8266 AP, used to detect that we joined it
#define LIGHT_STATIC_FALLBACK_IP   "192.168.4.10"   // static address used if the AP hands out no DHCP lease
#define LIGHT_DHCP_TIMEOUT_MS      8000             // wait for a DHCP lease from the ESP8266
#define LIGHT_STATIC_TIMEOUT_MS    3000             // wait after applying the static address
#define LIGHT_JOIN_TIMEOUT_MS      15000            // overall budget for joining LIGHT_AP_SSID
#define LIGHT_RESTORE_TIMEOUT_MS   20000            // wait for the original network to come back
#define LIGHT_REPORT_TIMEOUT_MS    15000            // wait for the LLM audio channel to open again
#define LIGHT_AUTO_REPORT          1                // 1 = reopen the audio channel and report the result to the LLM

#endif // _BOARD_CONFIG_H_
