# my-esp32s3-1.54tft

Custom board based on ESP32-S3 with a 1.54" ST7789 display.

Besides the standard xiaozhi features this firmware contains **voice control of
the ESP8266 light board**
([spartacus1988/esp8266-light](https://github.com/spartacus1988/esp8266-light/)).

## Voice light tools

| Tool | What it does |
|------|--------------|
| `self.light.turn_on` | Switch the relay ON (`GET /api?relay=on`) |
| `self.light.turn_off` | Switch the relay OFF (`GET /api?relay=off`) |
| `self.light.get_status` | Query relay state, control mode and the current **A0 (ADC)** value (`GET /api`) |
| `self.light.restore_a0_control` | Give the relay control back to the **A0 input** (auto mode, `GET /api?mode=auto`) and return the A0 reading |
| `self.light.reboot` | Reboot the ESP8266 (`GET /?reset=1`) |
| `self.light.get_last_report` | Return the remembered answer of the **last** command without switching networks |

Typical voice requests: «включи свет», «выключи свет», «какое состояние света и
что на входе A0», «верни управление по A0», «перезагрузи плату света».

## What happens on a command

The tool replies to the model **immediately** ("the command is running"), so the
user gets feedback right away. A deferred reply cannot survive the network
switch: leaving for `esp-light` kills the MQTT session of the tool call, and the
answer would come back under a new `session_id` that the server cannot match.

The ESP8266 creates its own access point, therefore every command makes the
xiaozhi board:

1. leave the current network and join `esp-light` (external scan mode is enabled
   so the Wi-Fi manager does not fight for the radio);
2. send the HTTP request to `http://192.168.4.1`;
3. **remember the answer of the ESP8266** (RAM + NVS namespace `light`);
4. reconnect to the network it was using before;
5. show the remembered answer on the display (notification + chat text) and keep
   it in NVS, so it survives a reboot.

The answer stays available to the model through `self.light.get_last_report`
("what was the result of the last light command"), without switching networks.

## Configuration

See the `LIGHT_*` block in [`config.h`](./config.h):

| Define | Default | Meaning |
|--------|---------|---------|
| `LIGHT_AP_SSID` | `esp-light` | Wi-Fi network of the ESP8266 (`AP_SSID` in its `config.py`) |
| `LIGHT_AP_PASSWORD` | `light1234` | Password of that network (a saved network with the same name overrides it) |
| `LIGHT_DEVICE_HOST` | `192.168.4.1` | Web server of the ESP8266 |
| `LIGHT_STATIC_FALLBACK_IP` | `192.168.4.10` | Static address used if the AP hands out no DHCP lease |

## Building

```bash
idf.py set-target esp32s3
idf.py build
```
