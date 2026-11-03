# 🌱 E-Ink Environment Clock

[![C](https://img.shields.io/badge/C-gnu11-A8B9CC?logo=c&logoColor=black)](https://en.cppreference.com/w/c)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.5-E7352C?logo=espressif&logoColor=white)](https://docs.espressif.com/projects/esp-idf/)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-espressif32%206-F5822A?logo=platformio&logoColor=white)](https://platformio.org/)
[![ESP32-C3](https://img.shields.io/badge/ESP32--C3-Super%20Mini-E7352C?logo=espressif&logoColor=white)](https://www.espressif.com/en/products/socs/esp32-c3)
[![Home Assistant](https://img.shields.io/badge/Home%20Assistant-MQTT%20discovery-18BCF2?logo=homeassistant&logoColor=white)](https://www.home-assistant.io/integrations/mqtt/)
[![MQTT](https://img.shields.io/badge/MQTT-3.1.1-660066?logo=mqtt&logoColor=white)](https://mqtt.org/)
[![Conventional Commits](https://img.shields.io/badge/Conventional%20Commits-1.0.0-FE5196?logo=conventionalcommits&logoColor=white)](https://www.conventionalcommits.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green)](./LICENSE)

**E-Ink Environment Clock** is a desk clock on an ESP32-C3 with a 2.9" e-paper panel that shows the time next to indoor CO₂, temperature, humidity and pressure. It is set up from its own web page, publishes its readings to Home Assistant over MQTT and has a local JSON API with no account or key.

The clock has no buttons and no screen you can type on, and it has to stay a clock when the network is gone. So everything is configured from a phone: with no saved network it opens a WPA2 access point and prints the name, password and a QR code on the panel. When a known network disappears it keeps showing the time from its RTC and puts the access point details in a strip at the bottom instead of taking over the screen. The panel itself is slow and ghosts, so it is redrawn only when something visible changes.

![Result](images/result.png)

## 📦 Dependencies

| Needed for | What | Install |
| --- | --- | --- |
| Building and flashing | [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html); it downloads ESP-IDF 5.5 and the toolchain itself | `pipx install platformio` |
| Host tests | A C compiler (`gcc` or `clang`) | your package manager |
| Releases | [Commitizen](https://commitizen-tools.github.io/commitizen/), configured in `.cz.toml` | `pipx install commitizen` |

Sensor drivers come from the ESP Component Registry on the first build ([esp-idf-lib](https://esp-idf-lib.readthedocs.io/) `bmp280`, `ds3231`, `mhz19b`, plus `espressif/mdns`).

## 🔩 Hardware

| Part | Model | ESP32-C3 pins |
| --- | --- | --- |
| Microcontroller | ESP32-C3 Super Mini, 4 MB flash | — |
| Display | WeAct Studio 2.9" B/W e-paper, SSD1680, 296×128 | SCK 4, MOSI 6, CS 7, DC 2, RST 3, BUSY 10 |
| Temperature, humidity, pressure | BME280 (BMP280 works without humidity) | I²C SDA 8, SCL 9 |
| Real-time clock | DS3231 | I²C SDA 8, SCL 9 |
| CO₂ | MH-Z19 | UART RX 5 ← sensor TX, TX 1 → sensor RX |
| Status LED | — | 0 |

The pins live in `firmware/src/app_config.h`. GPIO 9 is the BOOT button on the Super Mini and is taken by I²C, which is why the reset below works through power cycles rather than a button.

![Circuit](images/circuit_image.png)

## 🚀 Flashing

The partition table has two OTA slots and an NVS partition for settings.

> [!IMPORTANT]
> When the board still runs firmware with the old single-app partition table, erase the flash once before the first upload. Later uploads keep the settings.

```sh
cd firmware && pio run -t erase
```

Build and flash over the native USB, then watch the log:

```sh
cd firmware && pio run -t upload && pio device monitor
```

After that, updates go over the air: from GitHub releases or by uploading `.pio/build/esp32-c3-supermini/firmware.bin` on the **System** tab. If a new image fails to start, the bootloader returns to the previous one.

## 📶 Wi-Fi setup

| Situation | What the clock does |
| --- | --- |
| No saved network | Opens the access point `EinkClock-XXXX` and shows the setup screen |
| Saved network unreachable for 30 s | Opens the access point and keeps retrying the saved network; the clock face stays, with the access point details in a strip at the bottom |
| Connected | Shows its address at the bottom for two minutes; the access point stops one minute after connecting |

The access point is WPA2 with a fresh 8-digit password every time it starts, because it also comes up after power cuts when the router boots slower than the clock, and an open network would let anyone nearby point the clock at their own broker. The panel shows the network name, password, a Wi-Fi QR code and `192.168.4.1`. Phones open the setup page on their own: a small DNS server answers every name with that address.

To forget the Wi-Fi and MQTT settings without the web page, plug the clock in three times in a row, unplugging it within five seconds each time. Only power-on resets count, so restarts after an update or a crash never trigger it.

## 🖥️ Web interface

Open `http://<clock-ip>/` or `http://eink-clock-<id>.local/`. English and Russian, light and dark theme, works on a phone. The page is gzipped into the firmware and loads nothing from the internet, so it also works over the setup access point.

| Tab | Contents |
| --- | --- |
| Overview | What the panel shows now, readings, a message for the screen, status |
| Display | Layout, which readings appear and in which order, date and time format, time zone, units, white on black, upside down; a preview rendered by the clock before saving |
| Network | Wi-Fi scan and connection, MQTT broker |
| System | Name, CO₂ threshold, temperature correction, CO₂ calibration, firmware updates, restart and reset |
| Developer | Live device log, log level, API console, raw state |

There is no password on the web interface: it is meant for a home network.

## 🖼️ Screen

| Layout | What it shows |
| --- | --- |
| `classic` | Time and date on the left, enabled readings in a column on the right |
| `clock` | Time and date across the whole panel |
| `co2` | CO₂ in large digits, time and the other readings in a row below |
| `sensors` | Enabled readings in a grid, no clock |

Disabled readings free their space: the column re-centres, one reading is drawn large, and with none left the clock takes the full width. A CO₂ value above the threshold is drawn white on black.

Messages appear as a black strip along the bottom (one line in large type or two in small, cut with "…"), or across the whole panel with `fullscreen`. They can expire after a set time. The panel font is the classic 5×7 bitmap font with Cyrillic letters drawn on the same grid (`firmware/tools/gen_cyrillic_font.py`).

Refresh policy, against ghosting: a full refresh on boot and whenever the kind of screen changes, a partial refresh when the content changes but not more often than every 10 seconds, and a forced full refresh after 10 partial ones.

## 🏠 Home Assistant

Turn on MQTT on the **Network** tab and point it at the broker Home Assistant uses. The clock announces itself through device-based MQTT discovery (Home Assistant 2024.11 or newer) and appears as one device without any YAML.

| Entity | Type |
| --- | --- |
| Temperature, humidity, pressure, CO₂, Wi-Fi signal, uptime | `sensor` |
| CO₂ above threshold | `binary_sensor` |
| Message on the screen | `text` |
| Clear message, restart, calibrate CO₂ | `button` |
| Screen layout | `select` |
| CO₂ threshold | `number` |

Readings are published every 30 seconds, and the state right after a setting or a message changes. Messages with an expiry or on the full screen go to `eink-clock/<id>/notify`:

```yaml
action: mqtt.publish
data:
  topic: eink-clock/a1b2c3/notify
  payload: '{"text": "Open the window", "ttl": 600, "fullscreen": true}'
```

| Topic under `eink-clock/<id>/` | Direction | Payload |
| --- | --- | --- |
| `state` | out | JSON with all readings, retained |
| `status` | out | `online` / `offline` (last will) |
| `notify` | in | `{"text", "ttl", "fullscreen"}` or plain text |
| `notification/set` | in | plain text |
| `command` | in | `clear_notification`, `reboot`, `calibrate_co2` |
| `layout/set` | in | `classic`, `clock`, `co2`, `sensors` |
| `co2_threshold/set` | in | 400…5000 |

## ⬆️ Updates and releases

CI publishes a release for every version tag: it builds the firmware with the tag as its version and attaches `firmware.bin`. Tags follow the commitizen format in `.cz.toml` (`1.2.0`, no `v`); a tag with a `-` becomes a pre-release, which the clock does not offer.

```sh
cz bump && git push --follow-tags
```

The clock asks the GitHub API for the latest release once a day and offers it on the **System** tab with its notes and an **Install** button. With automatic updates turned on, a newer release is installed between 03:00 and 05:00 local time, and never while the clock does not know the time. Downloads go over HTTPS with the ESP-IDF certificate bundle.

Images are not signed: HTTPS proves the file came from GitHub, not who uploaded it. Anyone who can publish releases in this repository can ship firmware to clocks with automatic updates on.

Local builds take their version from `.cz.toml`, so a board flashed from the same commit as the latest release does not offer to "update" to it.

## 🔧 Configuration

Everything below is changed in the web interface or with `POST /api/config` (a partial object is enough) and stored in NVS as JSON. Settings survive updates: OTA writes only the app slots, keys the old firmware did not know get their defaults, unknown keys are ignored. CI fails if the NVS partition is moved in `partitions.csv`, because that would wipe them.

| Key | Default | What it is |
| --- | --- | --- |
| `name` | `E-Ink Clock` | Device name, also in Home Assistant |
| `display.language` | `en` | Screen language, `en` or `ru` |
| `display.hour24` | `false` | 24-hour time; the original firmware used 12-hour |
| `display.timezone`, `display.tz_posix` | `Europe/Moscow`, `MSK-3` | Shown name and the POSIX TZ string the firmware uses |
| `display.date_format` | `dmy_text` | `dmy_text` (3 Nov 2026), `weekday`, `dmy_dots`, `iso` |
| `display.show_date` | `true` | Date under the time |
| `display.layout` | `classic` | See the Screen section |
| `display.metrics` | all four, in order | Which readings appear and in which order |
| `display.invert`, `display.rotate180` | `false` | White on black, upside down |
| `display.temp_unit`, `display.pressure_unit` | `c`, `hpa` | Also `f`, `mmhg` |
| `sensors.temp_offset_c` | `-2.3` | Added to the BME280 reading, carried over from the original firmware; limited to ±10 °C |
| `sensors.co2_threshold_ppm` | `1000` | CO₂ alarm: inverted value, LED, Home Assistant binary sensor |
| `sensors.co2_led` | `true` | Blink the LED above the threshold |
| `mqtt.enabled`, `mqtt.host`, `mqtt.port` | `false`, empty, `1883` | Broker; the host may also be a full `mqtt://` URI; TLS (`mqtts://`) is not set up |
| `mqtt.username`, `mqtt.password` | empty | Broker login; the API never returns the password |
| `mqtt.prefix` | empty = `eink-clock/<id>` | Topic prefix |
| `mqtt.discovery`, `mqtt.discovery_prefix` | `true`, `homeassistant` | Home Assistant discovery |
| `update.check` | `true` | Look for a new release once a day |
| `update.auto` | `false` | Install new releases at night; off by default so nothing changes on a clock without the owner asking |

Wi-Fi credentials are set through `POST /api/wifi`, which also reconnects. Fixed timings (sensor interval 30 s, SNTP every 30 minutes, the Wi-Fi fallback above, display refresh limits) are in `firmware/src/app_config.h`.

## 🔌 HTTP API

No keys and no accounts. Everything is JSON except the screen images. CORS is open, so a page or script on another host can call it.

| Method | Path | Purpose |
| --- | --- | --- |
| `GET` | `/api/state` | Readings, time, Wi-Fi, MQTT, message, device info |
| `GET` / `POST` | `/api/config` | Settings; `POST` takes a partial object |
| `GET` | `/api/screen.bmp` | What the panel shows now |
| `POST` | `/api/preview` | Renders unsaved settings, returns a BMP |
| `POST` / `DELETE` | `/api/notify` | Show or clear a message |
| `GET` | `/api/wifi/scan` | Nearby networks |
| `POST` | `/api/wifi` | Save a network and connect |
| `POST` | `/api/co2/calibrate` | MH-Z19 zero point (400 ppm) |
| `POST` | `/api/ota` | Firmware image as the request body |
| `GET` | `/api/update` | Running and latest release, install progress |
| `POST` | `/api/update/check`, `/api/update/install` | Check GitHub now, install the latest release |
| `GET` | `/api/logs?since=N` | Device log after position `N` |
| `POST` | `/api/log-level` | `{"level": "debug", "tag": "*"}` |
| `POST` | `/api/system/reboot`, `/api/system/forget-wifi`, `/api/system/factory-reset` | Maintenance |

Show a message on the screen for an hour:

```sh
curl -X POST http://192.168.1.42/api/notify -d '{"text": "Meeting until 15:00", "ttl": 3600}'
```

## 🛠️ Developer tools

The **Developer** tab keeps the last 12 KB of the device log since boot in RAM and shows it live, with filters by level and text and a download button. It can change the log level per tag until the next restart, send any request to the API and show the raw `/api/state`. The same log is on the USB console via `pio device monitor`.

## 🧪 Tests

Everything that does not touch hardware is plain C and runs on the host: settings and their JSON, settings from older and newer firmware, layouts and text wrapping, the Cyrillic font, messages and their expiry, the power-cycle reset, captive-portal DNS replies, MQTT payloads and commands, version comparison and GitHub release parsing, the log ring buffer. That is 60 test cases. They use cJSON from the ESP-IDF package, so build the firmware once first.

```sh
cd firmware && pio test -e native
```

Set `PREVIEW_DIR` to get every screen the tests render as a PBM image:

```sh
cd firmware && PREVIEW_DIR=/tmp/screens pio test -e native -f test_ui
```

A passing suite says the logic and the drawing are right. It says nothing about Wi-Fi, the radio, the panel driver, the sensors or OTA, which only run on the board. CI runs the build and the tests on every push.

## 🧩 PCB

The KiCad project is in `pcb/eink-environment-clock/`, Gerber files in `pcb/pcb/`. Reference images:

```
images/
├── assembled_board_image.png
├── circuit_image.png
├── pcb_back_image.png
├── pcb_front_image.png
└── schematic_image.png
```

## 🧰 Case

```
3d_model/
├── stp
│   ├── backplate.stp
│   ├── center.stp
│   └── frontplate.stp
├── case.3mf
└── case.m3d
```

## ⚠️ Known issues

* Holes for the **MH-Z19** are too small.
* Some modules (**MH-Z19**) sit too close to the mounting holes, which makes assembly and fastening inconvenient.

## 📁 Source layout

```
firmware/
├── platformio.ini        # firmware env and the host test env
├── partitions.csv        # NVS + two OTA slots
├── web/index.html        # web interface, gzipped into the image at build time
├── test/                 # host unit tests
├── tools/                # web embedding, cJSON for tests, Cyrillic glyph generator
└── src/
    ├── app_main.c        # main loop: sensors, screen choice, refresh policy
    ├── app_config.h      # pins, timings, update repository
    ├── settings*.c       # settings, JSON <-> NVS
    ├── wifi_mgr.c        # station, setup access point, mDNS
    ├── dns_*.c           # captive-portal DNS
    ├── web_server.c      # HTTP API, preview, OTA, logs
    ├── mqtt_ha.c         # MQTT client; ha_*.c build and parse its messages
    ├── updater.c         # GitHub release check and install; release.c parses
    ├── log_*.c           # log capture into a RAM ring buffer
    ├── ui.c              # layouts, messages, setup and system screens
    ├── epd_canvas.c      # 1-bit canvas, UTF-8 text, fonts
    ├── epd_ssd1680.c     # e-paper driver
    └── third_party/      # QR Code generator (Project Nayuki, MIT)
```

The e-paper driver follows the panel vendor's reference code from [WeActStudio.EpaperModule](https://github.com/WeActStudio/WeActStudio.EpaperModule).

## 📜 License

[MIT](./LICENSE).
