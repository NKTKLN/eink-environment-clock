# 🖥️ E-Ink Environment Clock

[![License](https://img.shields.io/github/license/NKTKLN/eink-environment-clock?color=green)](./LICENSE)
[![Platform](https://img.shields.io/badge/platform-ESP32--C3-blue)](https://www.espressif.com/en/products/socs/esp32-c3)
[![Framework](https://img.shields.io/badge/framework-ESP--IDF%20%2F%20PlatformIO-orange)](https://platformio.org/)

**eink-environment-clock** is a minimal ESP32-C3 based E-Ink desk clock that displays time and indoor air data.
It combines a low-power E-Ink screen with CO₂, temperature, humidity, and pressure sensing, making it useful as both a clock and a compact environment monitor.

![Result](images/result.png)

## 🧩 PCB

PCB design and assembly are illustrated with the following reference images:

```bash
images/
├── assembled_board_image.png
├── circuit_image.png
├── pcb_back_image.png
├── pcb_front_image.png
└── schematic_image.png
```

### Circuit

![Circuit](images/circuit_image.png)

## 💾 Firmware

The firmware is a pure **ESP-IDF** PlatformIO project (no Arduino) located in:

```bash
firmware/
├── platformio.ini
└── src/
    ├── app_main.c        # application logic
    ├── app_config.h      # pins, Wi-Fi credentials, timings
    ├── secrets.h.example # Wi-Fi credentials template (copy to secrets.h)
    ├── epd_ssd1680.*     # e-paper driver (WeAct 2.9" B/W, SSD1680)
    ├── epd_canvas.*      # 1bpp canvas, bitmap fonts, icons
    ├── ui.*              # clock face rendering
    └── net_time.*        # Wi-Fi + SNTP time sync
```

Before building, set up your Wi-Fi credentials (only needed if `USE_WIFI_TIME_SYNC` is enabled in `app_config.h`):

```bash
cp firmware/src/secrets.h.example firmware/src/secrets.h
# then edit secrets.h with your WIFI_SSID / WIFI_PASSWORD
```

Build and flash (ESP32-C3 Super Mini over native USB):

```bash
cd firmware
pio run -t upload && pio device monitor
```

## 📚 Libraries

Sensors are handled by [esp-idf-lib](https://esp-idf-lib.readthedocs.io/) components (fetched automatically from the ESP Component Registry):

* **bmp280** — BME280 temperature / humidity / pressure
* **ds3231** — RTC
* **mhz19b** — CO₂ sensor

The e-paper driver is custom and follows the panel vendor's reference code from [WeActStudio.EpaperModule](https://github.com/WeActStudio/WeActStudio.EpaperModule).

## 📦 Case

The case model for this project is located in:

```bash
3d_model/
├── stp
│   ├── backplate.stp
│   ├── center.stp
│   └── frontplate.stp
├── case.3mf
└── case.m3d
```

## ⚠️ Known Issues

* Holes for the **MH-Z19** are too small.
* There are several issues with the mounting holes in general. Some modules are placed too close to the mounting holes, which sometimes makes assembly and fastening inconvenient.
* The wrong header type was selected for external modules such as the display. The headers I used turned out to be smaller than the ones I had, so I ended up soldering the connection cable directly.

## 📜 License

This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for details.
