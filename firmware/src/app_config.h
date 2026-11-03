#pragma once

/* =====================================================
 * Pin configuration (ESP32-C3 Super Mini)
 * ===================================================== */

/* E-paper display (SPI) */
#define PIN_EPD_DC   2
#define PIN_EPD_RST  3
#define PIN_SPI_SCK  4
#define PIN_SPI_MOSI 6
#define PIN_EPD_CS   7
#define PIN_EPD_BUSY 10

/* I2C bus (BME280 + DS3231) */
#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 9

/* MH-Z19 UART (ESP RX <- sensor TX, ESP TX -> sensor RX) */
#define PIN_MHZ_RX 5
#define PIN_MHZ_TX 1

/* Status LED */
#define PIN_LED 0

/* =====================================================
 * Fixed configuration. Everything a user may want to change (time zone,
 * 12/24 h, units, CO2 threshold, Wi-Fi, MQTT, ...) lives in settings.c and
 * is edited through the web interface.
 * ===================================================== */

#define NTP_SERVER "pool.ntp.org"

/* Firmware updates: GitHub releases of this repository, asset name. */
#define UPDATE_REPO  "NKTKLN/eink-environment-clock"
#define UPDATE_ASSET "firmware.bin"

#define SENSOR_UPDATE_INTERVAL_S 30   /* Read sensors every 30 seconds */
#define TIME_SYNC_INTERVAL_S     1800 /* SNTP resync every 30 minutes */

/* Wi-Fi: how long to try the saved network before opening the setup
 * access point, and how long the AP stays up after a successful
 * connection so the setup page can show the new address. */
#define WIFI_FALLBACK_AP_S     30
#define WIFI_AP_LINGER_S       60
#define WIFI_CONNECTED_NOTICE_S 120

/* Display refresh policy (anti-ghosting):
 *  - full (flashing) refresh on boot and when the screen type changes;
 *  - differential (partial) refresh when the displayed data changes,
 *    but not more often than once in DISPLAY_MIN_UPDATE_PERIOD_S;
 *  - forced full refresh after DISPLAY_FULL_REFRESH_EVERY_N partials. */
#define DISPLAY_MIN_UPDATE_PERIOD_S  10
#define DISPLAY_FULL_REFRESH_EVERY_N 10
