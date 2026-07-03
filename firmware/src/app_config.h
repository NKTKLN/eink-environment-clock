#pragma once

#include <stdbool.h>
#include <stdint.h>

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
 * Application configuration
 * ===================================================== */

#define USE_12_HOUR_FORMAT true

#define USE_WIFI_TIME_SYNC true
#include "secrets.h" /* defines WIFI_SSID / WIFI_PASSWORD, not tracked by git */

#define NTP_SERVER "pool.ntp.org"
#define TIME_ZONE  "MSK-3" /* POSIX TZ string */

/* BME280 */
#define BME280_TEMP_OFFSET_C (-2.3f)

/* MH-Z19 */
#define CO2_ALERT_THRESHOLD_PPM 1000

/* =====================================================
 * Timing configuration
 * ===================================================== */

#define SENSOR_UPDATE_INTERVAL_S   30   /* Read sensors every 30 seconds */
#define TIME_SYNC_INTERVAL_S       1800 /* SNTP resync every 30 minutes */

/* Display refresh policy (anti-ghosting):
 *  - full (flashing) refresh on boot;
 *  - differential (partial) refresh when the displayed data changes,
 *    but not more often than once in DISPLAY_MIN_UPDATE_PERIOD_S;
 *  - forced full refresh after DISPLAY_FULL_REFRESH_EVERY_N partials. */
#define DISPLAY_MIN_UPDATE_PERIOD_S   10
#define DISPLAY_FULL_REFRESH_EVERY_N  10
