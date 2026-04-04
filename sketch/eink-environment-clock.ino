#include <SPI.h>
#include <GxEPD2_BW.h>
#include <Adafruit_GFX.h>

#include <math.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <RTClib.h>
#include <time.h>
#include <MHZ19.h>

// =====================================================
// Pin configuration
// =====================================================

// E-paper display pins
constexpr uint8_t PIN_EPD_DC   = 2;
constexpr uint8_t PIN_EPD_RST  = 3;
constexpr uint8_t PIN_SPI_SCK  = 4;
constexpr uint8_t PIN_SPI_MOSI = 6;
constexpr uint8_t PIN_EPD_CS   = 7;
constexpr uint8_t PIN_EPD_BUSY = 10;

// I2C pins
constexpr uint8_t I2C_SDA_PIN = 8;
constexpr uint8_t I2C_SCL_PIN = 9;

// MH-Z19 UART pins
constexpr uint8_t MHZ_RX_PIN = 5;
constexpr uint8_t MHZ_TX_PIN = 1;

// Status LED pin
constexpr uint8_t LED_PIN = 0;

// =====================================================
// Application configuration
// =====================================================

// Time format
constexpr bool USE_12_HOUR_FORMAT = true;

// Time synchronization
constexpr bool USE_WIFI_TIME_SYNC = true;

// Wi-Fi credentials
constexpr char WIFI_SSID[] = "YourWiFiName";
constexpr char WIFI_PASSWORD[] = "YourWiFiPassword";

// NTP settings
constexpr char NTP_SERVER[] = "pool.ntp.org";
constexpr char TIME_ZONE[]  = "MSK-3";

// BME280 settings
constexpr uint8_t BME280_ADDRESS_PRIMARY   = 0x76;
constexpr uint8_t BME280_ADDRESS_SECONDARY = 0x77;
constexpr float BME280_TEMP_OFFSET_C = -2.3f;

// MZH19b settings
constexpr int CO2_ALERT_THRESHOLD_PPM = 1000;  // Blink the status LED when CO2 exceeds this level

// Screen settings
constexpr bool USE_DARK_THEME = false; // true - not recommended
constexpr bool SKIP_UPDATE_IF_DATA_UNCHANGED = true;

// =====================================================
// Timing configuration
// =====================================================

constexpr uint32_t SENSOR_UPDATE_INTERVAL_MS           = 30000;    // Read sensors every 30 seconds
constexpr uint32_t DISPLAY_UPDATE_INTERVAL_MS          = 1000;     // Update display every second
constexpr uint16_t DISPLAY_FULL_REFRESH_EVERY          = 200;      // Full refresh every N display updates
constexpr uint8_t  DISPLAY_EXTRA_REDRAWS_AFTER_PARTIAL = 4;        // Number of additional full redraws after a partial display update 
constexpr uint32_t WIFI_TIME_SYNC_INTERVAL_MS          = 1800000;  // Sync time every 30 minutes
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS             = 15000;    // Wi-Fi connection timeout
constexpr uint32_t NTP_SYNC_TIMEOUT_MS                 = 15000;    // NTP synchronization timeout
constexpr uint32_t LED_BLINK_INTERVAL_MS               = 1000;     // LED blink interval when CO2 is above the alert threshold

// =====================================================
// Timing state
// =====================================================

uint32_t lastSensorUpdateMs  = 0;
uint32_t lastDisplayUpdateMs = 0;
uint32_t lastTimeSyncMs      = 0;
uint32_t lastLedToggleMs     = 0;

// =====================================================
// Counters
// =====================================================

uint16_t updatesSinceFullRefresh = 0;
uint16_t displayTimeUpdateCount  = 0;
uint16_t displayDataUpdateCount  = 0;

// =====================================================
// Runtime flags
// =====================================================

bool wifiConnected     = false;
bool firstDisplayDraw  = true;
bool bme280Available   = false;
bool ds3231Available   = false;
bool mhz19Available    = false;
bool displayAvailable  = false;
bool ledOn             = false;
bool sensorDataChanged = false;
bool timeChanged       = false;

// =====================================================
// Global objects
// =====================================================

// E-paper display driver
GxEPD2_BW<GxEPD2_290_T94_V2, GxEPD2_290_T94_V2::HEIGHT> display(
  GxEPD2_290_T94_V2(PIN_EPD_CS, PIN_EPD_DC, PIN_EPD_RST, PIN_EPD_BUSY)
);

// Peripherals
Adafruit_BME280 bme;
RTC_DS3231 rtc;
MHZ19 mhz19;
HardwareSerial mhzSerial(1);

// =====================================================
// Sensor & time data
// =====================================================

struct SensorData {
  float temperatureC = NAN;
  float humidityPct  = NAN;
  float pressureHpa  = NAN;
  int   co2Ppm       = -1;
};

SensorData lastSensorData;
DateTime lastRtcTime;

// =====================================================
// Bitmap icons
// =====================================================

// 16x16 humidity icon
const unsigned char ICON_DROPLET_16X16[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0xc0, 0x03, 0xe0, 0x07, 0xe0, 0x07,
  0xf0, 0x0f, 0xf0, 0x0f, 0xf8, 0x1f, 0xd8, 0x1f, 0x98, 0x1f, 0x30, 0x0f,
  0x70, 0x0e, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x00
};

// 16x16 pressure icon
const unsigned char ICON_GAUGE_16X16[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0xe0, 0x07, 0xf0, 0x0f, 0xf8, 0x1f, 0x5c, 0x3a,
  0x7c, 0x3e, 0x6c, 0x36, 0x7c, 0x3e, 0x7c, 0x3e, 0x3c, 0x3c, 0x78, 0x1e,
  0xf0, 0x0f, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x00
};

// 16x16 plant icon
const unsigned char ICON_SEEDLING_16X16[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x1c, 0x3e, 0x7c, 0x3f, 0xfc, 0x3e,
  0xfc, 0x1c, 0xf8, 0x05, 0xf0, 0x01, 0xc0, 0x01, 0x80, 0x01, 0x80, 0x01,
  0x80, 0x01 ,0x80, 0x01, 0x00, 0x00, 0x00, 0x00
};

// 16x16 low temperature icon
const unsigned char ICON_TEMPERATURE_LOW_16X16[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0xe0, 0x38, 0xf0, 0x29, 0xf0, 0x39, 0xf0, 0x01,
  0xf0, 0x01, 0xf0, 0x01, 0xb0, 0x01, 0xb8, 0x03, 0x18, 0x03, 0x18, 0x03,
  0xf8, 0x03, 0xf0, 0x01, 0x00, 0x00, 0x00, 0x00
};

// =====================================================
// Structures
// =====================================================

struct BoxLayout {
  int16_t cursorX;
  int16_t cursorY;
  int16_t boxX;
  int16_t boxY;
  uint16_t boxW;
  uint16_t boxH;
};

// =====================================================
// Data helpers
// =====================================================

bool nearlyEqual(float a, float b, float eps = 0.05f) {
  if (isnan(a) && isnan(b)) return true;
  if (isnan(a) || isnan(b)) return false;
  return fabs(a - b) <= eps;
}

bool isSensorDataSimilar(const SensorData& a, const SensorData& b) {
  return nearlyEqual(a.temperatureC, b.temperatureC, 0.2f) &&
         nearlyEqual(a.humidityPct,  b.humidityPct,  2.0f) &&
         nearlyEqual(a.pressureHpa,  b.pressureHpa,  5.0f) &&
         abs(a.co2Ppm - b.co2Ppm) <= 50;
}

bool hasTimeChanged(const DateTime& a, const DateTime& b) {
  return a.year() != b.year() ||
         a.month() != b.month() ||
         a.day() != b.day() || 
         a.hour() != b.hour() ||
         a.minute() != b.minute();
}

// =====================================================
// Logging
// =====================================================

void logInfo(const char* module, const char* message) {
  Serial.print("[INFO] [");
  Serial.print(module);
  Serial.print("] ");
  Serial.println(message);
}

void logWarn(const char* module, const char* message) {
  Serial.print("[WARN] [");
  Serial.print(module);
  Serial.print("] ");
  Serial.println(message);
}

void logError(const char* module, const char* message) {
  Serial.print("[ERROR] [");
  Serial.print(module);
  Serial.print("] ");
  Serial.println(message);
}

// =====================================================
// Wi-Fi
// =====================================================

bool connectToWifi() {
  if (!USE_WIFI_TIME_SYNC) {
    logWarn("WiFi", "Disabled in configuration.");
    return false;
  }

  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("[INFO] [WiFi] Connecting ");

  const uint32_t startTimeMs = millis();

  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startTimeMs >= WIFI_CONNECT_TIMEOUT_MS) {
      Serial.println();
      logError("WiFi", "Connection timeout.");
      return false;
    }

    delay(500);
    Serial.print(".");
  }

  Serial.println();
  logInfo("WiFi", "Connected successfully.");
  Serial.print("[INFO] [WiFi] IP: ");
  Serial.println(WiFi.localIP());

  return true;
}

// =====================================================
// Time synchronization
// =====================================================

bool syncTimeFromNtp() {
  if (!wifiConnected) {
    logError("NTP", "Wi-Fi is not connected.");
    return false;
  }

  logInfo("NTP", "Starting synchronization...");

  configTzTime(TIME_ZONE, NTP_SERVER);

  struct tm timeInfo;
  const uint32_t startTimeMs = millis();

  while (!getLocalTime(&timeInfo)) {
    if (millis() - startTimeMs >= NTP_SYNC_TIMEOUT_MS) {
      logError("NTP", "Synchronization timeout.");
      return false;
    }

    delay(200);
  }

  logInfo("NTP", "Time synchronized successfully.");
  return true;
}

bool syncRtcFromNtp() {
  if (!ds3231Available) {
    logError("DS3231", "RTC is not available.");
    return false;
  }

  if (!syncTimeFromNtp()) {
    return false;
  }

  struct tm timeInfo;
  if (!getLocalTime(&timeInfo)) {
    logError("DS3231", "Failed to read synchronized time.");
    return false;
  }

  rtc.adjust(DateTime(
    timeInfo.tm_year + 1900,
    timeInfo.tm_mon + 1,
    timeInfo.tm_mday,
    timeInfo.tm_hour,
    timeInfo.tm_min,
    timeInfo.tm_sec
  ));

  logInfo("DS3231", "Time synced from NTP.");
  return true;
}

bool syncRtcFromCompileTime() {
  if (!ds3231Available) {
    logError("DS3231", "RTC is not available.");
    return false;
  }

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  logInfo("DS3231", "Time synced from compile time.");
  return true;
}

// =====================================================
// Initialization
// =====================================================

bool initBme280() {
  if (bme.begin(BME280_ADDRESS_PRIMARY) || bme.begin(BME280_ADDRESS_SECONDARY)) {
    bme.setSampling(
      Adafruit_BME280::MODE_FORCED,
      Adafruit_BME280::SAMPLING_X1,
      Adafruit_BME280::SAMPLING_X1,
      Adafruit_BME280::SAMPLING_X1,
      Adafruit_BME280::FILTER_OFF
    );

    logInfo("BME280", "Initialized.");
    return true;
  }

  logError("BME280", "Initialization failed.");
  return false;
}

bool initMhz19() {
  mhzSerial.begin(9600, SERIAL_8N1, MHZ_RX_PIN, MHZ_TX_PIN);
  mhz19.begin(mhzSerial);
  mhz19.autoCalibration(false);

  logInfo("MH-Z19", "UART started.");
  logInfo("MH-Z19", "Warm-up may take a few minutes.");
  return true;
}

bool initRtc() {
  if (rtc.begin()) {
    logInfo("DS3231", "Initialized.");
    return true;
  }
  
  logError("DS3231", "Initialization failed.");
  return false;
}

bool initEpd() {
  pinMode(PIN_EPD_BUSY, INPUT);
  SPI.begin(PIN_SPI_SCK, -1, PIN_SPI_MOSI, PIN_EPD_CS);

  logInfo("EPD", "Initializing...");

  display.init(115200);
  display.setRotation(3);

  logInfo("EPD", "Initialized.");
  return true;
}

void initLed() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  logInfo("Led", "Initialized.");
}

// =====================================================
// Sensor handling
// =====================================================

void readSensors() {
  SensorData newData = lastSensorData;

  if (bme280Available) {
    bme.takeForcedMeasurement();
    
    const float temperatureC = bme.readTemperature() + BME280_TEMP_OFFSET_C;
    const float humidityPct  = bme.readHumidity();
    const float pressureHpa  = bme.readPressure() / 100.0F;

    if (!isnan(temperatureC)) newData.temperatureC = temperatureC;
    if (!isnan(humidityPct)) newData.humidityPct = humidityPct;
    if (!isnan(pressureHpa)) newData.pressureHpa = pressureHpa;
  }

  if (mhz19Available) {
    const int co2Ppm = mhz19.getCO2();

    if (co2Ppm > 0 && co2Ppm <= 10000) newData.co2Ppm = co2Ppm;
  }

  if (!isSensorDataSimilar(lastSensorData, newData)) {
    lastSensorData = newData;
    sensorDataChanged = true;
  }
}

void readCurrentTime() {
  if (!ds3231Available) return;

  const DateTime now = rtc.now();

  if (hasTimeChanged(now, lastRtcTime)) {
    lastRtcTime = now;
    timeChanged = true;
  }
}

// =====================================================
// Serial data log
// =====================================================

void printCurrentTime() {
  if (!ds3231Available) {
    return;
  }

  const DateTime now = rtc.now();

  char buffer[32];
  snprintf(
    buffer,
    sizeof(buffer),
    "%02u.%02u.%04u %02u:%02u:%02u",
    now.day(),
    now.month(),
    now.year(),
    now.hour(),
    now.minute(),
    now.second()
  );

  Serial.print("[INFO] [RTC] ");
  Serial.println(buffer);
}

void printSensorData() {
  if (bme280Available) {
    Serial.print("[INFO] [BME280] T=");
    Serial.print(lastSensorData.temperatureC, 1);
    Serial.print(" C, H=");
    Serial.print(lastSensorData.humidityPct, 0);
    Serial.print(" %, P=");
    Serial.print(lastSensorData.pressureHpa, 0);
    Serial.println(" hPa");
  }

  if (mhz19Available && lastSensorData.co2Ppm > 0) {
    Serial.print("[INFO] [MH-Z19] CO2=");
    Serial.print(lastSensorData.co2Ppm);
    Serial.println(" ppm");
  }
}

void printFullData() {
  printCurrentTime();
  printSensorData();
}

// =====================================================
// LED handling
// =====================================================

void updateCo2Led() {
  if (lastSensorData.co2Ppm > CO2_ALERT_THRESHOLD_PPM) {
    const uint32_t nowMs = millis(); 

    if (nowMs - lastLedToggleMs >= LED_BLINK_INTERVAL_MS) {
      lastLedToggleMs = nowMs;
      ledOn = !ledOn;
      digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
    }
  } else {
    ledOn = false;
    digitalWrite(LED_PIN, LOW);
  }
}

// =====================================================
// Display helpers
// =====================================================

uint16_t getThemeBackgroundColor() {
  return USE_DARK_THEME ? GxEPD_BLACK : GxEPD_WHITE;
}

uint16_t getThemeForegroundColor() {
  return USE_DARK_THEME ? GxEPD_WHITE : GxEPD_BLACK;
}

void formatDisplayDate(char* buffer, size_t size) {
  static const char* MONTHS[] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
  };

  if (!ds3231Available) {
    snprintf(buffer, size, "--.--.----");
    return;
  }

  snprintf(
    buffer,
    size,
    "%u %s %u",
    lastRtcTime.day(),
    MONTHS[lastRtcTime.month() - 1],
    lastRtcTime.year()
  );
}

void formatDisplayTime(char* buffer, size_t size) {
  if (!ds3231Available) {
    snprintf(buffer, size, "--:--");
    return;
  }

  uint8_t hour = lastRtcTime.hour();

  if (USE_12_HOUR_FORMAT) {
    hour %= 12;
    if (hour == 0) {
      hour = 12;
    }
  }

  snprintf(buffer, size, "%02u:%02u", hour, lastRtcTime.minute());
}

// =====================================================
// Display render
// =====================================================

void drawCenteredText(const char* text, int16_t baselineY, uint8_t textSize, uint16_t areaWidth) {
  display.setFont();
  display.setTextSize(textSize);

  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

  int16_t x = (areaWidth - (int16_t)w) / 2 - x1;
  int16_t y = baselineY;

  display.getTextBounds(text, x, y, &x1, &y1, &w, &h);

  display.setCursor(x, y);
  display.print(text);
}

void drawFloatMetric(int16_t iconX, int16_t topY, const unsigned char* icon, float value, uint8_t decimals, const char* unit) {
  const uint16_t fg = getThemeForegroundColor();

  char valueText[16];
  if (isnan(value)) {
    snprintf(valueText, sizeof(valueText), "---");
  } else {
    snprintf(valueText, sizeof(valueText), "%.*f", decimals, value);
  }

  display.drawXBitmap(iconX, topY, icon, 16, 16, fg);

  display.setTextSize(2);
  display.setCursor(iconX + 24, topY);
  display.print(valueText);

  if (unit != nullptr && unit[0] != '\0') {
    const int16_t cursorX = display.getCursorX();
    display.setTextSize(1);
    display.setCursor(cursorX + 1, topY + 6);
    display.print(unit);
  }
}

void drawPartialRegion(int16_t x, int16_t y, int16_t w, int16_t h, void (*drawContent)()) {
  const uint16_t bg = getThemeBackgroundColor();
  const uint16_t fg = getThemeForegroundColor();

  display.setPartialWindow(x, y, w, h);
  display.firstPage();
  do {
    display.fillRect(x, y, w, h, bg);
    display.setTextColor(fg);
    display.setFont();
    drawContent();
  } while (display.nextPage());
}

// =====================================================
// Display UI
// =====================================================
void drawTimeRegion() {
  char dateBuffer[20];
  char timeBuffer[16];

  formatDisplayDate(dateBuffer, sizeof(dateBuffer));
  formatDisplayTime(timeBuffer, sizeof(timeBuffer));

  drawCenteredText(timeBuffer, 31, 5, 185);
  drawCenteredText(dateBuffer, 81, 2, 185);
}

void drawSensorRegion() {
  drawFloatMetric(190, 17, ICON_TEMPERATURE_LOW_16X16, lastSensorData.temperatureC, 1, "\xF7""C");
  drawFloatMetric(190, 43, ICON_DROPLET_16X16,         lastSensorData.humidityPct,  0, "%");
  drawFloatMetric(190, 69, ICON_GAUGE_16X16,           lastSensorData.pressureHpa,  0, "hPa");
  drawFloatMetric(190, 95, ICON_SEEDLING_16X16,        lastSensorData.co2Ppm,       0, "ppm");
}

void drawFullScreen() {
  const uint16_t bg = getThemeBackgroundColor();
  const uint16_t fg = getThemeForegroundColor();

  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(bg);
    display.setTextColor(fg);
    display.setFont();
    
    drawTimeRegion();
    drawSensorRegion();
  } while (display.nextPage());
}

void drawPartialScreen() {
  bool didPartial = false;

  if (timeChanged || !SKIP_UPDATE_IF_DATA_UNCHANGED) {
    drawPartialRegion(13, 24, 158, 80, drawTimeRegion);
    didPartial = true;

    if (++displayTimeUpdateCount > DISPLAY_EXTRA_REDRAWS_AFTER_PARTIAL) {
      timeChanged = false;
      displayTimeUpdateCount = 0;
    }
  }

  if (sensorDataChanged || !SKIP_UPDATE_IF_DATA_UNCHANGED) {
    drawPartialRegion(186, 8, 110, 112, drawSensorRegion);
    didPartial = true;

    if (++displayDataUpdateCount > DISPLAY_EXTRA_REDRAWS_AFTER_PARTIAL) {
      sensorDataChanged = false;
      displayDataUpdateCount = 0;
    }
  }

  if (didPartial) ++updatesSinceFullRefresh;
}

void drawDisplay() {
  if (firstDisplayDraw || (updatesSinceFullRefresh >= DISPLAY_FULL_REFRESH_EVERY)) {
    drawFullScreen();
    updatesSinceFullRefresh = 0;
    timeChanged = false;
    sensorDataChanged = false;
    firstDisplayDraw = false;
  }
  
  drawPartialScreen();
  firstDisplayDraw = false;
}

// =====================================================
// Setup
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  wifiConnected    = connectToWifi();
  bme280Available  = initBme280();
  mhz19Available   = initMhz19();
  ds3231Available  = initRtc();
  displayAvailable = initEpd();
  initLed();

  if (!syncRtcFromNtp()) {
    syncRtcFromCompileTime();
  }
}

// =====================================================
// Loop
// =====================================================

void loop() {
  const uint32_t nowMs = millis();

  if (USE_WIFI_TIME_SYNC && !wifiConnected) {
    wifiConnected = connectToWifi();
  }

  readCurrentTime();

  if (nowMs - lastSensorUpdateMs >= SENSOR_UPDATE_INTERVAL_MS) {
    lastSensorUpdateMs = nowMs;
    readSensors();
  }

  if (nowMs - lastDisplayUpdateMs >= DISPLAY_UPDATE_INTERVAL_MS) {
    lastDisplayUpdateMs = nowMs;
    drawDisplay();
    printFullData();
  }

  updateCo2Led();

  if (USE_WIFI_TIME_SYNC && wifiConnected &&
      nowMs - lastTimeSyncMs >= WIFI_TIME_SYNC_INTERVAL_MS) {
    lastTimeSyncMs = nowMs;
    syncRtcFromNtp();
  }
}
