/* BME280 (temperature/humidity/pressure), DS3231 RTC and MH-Z19 CO2. */
#pragma once

#include <stdbool.h>
#include <time.h>

#include "ui.h"

void sensors_init(void);

/* Updates the fields that could be read; others keep their old value. */
void sensors_read(sensor_data_t *data, float temp_offset_c);

/* Zero-point calibration: the sensor must have been in fresh (~400 ppm)
 * air for at least 20 minutes. */
bool sensors_calibrate_co2(void);

/* Local time from the RTC, falling back to the system clock. */
bool sensors_get_time(struct tm *out);

/* Writes the current system time (after SNTP) into the RTC. */
void sensors_sync_rtc(void);

/* Sets the system clock from the RTC (which keeps local time in the
 * current TZ). Lets time() work before SNTP and offline. */
bool sensors_set_system_time_from_rtc(void);
