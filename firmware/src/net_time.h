/* Wi-Fi station + SNTP time synchronization (background). */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

/* Brings up Wi-Fi and SNTP; reconnects and resyncs on its own. */
esp_err_t net_time_start(void);

/* Returns true once after each successful SNTP sync (clears the flag). */
bool net_time_take_synced(void);
