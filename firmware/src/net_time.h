/* SNTP time synchronization, started once the station gets an IP. */
#pragma once

#include <stdbool.h>

/* Safe to call repeatedly; only the first call starts SNTP. */
void net_time_start(void);

/* Returns true once after each successful SNTP sync (clears the flag). */
bool net_time_take_synced(void);

/* True once the system clock has been set by SNTP since boot. */
bool net_time_valid(void);
