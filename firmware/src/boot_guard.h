/*
 * Factory-style network reset without a button: power the clock on and off
 * BOOT_GUARD_CYCLES times in a row, each time for less than
 * BOOT_GUARD_WINDOW_MS, and Wi-Fi + MQTT settings are forgotten.
 *
 * Only power-on resets count, so OTA restarts, crashes and brownouts never
 * trigger it.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define BOOT_GUARD_CYCLES    3
#define BOOT_GUARD_WINDOW_MS 5000

/* Pure step function (host-tested): given the stored counter and whether
 * this boot is a power-on, returns the counter to store and sets *trigger
 * when the reset must happen (the counter is then already 0). */
uint8_t boot_guard_step(uint8_t stored, bool power_on, bool *trigger);

/* Reads/updates the counter in NVS. Returns true if the network settings
 * must be reset now. Requires NVS to be initialized. */
bool boot_guard_check(void);

/* Call once the device has been running for BOOT_GUARD_WINDOW_MS. */
void boot_guard_clear(void);
