#include "boot_guard.h"

#ifndef BOOT_GUARD_HOST_TEST
#include "esp_log.h"
#include "esp_system.h"
#include "nvs.h"

static const char *TAG = "boot_guard";
#define NVS_NAMESPACE "boot"
#define NVS_KEY       "quick"
#endif

uint8_t boot_guard_step(uint8_t stored, bool power_on, bool *trigger)
{
    *trigger = false;
    if (!power_on) {
        return 0;
    }
    uint8_t count = (uint8_t)(stored + 1);
    if (count >= BOOT_GUARD_CYCLES) {
        *trigger = true;
        return 0;
    }
    return count;
}

#ifndef BOOT_GUARD_HOST_TEST

static void store(uint8_t value)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, NVS_KEY, value);
        nvs_commit(h);
        nvs_close(h);
    }
}

bool boot_guard_check(void)
{
    uint8_t stored = 0;
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, NVS_KEY, &stored);
        nvs_close(h);
    }

    bool power_on = esp_reset_reason() == ESP_RST_POWERON;
    bool trigger;
    uint8_t next = boot_guard_step(stored, power_on, &trigger);
    if (next != stored) {
        store(next);
    }
    ESP_LOGI(TAG, "quick power cycles: %u/%d", next, BOOT_GUARD_CYCLES);
    return trigger;
}

void boot_guard_clear(void)
{
    store(0);
}

#endif
