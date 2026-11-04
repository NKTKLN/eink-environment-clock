#include "net_time.h"

#include <stdatomic.h>

#include "esp_log.h"
#include "esp_sntp.h"

#include "app_config.h"

static const char *TAG = "net_time";

static atomic_bool s_synced;
static atomic_bool s_valid;
static atomic_bool s_started;

static void on_time_sync(struct timeval *tv)
{
    (void)tv;
    ESP_LOGI(TAG, "time synchronized via SNTP");
    atomic_store(&s_valid, true);
    atomic_store(&s_synced, true);
}

void net_time_start(void)
{
    if (atomic_exchange(&s_started, true)) {
        return;
    }
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, NTP_SERVER);
    sntp_set_time_sync_notification_cb(on_time_sync);
    sntp_set_sync_interval((uint32_t)TIME_SYNC_INTERVAL_S * 1000);
    esp_sntp_init();
}

bool net_time_take_synced(void)
{
    return atomic_exchange(&s_synced, false);
}

bool net_time_valid(void)
{
    return atomic_load(&s_valid);
}
