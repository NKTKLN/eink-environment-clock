#include "wifi_mgr.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "mdns.h"

#include "app_config.h"
#include "app_state.h"
#include "dns_server.h"
#include "net_time.h"
#include "settings_store.h"
#include "ui.h"

static const char *TAG = "wifi";

/* Retry delays. With the AP up every attempt makes the radio hop channels,
 * which disturbs the phone connected to the AP, so retry less often. */
#define RETRY_MS        3000
#define RETRY_AP_MS     20000

static SemaphoreHandle_t s_lock;
static wifi_status_t s_st;
static volatile uint32_t s_gen;
static bool s_scanning;

static esp_netif_t *s_sta_netif;
static esp_timer_handle_t s_fallback_timer;
static esp_timer_handle_t s_retry_timer;
static esp_timer_handle_t s_ap_stop_timer;

#define LOCK()   xSemaphoreTake(s_lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(s_lock)

static void bump(void)
{
    s_gen++;
    app_request_redraw();
}

static void start_timer(esp_timer_handle_t t, int ms)
{
    esp_timer_stop(t); /* restart if already running */
    esp_timer_start_once(t, (uint64_t)ms * 1000);
}

/* ===================================================== */

static void ap_start(bool after_failure)
{
    LOCK();
    bool already = s_st.ap_active;
    s_st.ap_after_failure = after_failure;
    if (!already) {
        snprintf(s_st.ap_password, sizeof(s_st.ap_password), "%08lu",
                 (unsigned long)(esp_random() % 100000000UL));
        s_st.ap_active = true;
        s_st.ap_clients = 0;
    }
    wifi_config_t cfg = { 0 };
    strlcpy((char *)cfg.ap.ssid, s_st.ap_ssid, sizeof(cfg.ap.ssid));
    cfg.ap.ssid_len = (uint8_t)strlen(s_st.ap_ssid);
    strlcpy((char *)cfg.ap.password, s_st.ap_password,
            sizeof(cfg.ap.password));
    UNLOCK();

    if (already) {
        bump();
        return;
    }

    cfg.ap.channel = 1;
    cfg.ap.max_connection = 4;
    cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    cfg.ap.pmf_cfg.required = false;

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err == ESP_OK) {
        err = esp_wifi_set_config(WIFI_IF_AP, &cfg);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AP start failed: %s", esp_err_to_name(err));
        LOCK();
        s_st.ap_active = false;
        UNLOCK();
        return;
    }
    dns_server_start();
    ESP_LOGW(TAG, "setup AP \"%s\" started", s_st.ap_ssid);
    bump();
}

static void ap_stop(void)
{
    LOCK();
    bool active = s_st.ap_active;
    s_st.ap_active = false;
    s_st.ap_clients = 0;
    UNLOCK();
    if (!active) {
        return;
    }
    dns_server_stop();
    esp_wifi_set_mode(WIFI_MODE_STA);
    ESP_LOGI(TAG, "setup AP stopped");
    bump();
}

static void connect_now(void)
{
    if (s_st.sta_configured && !s_scanning) {
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "connect: %s", esp_err_to_name(err));
        }
    }
}

static void on_fallback_timer(void *arg)
{
    if (!s_st.sta_connected) {
        ESP_LOGW(TAG, "no connection for %d s, opening setup AP",
                 WIFI_FALLBACK_AP_S);
        ap_start(true);
    }
}

static void on_retry_timer(void *arg)
{
    connect_now();
}

static void on_ap_stop_timer(void *arg)
{
    if (s_st.sta_connected) {
        ap_stop();
    }
}

/* ===================================================== */

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id,
                          void *data)
{
    switch (id) {
    case WIFI_EVENT_STA_START:
        connect_now();
        break;

    case WIFI_EVENT_STA_DISCONNECTED: {
        wifi_event_sta_disconnected_t *e = data;
        bool was_connected;
        LOCK();
        was_connected = s_st.sta_connected;
        s_st.sta_connected = false;
        s_st.ip[0] = '\0';
        s_st.rssi = 0;
        UNLOCK();
        ESP_LOGW(TAG, "disconnected (reason %d)", e->reason);
        if (was_connected) {
            bump();
        }
        if (s_st.sta_configured && !s_scanning) {
            start_timer(s_retry_timer,
                        s_st.ap_active ? RETRY_AP_MS : RETRY_MS);
            if (!s_st.ap_active && !esp_timer_is_active(s_fallback_timer)) {
                start_timer(s_fallback_timer, WIFI_FALLBACK_AP_S * 1000);
            }
        }
        break;
    }

    case WIFI_EVENT_AP_STACONNECTED:
    case WIFI_EVENT_AP_STADISCONNECTED: {
        wifi_sta_list_t list;
        LOCK();
        if (esp_wifi_ap_get_sta_list(&list) == ESP_OK) {
            s_st.ap_clients = list.num;
        }
        UNLOCK();
        ESP_LOGI(TAG, "AP clients: %d", s_st.ap_clients);
        break;
    }

    default:
        break;
    }
}

static void on_got_ip(void *arg, esp_event_base_t base, int32_t id,
                      void *data)
{
    ip_event_got_ip_t *e = data;

    LOCK();
    s_st.sta_connected = true;
    snprintf(s_st.ip, sizeof(s_st.ip), IPSTR, IP2STR(&e->ip_info.ip));
    bool ap = s_st.ap_active;
    UNLOCK();

    ESP_LOGI(TAG, "connected, IP %s", s_st.ip);
    esp_timer_stop(s_fallback_timer);
    esp_timer_stop(s_retry_timer);
    if (ap) {
        start_timer(s_ap_stop_timer, WIFI_AP_LINGER_S * 1000);
    }

    net_time_start();

    settings_t s;
    settings_get(&s);
    char msg[64];
    snprintf(msg, sizeof(msg), ui_str(&s, UI_STR_CONNECTED), s_st.ip);
    app_system_notice(msg, WIFI_CONNECTED_NOTICE_S);
    bump();
}

/* ===================================================== */

static void start_mdns(void)
{
    if (mdns_init() != ESP_OK) {
        ESP_LOGW(TAG, "mDNS init failed");
        return;
    }
    settings_t s;
    settings_get(&s);
    mdns_hostname_set(s_st.hostname);
    mdns_instance_name_set(s.name);
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    ESP_LOGI(TAG, "mDNS: http://%s.local/", s_st.hostname);
}

static void apply_sta_config(const char *ssid, const char *password)
{
    wifi_config_t cfg = { 0 };
    strlcpy((char *)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, password, sizeof(cfg.sta.password));
    cfg.sta.threshold.authmode =
        password[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
    cfg.sta.pmf_cfg.capable = true;
    cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));

    LOCK();
    strlcpy(s_st.sta_ssid, ssid, sizeof(s_st.sta_ssid));
    s_st.sta_configured = ssid[0] != '\0';
    UNLOCK();
}

esp_err_t wifi_mgr_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_st.ap_ssid, sizeof(s_st.ap_ssid), "EinkClock-%02X%02X",
             mac[4], mac[5]);
    snprintf(s_st.hostname, sizeof(s_st.hostname), "eink-clock-%s",
             app_device_id());
    esp_netif_set_hostname(s_sta_netif, s_st.hostname);

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    /* Credentials live in our settings, not in the driver's NVS copy. */
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               on_wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               on_got_ip, NULL));

    const esp_timer_create_args_t fallback = { .callback = on_fallback_timer,
                                               .name = "wifi_fallback" };
    const esp_timer_create_args_t retry = { .callback = on_retry_timer,
                                            .name = "wifi_retry" };
    const esp_timer_create_args_t ap_stop_t = { .callback = on_ap_stop_timer,
                                                .name = "wifi_ap_stop" };
    ESP_ERROR_CHECK(esp_timer_create(&fallback, &s_fallback_timer));
    ESP_ERROR_CHECK(esp_timer_create(&retry, &s_retry_timer));
    ESP_ERROR_CHECK(esp_timer_create(&ap_stop_t, &s_ap_stop_timer));

    settings_t s;
    settings_get(&s);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    apply_sta_config(s.wifi.ssid, s.wifi.password);
    if (s_st.sta_configured) {
        ESP_LOGI(TAG, "connecting to \"%s\"", s.wifi.ssid);
        start_timer(s_fallback_timer, WIFI_FALLBACK_AP_S * 1000);
    } else {
        ESP_LOGI(TAG, "no saved network");
        ap_start(false);
    }

    esp_err_t err = esp_wifi_start();
    if (err == ESP_OK) {
        start_mdns();
    }
    return err;
}

void wifi_mgr_get_status(wifi_status_t *out)
{
    LOCK();
    *out = s_st;
    UNLOCK();
    if (out->sta_connected) {
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            out->rssi = ap.rssi;
        }
    }
}

uint32_t wifi_mgr_generation(void)
{
    return s_gen;
}

esp_err_t wifi_mgr_set_credentials(const char *ssid, const char *password)
{
    settings_t s;
    settings_get(&s);
    strlcpy(s.wifi.ssid, ssid, sizeof(s.wifi.ssid));
    strlcpy(s.wifi.password, password, sizeof(s.wifi.password));
    esp_err_t err = settings_set(&s);
    if (err != ESP_OK) {
        return err;
    }

    esp_timer_stop(s_retry_timer);
    esp_timer_stop(s_ap_stop_timer);
    esp_wifi_disconnect();
    apply_sta_config(ssid, password);

    if (!s_st.sta_configured) {
        esp_timer_stop(s_fallback_timer);
        ap_start(false);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "connecting to \"%s\"", ssid);
    connect_now();
    if (!s_st.ap_active) {
        start_timer(s_fallback_timer, WIFI_FALLBACK_AP_S * 1000);
    }
    bump();
    return ESP_OK;
}

int wifi_mgr_scan(wifi_ap_record_t *out, int max)
{
    s_scanning = true;
    esp_timer_stop(s_retry_timer);
    if (!s_st.sta_connected) {
        /* A pending connection attempt makes the scan fail. */
        esp_wifi_disconnect();
    }

    wifi_scan_config_t cfg = { .show_hidden = false };
    uint16_t n = 0;
    esp_err_t err = esp_wifi_scan_start(&cfg, true);
    if (err == ESP_OK) {
        n = (uint16_t)max;
        esp_wifi_scan_get_ap_records(&n, out);
    } else {
        ESP_LOGW(TAG, "scan failed: %s", esp_err_to_name(err));
        esp_wifi_clear_ap_list();
    }

    s_scanning = false;
    if (!s_st.sta_connected) {
        connect_now();
    }
    return n;
}
