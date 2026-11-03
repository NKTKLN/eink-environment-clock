#include "mqtt_ha.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "mqtt_client.h"

#include "app_config.h"
#include "app_state.h"
#include "ha_command.h"
#include "ha_discovery.h"
#include "settings_store.h"
#include "wifi_mgr.h"

static const char *TAG = "mqtt";

static esp_mqtt_client_handle_t s_client;
static volatile bool s_connected;
static volatile bool s_publish_now;
static uint32_t s_settings_gen = UINT32_MAX;
static bool s_wifi_was_up;
static int64_t s_last_publish_ms;
static char s_base[64];
static char s_uri[96];

static void topic(char *buf, size_t size, const char *suffix)
{
    snprintf(buf, size, "%s/%s", s_base, suffix);
}

static void publish_discovery(const settings_t *s)
{
    if (!s->mqtt.discovery) {
        return;
    }
    wifi_status_t ws;
    wifi_mgr_get_status(&ws);
    char url[40];
    snprintf(url, sizeof(url), "http://%s/", ws.ip);

    ha_device_t dev = {
        .device_id = app_device_id(),
        .name = s->name,
        .base = s_base,
        .discovery_prefix = s->mqtt.discovery_prefix,
        .sw_version = app_version(),
        .config_url = ws.ip[0] ? url : NULL,
    };
    char t[128];
    ha_discovery_topic(t, sizeof(t), &dev);
    char *payload = ha_discovery_payload(&dev);
    if (payload != NULL) {
        esp_mqtt_client_enqueue(s_client, t, payload, 0, 1, 1, true);
        cJSON_free(payload);
    }
}

static void publish_state(void)
{
    settings_t s;
    settings_get(&s);
    wifi_status_t ws;
    wifi_mgr_get_status(&ws);
    app_notice_t notice;
    bool has_notice = app_get_notice(&notice, NULL) && !notice.is_system;

    ha_state_t st = {
        .co2_threshold_ppm = s.sensors.co2_threshold_ppm,
        .layout = s.display.layout,
        .rssi_dbm = ws.rssi,
        .uptime_s = (long)(app_uptime_ms() / 1000),
        .notification = has_notice ? notice.text : "",
    };
    app_get_sensors(&st.data);

    char t[96];
    topic(t, sizeof(t), HA_TOPIC_STATE);
    char *payload = ha_state_payload(&st);
    if (payload != NULL) {
        esp_mqtt_client_enqueue(s_client, t, payload, 0, 0, 1, true);
        cJSON_free(payload);
    }
    s_last_publish_ms = app_uptime_ms();
}

static void subscribe_all(const settings_t *s)
{
    const char *subs[] = { HA_TOPIC_NOTIFY, HA_TOPIC_NOTIFICATION,
                           HA_TOPIC_COMMAND, HA_TOPIC_LAYOUT,
                           HA_TOPIC_THRESHOLD };
    char t[96];
    for (size_t i = 0; i < sizeof(subs) / sizeof(subs[0]); i++) {
        topic(t, sizeof(t), subs[i]);
        esp_mqtt_client_subscribe(s_client, t, 0);
    }
    if (s->mqtt.discovery) {
        /* Home Assistant announces restarts here: re-send discovery. */
        snprintf(t, sizeof(t), "%s/status", s->mqtt.discovery_prefix);
        esp_mqtt_client_subscribe(s_client, t, 0);
    }
}

/* Saves one setting through a JSON patch, so a concurrent save from the web
 * interface is never overwritten with stale values. */
static void patch_setting(const char *section, const char *key, cJSON *value)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *sec = cJSON_AddObjectToObject(root, section);
    cJSON_AddItemToObject(sec, key, value);
    settings_patch_json(root);
    cJSON_Delete(root);
}

static void handle_message(const esp_mqtt_event_t *e)
{
    /* Fragmented messages (> buffer) are not supported. */
    if (e->current_data_offset != 0 || e->data_len != e->total_data_len) {
        return;
    }

    settings_t s;
    settings_get(&s);
    static ha_command_t cmd; /* ~300 bytes, keep it off the MQTT task stack */
    ha_parse_command(s_base, s.mqtt.discovery_prefix, e->topic, e->topic_len,
                     e->data, e->data_len, &cmd);

    switch (cmd.action) {
    case HA_ACT_NONE:
        return;
    case HA_ACT_INVALID:
        ESP_LOGW(TAG, "ignoring malformed message on %.*s", e->topic_len,
                 e->topic);
        return;
    case HA_ACT_HA_ONLINE:
        ESP_LOGI(TAG, "Home Assistant restarted, re-sending discovery");
        publish_discovery(&s);
        break;
    case HA_ACT_NOTIFY:
        app_notify(cmd.text, cmd.ttl_s, cmd.fullscreen);
        app_request_redraw();
        break;
    case HA_ACT_CLEAR:
        app_notify_clear();
        app_request_redraw();
        break;
    case HA_ACT_REBOOT:
        app_restart_later(OVERLAY_RESTART, RESTART_DELAY_MS);
        break;
    case HA_ACT_CALIBRATE:
        app_request_co2_calibration();
        break;
    case HA_ACT_LAYOUT:
        patch_setting("display", "layout",
                      cJSON_CreateString(layout_name(cmd.layout)));
        break;
    case HA_ACT_THRESHOLD:
        patch_setting("sensors", "co2_threshold_ppm",
                      cJSON_CreateNumber(cmd.threshold));
        break;
    }
    s_publish_now = true;
}

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    esp_mqtt_event_t *e = data;
    settings_t s;

    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED: {
        ESP_LOGI(TAG, "connected to %s", s_uri);
        s_connected = true;
        settings_get(&s);
        char t[96];
        topic(t, sizeof(t), HA_TOPIC_STATUS);
        esp_mqtt_client_enqueue(s_client, t, "online", 0, 1, 1, true);
        subscribe_all(&s);
        publish_discovery(&s);
        s_publish_now = true;
        break;
    }
    case MQTT_EVENT_DISCONNECTED:
        if (s_connected) {
            ESP_LOGW(TAG, "disconnected");
        }
        s_connected = false;
        break;
    case MQTT_EVENT_DATA:
        handle_message(e);
        break;
    case MQTT_EVENT_ERROR:
        if (e->error_handle != NULL &&
            e->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGE(TAG, "connection refused (bad credentials?)");
        }
        break;
    default:
        break;
    }
}

static void stop_client(void)
{
    if (s_client == NULL) {
        return;
    }
    if (s_connected) {
        char t[96];
        topic(t, sizeof(t), HA_TOPIC_STATUS);
        esp_mqtt_client_publish(s_client, t, "offline", 0, 1, 1);
    }
    esp_mqtt_client_stop(s_client);
    esp_mqtt_client_destroy(s_client);
    s_client = NULL;
    s_connected = false;
}

static void start_client(const settings_t *s)
{
    if (s->mqtt.prefix[0] != '\0') {
        strlcpy(s_base, s->mqtt.prefix, sizeof(s_base));
    } else {
        snprintf(s_base, sizeof(s_base), "eink-clock/%s", app_device_id());
    }

    /* Accept "host", "host:port" is not parsed: the port is separate. A full
     * URI (mqtt://, mqtts://, ws://) in the host field is used as is. */
    if (strstr(s->mqtt.host, "://") != NULL) {
        strlcpy(s_uri, s->mqtt.host, sizeof(s_uri));
    } else {
        snprintf(s_uri, sizeof(s_uri), "mqtt://%s:%u", s->mqtt.host,
                 s->mqtt.port);
    }

    static char lwt_topic[96];
    topic(lwt_topic, sizeof(lwt_topic), HA_TOPIC_STATUS);
    static char client_id[32];
    snprintf(client_id, sizeof(client_id), "eink-clock-%s", app_device_id());

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = s_uri,
        .credentials.client_id = client_id,
        .credentials.username = s->mqtt.username[0] ? s->mqtt.username : NULL,
        .credentials.authentication.password =
            s->mqtt.password[0] ? s->mqtt.password : NULL,
        .session.last_will = {
            .topic = lwt_topic,
            .msg = "offline",
            .qos = 1,
            .retain = true,
        },
        .session.keepalive = 60,
        .network.reconnect_timeout_ms = 10000,
        .buffer.size = 2048,
        .buffer.out_size = 4096,
        .task.stack_size = 6144,
    };

    s_client = esp_mqtt_client_init(&cfg);
    if (s_client == NULL) {
        ESP_LOGE(TAG, "client init failed");
        return;
    }
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, on_event, NULL);
    esp_mqtt_client_start(s_client);
    ESP_LOGI(TAG, "connecting to %s, topics under %s/", s_uri, s_base);
}

void mqtt_ha_tick(bool sensors_updated)
{
    wifi_status_t ws;
    wifi_mgr_get_status(&ws);
    uint32_t gen = settings_generation();

    if (gen != s_settings_gen || ws.sta_connected != s_wifi_was_up) {
        settings_t s;
        settings_get(&s);
        bool changed = gen != s_settings_gen;
        s_settings_gen = gen;
        s_wifi_was_up = ws.sta_connected;

        bool want = s.mqtt.enabled && s.mqtt.host[0] != '\0' &&
                    ws.sta_connected;
        /* Any settings change restarts the client: cheap and covers broker,
         * credentials, prefix and the device name in discovery. Layout and
         * threshold changes only need a state publish. */
        static char last_cfg[512];
        char cfg_key[512];
        snprintf(cfg_key, sizeof(cfg_key), "%d|%s|%u|%s|%s|%s|%d|%s|%s",
                 want, s.mqtt.host, s.mqtt.port, s.mqtt.username,
                 s.mqtt.password, s.mqtt.prefix, s.mqtt.discovery,
                 s.mqtt.discovery_prefix, s.name);
        if (strcmp(cfg_key, last_cfg) != 0) {
            strlcpy(last_cfg, cfg_key, sizeof(last_cfg));
            stop_client();
            if (want) {
                start_client(&s);
            }
        } else if (changed) {
            s_publish_now = true;
        }
    }

    if (!s_connected) {
        return;
    }
    if (s_publish_now || sensors_updated ||
        app_uptime_ms() - s_last_publish_ms >=
            SENSOR_UPDATE_INTERVAL_S * 1000) {
        s_publish_now = false;
        publish_state();
    }
}

void mqtt_ha_notify_changed(void)
{
    s_publish_now = true;
}

bool mqtt_ha_connected(void)
{
    return s_connected;
}

