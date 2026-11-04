#include "ha_discovery.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define REPO_URL "https://github.com/NKTKLN/eink-environment-clock"

void ha_discovery_topic(char *buf, size_t size, const ha_device_t *dev)
{
    snprintf(buf, size, "%s/device/eink_clock_%s/config",
             dev->discovery_prefix, dev->device_id);
}

static void topic(char *buf, size_t size, const ha_device_t *dev,
                  const char *suffix)
{
    snprintf(buf, size, "%s/%s", dev->base, suffix);
}

static cJSON *component(cJSON *cmps, const ha_device_t *dev, const char *key,
                        const char *platform, const char *name)
{
    char uid[64];
    snprintf(uid, sizeof(uid), "eink_clock_%s_%s", dev->device_id, key);

    cJSON *c = cJSON_AddObjectToObject(cmps, key);
    cJSON_AddStringToObject(c, "platform", platform);
    cJSON_AddStringToObject(c, "name", name);
    cJSON_AddStringToObject(c, "unique_id", uid);
    return c;
}

static cJSON *state_component(cJSON *cmps, const ha_device_t *dev,
                              const char *key, const char *platform,
                              const char *name, const char *state_topic)
{
    cJSON *c = component(cmps, dev, key, platform, name);
    char tpl[96];
    snprintf(tpl, sizeof(tpl), "{{ value_json.%s }}", key);
    cJSON_AddStringToObject(c, "state_topic", state_topic);
    cJSON_AddStringToObject(c, "value_template", tpl);
    return c;
}

static void sensor(cJSON *cmps, const ha_device_t *dev, const char *st,
                   const char *key, const char *name, const char *dev_class,
                   const char *unit, int precision, bool diagnostic)
{
    cJSON *c = state_component(cmps, dev, key, "sensor", name, st);
    if (dev_class != NULL) {
        cJSON_AddStringToObject(c, "device_class", dev_class);
    }
    cJSON_AddStringToObject(c, "unit_of_measurement", unit);
    cJSON_AddStringToObject(c, "state_class", "measurement");
    if (precision >= 0) {
        cJSON_AddNumberToObject(c, "suggested_display_precision", precision);
    }
    if (diagnostic) {
        cJSON_AddStringToObject(c, "entity_category", "diagnostic");
    }
}

static void button(cJSON *cmps, const ha_device_t *dev, const char *cmd_topic,
                   const char *key, const char *name, const char *payload,
                   const char *dev_class, const char *category)
{
    cJSON *c = component(cmps, dev, key, "button", name);
    cJSON_AddStringToObject(c, "command_topic", cmd_topic);
    cJSON_AddStringToObject(c, "payload_press", payload);
    if (dev_class != NULL) {
        cJSON_AddStringToObject(c, "device_class", dev_class);
    }
    if (category != NULL) {
        cJSON_AddStringToObject(c, "entity_category", category);
    }
}

static void text_entity(cJSON *cmps, const ha_device_t *dev,
                        const char *state_topic, const char *key,
                        const char *name, const char *suffix, const char *icon)
{
    char buf[128];
    cJSON *c = state_component(cmps, dev, key, "text", name, state_topic);
    topic(buf, sizeof(buf), dev, suffix);
    cJSON_AddStringToObject(c, "command_topic", buf);
    cJSON_AddNumberToObject(c, "min", 0);
    cJSON_AddNumberToObject(c, "max", 255);
    cJSON_AddStringToObject(c, "icon", icon);
}

char *ha_discovery_payload(const ha_device_t *dev)
{
    char st[128], status[128], cmd[128], buf[128];
    topic(st, sizeof(st), dev, HA_TOPIC_STATE);
    topic(status, sizeof(status), dev, HA_TOPIC_STATUS);
    topic(cmd, sizeof(cmd), dev, HA_TOPIC_COMMAND);

    cJSON *root = cJSON_CreateObject();

    cJSON *d = cJSON_AddObjectToObject(root, "device");
    cJSON *ids = cJSON_AddArrayToObject(d, "identifiers");
    snprintf(buf, sizeof(buf), "eink_clock_%s", dev->device_id);
    cJSON_AddItemToArray(ids, cJSON_CreateString(buf));
    cJSON_AddStringToObject(d, "name", dev->name);
    cJSON_AddStringToObject(d, "manufacturer", "NKTKLN");
    cJSON_AddStringToObject(d, "model", "E-Ink Environment Clock");
    cJSON_AddStringToObject(d, "sw_version", dev->sw_version);
    if (dev->config_url != NULL) {
        cJSON_AddStringToObject(d, "configuration_url", dev->config_url);
    }

    cJSON *o = cJSON_AddObjectToObject(root, "origin");
    cJSON_AddStringToObject(o, "name", "eink-environment-clock");
    cJSON_AddStringToObject(o, "sw_version", dev->sw_version);
    cJSON_AddStringToObject(o, "support_url", REPO_URL);

    cJSON_AddStringToObject(root, "availability_topic", status);
    cJSON_AddNumberToObject(root, "qos", 0);

    cJSON *cmps = cJSON_AddObjectToObject(root, "components");

    sensor(cmps, dev, st, "temperature", "Temperature", "temperature", "°C",
           1, false);
    sensor(cmps, dev, st, "humidity", "Humidity", "humidity", "%", 0, false);
    sensor(cmps, dev, st, "pressure", "Pressure", "atmospheric_pressure",
           "hPa", 0, false);
    sensor(cmps, dev, st, "co2", "CO2", "carbon_dioxide", "ppm", 0, false);
    sensor(cmps, dev, st, "rssi", "Wi-Fi signal", "signal_strength", "dBm",
           0, true);

    cJSON *up = state_component(cmps, dev, "uptime", "sensor", "Uptime", st);
    cJSON_AddStringToObject(up, "device_class", "duration");
    cJSON_AddStringToObject(up, "unit_of_measurement", "s");
    cJSON_AddStringToObject(up, "entity_category", "diagnostic");

    cJSON *hi = state_component(cmps, dev, "co2_high", "binary_sensor",
                                "CO2 above threshold", st);
    cJSON_AddStringToObject(hi, "device_class", "problem");
    cJSON_AddStringToObject(hi, "payload_on", "ON");
    cJSON_AddStringToObject(hi, "payload_off", "OFF");

    text_entity(cmps, dev, st, "notification", "Permanent notification",
                HA_TOPIC_NOTIFICATION, "mdi:message-text");
    text_entity(cmps, dev, st, "notification_temp", "Temporary notification",
                HA_TOPIC_NOTIFICATION_TEMP, "mdi:message-text-clock");

    cJSON *nt = state_component(cmps, dev, "notification_time", "number",
                                "Temporary notification time", st);
    topic(buf, sizeof(buf), dev, HA_TOPIC_NOTIFICATION_TIME);
    cJSON_AddStringToObject(nt, "command_topic", buf);
    cJSON_AddNumberToObject(nt, "min", TEMP_NOTICE_MIN_MIN);
    cJSON_AddNumberToObject(nt, "max", TEMP_NOTICE_MAX_MIN);
    cJSON_AddNumberToObject(nt, "step", 1);
    cJSON_AddStringToObject(nt, "unit_of_measurement", "min");
    cJSON_AddStringToObject(nt, "mode", "box");
    cJSON_AddStringToObject(nt, "icon", "mdi:timer-outline");

    button(cmps, dev, cmd, "clear_notification", "Clear notification",
           HA_CMD_CLEAR_NOTIFICATION, NULL, NULL);
    button(cmps, dev, cmd, "reboot", "Restart", HA_CMD_REBOOT, "restart",
           "config");
    button(cmps, dev, cmd, "calibrate_co2", "Calibrate CO2 (400 ppm)",
           HA_CMD_CALIBRATE_CO2, NULL, "config");

    cJSON *sel = state_component(cmps, dev, "layout", "select",
                                 "Screen layout", st);
    topic(buf, sizeof(buf), dev, HA_TOPIC_LAYOUT);
    cJSON_AddStringToObject(sel, "command_topic", buf);
    cJSON *opts = cJSON_AddArrayToObject(sel, "options");
    for (int i = 0; i < LAYOUT_COUNT; i++) {
        cJSON_AddItemToArray(opts,
                             cJSON_CreateString(layout_name((layout_t)i)));
    }
    cJSON_AddStringToObject(sel, "icon", "mdi:view-dashboard");

    cJSON *num = state_component(cmps, dev, "co2_threshold", "number",
                                 "CO2 threshold", st);
    topic(buf, sizeof(buf), dev, HA_TOPIC_THRESHOLD);
    cJSON_AddStringToObject(num, "command_topic", buf);
    cJSON_AddNumberToObject(num, "min", CO2_THRESHOLD_MIN);
    cJSON_AddNumberToObject(num, "max", CO2_THRESHOLD_MAX);
    cJSON_AddNumberToObject(num, "step", 50);
    cJSON_AddStringToObject(num, "unit_of_measurement", "ppm");
    cJSON_AddStringToObject(num, "mode", "box");
    cJSON_AddStringToObject(num, "entity_category", "config");

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return out;
}

static void add_number_or_null(cJSON *obj, const char *key, float v,
                                int decimals)
{
    if (isnan(v)) {
        cJSON_AddNullToObject(obj, key);
    } else {
        double p = pow(10, decimals);
        cJSON_AddNumberToObject(obj, key, round(v * p) / p);
    }
}

char *ha_state_payload(const ha_state_t *st)
{
    cJSON *root = cJSON_CreateObject();
    add_number_or_null(root, "temperature", st->data.temperature_c, 1);
    add_number_or_null(root, "humidity", st->data.humidity_pct, 0);
    add_number_or_null(root, "pressure", st->data.pressure_hpa, 1);
    if (st->data.co2_ppm > 0) {
        cJSON_AddNumberToObject(root, "co2", st->data.co2_ppm);
        cJSON_AddStringToObject(root, "co2_high",
                                st->data.co2_ppm > st->co2_threshold_ppm
                                    ? "ON"
                                    : "OFF");
    } else {
        cJSON_AddNullToObject(root, "co2");
        cJSON_AddNullToObject(root, "co2_high");
    }
    if (st->rssi_dbm != 0) {
        cJSON_AddNumberToObject(root, "rssi", st->rssi_dbm);
    } else {
        cJSON_AddNullToObject(root, "rssi");
    }
    cJSON_AddNumberToObject(root, "uptime", (double)st->uptime_s);
    cJSON_AddStringToObject(root, "layout", layout_name(st->layout));
    cJSON_AddNumberToObject(root, "co2_threshold", st->co2_threshold_ppm);
    cJSON_AddStringToObject(root, "notification",
                            st->notification != NULL ? st->notification : "");
    cJSON_AddStringToObject(root, "notification_temp",
                            st->notification_temp != NULL
                                ? st->notification_temp
                                : "");
    cJSON_AddNumberToObject(root, "notification_time", st->temp_notice_min);

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return out;
}
