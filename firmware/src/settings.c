#include "settings.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *METRIC_NAMES[METRIC_COUNT] = {
    "temperature", "humidity", "pressure", "co2",
};
static const char *LAYOUT_NAMES[LAYOUT_COUNT] = {
    "classic", "clock", "co2", "sensors",
};
static const char *DATE_NAMES[DATE_FORMAT_COUNT] = {
    "dmy_text", "weekday", "dmy_dots", "iso",
};
static const char *LANG_NAMES[LANG_COUNT] = { "en", "ru" };
static const char *TEMP_UNIT_NAMES[TEMP_UNIT_COUNT] = { "c", "f" };
static const char *PRESSURE_NAMES[PRESSURE_UNIT_COUNT] = { "hpa", "mmhg" };

const char *metric_name(metric_id_t id)
{
    return (unsigned)id < METRIC_COUNT ? METRIC_NAMES[id] : "";
}

const char *layout_name(layout_t l)
{
    return (unsigned)l < LAYOUT_COUNT ? LAYOUT_NAMES[l] : "";
}

static int name_index(const char *const *names, int count, const char *name)
{
    if (name == NULL) {
        return -1;
    }
    for (int i = 0; i < count; i++) {
        if (strcmp(names[i], name) == 0) {
            return i;
        }
    }
    return -1;
}

bool layout_from_name(const char *name, layout_t *out)
{
    int i = name_index(LAYOUT_NAMES, LAYOUT_COUNT, name);
    if (i < 0) {
        return false;
    }
    *out = (layout_t)i;
    return true;
}

void settings_defaults(settings_t *s)
{
    memset(s, 0, sizeof(*s));
    strcpy(s->name, "E-Ink Clock");

    s->display.language = LANG_EN;
    s->display.hour24 = false;
    strcpy(s->display.timezone, "Europe/Moscow");
    strcpy(s->display.tz_posix, "MSK-3");
    s->display.date_format = DATE_DMY_TEXT;
    s->display.show_date = true;
    s->display.layout = LAYOUT_CLASSIC;
    for (int i = 0; i < METRIC_COUNT; i++) {
        s->display.metrics[i].id = (metric_id_t)i;
        s->display.metrics[i].enabled = true;
    }
    s->display.temp_unit = TEMP_UNIT_C;
    s->display.pressure_unit = PRESSURE_HPA;

    s->sensors.temp_offset_c = -2.3f;
    s->sensors.co2_threshold_ppm = 1000;
    s->sensors.co2_led = true;

    s->mqtt.port = 1883;
    s->mqtt.discovery = true;
    strcpy(s->mqtt.discovery_prefix, "homeassistant");
}

/* ===================================================== */

static bool fix_enum(int *v, int count, int fallback)
{
    if (*v < 0 || *v >= count) {
        *v = fallback;
        return true;
    }
    return false;
}

/* Every metric must appear once: drop duplicates/garbage, append missing
 * ones (disabled) at the end. */
static bool fix_metric_order(metric_slot_t *m)
{
    metric_slot_t out[METRIC_COUNT];
    bool seen[METRIC_COUNT] = { false };
    int n = 0;
    bool changed = false;

    for (int i = 0; i < METRIC_COUNT; i++) {
        int id = (int)m[i].id;
        if (id < 0 || id >= METRIC_COUNT || seen[id]) {
            changed = true;
            continue;
        }
        seen[id] = true;
        out[n++] = m[i];
    }
    for (int id = 0; id < METRIC_COUNT; id++) {
        if (!seen[id]) {
            out[n].id = (metric_id_t)id;
            out[n].enabled = false;
            n++;
            changed = true;
        }
    }
    memcpy(m, out, sizeof(out));
    return changed;
}

/* POSIX TZ must start with a 3+ letter name or <...>; anything else makes
 * newlib silently fall back to UTC, which is worse than keeping MSK-3. */
static bool tz_posix_valid(const char *tz)
{
    if (tz[0] == '<') {
        return strchr(tz, '>') != NULL;
    }
    int letters = 0;
    while ((tz[letters] >= 'A' && tz[letters] <= 'Z') ||
           (tz[letters] >= 'a' && tz[letters] <= 'z')) {
        letters++;
    }
    return letters >= 3;
}

/* Topic segments must not contain MQTT wildcards. */
static bool strip_mqtt_wildcards(char *s)
{
    bool changed = false;
    for (char *p = s; *p; p++) {
        if (*p == '#' || *p == '+') {
            *p = '_';
            changed = true;
        }
    }
    size_t len = strlen(s);
    while (len > 0 && s[len - 1] == '/') {
        s[--len] = '\0';
        changed = true;
    }
    return changed;
}

bool settings_sanitize(settings_t *s)
{
    bool changed = false;

    changed |= fix_enum((int *)&s->display.language, LANG_COUNT, LANG_EN);
    changed |= fix_enum((int *)&s->display.date_format, DATE_FORMAT_COUNT,
                        DATE_DMY_TEXT);
    changed |= fix_enum((int *)&s->display.layout, LAYOUT_COUNT,
                        LAYOUT_CLASSIC);
    changed |= fix_enum((int *)&s->display.temp_unit, TEMP_UNIT_COUNT,
                        TEMP_UNIT_C);
    changed |= fix_enum((int *)&s->display.pressure_unit, PRESSURE_UNIT_COUNT,
                        PRESSURE_HPA);
    changed |= fix_metric_order(s->display.metrics);

    if (!tz_posix_valid(s->display.tz_posix)) {
        strcpy(s->display.tz_posix, "MSK-3");
        strcpy(s->display.timezone, "Europe/Moscow");
        changed = true;
    }

    if (s->sensors.co2_threshold_ppm < CO2_THRESHOLD_MIN) {
        s->sensors.co2_threshold_ppm = CO2_THRESHOLD_MIN;
        changed = true;
    } else if (s->sensors.co2_threshold_ppm > CO2_THRESHOLD_MAX) {
        s->sensors.co2_threshold_ppm = CO2_THRESHOLD_MAX;
        changed = true;
    }

    float off = s->sensors.temp_offset_c;
    if (isnan(off)) {
        s->sensors.temp_offset_c = 0.0f;
        changed = true;
    } else if (off > TEMP_OFFSET_LIMIT || off < -TEMP_OFFSET_LIMIT) {
        s->sensors.temp_offset_c =
            off > 0 ? TEMP_OFFSET_LIMIT : -TEMP_OFFSET_LIMIT;
        changed = true;
    }

    if (s->mqtt.port == 0) {
        s->mqtt.port = 1883;
        changed = true;
    }
    changed |= strip_mqtt_wildcards(s->mqtt.prefix);
    changed |= strip_mqtt_wildcards(s->mqtt.discovery_prefix);
    if (s->mqtt.discovery_prefix[0] == '\0') {
        strcpy(s->mqtt.discovery_prefix, "homeassistant");
        changed = true;
    }
    if (s->name[0] == '\0') {
        strcpy(s->name, "E-Ink Clock");
        changed = true;
    }

    return changed;
}

/* ===================================================== */

static void get_str(const cJSON *obj, const char *key, char *dst, size_t size)
{
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsString(it)) {
        snprintf(dst, size, "%s", it->valuestring);
    }
}

static void get_bool(const cJSON *obj, const char *key, bool *dst)
{
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsBool(it)) {
        *dst = cJSON_IsTrue(it);
    }
}

static void get_int(const cJSON *obj, const char *key, int *dst)
{
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsNumber(it)) {
        *dst = (int)it->valuedouble;
    }
}

static void get_float(const cJSON *obj, const char *key, float *dst)
{
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsNumber(it)) {
        *dst = (float)it->valuedouble;
    }
}

/* Enums travel as strings; unknown names leave the value untouched. */
static void get_enum(const cJSON *obj, const char *key,
                     const char *const *names, int count, int *dst)
{
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsString(it)) {
        int i = name_index(names, count, it->valuestring);
        if (i >= 0) {
            *dst = i;
        }
    }
}

static void apply_metrics(settings_t *s, const cJSON *arr)
{
    if (!cJSON_IsArray(arr)) {
        return;
    }

    metric_slot_t m[METRIC_COUNT];
    int n = 0;
    const cJSON *it;
    cJSON_ArrayForEach(it, arr) {
        if (n >= METRIC_COUNT) {
            break;
        }
        const cJSON *id = cJSON_GetObjectItemCaseSensitive(it, "id");
        int idx = cJSON_IsString(id)
                      ? name_index(METRIC_NAMES, METRIC_COUNT, id->valuestring)
                      : -1;
        if (idx < 0) {
            continue;
        }
        m[n].id = (metric_id_t)idx;
        m[n].enabled = true;
        get_bool(it, "enabled", &m[n].enabled);
        n++;
    }
    /* Unlisted metrics get id = -1 here and are re-added by sanitize. */
    for (int i = n; i < METRIC_COUNT; i++) {
        m[i].id = (metric_id_t)-1;
        m[i].enabled = false;
    }
    memcpy(s->display.metrics, m, sizeof(m));
}

void settings_apply_json(settings_t *s, const cJSON *json)
{
    if (!cJSON_IsObject(json)) {
        return;
    }

    get_str(json, "name", s->name, sizeof(s->name));

    const cJSON *wifi = cJSON_GetObjectItemCaseSensitive(json, "wifi");
    if (cJSON_IsObject(wifi)) {
        get_str(wifi, "ssid", s->wifi.ssid, sizeof(s->wifi.ssid));
        get_str(wifi, "password", s->wifi.password,
                sizeof(s->wifi.password));
    }

    const cJSON *d = cJSON_GetObjectItemCaseSensitive(json, "display");
    if (cJSON_IsObject(d)) {
        get_enum(d, "language", LANG_NAMES, LANG_COUNT,
                 (int *)&s->display.language);
        get_bool(d, "hour24", &s->display.hour24);
        get_str(d, "timezone", s->display.timezone,
                sizeof(s->display.timezone));
        get_str(d, "tz_posix", s->display.tz_posix,
                sizeof(s->display.tz_posix));
        get_enum(d, "date_format", DATE_NAMES, DATE_FORMAT_COUNT,
                 (int *)&s->display.date_format);
        get_bool(d, "show_date", &s->display.show_date);
        get_enum(d, "layout", LAYOUT_NAMES, LAYOUT_COUNT,
                 (int *)&s->display.layout);
        apply_metrics(s, cJSON_GetObjectItemCaseSensitive(d, "metrics"));
        get_bool(d, "invert", &s->display.invert);
        get_bool(d, "rotate180", &s->display.rotate180);
        get_enum(d, "temp_unit", TEMP_UNIT_NAMES, TEMP_UNIT_COUNT,
                 (int *)&s->display.temp_unit);
        get_enum(d, "pressure_unit", PRESSURE_NAMES, PRESSURE_UNIT_COUNT,
                 (int *)&s->display.pressure_unit);
    }

    const cJSON *sn = cJSON_GetObjectItemCaseSensitive(json, "sensors");
    if (cJSON_IsObject(sn)) {
        get_float(sn, "temp_offset_c", &s->sensors.temp_offset_c);
        get_int(sn, "co2_threshold_ppm", &s->sensors.co2_threshold_ppm);
        get_bool(sn, "co2_led", &s->sensors.co2_led);
    }

    const cJSON *m = cJSON_GetObjectItemCaseSensitive(json, "mqtt");
    if (cJSON_IsObject(m)) {
        get_bool(m, "enabled", &s->mqtt.enabled);
        get_str(m, "host", s->mqtt.host, sizeof(s->mqtt.host));
        int port = s->mqtt.port;
        get_int(m, "port", &port);
        s->mqtt.port = (port > 0 && port <= 65535) ? (uint16_t)port : 1883;
        get_str(m, "username", s->mqtt.username, sizeof(s->mqtt.username));
        get_str(m, "password", s->mqtt.password, sizeof(s->mqtt.password));
        get_str(m, "prefix", s->mqtt.prefix, sizeof(s->mqtt.prefix));
        get_bool(m, "discovery", &s->mqtt.discovery);
        get_str(m, "discovery_prefix", s->mqtt.discovery_prefix,
                sizeof(s->mqtt.discovery_prefix));
    }

    settings_sanitize(s);
}

cJSON *settings_to_json(const settings_t *s, bool include_secrets)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "name", s->name);

    cJSON *wifi = cJSON_AddObjectToObject(root, "wifi");
    cJSON_AddStringToObject(wifi, "ssid", s->wifi.ssid);
    if (include_secrets) {
        cJSON_AddStringToObject(wifi, "password", s->wifi.password);
    } else {
        cJSON_AddBoolToObject(wifi, "password_set", s->wifi.password[0]);
    }

    cJSON *d = cJSON_AddObjectToObject(root, "display");
    cJSON_AddStringToObject(d, "language", LANG_NAMES[s->display.language]);
    cJSON_AddBoolToObject(d, "hour24", s->display.hour24);
    cJSON_AddStringToObject(d, "timezone", s->display.timezone);
    cJSON_AddStringToObject(d, "tz_posix", s->display.tz_posix);
    cJSON_AddStringToObject(d, "date_format",
                            DATE_NAMES[s->display.date_format]);
    cJSON_AddBoolToObject(d, "show_date", s->display.show_date);
    cJSON_AddStringToObject(d, "layout", LAYOUT_NAMES[s->display.layout]);
    cJSON *metrics = cJSON_AddArrayToObject(d, "metrics");
    for (int i = 0; i < METRIC_COUNT; i++) {
        cJSON *it = cJSON_CreateObject();
        cJSON_AddStringToObject(it, "id",
                                metric_name(s->display.metrics[i].id));
        cJSON_AddBoolToObject(it, "enabled", s->display.metrics[i].enabled);
        cJSON_AddItemToArray(metrics, it);
    }
    cJSON_AddBoolToObject(d, "invert", s->display.invert);
    cJSON_AddBoolToObject(d, "rotate180", s->display.rotate180);
    cJSON_AddStringToObject(d, "temp_unit",
                            TEMP_UNIT_NAMES[s->display.temp_unit]);
    cJSON_AddStringToObject(d, "pressure_unit",
                            PRESSURE_NAMES[s->display.pressure_unit]);

    cJSON *sn = cJSON_AddObjectToObject(root, "sensors");
    /* Round to 0.1 so -2.3f does not come out as -2.2999999523. */
    cJSON_AddNumberToObject(sn, "temp_offset_c",
                            roundf(s->sensors.temp_offset_c * 10.0f) / 10.0);
    cJSON_AddNumberToObject(sn, "co2_threshold_ppm",
                            s->sensors.co2_threshold_ppm);
    cJSON_AddBoolToObject(sn, "co2_led", s->sensors.co2_led);

    cJSON *m = cJSON_AddObjectToObject(root, "mqtt");
    cJSON_AddBoolToObject(m, "enabled", s->mqtt.enabled);
    cJSON_AddStringToObject(m, "host", s->mqtt.host);
    cJSON_AddNumberToObject(m, "port", s->mqtt.port);
    cJSON_AddStringToObject(m, "username", s->mqtt.username);
    if (include_secrets) {
        cJSON_AddStringToObject(m, "password", s->mqtt.password);
    } else {
        cJSON_AddBoolToObject(m, "password_set", s->mqtt.password[0]);
    }
    cJSON_AddStringToObject(m, "prefix", s->mqtt.prefix);
    cJSON_AddBoolToObject(m, "discovery", s->mqtt.discovery);
    cJSON_AddStringToObject(m, "discovery_prefix", s->mqtt.discovery_prefix);

    return root;
}
