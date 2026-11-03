/*
 * User settings: defaults, validation and JSON (de)serialization.
 *
 * Pure C + cJSON, no ESP-IDF dependencies, so it is unit-tested on the host.
 * Persistence lives in settings_store.c.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cJSON.h"

typedef enum {
    METRIC_TEMPERATURE,
    METRIC_HUMIDITY,
    METRIC_PRESSURE,
    METRIC_CO2,
    METRIC_COUNT,
} metric_id_t;

typedef enum {
    LAYOUT_CLASSIC, /* clock on the left, metrics column on the right */
    LAYOUT_CLOCK,   /* big clock over the whole screen */
    LAYOUT_CO2,     /* big CO2 value, clock and other metrics small */
    LAYOUT_SENSORS, /* metrics only, 2x2 grid */
    LAYOUT_COUNT,
} layout_t;

typedef enum {
    DATE_DMY_TEXT,    /* 3 Nov 2026 */
    DATE_WEEKDAY,     /* Tue, 3 Nov */
    DATE_DMY_DOTS,    /* 03.11.2026 */
    DATE_ISO,         /* 2026-11-03 */
    DATE_FORMAT_COUNT,
} date_format_t;

typedef enum { LANG_EN, LANG_RU, LANG_COUNT } language_t;
typedef enum { TEMP_UNIT_C, TEMP_UNIT_F, TEMP_UNIT_COUNT } temp_unit_t;
typedef enum { PRESSURE_HPA, PRESSURE_MMHG, PRESSURE_UNIT_COUNT } pressure_unit_t;

typedef struct {
    metric_id_t id;
    bool enabled;
} metric_slot_t;

typedef struct {
    char name[32];

    struct {
        char ssid[33];
        char password[65];
    } wifi;

    struct {
        language_t language;
        bool hour24;
        char timezone[48]; /* IANA name, informational (shown in the UI) */
        char tz_posix[48]; /* what the firmware actually uses */
        date_format_t date_format;
        bool show_date;
        layout_t layout;
        metric_slot_t metrics[METRIC_COUNT]; /* display order */
        bool invert;
        bool rotate180;
        temp_unit_t temp_unit;
        pressure_unit_t pressure_unit;
    } display;

    struct {
        float temp_offset_c;
        int co2_threshold_ppm;
        bool co2_led;
    } sensors;

    struct {
        bool enabled;
        char host[64];
        uint16_t port;
        char username[64];
        char password[64];
        char prefix[64]; /* empty = eink-clock/<id> */
        bool discovery;
        char discovery_prefix[32];
    } mqtt;

    struct {
        bool check; /* look for new GitHub releases once a day */
        bool auto_install; /* install them at night (03:00-05:00) */
    } update;
} settings_t;

#define CO2_THRESHOLD_MIN 400
#define CO2_THRESHOLD_MAX 5000
#define TEMP_OFFSET_LIMIT 10.0f

void settings_defaults(settings_t *s);

/* Clamps numbers, fixes enums and repairs the metric order so that every
 * metric appears exactly once. Returns true if anything had to change. */
bool settings_sanitize(settings_t *s);

/* Merges a (possibly partial) JSON object into *s. Unknown keys are ignored,
 * missing keys keep their current value. Secrets are only touched when the
 * key is present. Calls settings_sanitize() at the end. */
void settings_apply_json(settings_t *s, const cJSON *json);

/* include_secrets = false replaces passwords with "*_set" booleans:
 * this is the form served by the HTTP API. true is the storage form. */
cJSON *settings_to_json(const settings_t *s, bool include_secrets);

/* Name <-> id helpers shared by the HTTP API, MQTT and the UI. */
const char *metric_name(metric_id_t id);
const char *layout_name(layout_t l);
bool layout_from_name(const char *name, layout_t *out);
