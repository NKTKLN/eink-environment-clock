/*
 * Home Assistant MQTT discovery and state payloads. Pure C + cJSON
 * (host-tested); the MQTT connection itself lives in mqtt_ha.c.
 *
 * Uses device-based discovery (one retained message describing the device
 * and all its entities), supported since Home Assistant 2024.11.
 */
#pragma once

#include <stdbool.h>

#include "settings.h"
#include "ui.h"

typedef struct {
    const char *device_id;  /* "a1b2c3", from the MAC */
    const char *name;       /* user-visible device name */
    const char *base;       /* topic prefix, e.g. "eink-clock/a1b2c3" */
    const char *discovery_prefix;
    const char *sw_version;
    const char *config_url; /* "http://192.168.1.42/" or NULL */
} ha_device_t;

/* Topic suffixes below <base>/ */
#define HA_TOPIC_STATE        "state"
#define HA_TOPIC_STATUS       "status" /* online/offline, LWT */
#define HA_TOPIC_NOTIFY       "notify" /* JSON {"text","ttl","fullscreen"} */
#define HA_TOPIC_NOTIFICATION "notification/set" /* plain text */
#define HA_TOPIC_COMMAND      "command"
#define HA_TOPIC_LAYOUT       "layout/set"
#define HA_TOPIC_THRESHOLD    "co2_threshold/set"

#define HA_CMD_CLEAR_NOTIFICATION "clear_notification"
#define HA_CMD_REBOOT             "reboot"
#define HA_CMD_CALIBRATE_CO2      "calibrate_co2"

/* "<discovery_prefix>/device/<id>/config" */
void ha_discovery_topic(char *buf, size_t size, const ha_device_t *dev);

/* Retained discovery message. Caller frees with cJSON_free(). */
char *ha_discovery_payload(const ha_device_t *dev);

typedef struct {
    sensor_data_t data;
    int co2_threshold_ppm;
    layout_t layout;
    int rssi_dbm; /* 0 = unknown */
    long uptime_s;
    const char *notification;
} ha_state_t;

/* State JSON published to <base>/state. Caller frees with cJSON_free(). */
char *ha_state_payload(const ha_state_t *st);
