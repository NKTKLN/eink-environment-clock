/* Decoding of incoming MQTT messages (pure, host-tested). */
#pragma once

#include <stdbool.h>

#include "notify.h"
#include "settings.h"

typedef enum {
    HA_ACT_NONE,      /* not ours / ignored */
    HA_ACT_INVALID,   /* ours, but the payload is malformed */
    HA_ACT_NOTIFY,    /* text, ttl, fullscreen */
    HA_ACT_NOTIFY_TEMP, /* text; shown for settings temp_notice_min */
    HA_ACT_CLEAR,
    HA_ACT_REBOOT,
    HA_ACT_CALIBRATE,
    HA_ACT_LAYOUT,    /* layout */
    HA_ACT_THRESHOLD, /* threshold */
    HA_ACT_TEMP_TIME, /* minutes */
    HA_ACT_HA_ONLINE, /* Home Assistant restarted: resend discovery */
} ha_action_t;

typedef struct {
    ha_action_t action;
    char text[NOTIFY_TEXT_MAX];
    int ttl_s;
    bool fullscreen;
    layout_t layout;
    int threshold;
    int minutes;
} ha_command_t;

/* topic/data are not NUL-terminated (as delivered by esp-mqtt). */
void ha_parse_command(const char *base, const char *discovery_prefix,
                      const char *topic, int topic_len, const char *data,
                      int data_len, ha_command_t *out);
