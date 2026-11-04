#include "ha_command.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ha_discovery.h"

/* topic == prefix + "/" + suffix */
static bool topic_is(const char *topic, int len, const char *prefix,
                     const char *suffix)
{
    size_t plen = strlen(prefix);
    size_t slen = strlen(suffix);
    return (size_t)len == plen + 1 + slen && strncmp(topic, prefix, plen) == 0 &&
           topic[plen] == '/' && strncmp(topic + plen + 1, suffix, slen) == 0;
}

static bool payload_is(const char *data, int len, const char *s)
{
    return (size_t)len == strlen(s) && strncmp(data, s, (size_t)len) == 0;
}

/* Copies the payload into a small NUL-terminated buffer, trimming spaces. */
static void small_copy(char *dst, size_t size, const char *data, int len)
{
    while (len > 0 && (*data == ' ' || *data == '\n' || *data == '\r')) {
        data++;
        len--;
    }
    while (len > 0 && (data[len - 1] == ' ' || data[len - 1] == '\n' ||
                       data[len - 1] == '\r')) {
        len--;
    }
    size_t n = (size_t)len < size - 1 ? (size_t)len : size - 1;
    memcpy(dst, data, n);
    dst[n] = '\0';
}

/* Plain text from a HA text entity, even if it looks like JSON. */
static void plain_text(char *dst, size_t size, const char *data, int len)
{
    size_t n = (size_t)len < size - 1 ? (size_t)len : size - 1;
    while (n > 0 && n < (size_t)len && ((unsigned char)data[n] & 0xC0) == 0x80) {
        n--; /* do not cut a UTF-8 character */
    }
    memcpy(dst, data, n);
    dst[n] = '\0';
}

/* Number entity payload ("1250", "1250.0") within [min, max]. */
static bool parse_number(const char *data, int len, int min, int max, int *out)
{
    char buf[24];
    small_copy(buf, sizeof(buf), data, len);
    char *end;
    float v = strtof(buf, &end);
    if (end == buf || *end != '\0' || v < min || v > max) {
        return false;
    }
    *out = (int)(v + 0.5f);
    return true;
}

void ha_parse_command(const char *base, const char *discovery_prefix,
                      const char *topic, int topic_len, const char *data,
                      int data_len, ha_command_t *out)
{
    memset(out, 0, sizeof(*out));
    out->action = HA_ACT_NONE;

    if (topic_is(topic, topic_len, discovery_prefix, "status")) {
        if (payload_is(data, data_len, "online")) {
            out->action = HA_ACT_HA_ONLINE;
        }
        return;
    }

    if (topic_is(topic, topic_len, base, HA_TOPIC_NOTIFY)) {
        out->action = notify_parse(data, data_len, out->text,
                                   sizeof(out->text), &out->ttl_s,
                                   &out->fullscreen)
                          ? HA_ACT_NOTIFY
                          : HA_ACT_INVALID;
        return;
    }

    if (topic_is(topic, topic_len, base, HA_TOPIC_NOTIFICATION)) {
        plain_text(out->text, sizeof(out->text), data, data_len);
        out->action = HA_ACT_NOTIFY; /* permanent: ttl 0 */
        return;
    }

    if (topic_is(topic, topic_len, base, HA_TOPIC_NOTIFICATION_TEMP)) {
        plain_text(out->text, sizeof(out->text), data, data_len);
        out->action = HA_ACT_NOTIFY_TEMP;
        return;
    }

    if (topic_is(topic, topic_len, base, HA_TOPIC_COMMAND)) {
        if (payload_is(data, data_len, HA_CMD_CLEAR_NOTIFICATION)) {
            out->action = HA_ACT_CLEAR;
        } else if (payload_is(data, data_len, HA_CMD_REBOOT)) {
            out->action = HA_ACT_REBOOT;
        } else if (payload_is(data, data_len, HA_CMD_CALIBRATE_CO2)) {
            out->action = HA_ACT_CALIBRATE;
        } else {
            out->action = HA_ACT_INVALID;
        }
        return;
    }

    char buf[24];
    if (topic_is(topic, topic_len, base, HA_TOPIC_LAYOUT)) {
        small_copy(buf, sizeof(buf), data, data_len);
        out->action = layout_from_name(buf, &out->layout) ? HA_ACT_LAYOUT
                                                          : HA_ACT_INVALID;
        return;
    }

    if (topic_is(topic, topic_len, base, HA_TOPIC_THRESHOLD)) {
        out->action = parse_number(data, data_len, CO2_THRESHOLD_MIN,
                                   CO2_THRESHOLD_MAX, &out->threshold)
                           ? HA_ACT_THRESHOLD
                           : HA_ACT_INVALID;
        return;
    }

    if (topic_is(topic, topic_len, base, HA_TOPIC_NOTIFICATION_TIME)) {
        out->action = parse_number(data, data_len, TEMP_NOTICE_MIN_MIN,
                                   TEMP_NOTICE_MAX_MIN, &out->minutes)
                           ? HA_ACT_TEMP_TIME
                           : HA_ACT_INVALID;
        return;
    }
}
