#include "web_server.h"

#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#include "esp_chip_info.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "app_state.h"
#include "bmp.h"
#include "epd_canvas.h"
#include "mqtt_ha.h"
#include "net_time.h"
#include "notify.h"
#include "sensors.h"
#include "settings_store.h"
#include "web_index.h"
#include "wifi_mgr.h"

static const char *TAG = "web";

#define MAX_JSON_BODY 4096

/* ===================================================== */
/* Helpers                                               */
/* ===================================================== */

static void cors(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
}

static esp_err_t send_json(httpd_req_t *req, cJSON *json)
{
    char *text = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    if (text == NULL) {
        return httpd_resp_send_500(req);
    }
    cors(req);
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_sendstr(req, text);
    cJSON_free(text);
    return err;
}

static esp_err_t send_ok(httpd_req_t *req)
{
    cJSON *j = cJSON_CreateObject();
    cJSON_AddBoolToObject(j, "ok", true);
    return send_json(req, j);
}

static esp_err_t send_error(httpd_req_t *req, const char *status,
                            const char *message)
{
    cJSON *j = cJSON_CreateObject();
    cJSON_AddStringToObject(j, "error", message);
    httpd_resp_set_status(req, status);
    return send_json(req, j);
}

/* Reads the whole body into a NUL-terminated heap buffer. */
static char *read_body(httpd_req_t *req, size_t max, int *out_len)
{
    if (req->content_len > max) {
        return NULL;
    }
    char *buf = malloc(req->content_len + 1);
    if (buf == NULL) {
        return NULL;
    }
    size_t got = 0;
    while (got < req->content_len) {
        int r = httpd_req_recv(req, buf + got, req->content_len - got);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (r <= 0) {
            free(buf);
            return NULL;
        }
        got += (size_t)r;
    }
    buf[got] = '\0';
    if (out_len != NULL) {
        *out_len = (int)got;
    }
    return buf;
}

static cJSON *read_json(httpd_req_t *req)
{
    char *body = read_body(req, MAX_JSON_BODY, NULL);
    if (body == NULL) {
        return NULL;
    }
    cJSON *json = cJSON_Parse(body);
    free(body);
    return json;
}

static void add_num_or_null(cJSON *o, const char *key, float v)
{
    if (isnan(v)) {
        cJSON_AddNullToObject(o, key);
    } else {
        cJSON_AddNumberToObject(o, key, roundf(v * 10.0f) / 10.0);
    }
}

static const char *reset_reason_name(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON: return "power_on";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_DEEPSLEEP: return "deep_sleep";
    case ESP_RST_USB: return "usb";
    default: return "other";
    }
}

/* ===================================================== */
/* Static page and captive portal                        */
/* ===================================================== */

static esp_err_t index_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, (const char *)WEB_INDEX_GZ, WEB_INDEX_GZ_LEN);
}

/* Any unknown GET. While the setup AP is up, requests for foreign hosts
 * (connectivity checks: generate_204, hotspot-detect.html, ...) are
 * redirected to the setup page, which makes phones show the portal. */
static bool via_access_point(httpd_req_t *req)
{
    struct sockaddr_in6 addr;
    socklen_t len = sizeof(addr);
    if (getsockname(httpd_req_to_sockfd(req), (struct sockaddr *)&addr,
                    &len) != 0) {
        return false;
    }
    char ip[48] = "";
    if (addr.sin6_family == AF_INET6) {
        /* IPv4-mapped: the IPv4 address sits in the last 4 bytes */
        inet_ntop(AF_INET, &addr.sin6_addr.un.u32_addr[3], ip, sizeof(ip));
    } else {
        inet_ntop(AF_INET, &((struct sockaddr_in *)&addr)->sin_addr, ip,
                  sizeof(ip));
    }
    return strcmp(ip, WIFI_AP_IP) == 0;
}

static esp_err_t fallback_get(httpd_req_t *req)
{
    char host[64] = "";
    httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host));

    if (via_access_point(req) && strcmp(host, WIFI_AP_IP) != 0) {
        httpd_resp_set_status(req, "302 Found");
        httpd_resp_set_hdr(req, "Location", "http://" WIFI_AP_IP "/");
        return httpd_resp_send(req, NULL, 0);
    }
    if (strncmp(req->uri, "/api/", 5) == 0) {
        return send_error(req, "404 Not Found", "unknown endpoint");
    }
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t options_any(httpd_req_t *req)
{
    cors(req);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods",
                       "GET, POST, DELETE, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

/* ===================================================== */
/* State and settings                                    */
/* ===================================================== */

static esp_err_t state_get(httpd_req_t *req)
{
    settings_t s;
    settings_get(&s);
    sensor_data_t d;
    app_get_sensors(&d);
    wifi_status_t ws;
    wifi_mgr_get_status(&ws);

    cJSON *root = cJSON_CreateObject();

    cJSON *dev = cJSON_AddObjectToObject(root, "device");
    cJSON_AddStringToObject(dev, "id", app_device_id());
    cJSON_AddStringToObject(dev, "name", s.name);
    cJSON_AddStringToObject(dev, "version", app_version());
    cJSON_AddStringToObject(dev, "idf", esp_get_idf_version());
    cJSON_AddNumberToObject(dev, "uptime_s", (double)(app_uptime_ms() / 1000));
    cJSON_AddNumberToObject(dev, "heap_free", esp_get_free_heap_size());
    cJSON_AddNumberToObject(dev, "heap_min",
                            esp_get_minimum_free_heap_size());
    cJSON_AddStringToObject(dev, "reset_reason",
                            reset_reason_name(esp_reset_reason()));
    cJSON_AddStringToObject(dev, "hostname", ws.hostname);
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char macs[18];
    snprintf(macs, sizeof(macs), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0],
             mac[1], mac[2], mac[3], mac[4], mac[5]);
    cJSON_AddStringToObject(dev, "mac", macs);
    const esp_partition_t *part = esp_ota_get_running_partition();
    cJSON_AddStringToObject(dev, "partition", part ? part->label : "");

    cJSON *sn = cJSON_AddObjectToObject(root, "sensors");
    add_num_or_null(sn, "temperature_c", d.temperature_c);
    add_num_or_null(sn, "humidity_pct", d.humidity_pct);
    add_num_or_null(sn, "pressure_hpa", d.pressure_hpa);
    if (d.co2_ppm > 0) {
        cJSON_AddNumberToObject(sn, "co2_ppm", d.co2_ppm);
    } else {
        cJSON_AddNullToObject(sn, "co2_ppm");
    }
    cJSON_AddBoolToObject(sn, "co2_high",
                          d.co2_ppm > s.sensors.co2_threshold_ppm);

    struct tm now;
    bool valid = sensors_get_time(&now);
    cJSON *t = cJSON_AddObjectToObject(root, "time");
    cJSON_AddBoolToObject(t, "valid", valid);
    cJSON_AddBoolToObject(t, "ntp_synced", net_time_valid());
    char iso[24];
    strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%S", &now);
    cJSON_AddStringToObject(t, "local", valid ? iso : "");
    cJSON_AddStringToObject(t, "timezone", s.display.timezone);

    cJSON *w = cJSON_AddObjectToObject(root, "wifi");
    cJSON_AddBoolToObject(w, "configured", ws.sta_configured);
    cJSON_AddBoolToObject(w, "connected", ws.sta_connected);
    cJSON_AddStringToObject(w, "ssid", ws.sta_ssid);
    cJSON_AddStringToObject(w, "ip", ws.ip);
    if (ws.sta_connected) {
        cJSON_AddNumberToObject(w, "rssi", ws.rssi);
    } else {
        cJSON_AddNullToObject(w, "rssi");
    }
    cJSON_AddBoolToObject(w, "ap_active", ws.ap_active);
    cJSON_AddStringToObject(w, "ap_ssid", ws.ap_ssid);
    cJSON_AddNumberToObject(w, "ap_clients", ws.ap_clients);

    cJSON *m = cJSON_AddObjectToObject(root, "mqtt");
    cJSON_AddBoolToObject(m, "enabled", s.mqtt.enabled);
    cJSON_AddBoolToObject(m, "connected", mqtt_ha_connected());
    char base[80];
    if (s.mqtt.prefix[0]) {
        strlcpy(base, s.mqtt.prefix, sizeof(base));
    } else {
        snprintf(base, sizeof(base), "eink-clock/%s", app_device_id());
    }
    cJSON_AddStringToObject(m, "base_topic", base);

    app_notice_t n;
    if (app_get_notice(&n, NULL)) {
        cJSON *nj = cJSON_AddObjectToObject(root, "notification");
        cJSON_AddStringToObject(nj, "text", n.text);
        cJSON_AddBoolToObject(nj, "fullscreen", n.fullscreen);
        cJSON_AddBoolToObject(nj, "system", n.is_system);
        if (n.remaining_s >= 0) {
            cJSON_AddNumberToObject(nj, "remaining_s", n.remaining_s);
        } else {
            cJSON_AddNullToObject(nj, "remaining_s");
        }
    } else {
        cJSON_AddNullToObject(root, "notification");
    }

    return send_json(req, root);
}

static esp_err_t config_get(httpd_req_t *req)
{
    settings_t s;
    settings_get(&s);
    return send_json(req, settings_to_json(&s, false));
}

static esp_err_t config_post(httpd_req_t *req)
{
    cJSON *json = read_json(req);
    if (!cJSON_IsObject(json)) {
        cJSON_Delete(json);
        return send_error(req, "400 Bad Request", "expected a JSON object");
    }
    /* Wi-Fi goes through /api/wifi, which also reconnects. */
    cJSON_DeleteItemFromObjectCaseSensitive(json, "wifi");
    esp_err_t err = settings_patch_json(json);
    cJSON_Delete(json);
    if (err != ESP_OK) {
        return send_error(req, "500 Internal Server Error", "save failed");
    }
    app_request_redraw();
    mqtt_ha_notify_changed();
    return config_get(req);
}

/* ===================================================== */
/* Screen                                                */
/* ===================================================== */

static esp_err_t send_bmp(httpd_req_t *req, const uint8_t *fb)
{
    uint8_t *bmp = malloc(BMP_SIZE);
    if (bmp == NULL) {
        return httpd_resp_send_500(req);
    }
    bmp_from_canvas(fb, bmp);
    cors(req);
    httpd_resp_set_type(req, "image/bmp");
    esp_err_t err = httpd_resp_send(req, (const char *)bmp, BMP_SIZE);
    free(bmp);
    return err;
}

static esp_err_t screen_get(httpd_req_t *req)
{
    uint8_t *fb = malloc(EPD_FRAMEBUFFER_SIZE);
    if (fb == NULL) {
        return httpd_resp_send_500(req);
    }
    app_copy_frame(fb);
    esp_err_t err = send_bmp(req, fb);
    free(fb);
    return err;
}

/* Renders unsaved settings for the live preview in the editor:
 * {"settings": {...partial}, "notification": {"text","fullscreen"},
 *  "screen": "clock" | "setup"} */
static esp_err_t preview_post(httpd_req_t *req)
{
    cJSON *json = read_json(req);
    if (!cJSON_IsObject(json)) {
        cJSON_Delete(json);
        return send_error(req, "400 Bad Request", "expected a JSON object");
    }

    settings_t s;
    settings_get(&s);
    settings_apply_json(&s, cJSON_GetObjectItem(json, "settings"));

    sensor_data_t d;
    app_get_sensors(&d);
    struct tm now;
    bool valid = sensors_get_time(&now);

    uint8_t *fb = malloc(EPD_FRAMEBUFFER_SIZE);
    if (fb == NULL) {
        cJSON_Delete(json);
        return httpd_resp_send_500(req);
    }

    const cJSON *screen = cJSON_GetObjectItem(json, "screen");
    if (cJSON_IsString(screen) && strcmp(screen->valuestring, "setup") == 0) {
        wifi_status_t ws;
        wifi_mgr_get_status(&ws);
        ui_render_setup(fb, &s, ws.ap_ssid, "12345678", WIFI_AP_IP, NULL);
    } else {
        ui_notice_t notice = { 0 };
        const cJSON *n = cJSON_GetObjectItem(json, "notification");
        const cJSON *text = cJSON_GetObjectItem(n, "text");
        if (cJSON_IsString(text)) {
            notice.text = text->valuestring;
            notice.fullscreen = cJSON_IsTrue(cJSON_GetObjectItem(n, "fullscreen"));
        }
        ui_render_clock(fb, &s, &now, valid, &d, &notice);
    }
    cJSON_Delete(json);

    esp_err_t err = send_bmp(req, fb);
    free(fb);
    return err;
}

/* ===================================================== */
/* Notifications                                         */
/* ===================================================== */

static esp_err_t notify_post(httpd_req_t *req)
{
    int len = 0;
    char *body = read_body(req, 2048, &len);
    if (body == NULL) {
        return send_error(req, "400 Bad Request", "body too large");
    }
    char text[NOTIFY_TEXT_MAX];
    int ttl;
    bool fullscreen;
    bool ok = notify_parse(body, len, text, sizeof(text), &ttl, &fullscreen);
    free(body);
    if (!ok) {
        return send_error(req, "400 Bad Request",
                          "expected {\"text\": string, \"ttl\": number, "
                          "\"fullscreen\": bool}");
    }
    app_notify(text, ttl, fullscreen);
    app_request_redraw();
    mqtt_ha_notify_changed();
    return send_ok(req);
}

static esp_err_t notify_delete(httpd_req_t *req)
{
    app_notify_clear();
    app_request_redraw();
    mqtt_ha_notify_changed();
    return send_ok(req);
}

/* ===================================================== */
/* Wi-Fi                                                 */
/* ===================================================== */

static esp_err_t wifi_scan_get(httpd_req_t *req)
{
    const int max = 20;
    wifi_ap_record_t *recs = calloc(max, sizeof(*recs));
    if (recs == NULL) {
        return httpd_resp_send_500(req);
    }
    int n = wifi_mgr_scan(recs, max);

    cJSON *arr = cJSON_CreateArray();
    for (int i = 0; i < n; i++) {
        if (recs[i].ssid[0] == '\0') {
            continue;
        }
        /* The driver lists one entry per access point: merge by name. */
        bool dup = false;
        for (int j = 0; j < i; j++) {
            if (strcmp((char *)recs[i].ssid, (char *)recs[j].ssid) == 0) {
                dup = true;
                break;
            }
        }
        if (dup) {
            continue;
        }
        cJSON *it = cJSON_CreateObject();
        cJSON_AddStringToObject(it, "ssid", (char *)recs[i].ssid);
        cJSON_AddNumberToObject(it, "rssi", recs[i].rssi);
        cJSON_AddBoolToObject(it, "secure",
                              recs[i].authmode != WIFI_AUTH_OPEN);
        cJSON_AddItemToArray(arr, it);
    }
    free(recs);
    return send_json(req, arr);
}

static esp_err_t wifi_post(httpd_req_t *req)
{
    cJSON *json = read_json(req);
    const cJSON *ssid = cJSON_GetObjectItem(json, "ssid");
    const cJSON *pass = cJSON_GetObjectItem(json, "password");
    if (!cJSON_IsString(ssid) || strlen(ssid->valuestring) > 32 ||
        (pass != NULL && !cJSON_IsString(pass)) ||
        (cJSON_IsString(pass) && strlen(pass->valuestring) > 64)) {
        cJSON_Delete(json);
        return send_error(req, "400 Bad Request",
                          "expected {\"ssid\": string, \"password\": string}");
    }

    /* Without a password field the saved one is kept for the same SSID. */
    settings_t s;
    settings_get(&s);
    const char *password = "";
    if (cJSON_IsString(pass)) {
        password = pass->valuestring;
    } else if (strcmp(s.wifi.ssid, ssid->valuestring) == 0) {
        password = s.wifi.password;
    }
    char ssid_copy[33];
    char pass_copy[65];
    strlcpy(ssid_copy, ssid->valuestring, sizeof(ssid_copy));
    strlcpy(pass_copy, password, sizeof(pass_copy));
    cJSON_Delete(json);

    /* Reply first: in AP+STA mode the connection attempt may move the
     * radio to another channel and briefly drop the phone. */
    esp_err_t err = send_ok(req);
    wifi_mgr_set_credentials(ssid_copy, pass_copy);
    return err;
}

/* ===================================================== */
/* System                                                */
/* ===================================================== */

static esp_err_t calibrate_post(httpd_req_t *req)
{
    app_request_co2_calibration();
    return send_ok(req);
}

static esp_err_t reboot_post(httpd_req_t *req)
{
    esp_err_t err = send_ok(req);
    app_restart_later(OVERLAY_RESTART, 2000);
    return err;
}

static esp_err_t forget_wifi_post(httpd_req_t *req)
{
    settings_forget_network();
    esp_err_t err = send_ok(req);
    app_restart_later(OVERLAY_RESET, 2000);
    return err;
}

static esp_err_t factory_reset_post(httpd_req_t *req)
{
    settings_factory_reset();
    esp_err_t err = send_ok(req);
    app_restart_later(OVERLAY_RESET, 2000);
    return err;
}

/* Raw firmware.bin as the request body. */
static esp_err_t ota_post(httpd_req_t *req)
{
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (target == NULL) {
        return send_error(req, "500 Internal Server Error",
                          "no OTA partition");
    }
    if (req->content_len == 0 || req->content_len > target->size) {
        return send_error(req, "400 Bad Request", "bad image size");
    }

    char *buf = malloc(4096);
    if (buf == NULL) {
        return httpd_resp_send_500(req);
    }

    esp_ota_handle_t ota;
    esp_err_t err = esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &ota);
    if (err != ESP_OK) {
        free(buf);
        return send_error(req, "500 Internal Server Error",
                          esp_err_to_name(err));
    }
    ESP_LOGW(TAG, "OTA: %u bytes -> %s", (unsigned)req->content_len,
             target->label);

    size_t total = req->content_len;
    size_t done = 0;
    int last_pct = -1;
    app_set_overlay(OVERLAY_OTA, 0);

    while (done < total) {
        int r = httpd_req_recv(req, buf, MIN(4096, total - done));
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (r <= 0) {
            err = ESP_FAIL;
            break;
        }
        err = esp_ota_write(ota, buf, (size_t)r);
        if (err != ESP_OK) {
            break;
        }
        done += (size_t)r;
        int pct = (int)(done * 100 / total);
        if (pct / 10 != last_pct / 10) {
            last_pct = pct;
            app_set_overlay(OVERLAY_OTA, pct);
        }
    }
    free(buf);

    if (err == ESP_OK) {
        err = esp_ota_end(ota); /* verifies the image */
    } else {
        esp_ota_abort(ota);
    }
    if (err == ESP_OK) {
        err = esp_ota_set_boot_partition(target);
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(err));
        app_set_overlay(OVERLAY_NONE, -1);
        app_request_redraw();
        return send_error(req, "400 Bad Request",
                          err == ESP_ERR_OTA_VALIDATE_FAILED
                              ? "image is not valid firmware"
                              : esp_err_to_name(err));
    }

    ESP_LOGW(TAG, "OTA done, restarting");
    esp_err_t res = send_ok(req);
    app_restart_later(OVERLAY_RESTART, 1500);
    return res;
}

/* ===================================================== */

esp_err_t web_server_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 24;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.stack_size = 8192;
    cfg.lru_purge_enable = true;
    cfg.max_open_sockets = 5;
    cfg.recv_wait_timeout = 10;
    cfg.send_wait_timeout = 10;

    httpd_handle_t server;
    esp_err_t err = httpd_start(&server, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "start failed: %s", esp_err_to_name(err));
        return err;
    }

    const httpd_uri_t routes[] = {
        { "/", HTTP_GET, index_get, NULL },
        { "/api/state", HTTP_GET, state_get, NULL },
        { "/api/config", HTTP_GET, config_get, NULL },
        { "/api/config", HTTP_POST, config_post, NULL },
        { "/api/screen.bmp", HTTP_GET, screen_get, NULL },
        { "/api/preview", HTTP_POST, preview_post, NULL },
        { "/api/notify", HTTP_POST, notify_post, NULL },
        { "/api/notify", HTTP_DELETE, notify_delete, NULL },
        { "/api/wifi/scan", HTTP_GET, wifi_scan_get, NULL },
        { "/api/wifi", HTTP_POST, wifi_post, NULL },
        { "/api/co2/calibrate", HTTP_POST, calibrate_post, NULL },
        { "/api/system/reboot", HTTP_POST, reboot_post, NULL },
        { "/api/system/forget-wifi", HTTP_POST, forget_wifi_post, NULL },
        { "/api/system/factory-reset", HTTP_POST, factory_reset_post, NULL },
        { "/api/ota", HTTP_POST, ota_post, NULL },
        { "/api/*", HTTP_OPTIONS, options_any, NULL },
        { "/*", HTTP_GET, fallback_get, NULL },
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        httpd_register_uri_handler(server, &routes[i]);
    }
    ESP_LOGI(TAG, "HTTP server started");
    return ESP_OK;
}
