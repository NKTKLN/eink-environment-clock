/*
 * Wi-Fi connection manager.
 *
 *  - no saved network: the setup access point starts right away;
 *  - saved network unreachable for WIFI_FALLBACK_AP_S: the access point
 *    starts as well (AP+STA), while the station keeps retrying;
 *  - once the station gets an IP the access point stops after
 *    WIFI_AP_LINGER_S, so the setup page can still show the new address.
 *
 * The AP is WPA2 with a fresh random 8-digit password every time it
 * starts; the display shows it together with a Wi-Fi QR code.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_wifi_types.h"

#define WIFI_AP_IP "192.168.4.1"

typedef struct {
    bool sta_configured;
    bool sta_connected;
    char sta_ssid[33];
    char ip[16];
    int rssi;
    bool ap_active;
    bool ap_after_failure; /* AP opened because the saved network failed */
    char ap_ssid[33];
    char ap_password[9];
    int ap_clients;
    char hostname[32];
} wifi_status_t;

esp_err_t wifi_mgr_start(void);

void wifi_mgr_get_status(wifi_status_t *out);

/* Bumped on every visible status change (used to trigger redraws). */
uint32_t wifi_mgr_generation(void);

/* Saves the credentials and connects. The AP (if up) stays up until the
 * connection succeeds. */
esp_err_t wifi_mgr_set_credentials(const char *ssid, const char *password);

/* Blocking scan (~2-3 s). Returns the number of records written. */
int wifi_mgr_scan(wifi_ap_record_t *out, int max);
