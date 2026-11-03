idf_component_register(
    SRCS
        "app_main.c"
        "app_state.c"
        "bmp.c"
        "boot_guard.c"
        "dns_packet.c"
        "dns_server.c"
        "epd_canvas.c"
        "epd_ssd1680.c"
        "ha_command.c"
        "ha_discovery.c"
        "mqtt_ha.c"
        "net_time.c"
        "notify.c"
        "sensors.c"
        "settings.c"
        "settings_store.c"
        "ui.c"
        "web_server.c"
        "wifi_mgr.c"
        "third_party/qrcodegen.c"
    INCLUDE_DIRS "." "third_party"
    PRIV_REQUIRES
        driver esp_wifi nvs_flash esp_event esp_netif esp_timer lwip
        esp_http_server app_update json mqtt esp_app_format
)
