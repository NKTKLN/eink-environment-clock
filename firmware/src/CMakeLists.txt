idf_component_register(
    SRCS
        "app_main.c"
        "epd_ssd1680.c"
        "epd_canvas.c"
        "ui.c"
        "net_time.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_wifi nvs_flash esp_event esp_netif esp_timer lwip
)
