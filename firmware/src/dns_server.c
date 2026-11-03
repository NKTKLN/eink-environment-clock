#include "dns_server.h"
#include "dns_packet.h"

#include <stdatomic.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "lwip/sockets.h"

static const char *TAG = "dns";

#define DNS_PORT    53
#define DNS_MAX_LEN 512
#define AP_IP       0x0104A8C0 /* 192.168.4.1, network byte order */

static atomic_bool s_running;
static TaskHandle_t s_task;

static void dns_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket failed");
        goto done;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(DNS_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind failed");
        close(sock);
        goto done;
    }
    struct timeval tv = { .tv_sec = 1 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ESP_LOGI(TAG, "captive DNS started");

    static uint8_t buf[DNS_MAX_LEN];
    while (atomic_load(&s_running)) {
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int len = recvfrom(sock, buf, sizeof(buf), 0,
                           (struct sockaddr *)&from, &from_len);
        if (len <= 0) {
            continue;
        }
        int out = dns_build_reply(buf, len, sizeof(buf), AP_IP);
        if (out > 0) {
            sendto(sock, buf, out, 0, (struct sockaddr *)&from, from_len);
        }
    }
    close(sock);
    ESP_LOGI(TAG, "captive DNS stopped");

done:
    s_task = NULL;
    vTaskDelete(NULL);
}

void dns_server_start(void)
{
    if (atomic_exchange(&s_running, true)) {
        return;
    }
    /* A previous task either sees s_running again and keeps serving, or is
     * already past its loop check and exits within a few milliseconds. */
    for (int i = 0; i < 30 && s_task != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (s_task != NULL) {
        return;
    }
    xTaskCreate(dns_task, "dns", 3072, NULL, 4, &s_task);
}

void dns_server_stop(void)
{
    atomic_store(&s_running, false);
}
