#include "dns_server.h"

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

/* Skips the QNAME labels; returns the offset after it or -1. */
static int skip_name(const uint8_t *buf, int len, int off)
{
    while (off < len) {
        uint8_t l = buf[off];
        if (l == 0) {
            return off + 1;
        }
        if ((l & 0xC0) == 0xC0) {
            return off + 2;
        }
        off += l + 1;
    }
    return -1;
}

static int build_reply(uint8_t *buf, int len)
{
    if (len < 12) {
        return -1;
    }
    uint16_t qdcount = (uint16_t)(buf[4] << 8 | buf[5]);
    if ((buf[2] & 0x80) || qdcount != 1) {
        return -1; /* not a query, or more than one question */
    }

    int qend = skip_name(buf, len, 12);
    if (qend < 0 || qend + 4 > len) {
        return -1;
    }
    uint16_t qtype = (uint16_t)(buf[qend] << 8 | buf[qend + 1]);
    int out = qend + 4; /* drop anything after the question */

    buf[2] = 0x84 | (buf[2] & 0x01); /* response, authoritative, keep RD */
    buf[3] = 0x80;                    /* RA, no error */
    buf[6] = buf[7] = 0;              /* ANCOUNT */
    buf[8] = buf[9] = 0;              /* NSCOUNT */
    buf[10] = buf[11] = 0;            /* ARCOUNT */

    if (qtype == 1 /* A */ && out + 16 <= DNS_MAX_LEN) {
        const uint8_t answer[] = {
            0xC0, 0x0C,             /* name: pointer to the question */
            0x00, 0x01, 0x00, 0x01, /* type A, class IN */
            0x00, 0x00, 0x00, 0x3C, /* TTL 60 s */
            0x00, 0x04,             /* RDLENGTH */
        };
        memcpy(buf + out, answer, sizeof(answer));
        uint32_t ip = AP_IP;
        memcpy(buf + out + sizeof(answer), &ip, 4);
        out += sizeof(answer) + 4;
        buf[7] = 1;
    }
    return out;
}

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
        int out = build_reply(buf, len);
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
