#include "diag.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "health";

/* Our tasks plus the ESP-IDF ones we configure or load heavily. */
static const char *TASKS[] = {
    "main", "httpd", "mqtt_task", "updater", "dns", "esp_timer", "tiT", "sys_evt",
};

static int stack_free(const char *name)
{
    TaskHandle_t h = xTaskGetHandle(name);
    /* ESP-IDF reports the high-water mark in bytes */
    return h != NULL ? (int)uxTaskGetStackHighWaterMark(h) : -1;
}

void diag_log(void)
{
    char tasks[160];
    size_t o = 0;
    for (size_t i = 0; i < sizeof(TASKS) / sizeof(TASKS[0]); i++) {
        int f = stack_free(TASKS[i]);
        if (f >= 0 && o < sizeof(tasks)) {
            o += (size_t)snprintf(tasks + o, sizeof(tasks) - o, " %s=%d",
                                  TASKS[i], f);
        }
    }
    ESP_LOGI(TAG, "heap free=%u min=%u largest=%u | stack free:%s",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
             o ? tasks : " -");
}

void diag_to_json(cJSON *obj)
{
    cJSON *heap = cJSON_AddObjectToObject(obj, "heap");
    cJSON_AddNumberToObject(heap, "free", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    cJSON_AddNumberToObject(heap, "min_free", heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
    cJSON_AddNumberToObject(heap, "largest_block", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    cJSON *tasks = cJSON_AddObjectToObject(obj, "stack_free");
    for (size_t i = 0; i < sizeof(TASKS) / sizeof(TASKS[0]); i++) {
        int f = stack_free(TASKS[i]);
        if (f >= 0) {
            cJSON_AddNumberToObject(tasks, TASKS[i], f);
        }
    }
}
