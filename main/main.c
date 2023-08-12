#include <esp_sleep.h>
#include <nvs_flash.h>
#include <esp_https_ota.h>
#include <esp_wifi.h>
#include "common.h"
#include "solar.h"

#if defined(CONFIG_IDF_TARGET_ESP8266)
#include "connect_esp8266.h"
#else
#include "connect_esp32.h"
#endif

#include "homie.h"

bool run_all = false;
QueueHandle_t queue;

static esp_err_t increment_restart_counter(int32_t *out_value)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("app", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        goto fail;
    }
    *out_value = 0;
    err = nvs_get_i32(nvs_handle, "restart_counter", out_value);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        goto fail;
    }
    *out_value += 1;
    err = nvs_set_i32(nvs_handle, "restart_counter", *out_value);
    if (err != ESP_OK) {
        goto fail;
    }
    err = nvs_commit(nvs_handle);
fail:
    nvs_close(nvs_handle);
    return err;
}

static void ota_init(void)
{
    esp_http_client_config_t ota_config = {
        .url = CONFIG_OTA_URL,
    };

    if (esp_https_ota(&ota_config) == ESP_OK) {
        nvs_handle_t nvs;
        nvs_open("app", NVS_READWRITE, &nvs);
        nvs_set_i32(nvs, "restart_counter", 0);
        nvs_close(nvs);
    }
}

void connected_handler()
{
    if (run_all) {
        publish_node_attributes();
    }

    queue_item_t res;
    char subtopic[HOMIE_MAX_TOPIC_LEN];
    while (true) {
        if (xQueueReceive(queue, &res, 500 / portTICK_PERIOD_MS)) {
            snprintf(subtopic, sizeof(subtopic), "%s/%s", res.node->id, res.prop->id);
            homie_publishf(subtopic, QOS_1, RETAINED, res.prop->format, res.val);
        } else if (xEventGroupGetBits(services_event_group) & READ_COMPLETED_BIT) {
            break;
        }
    }
    homie_publish("$state", QOS_1, RETAINED, "sleeping");
    xEventGroupSetBits(services_event_group, PUBLISH_COMPLETED_BIT);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    services_event_group = xEventGroupCreate();

    int32_t restart_counter = 0;
    ESP_ERROR_CHECK(increment_restart_counter(&restart_counter));
    run_all = (restart_counter % 30 == 1);

    queue = xQueueCreate(16, sizeof(queue_item_t));
    xTaskCreate(task_solar_read, "task_solar_read", configMINIMAL_STACK_SIZE * 10, queue, 5, NULL);
    wifi_init();
    if (!(xEventGroupWaitBits(services_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, 20 * 1000 / portTICK_PERIOD_MS) & WIFI_CONNECTED_BIT)) {
        // wait at most 20 secs for wifi to connect
        goto sleep;
    }

    homie_config_t homie_conf = {
        .mqtt_config = {
            .event_handle = NULL,
            .client_id = "epsolar",
            .username = CONFIG_MQTT_USERNAME,
            .password = CONFIG_MQTT_PASSWORD,
            .uri = CONFIG_MQTT_URI,
            .keepalive = 15,
            .task_stack = configMINIMAL_STACK_SIZE * 10,
            .cert_pem = NULL,
        },
        .device_name = "EPSolar Sensor",
        .base_topic = "homie/epsolar",
        .node_list = "array,battery,device,load",
        .stats_interval = CONFIG_SLEEP_DURATION,
        .loop = false,
        .disable_publish_attributes = !run_all,
        .connected_handler = connected_handler,
        .msg_handler = NULL,
    };
    homie_init(&homie_conf);

    // wait at most 20 seconds for the publish to complete
    xEventGroupWaitBits(services_event_group, PUBLISH_COMPLETED_BIT, pdFALSE, pdTRUE, 20 * 1000 / portTICK_PERIOD_MS);
    homie_disconnect();

    if (run_all) {
        ota_init();
    }
sleep:
    esp_wifi_stop();
    esp_deep_sleep(CONFIG_SLEEP_DURATION * 1000 * 1000);

    // not reached unless deep sleep fails somehow
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_restart();
}