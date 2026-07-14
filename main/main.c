#include <inttypes.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif
#include "esp_zigbee.h"
#include "ezbee/platform/radio.h"
#include "ezbee/zha.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "solar.h"

#define EPSOLAR_STORAGE_PARTITION "zb_storage"
#define EPSOLAR_MODBUS_INIT_RETRY_MS 5000
#define RF_SWITCH_POWER_GPIO GPIO_NUM_3
#define RF_SWITCH_SELECT_GPIO GPIO_NUM_14

#define EPSOLAR_PRIVATE_PROFILE_ID 0xc000
#define EPSOLAR_DC_DEVICE_ID 0x0001
#define EPSOLAR_STATUS_DEVICE_ID 0x0002
#define EPSOLAR_ARRAY_ENDPOINT 1
#define EPSOLAR_BATTERY_ENDPOINT 2
#define EPSOLAR_CONTROLLER_ENDPOINT 3
#define EPSOLAR_LOAD_ENDPOINT 4
#define EPSOLAR_BATTERY_STATUS_ENDPOINT 5
#define EPSOLAR_CHARGING_STATUS_ENDPOINT 6
#define EPSOLAR_DISCHARGING_STATUS_ENDPOINT 7
#define EPSOLAR_ENDPOINT_COUNT 7

#define EPSOLAR_MANUFACTURER_NAME "\x09" "DIY Solar"
#define EPSOLAR_MODEL_IDENTIFIER "\x0e" "EPSolar Zigbee"

static const char *TAG = "epsolar_zigbee";
static bool telemetry_task_started;

#ifdef CONFIG_PM_ENABLE
static void configure_power_management(void)
{
    esp_pm_config_t config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .light_sleep_enable = true,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&config));
}
#endif

static void select_external_antenna(void)
{
    ESP_ERROR_CHECK(gpio_hold_dis(RF_SWITCH_POWER_GPIO));
    ESP_ERROR_CHECK(gpio_hold_dis(RF_SWITCH_SELECT_GPIO));

    gpio_config_t config = {
        .pin_bit_mask =
            (1ULL << RF_SWITCH_POWER_GPIO) |
            (1ULL << RF_SWITCH_SELECT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));

    ESP_ERROR_CHECK(gpio_set_level(RF_SWITCH_POWER_GPIO, 0));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(gpio_set_level(RF_SWITCH_SELECT_GPIO, 1));
    ESP_ERROR_CHECK(gpio_hold_en(RF_SWITCH_POWER_GPIO));
    ESP_ERROR_CHECK(gpio_hold_en(RF_SWITCH_SELECT_GPIO));
}

static int16_t clamp_zcl_measurement(int64_t value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value <= INT16_MIN) {
        return INT16_MIN + 1;
    }
    return (int16_t)value;
}

static void add_basic_identity(ezb_af_ep_desc_t endpoint)
{
    ezb_zcl_cluster_desc_t basic = ezb_af_endpoint_get_cluster_desc(
        endpoint,
        EZB_ZCL_CLUSTER_ID_BASIC,
        EZB_ZCL_CLUSTER_SERVER
    );
    ESP_ERROR_CHECK(ezb_zcl_basic_cluster_desc_add_attr(
        basic,
        EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
        EPSOLAR_MANUFACTURER_NAME
    ));
    ESP_ERROR_CHECK(ezb_zcl_basic_cluster_desc_add_attr(
        basic,
        EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
        EPSOLAR_MODEL_IDENTIFIER
    ));
}

static void add_dc_electrical_cluster(ezb_af_ep_desc_t endpoint, bool include_power)
{
    ezb_zcl_electrical_measurement_cluster_server_config_t electrical_config = {
        .measurement_type = EZB_ZCL_ELECTRICAL_MEASUREMENT_MEASUREMENT_TYPE_DC_MEASUREMENT,
    };
    ezb_zcl_cluster_desc_t electrical =
        ezb_zcl_electrical_measurement_create_cluster_desc(&electrical_config, EZB_ZCL_CLUSTER_SERVER);

    int16_t voltage = 0;
    int16_t current = 0;
    int16_t power = 0;
    uint16_t multiplier = 1;
    uint16_t centi_divisor = 100;
    ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
        electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID, &voltage
    ));
    ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
        electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID, &current
    ));
    ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
        electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_MULTIPLIER_ID, &multiplier
    ));
    ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
        electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_DIVISOR_ID, &centi_divisor
    ));
    ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
        electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_MULTIPLIER_ID, &multiplier
    ));
    ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
        electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_DIVISOR_ID, &centi_divisor
    ));
    if (include_power) {
        ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
            electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_ID, &power
        ));
        ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
            electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_MULTIPLIER_ID, &multiplier
        ));
        ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
            electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_DIVISOR_ID, &centi_divisor
        ));
    }
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(endpoint, electrical));
}

static ezb_af_ep_desc_t create_dc_electrical_endpoint(uint8_t endpoint_id, bool include_basic)
{
    ezb_af_ep_config_t endpoint_config = {
        .ep_id = endpoint_id,
        .app_profile_id = EPSOLAR_PRIVATE_PROFILE_ID,
        .app_device_id = EPSOLAR_DC_DEVICE_ID,
        .app_device_version = 1,
    };
    ezb_af_ep_desc_t endpoint = ezb_af_create_endpoint_desc(&endpoint_config);
    ESP_ERROR_CHECK(endpoint == EZB_INVALID_AF_EP_DESC ? ESP_ERR_NO_MEM : ESP_OK);

    if (include_basic) {
        ezb_zcl_basic_cluster_server_config_t basic_config = {
            .zcl_version = EZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
            .power_source = EZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE,
        };
        ezb_zcl_cluster_desc_t basic =
            ezb_zcl_basic_create_cluster_desc(&basic_config, EZB_ZCL_CLUSTER_SERVER);
        ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(endpoint, basic));
        add_basic_identity(endpoint);
    }

    add_dc_electrical_cluster(endpoint, true);
    return endpoint;
}

static void add_analog_input_cluster(
    ezb_af_ep_desc_t endpoint,
    const char *description,
    uint8_t application_type,
    uint16_t application_index
)
{
    ezb_zcl_analog_input_cluster_server_config_t analog_config = {
        .out_of_service = false,
        .present_value = 0,
        .status_flags = 0,
    };
    ezb_zcl_cluster_desc_t analog =
        ezb_zcl_analog_input_create_cluster_desc(&analog_config, EZB_ZCL_CLUSTER_SERVER);
    uint32_t encoded_application_type = ((uint32_t)application_type << 16) | application_index;
    ESP_ERROR_CHECK(ezb_zcl_analog_input_cluster_desc_add_attr(
        analog, EZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID, description
    ));
    ESP_ERROR_CHECK(ezb_zcl_analog_input_cluster_desc_add_attr(
        analog, EZB_ZCL_ATTR_ANALOG_INPUT_APPLICATION_TYPE_ID, &encoded_application_type
    ));
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(endpoint, analog));
}

static ezb_af_ep_desc_t create_analog_endpoint(
    uint8_t endpoint_id,
    const char *description,
    uint8_t application_type,
    uint16_t application_index
)
{
    ezb_af_ep_config_t endpoint_config = {
        .ep_id = endpoint_id,
        .app_profile_id = EPSOLAR_PRIVATE_PROFILE_ID,
        .app_device_id = EPSOLAR_STATUS_DEVICE_ID,
        .app_device_version = 1,
    };
    ezb_af_ep_desc_t endpoint = ezb_af_create_endpoint_desc(&endpoint_config);
    ESP_ERROR_CHECK(endpoint == EZB_INVALID_AF_EP_DESC ? ESP_ERR_NO_MEM : ESP_OK);

    add_analog_input_cluster(endpoint, description, application_type, application_index);
    return endpoint;
}

static void register_device(void)
{
    static const char battery_level_description[] = "\x17" "Battery state of charge";
    static const struct {
        uint8_t endpoint;
        const char *description;
    } status_endpoints[] = {
        {EPSOLAR_BATTERY_STATUS_ENDPOINT, "\x17" "Battery status register"},
        {EPSOLAR_CHARGING_STATUS_ENDPOINT, "\x18" "Charging status register"},
        {EPSOLAR_DISCHARGING_STATUS_ENDPOINT, "\x1b" "Discharging status register"},
    };

    ESP_ERROR_CHECK(ezb_af_dev_set_max_endpoint_num(EPSOLAR_ENDPOINT_COUNT));
    ezb_af_device_desc_t device = ezb_af_create_device_desc();
    ESP_ERROR_CHECK(device == EZB_INVALID_AF_DEVICE_DESC ? ESP_ERR_NO_MEM : ESP_OK);

    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(
        device, create_dc_electrical_endpoint(EPSOLAR_ARRAY_ENDPOINT, true)
    ));

    ezb_zha_temperature_sensor_config_t battery_config = EZB_ZHA_TEMPERATURE_SENSOR_CONFIG();
    battery_config.temp_meas_cfg.min_measured_value = -4000;
    battery_config.temp_meas_cfg.max_measured_value = 10000;
    ezb_af_ep_desc_t battery =
        ezb_zha_create_temperature_sensor(EPSOLAR_BATTERY_ENDPOINT, &battery_config);
    add_basic_identity(battery);
    add_dc_electrical_cluster(battery, false);
    add_analog_input_cluster(
        battery,
        battery_level_description,
        EZB_ZCL_ANALOG_INPUT_APPLICATION_TYPE_PERCENTAGE,
        0
    );
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(device, battery));

    ezb_zha_temperature_sensor_config_t controller_config = EZB_ZHA_TEMPERATURE_SENSOR_CONFIG();
    controller_config.temp_meas_cfg.min_measured_value = -4000;
    controller_config.temp_meas_cfg.max_measured_value = 12500;
    ezb_af_ep_desc_t controller =
        ezb_zha_create_temperature_sensor(EPSOLAR_CONTROLLER_ENDPOINT, &controller_config);
    add_basic_identity(controller);
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(device, controller));

    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(
        device, create_dc_electrical_endpoint(EPSOLAR_LOAD_ENDPOINT, false)
    ));
    for (size_t i = 0; i < sizeof(status_endpoints) / sizeof(status_endpoints[0]); ++i) {
        ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(
            device,
            create_analog_endpoint(
                status_endpoints[i].endpoint,
                status_endpoints[i].description,
                EZB_ZCL_ANALOG_INPUT_APPLICATION_TYPE_COUNT_UNITLESS,
                i + 1
            )
        ));
    }

    ESP_ERROR_CHECK(ezb_af_device_desc_register(device));
}

static bool set_attribute(uint8_t endpoint, uint16_t cluster, uint16_t attribute, void *value)
{
    ezb_zcl_status_t status = ezb_zcl_set_attr_value(
        endpoint,
        cluster,
        EZB_ZCL_CLUSTER_SERVER,
        attribute,
        EZB_ZCL_STD_MANUF_CODE,
        value,
        false
    );
    if (status == EZB_ZCL_STATUS_SUCCESS) {
        return true;
    }

    ESP_LOGW(
        TAG,
        "Attribute update failed: endpoint=%u cluster=0x%04x attribute=0x%04x status=0x%02x",
        endpoint,
        cluster,
        attribute,
        status
    );
    return false;
}

static bool set_analog_value(uint8_t endpoint, float value)
{
    return set_attribute(
        endpoint,
        EZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
        EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
        &value
    );
}


static bool update_dc_electrical_endpoint(
    uint8_t endpoint,
    uint16_t voltage_cV,
    uint16_t current_cA,
    uint32_t power_cW
)
{
    int16_t voltage = clamp_zcl_measurement(voltage_cV);
    int16_t current = clamp_zcl_measurement(current_cA);
    int16_t power = clamp_zcl_measurement(power_cW);
    bool success = set_attribute(
        endpoint,
        EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
        EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID,
        &voltage
    );
    success &= set_attribute(
        endpoint,
        EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
        EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID,
        &current
    );
    success &= set_attribute(
        endpoint,
        EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
        EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_ID,
        &power
    );
    return success;
}

static bool update_telemetry(const epsolar_telemetry_t *telemetry)
{
    if (!esp_zigbee_lock_acquire(portMAX_DELAY)) {
        ESP_LOGW(TAG, "Unable to acquire Zigbee lock for telemetry update");
        return false;
    }

    bool success = true;
    if (telemetry->valid & EPSOLAR_VALID_ARRAY) {
        success &= update_dc_electrical_endpoint(
            EPSOLAR_ARRAY_ENDPOINT,
            telemetry->array_voltage_cV,
            telemetry->array_current_cA,
            telemetry->array_power_cW
        );
    }
    if (telemetry->valid & EPSOLAR_VALID_LOAD) {
        success &= update_dc_electrical_endpoint(
            EPSOLAR_LOAD_ENDPOINT,
            telemetry->load_voltage_cV,
            telemetry->load_current_cA,
            telemetry->load_power_cW
        );
    }
    if (telemetry->valid & EPSOLAR_VALID_TEMPERATURES) {
        int16_t battery_temperature = telemetry->battery_temperature_cC;
        int16_t controller_temperature = telemetry->controller_temperature_cC;
        success &= set_attribute(
            EPSOLAR_BATTERY_ENDPOINT,
            EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
            EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID,
            &battery_temperature
        );
        success &= set_attribute(
            EPSOLAR_CONTROLLER_ENDPOINT,
            EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
            EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID,
            &controller_temperature
        );
    }
    if (telemetry->valid & EPSOLAR_VALID_BATTERY_LEVEL) {
        success &= set_analog_value(
            EPSOLAR_BATTERY_ENDPOINT,
            telemetry->battery_level_percent
        );
    }
    if (telemetry->valid & EPSOLAR_VALID_BATTERY_ELECTRICAL) {
        int16_t battery_voltage = clamp_zcl_measurement(telemetry->battery_voltage_cV);
        int16_t battery_current = clamp_zcl_measurement(telemetry->battery_current_cA);
        success &= set_attribute(
            EPSOLAR_BATTERY_ENDPOINT,
            EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
            EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID,
            &battery_voltage
        );
        success &= set_attribute(
            EPSOLAR_BATTERY_ENDPOINT,
            EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
            EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID,
            &battery_current
        );
    }
    if (telemetry->valid & EPSOLAR_VALID_STATUS) {
        success &= set_analog_value(
            EPSOLAR_BATTERY_STATUS_ENDPOINT,
            telemetry->battery_status
        );
        success &= set_analog_value(
            EPSOLAR_CHARGING_STATUS_ENDPOINT,
            telemetry->charging_status
        );
        success &= set_analog_value(
            EPSOLAR_DISCHARGING_STATUS_ENDPOINT,
            telemetry->discharging_status
        );
    }

    esp_zigbee_lock_release();
    return success;
}

static void telemetry_task(void *arg)
{
    epsolar_modbus_t modbus = {0};
    esp_err_t err;
    while ((err = epsolar_modbus_init(&modbus)) != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Modbus initialization failed: %s; retrying in %u ms",
            esp_err_to_name(err),
            EPSOLAR_MODBUS_INIT_RETRY_MS
        );
        epsolar_modbus_deinit(&modbus);
        vTaskDelay(pdMS_TO_TICKS(EPSOLAR_MODBUS_INIT_RETRY_MS));
    }

    while (true) {
        epsolar_telemetry_t telemetry;
        err = epsolar_read_telemetry(&modbus, &telemetry);
        if (err == ESP_OK) {
            if (update_telemetry(&telemetry)) {
                ESP_LOGI(
                    TAG,
                    "Updated Zigbee telemetry (valid mask 0x%02" PRIx32 ")",
                    telemetry.valid
                );
            }
        } else {
            ESP_LOGW(TAG, "No EPSolar telemetry available");
        }
        vTaskDelay(pdMS_TO_TICKS(CONFIG_EPSOLAR_UPDATE_INTERVAL_SECONDS * 1000U));
    }
}

static void start_telemetry_task(void)
{
    if (telemetry_task_started) {
        return;
    }
    BaseType_t created = xTaskCreate(telemetry_task, "epsolar_read", 6144, NULL, 4, NULL);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    telemetry_task_started = true;
}

static void commissioning_retry_task(void *arg)
{
    uint8_t mode = (uint8_t)(uintptr_t)arg;
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_err_t err = ezb_bdb_start_top_level_commissioning(mode);
    esp_zigbee_lock_release();
    if (err != EZB_ERR_NONE) {
        ESP_LOGW(TAG, "Unable to retry Zigbee commissioning: 0x%04x", err);
    }
    vTaskDelete(NULL);
}

static void schedule_commissioning_retry(uint8_t mode)
{
    if (xTaskCreate(
            commissioning_retry_task,
            "zb_commission",
            3072,
            (void *)(uintptr_t)mode,
            3,
            NULL
        ) != pdPASS) {
        ESP_LOGE(TAG, "Unable to schedule Zigbee commissioning retry");
    }
}

static bool zigbee_signal_handler(const ezb_app_signal_t *signal)
{
    ezb_app_signal_type_t type = ezb_app_signal_get_type(signal);
    switch (type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initializing Zigbee stack");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;

    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        ezb_bdb_comm_status_t status = *(ezb_bdb_comm_status_t *)ezb_app_signal_get_params(signal);
        if (status != EZB_BDB_STATUS_SUCCESS) {
            ESP_LOGW(TAG, "Zigbee initialization failed: 0x%02x", status);
            schedule_commissioning_retry(EZB_BDB_MODE_INITIALIZATION);
            break;
        }
        start_telemetry_task();
        if (ezb_bdb_is_factory_new()) {
            ESP_LOGI(TAG, "Starting Zigbee network steering");
            ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGI(TAG, "Zigbee device resumed its saved network");
        }
        break;
    }

    case EZB_BDB_SIGNAL_STEERING: {
        ezb_bdb_comm_status_t status = *(ezb_bdb_comm_status_t *)ezb_app_signal_get_params(signal);
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ezb_extpanid_t pan_id;
            ezb_nwk_get_extended_panid(&pan_id);
            ESP_LOGI(
                TAG,
                "Joined Zigbee network: PAN=0x%04x EXT=0x%016" PRIx64 " channel=%u address=0x%04x",
                ezb_nwk_get_panid(),
                pan_id.u64,
                ezb_nwk_get_current_channel(),
                ezb_nwk_get_short_address()
            );
        } else {
            ESP_LOGW(TAG, "Zigbee network steering failed: 0x%02x", status);
            schedule_commissioning_retry(EZB_BDB_MODE_NETWORK_STEERING);
        }
        break;
    }

    default:
        ESP_LOGI(TAG, "Zigbee signal: %s (0x%02x)", ezb_app_signal_to_string(type), type);
        break;
    }
    return true;
}

static void zigbee_task(void *arg)
{
    esp_zigbee_config_t config = {
        .device_config = {
            .device_type = EZB_NWK_DEVICE_TYPE_END_DEVICE,
            .install_code_policy = false,
            .zed_config = {
                .ed_timeout = EZB_NWK_ED_TIMEOUT_64MIN,
                .keep_alive = 3000,
            },
        },
        .platform_config = {
            .storage_partition_name = EPSOLAR_STORAGE_PARTITION,
            .radio_config = {
                .radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE,
            },
        },
    };

    ESP_ERROR_CHECK(esp_zigbee_init(&config));
    ezb_nwk_set_rx_on_when_idle(false);
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(EZB_RADIO_2P4GHZ_ALL_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(zigbee_signal_handler));
    register_device();
    ESP_ERROR_CHECK(esp_zigbee_start(false));
    esp_zigbee_launch_mainloop();

    ESP_LOGE(TAG, "Zigbee main loop exited");
    esp_zigbee_deinit();
    vTaskDelete(NULL);
}

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = nvs_flash_init_partition(EPSOLAR_STORAGE_PARTITION);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase_partition(EPSOLAR_STORAGE_PARTITION));
        err = nvs_flash_init_partition(EPSOLAR_STORAGE_PARTITION);
    }
    ESP_ERROR_CHECK(err);
}

void app_main(void)
{
    select_external_antenna();
    initialize_nvs();
#ifdef CONFIG_PM_ENABLE
    configure_power_management();
#endif
    ESP_LOGI(TAG, "Starting ESP32-C6 EPSolar Zigbee sensor");
    ESP_ERROR_CHECK(
        xTaskCreate(zigbee_task, "zigbee_main", 6144, NULL, 5, NULL) == pdPASS
            ? ESP_OK
            : ESP_ERR_NO_MEM
    );
}