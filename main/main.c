#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_zigbee.h"
#include "ezbee/platform/radio.h"
#include "ezbee/zha.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "solar.h"

#define EPSOLAR_STORAGE_PARTITION "zb_storage"

#define EPSOLAR_ARRAY_ENDPOINT 1
#define EPSOLAR_BATTERY_ENDPOINT 2
#define EPSOLAR_CONTROLLER_ENDPOINT 3
#define EPSOLAR_LOAD_ENDPOINT 4
#define EPSOLAR_BATTERY_STATUS_ENDPOINT 5
#define EPSOLAR_BATTERY_CURRENT_ENDPOINT 6
#define EPSOLAR_CHARGING_STATUS_ENDPOINT 7
#define EPSOLAR_DISCHARGING_STATUS_ENDPOINT 8
#define EPSOLAR_ENDPOINT_COUNT 8

#define EPSOLAR_MANUFACTURER_NAME "\x09" "DIY Solar"
#define EPSOLAR_MODEL_IDENTIFIER "\x0e" "EPSolar Zigbee"

static const char *TAG = "epsolar_zigbee";
static bool telemetry_task_started;

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

static ezb_af_ep_desc_t create_electrical_endpoint(uint8_t endpoint_id, bool include_basic)
{
    ezb_af_ep_config_t endpoint_config = {
        .ep_id = endpoint_id,
        .app_profile_id = EZB_AF_HA_PROFILE_ID,
        .app_device_id = EZB_ZHA_CONSUMPTION_AWARENESS_DEVICE_ID,
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

    ezb_zcl_electrical_measurement_cluster_server_config_t electrical_config = {
        .measurement_type =
            EZB_ZCL_ELECTRICAL_MEASUREMENT_MEASUREMENT_TYPE_ACTIVE_MEASUREMENT_AC |
            EZB_ZCL_ELECTRICAL_MEASUREMENT_MEASUREMENT_TYPE_PHASE_A_MEASUREMENT,
    };
    ezb_zcl_cluster_desc_t electrical =
        ezb_zcl_electrical_measurement_create_cluster_desc(&electrical_config, EZB_ZCL_CLUSTER_SERVER);

    uint16_t voltage = 0;
    uint16_t current = 0;
    int16_t power = 0;
    uint16_t multiplier = 1;
    uint16_t voltage_divisor = 100;
    uint16_t current_divisor = 100;
    uint16_t power_divisor = 1;
    ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
        electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE_ID, &voltage
    ));
    ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
        electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_CURRENT_ID, &current
    ));
    ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
        electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_ID, &power
    ));
    ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
        electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_VOLTAGE_MULTIPLIER_ID, &multiplier
    ));
    ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
        electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_VOLTAGE_DIVISOR_ID, &voltage_divisor
    ));
    ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
        electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_CURRENT_MULTIPLIER_ID, &multiplier
    ));
    ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
        electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_CURRENT_DIVISOR_ID, &current_divisor
    ));
    ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
        electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_POWER_MULTIPLIER_ID, &multiplier
    ));
    ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
        electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_AC_POWER_DIVISOR_ID, &power_divisor
    ));
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(endpoint, electrical));
    return endpoint;
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
        .app_profile_id = EZB_AF_HA_PROFILE_ID,
        .app_device_id = EZB_ZHA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 1,
    };
    ezb_af_ep_desc_t endpoint = ezb_af_create_endpoint_desc(&endpoint_config);
    ESP_ERROR_CHECK(endpoint == EZB_INVALID_AF_EP_DESC ? ESP_ERR_NO_MEM : ESP_OK);

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
    return endpoint;
}

static void register_device(void)
{
    static const char battery_status_description[] = "\x0e" "Battery status";
    static const char battery_current_description[] = "\x0f" "Battery current";
    static const char charging_status_description[] = "\x0f" "Charging status";
    static const char discharging_status_description[] = "\x12" "Discharging status";

    ESP_ERROR_CHECK(ezb_af_dev_set_max_endpoint_num(EPSOLAR_ENDPOINT_COUNT));
    ezb_af_device_desc_t device = ezb_af_create_device_desc();
    ESP_ERROR_CHECK(device == EZB_INVALID_AF_DEVICE_DESC ? ESP_ERR_NO_MEM : ESP_OK);

    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(
        device, create_electrical_endpoint(EPSOLAR_ARRAY_ENDPOINT, true)
    ));

    ezb_zha_temperature_sensor_config_t battery_config = EZB_ZHA_TEMPERATURE_SENSOR_CONFIG();
    battery_config.temp_meas_cfg.min_measured_value = -4000;
    battery_config.temp_meas_cfg.max_measured_value = 10000;
    ezb_af_ep_desc_t battery =
        ezb_zha_create_temperature_sensor(EPSOLAR_BATTERY_ENDPOINT, &battery_config);
    add_basic_identity(battery);

    ezb_zcl_power_config_cluster_server_config_t power_config = {0};
    ezb_zcl_cluster_desc_t power =
        ezb_zcl_power_config_create_cluster_desc(&power_config, EZB_ZCL_CLUSTER_SERVER);
    uint8_t battery_voltage = 0xff;
    uint8_t battery_percentage = 0xff;
    uint8_t battery_size = EZB_ZCL_POWER_CONFIG_BATTERY_SIZE_OTHER;
    ESP_ERROR_CHECK(ezb_zcl_power_config_cluster_desc_add_attr(
        power, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &battery_voltage
    ));
    ESP_ERROR_CHECK(ezb_zcl_power_config_cluster_desc_add_attr(
        power, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &battery_percentage
    ));
    ESP_ERROR_CHECK(ezb_zcl_power_config_cluster_desc_add_attr(
        power, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_SIZE_ID, &battery_size
    ));
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(battery, power));
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(device, battery));

    ezb_zha_temperature_sensor_config_t controller_config = EZB_ZHA_TEMPERATURE_SENSOR_CONFIG();
    controller_config.temp_meas_cfg.min_measured_value = -4000;
    controller_config.temp_meas_cfg.max_measured_value = 12500;
    ezb_af_ep_desc_t controller =
        ezb_zha_create_temperature_sensor(EPSOLAR_CONTROLLER_ENDPOINT, &controller_config);
    add_basic_identity(controller);
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(device, controller));

    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(
        device, create_electrical_endpoint(EPSOLAR_LOAD_ENDPOINT, false)
    ));
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(
        device,
        create_analog_endpoint(
            EPSOLAR_BATTERY_STATUS_ENDPOINT,
            battery_status_description,
            EZB_ZCL_ANALOG_INPUT_APPLICATION_TYPE_COUNT_UNITLESS,
            0
        )
    ));
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(
        device,
        create_analog_endpoint(
            EPSOLAR_BATTERY_CURRENT_ENDPOINT,
            battery_current_description,
            EZB_ZCL_ANALOG_INPUT_APPLICATION_TYPE_CURRENT_IN_AMPS,
            1
        )
    ));
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(
        device,
        create_analog_endpoint(
            EPSOLAR_CHARGING_STATUS_ENDPOINT,
            charging_status_description,
            EZB_ZCL_ANALOG_INPUT_APPLICATION_TYPE_COUNT_UNITLESS,
            2
        )
    ));
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(
        device,
        create_analog_endpoint(
            EPSOLAR_DISCHARGING_STATUS_ENDPOINT,
            discharging_status_description,
            EZB_ZCL_ANALOG_INPUT_APPLICATION_TYPE_COUNT_UNITLESS,
            3
        )
    ));

    ESP_ERROR_CHECK(ezb_af_device_desc_register(device));
}

static void set_attribute(uint8_t endpoint, uint16_t cluster, uint16_t attribute, void *value)
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
    if (status != EZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(
            TAG,
            "Attribute update failed: endpoint=%u cluster=0x%04x attribute=0x%04x status=0x%02x",
            endpoint,
            cluster,
            attribute,
            status
        );
    }
}

static int16_t power_watts(uint32_t power_cW)
{
    uint32_t rounded = (power_cW + 50) / 100;
    return rounded > INT16_MAX ? INT16_MAX : (int16_t)rounded;
}

static void update_electrical_endpoint(
    uint8_t endpoint,
    uint16_t voltage_cV,
    uint16_t current_cA,
    uint32_t power_cW
)
{
    int16_t power = power_watts(power_cW);
    set_attribute(
        endpoint,
        EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
        EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_VOLTAGE_ID,
        &voltage_cV
    );
    set_attribute(
        endpoint,
        EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
        EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMS_CURRENT_ID,
        &current_cA
    );
    set_attribute(
        endpoint,
        EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
        EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACTIVE_POWER_ID,
        &power
    );
}

static void update_telemetry(const epsolar_telemetry_t *telemetry)
{
    esp_zigbee_lock_acquire(portMAX_DELAY);

    if (telemetry->valid & EPSOLAR_VALID_ARRAY) {
        update_electrical_endpoint(
            EPSOLAR_ARRAY_ENDPOINT,
            telemetry->array_voltage_cV,
            telemetry->array_current_cA,
            telemetry->array_power_cW
        );
    }
    if (telemetry->valid & EPSOLAR_VALID_LOAD) {
        update_electrical_endpoint(
            EPSOLAR_LOAD_ENDPOINT,
            telemetry->load_voltage_cV,
            telemetry->load_current_cA,
            telemetry->load_power_cW
        );
    }
    if (telemetry->valid & EPSOLAR_VALID_TEMPERATURES) {
        int16_t battery_temperature = telemetry->battery_temperature_cC;
        int16_t controller_temperature = telemetry->controller_temperature_cC;
        set_attribute(
            EPSOLAR_BATTERY_ENDPOINT,
            EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
            EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID,
            &battery_temperature
        );
        set_attribute(
            EPSOLAR_CONTROLLER_ENDPOINT,
            EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
            EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID,
            &controller_temperature
        );
    }
    if (telemetry->valid & EPSOLAR_VALID_BATTERY_LEVEL) {
        uint16_t level = telemetry->battery_level_percent > 100 ? 100 : telemetry->battery_level_percent;
        uint8_t battery_percentage = (uint8_t)(level * 2);
        set_attribute(
            EPSOLAR_BATTERY_ENDPOINT,
            EZB_ZCL_CLUSTER_ID_POWER_CONFIG,
            EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
            &battery_percentage
        );
    }
    if (telemetry->valid & EPSOLAR_VALID_BATTERY_ELECTRICAL) {
        uint16_t voltage_dV = (telemetry->battery_voltage_cV + 5) / 10;
        uint8_t battery_voltage = voltage_dV > 254 ? 254 : (uint8_t)voltage_dV;
        float battery_current = telemetry->battery_current_cA / 100.0f;
        set_attribute(
            EPSOLAR_BATTERY_ENDPOINT,
            EZB_ZCL_CLUSTER_ID_POWER_CONFIG,
            EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
            &battery_voltage
        );
        set_attribute(
            EPSOLAR_BATTERY_CURRENT_ENDPOINT,
            EZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
            EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
            &battery_current
        );
    }
    if (telemetry->valid & EPSOLAR_VALID_STATUS) {
        float battery_status = telemetry->battery_status;
        float charging_status = telemetry->charging_status;
        float discharging_status = telemetry->discharging_status;
        set_attribute(
            EPSOLAR_BATTERY_STATUS_ENDPOINT,
            EZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
            EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
            &battery_status
        );
        set_attribute(
            EPSOLAR_CHARGING_STATUS_ENDPOINT,
            EZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
            EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
            &charging_status
        );
        set_attribute(
            EPSOLAR_DISCHARGING_STATUS_ENDPOINT,
            EZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
            EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
            &discharging_status
        );
    }

    esp_zigbee_lock_release();
}

static void telemetry_task(void *arg)
{
    epsolar_modbus_t modbus = {0};
    esp_err_t err = epsolar_modbus_init(&modbus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Modbus initialization failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        epsolar_telemetry_t telemetry;
        err = epsolar_read_telemetry(&modbus, &telemetry);
        if (err == ESP_OK) {
            update_telemetry(&telemetry);
            ESP_LOGI(TAG, "Updated Zigbee telemetry (valid mask 0x%02" PRIx32 ")", telemetry.valid);
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
    if (created == pdPASS) {
        telemetry_task_started = true;
    } else {
        ESP_LOGE(TAG, "Unable to create telemetry task");
    }
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
    initialize_nvs();
    ESP_LOGI(TAG, "Starting ESP32-C6 EPSolar Zigbee sensor");
    ESP_ERROR_CHECK(
        xTaskCreate(zigbee_task, "zigbee_main", 6144, NULL, 5, NULL) == pdPASS
            ? ESP_OK
            : ESP_ERR_NO_MEM
    );
}