#include <inttypes.h>
#include <math.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_check.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
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
#define EPSOLAR_REPORT_MIN_INTERVAL_S 30
#define EPSOLAR_REPORT_MAX_INTERVAL_S 180
/* Nothing above the application notices when its frames stop reaching the
 * coordinator: attribute writes keep succeeding locally, so the confirmed link
 * probe is the only liveness signal. Repairs escalate one step per stall
 * window, and the window restarts on every repair, so no repair is judged on
 * silence it never had the chance to break: the restart at the end of the
 * ladder needs EPSOLAR_LINK_STALL_S * (EPSOLAR_LINK_REPAIR_ATTEMPTS + 1) of
 * uninterrupted silence. */
#define EPSOLAR_LINK_STALL_S 600
#define EPSOLAR_LINK_REPAIR_ATTEMPTS 3
#define EPSOLAR_REJOIN_BACKOFF_S 300
/* Upper bound on how long one telemetry cycle's Modbus read, report burst and
 * APS ack exchange may hold off light sleep. The probe's confirm - success,
 * or 0xa7 once the APS retries exhaust - normally closes the window well
 * inside this; the timeout only covers a confirm that never comes. */
#define EPSOLAR_REPORT_WINDOW_TIMEOUT_MS 15000
#define EPSOLAR_ZIGBEE_KEEP_ALIVE_MS 60000
#define EPSOLAR_MODBUS_INIT_RETRY_MS 5000
/* Hold out for a parent with some link margin, not one at the edge of hearing. */
#define EPSOLAR_ZIGBEE_MIN_JOIN_LQI 40
/* Cycles of silence from the telemetry task before the node restarts itself.
 * Nothing else notices that it stopped: a Modbus transaction that never
 * returns, a wake that never happens and a task that never runs all look
 * identical from the outside, and on battery there is no console to see it
 * on. esp_restart() keeps RTC RAM, so the counters below survive into the
 * next boot line. */
#define EPSOLAR_STALL_RESTART_CYCLES 3
#if defined(CONFIG_PM_ENABLE) && defined(CONFIG_EPSOLAR_LIGHT_SLEEP)
#define EPSOLAR_LIGHT_SLEEP_ENABLED true
#else
#define EPSOLAR_LIGHT_SLEEP_ENABLED false
#endif
#define RF_SWITCH_POWER_GPIO GPIO_NUM_3
#define RF_SWITCH_SELECT_GPIO GPIO_NUM_14

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

/* ZCL temperature MeasuredValue bounds, in hundredths of a degree Celsius */
#define EPSOLAR_TEMPERATURE_MIN_cC (-4000)
#define EPSOLAR_BATTERY_TEMPERATURE_MAX_cC 10000
#define EPSOLAR_CONTROLLER_TEMPERATURE_MAX_cC 12500

static const char *TAG = "epsolar_zigbee";
static bool telemetry_task_started;

/* Reading these back costs a power cycle: USB cannot be attached while the
 * node runs on its own supply, and pulling that supply is a power-on reset,
 * which is the one event RTC RAM does not survive. So NVS is the channel that
 * matters. RTC RAM adds nothing across a light sleep - all of SRAM survives
 * one - and carries the counters only across the software restarts the node
 * performs itself, where .bss would have been zeroed.
 *
 * Nothing is written to NVS at boot, and save_diagnostics() refuses to write
 * at all while a USB host is present, so the snapshot of the failed run
 * survives the boot that attaching the cable causes and every reset the
 * monitor does after it. Readout is idempotent: reset as often as you like. */
#define EPSOLAR_DIAG_MAGIC 0x45505335U /* "EPS5" */
#define EPSOLAR_DIAG_NAMESPACE "epsolar"
#define EPSOLAR_DIAG_KEY "diag"
#define EPSOLAR_DIAG_DENSE_CYCLES 30
#define EPSOLAR_DIAG_SAVE_CYCLES 10

/* Counters are bumped from both the telemetry task and the Zigbee stack task.
 * The increments are not atomic; a lost count in a diagnostic is cheaper than
 * a lock on the stack's confirm path. */
typedef struct {
    uint32_t magic;
    uint32_t boots;
    uint32_t cycles;
    uint32_t modbus_failures;
    uint32_t publish_failures;
    uint32_t report_confirms;
    uint32_t report_failures;
    uint32_t probe_streak;
    uint32_t announces;
    uint32_t rejoins;
    uint32_t stall_restarts;
    /* Reset reasons of the last four boots, most recent in the low byte. The
     * live one is only ever seen by a console that was not attached when it
     * mattered; this is how a brownout, a watchdog or a panic that happened
     * hours ago still shows up in the next readout. */
    uint32_t reset_reasons;
    uint32_t light_sleeps;
    uint64_t light_sleep_us;
    uint32_t longest_light_sleep_ms;
    uint32_t last_wakeup_causes;
    uint32_t last_publish_uptime_s;
    uint32_t last_confirm_uptime_s;
    uint32_t last_cycle_uptime_s;
    uint8_t last_probe_status;
    bool light_sleep_enabled;
} epsolar_diag_t;

static RTC_NOINIT_ATTR epsolar_diag_t s_diag;

static uint32_t uptime_seconds(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000);
}

static void save_diagnostics(void)
{
    /* A USB host means somebody is reading, not that the node is running: the
     * supply has to be pulled before the cable goes in, so every readout
     * starts a fresh boot, and idf.py monitor resets the board again on top of
     * that. Either of those runs would otherwise overwrite the snapshot of the
     * run being investigated before it had been read. Freeze it instead. */
    if (usb_serial_jtag_is_connected()) {
        static bool announced;
        if (!announced) {
            announced = true;
            ESP_LOGW(TAG, "USB attached; retained diagnostics frozen for readout");
        }
        return;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(EPSOLAR_DIAG_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Unable to open diagnostics storage: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_blob(nvs, EPSOLAR_DIAG_KEY, &s_diag, sizeof(s_diag));
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Unable to store diagnostics: %s", esp_err_to_name(err));
    }
    nvs_close(nvs);
}

static bool load_diagnostics(epsolar_diag_t *diag)
{
    nvs_handle_t nvs;
    if (nvs_open(EPSOLAR_DIAG_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return false;
    }
    size_t length = sizeof(*diag);
    bool loaded = nvs_get_blob(nvs, EPSOLAR_DIAG_KEY, diag, &length) == ESP_OK
        && length == sizeof(*diag)
        && diag->magic == EPSOLAR_DIAG_MAGIC;
    nvs_close(nvs);
    return loaded;
}

static void log_previous_run(const char *source, esp_reset_reason_t reason, const epsolar_diag_t *previous)
{
    ESP_LOGW(
        TAG,
        "Boot %" PRIu32 " (reset reason %d, history 0x%08" PRIx32
        "); previous run (%s): cycles=%" PRIu32
        " modbus_failures=%" PRIu32 " publish_failures=%" PRIu32
        " report_confirms=%" PRIu32 " report_failures=%" PRIu32
        " last_probe_status=0x%02x announces=%" PRIu32 " rejoins=%" PRIu32
        " stall_restarts=%" PRIu32 " last_cycle=%" PRIu32 "s last_publish=%" PRIu32
        "s last_confirm=%" PRIu32 "s light_sleep=%d light_sleeps=%" PRIu32
        " slept=%" PRIu32 "s longest_sleep=%" PRIu32 "ms wakeup_causes=0x%08" PRIx32,
        previous->boots + 1,
        reason,
        previous->reset_reasons,
        source,
        previous->cycles,
        previous->modbus_failures,
        previous->publish_failures,
        previous->report_confirms,
        previous->report_failures,
        previous->last_probe_status,
        previous->announces,
        previous->rejoins,
        previous->stall_restarts,
        previous->last_cycle_uptime_s,
        previous->last_publish_uptime_s,
        previous->last_confirm_uptime_s,
        previous->light_sleep_enabled,
        previous->light_sleeps,
        (uint32_t)(previous->light_sleep_us / 1000000U),
        previous->longest_light_sleep_ms,
        previous->last_wakeup_causes
    );
}

/* Report both sources, never whichever one happens to be valid.
 *
 * A USB reset - reset reason 11, which is what idf.py monitor performs when it
 * opens the port - keeps RTC RAM. So the readout sequence poisons its own
 * evidence: attaching the cable is a power-on reset that wipes RTC RAM and
 * reads NVS, but that boot line is emitted while the port is still enumerating
 * and nobody sees it; the monitor then resets over USB, RTC RAM survives from
 * the few seconds the board was up, and every boot from then on reports that
 * stub run in preference to the field run sitting in NVS. Reset as often as
 * you like was true of the stored snapshot and false of what got printed.
 *
 * The two sources answer different questions and neither substitutes for the
 * other: RTC RAM is the more recent, and is the only record of a run the node
 * ended itself with esp_restart(); NVS is frozen while USB is attached and is
 * therefore the only record of a run on external power. Print both. */
static void report_boot_diagnostics(void)
{
    esp_reset_reason_t reason = esp_reset_reason();
    epsolar_diag_t retained = s_diag;
    epsolar_diag_t stored;
    bool have_retained = retained.magic == EPSOLAR_DIAG_MAGIC;
    bool have_stored = load_diagnostics(&stored);

    if (have_retained) {
        log_previous_run("RTC RAM", reason, &retained);
    }
    if (have_stored) {
        log_previous_run("NVS, as of its last cycle", reason, &stored);
    }
    if (!have_retained && !have_stored) {
        ESP_LOGI(TAG, "Boot 1 (reset reason %d); no retained diagnostics", reason);
    }

    /* Counters continue from the fresher source. RTC RAM outranks NVS when it
     * survived, because NVS lags it by up to EPSOLAR_DIAG_SAVE_CYCLES. */
    epsolar_diag_t previous = (epsolar_diag_t){0};
    if (have_retained) {
        previous = retained;
    } else if (have_stored) {
        previous = stored;
    }

    s_diag = (epsolar_diag_t){
        .magic = EPSOLAR_DIAG_MAGIC,
        .boots = previous.boots + 1,
        .stall_restarts = previous.stall_restarts,
        .reset_reasons = (previous.reset_reasons << 8) | (uint8_t)reason,
    };

    /* A run that dies before its first telemetry cycle otherwise leaves nothing
     * at all: save_diagnostics() is only reached from the cycle loop and the
     * repair ladder, and the cycle loop only starts once the node has joined.
     * Stamping the boot here makes "booted, never joined" - cycles=0 with a
     * boots that moved - distinguishable from "never booted", and keeps the
     * reset-reason history unbroken across a run that never cycles. Refuses to
     * write while USB is attached, like every other save, so the snapshot being
     * read out survives this. */
    save_diagnostics();
}

/* Decide once at boot: a USB host present means the no-sleep reference run,
 * and USB Serial JTAG does not survive a light sleep. */
#ifdef CONFIG_PM_ENABLE
/* A sleepy end device receives nothing except in the short receive window
 * that follows one of its own MAC polls, and an APS-acknowledged frame is
 * only complete once the ack has been fetched that way: the coordinator's
 * ack rides back as an indirect transmission the parent holds for 7.68 s,
 * and the stack is supposed to short-poll until it lands. Under automatic
 * light sleep that phase is exactly what failed in the field - attribute
 * writes and transmissions succeeded, and every APSDE-DATA.confirm came back
 * 0xa7 "no APS ack", so the chip was sleeping through or mistiming its own
 * ack polls. Take that decision away from the stack: hold
 * ESP_PM_NO_LIGHT_SLEEP from the top of each telemetry cycle until the link
 * probe's confirm arrives (either way) or a failsafe timeout fires, so the
 * Modbus transaction, the report burst and the ack exchange all run on a
 * chip that stays awake, and the node sleeps only the quiet remainder of
 * the minute. Costs a few seconds of awake time per cycle; buys delivery. */
static esp_pm_lock_handle_t s_report_window_lock;
static esp_timer_handle_t s_report_window_timer;
static portMUX_TYPE s_report_window_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_report_window_held;

static void close_report_window(void)
{
    if (s_report_window_lock == NULL) {
        return;
    }
    esp_timer_stop(s_report_window_timer);
    bool release = false;
    portENTER_CRITICAL(&s_report_window_mux);
    if (s_report_window_held) {
        s_report_window_held = false;
        release = true;
    }
    portEXIT_CRITICAL(&s_report_window_mux);
    if (release) {
        ESP_ERROR_CHECK(esp_pm_lock_release(s_report_window_lock));
    }
}

static void report_window_timeout(void *arg)
{
    (void)arg;
    close_report_window();
}

static void open_report_window(void)
{
    if (s_report_window_lock == NULL) {
        return;
    }
    bool acquire = false;
    portENTER_CRITICAL(&s_report_window_mux);
    if (!s_report_window_held) {
        s_report_window_held = true;
        acquire = true;
    }
    portEXIT_CRITICAL(&s_report_window_mux);
    if (acquire) {
        ESP_ERROR_CHECK(esp_pm_lock_acquire(s_report_window_lock));
    }
    esp_timer_stop(s_report_window_timer);
    ESP_ERROR_CHECK(esp_timer_start_once(
        s_report_window_timer,
        (uint64_t)EPSOLAR_REPORT_WINDOW_TIMEOUT_MS * 1000ULL
    ));
}

static void configure_power_management(void)
{
    bool light_sleep = EPSOLAR_LIGHT_SLEEP_ENABLED;
    if (light_sleep && usb_serial_jtag_is_connected()) {
        light_sleep = false;
        ESP_LOGW(TAG, "USB host attached at boot; automatic light sleep disabled for this run");
    }
    esp_pm_config_t config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .light_sleep_enable = light_sleep,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&config));
    s_diag.light_sleep_enabled = light_sleep;
    if (light_sleep) {
        ESP_ERROR_CHECK(esp_pm_lock_create(
            ESP_PM_NO_LIGHT_SLEEP, 0, "epsolar_report", &s_report_window_lock
        ));
        const esp_timer_create_args_t window_timer = {
            .callback = report_window_timeout,
            .name = "epsolar_report_window",
        };
        ESP_ERROR_CHECK(esp_timer_create(&window_timer, &s_report_window_timer));
    }
}
#endif

#if CONFIG_PM_LIGHT_SLEEP_CALLBACKS
/* Runs in the idle task inside the tickless-idle critical section, once per
 * automatic light sleep. Touch the RTC RAM counters and nothing else.
 *
 * These counters answer the only question the outside world cannot: whether
 * the node is still waking up at all. Cycles that keep advancing while reports
 * stop means the radio path died; cycles that stop with light_sleeps frozen
 * means the chip never came back out of sleep.
 *
 * slept_us is zero when the framework decided the idle window was too short
 * and skipped the sleep, which is not a wake. */
static esp_err_t light_sleep_exited(int64_t slept_us, void *arg)
{
    (void)arg;
    if (slept_us <= 0) {
        return ESP_OK;
    }
    s_diag.light_sleeps++;
    s_diag.light_sleep_us += (uint64_t)slept_us;
    uint32_t slept_ms = (uint32_t)(slept_us / 1000);
    if (slept_ms > s_diag.longest_light_sleep_ms) {
        s_diag.longest_light_sleep_ms = slept_ms;
    }
    s_diag.last_wakeup_causes = esp_sleep_get_wakeup_causes();
    return ESP_OK;
}

static void register_light_sleep_counters(void)
{
    esp_pm_sleep_cbs_register_config_t callbacks = {.exit_cb = light_sleep_exited};
    ESP_ERROR_CHECK(esp_pm_light_sleep_register_cbs(&callbacks));
}
#endif

#if !CONFIG_PM_LIGHT_SLEEP_CALLBACKS
static void register_light_sleep_counters(void) {}
#endif

#ifndef CONFIG_PM_ENABLE
static void configure_power_management(void) {}
static void open_report_window(void) {}
static void close_report_window(void) {}
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

static uint8_t clamp_battery_percentage(uint16_t percentage)
{
    return percentage > 100U ? 100U : (uint8_t)percentage;
}

static uint8_t battery_power_source_level(uint8_t percentage)
{
    if (percentage == 0U) {
        return EZB_AF_NODE_POWER_SOURCE_LEVEL_CRITICAL;
    }
    if (percentage <= 33U) {
        return EZB_AF_NODE_POWER_SOURCE_LEVEL_33_PERCENT;
    }
    if (percentage <= 66U) {
        return EZB_AF_NODE_POWER_SOURCE_LEVEL_66_PERCENT;
    }
    return EZB_AF_NODE_POWER_SOURCE_LEVEL_100_PERCENT;
}

static bool set_battery_power_descriptor(uint8_t source_level)
{
    ezb_af_node_power_desc_t descriptor = {
        .current_power_mode = EZB_AF_NODE_POWER_MODE_SYNC_ON_WHEN_IDLE,
        .available_power_sources = EZB_AF_NODE_POWER_SOURCE_RECHARGEABLE_BATTERY,
        .current_power_source = EZB_AF_NODE_POWER_SOURCE_RECHARGEABLE_BATTERY,
        .current_power_source_level = source_level,
    };
    ezb_err_t err = ezb_af_set_node_power_desc(&descriptor);
    if (err == EZB_ERR_NONE) {
        return true;
    }

    ESP_LOGW(TAG, "Unable to set Zigbee power descriptor: 0x%04x", err);
    return false;
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

/* Only a handful of standard attributes ship with EZB_ZCL_ATTR_ACCESS_REPORTING
 * (temperature MeasuredValue is one; the DC electrical, analog input and
 * battery attributes are not). The stack allocates a reporting slot per
 * reportable attribute while the data model is registered, so the flag has to
 * be set here, on the cluster descriptor, before ezb_af_device_desc_register().
 * Without a slot, Configure Reporting from the coordinator and
 * ezb_zcl_report_attr_cmd_req() from us both fail with EZB_ERR_FAIL. */
static void mark_reportable(ezb_zcl_cluster_desc_t cluster, uint16_t attribute)
{
    ezb_zcl_attr_desc_t attr =
        ezb_zcl_cluster_get_attr_desc(cluster, attribute, EZB_ZCL_STD_MANUF_CODE);
    ESP_ERROR_CHECK(attr == EZB_INVALID_ZCL_ATTR_DESC ? ESP_ERR_NOT_FOUND : ESP_OK);
    if (ezb_zcl_attr_is_reportable(attr)) {
        return;
    }
    ezb_err_t err = ezb_zcl_attr_desc_set_access(
        attr,
        ezb_zcl_attr_desc_get_access(attr) | EZB_ZCL_ATTR_ACCESS_REPORTING
    );
    ESP_ERROR_CHECK(err == EZB_ERR_NONE ? ESP_OK : ESP_FAIL);
}

static void add_dc_electrical_cluster(ezb_af_ep_desc_t endpoint, bool include_power)
{
    ezb_zcl_electrical_measurement_cluster_server_config_t electrical_config = {
        .measurement_type = EZB_ZCL_ELECTRICAL_MEASUREMENT_MEASUREMENT_TYPE_DC_MEASUREMENT,
    };
    ezb_zcl_cluster_desc_t electrical =
        ezb_zcl_electrical_measurement_create_cluster_desc(&electrical_config, EZB_ZCL_CLUSTER_SERVER);

    /* INT16_MIN is the ZCL "invalid" sentinel: readings stay unknown until the
     * first Modbus poll succeeds. Power uses divisor 10 (deciwatts) because
     * divisor 100 would clamp the int16 attribute at 327 W. */
    int16_t unmeasured = INT16_MIN;
    uint16_t multiplier = 1;
    uint16_t centi_divisor = 100;
    uint16_t deci_divisor = 10;
    ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
        electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID, &unmeasured
    ));
    ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
        electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID, &unmeasured
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
    mark_reportable(electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID);
    mark_reportable(electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID);
    if (include_power) {
        ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
            electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_ID, &unmeasured
        ));
        ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
            electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_MULTIPLIER_ID, &multiplier
        ));
        ESP_ERROR_CHECK(ezb_zcl_electrical_measurement_cluster_desc_add_attr(
            electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_DIVISOR_ID, &deci_divisor
        ));
        mark_reportable(electrical, EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_ID);
    }
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(endpoint, electrical));
}

static ezb_af_ep_desc_t create_dc_electrical_endpoint(uint8_t endpoint_id, bool include_basic)
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
            .power_source = EZB_ZCL_BASIC_POWER_SOURCE_BATTERY,
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
        .present_value = NAN,
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
    mark_reportable(analog, EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID);
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(endpoint, analog));
}

static void add_battery_power_cluster(ezb_af_ep_desc_t endpoint)
{
    uint8_t percentage_remaining = UINT8_MAX;
    ezb_zcl_cluster_desc_t power_config =
        ezb_zcl_power_config_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_SERVER);
    ESP_ERROR_CHECK(ezb_zcl_power_config_cluster_desc_add_attr(
        power_config,
        EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
        &percentage_remaining
    ));
    mark_reportable(power_config, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID);
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(endpoint, power_config));
}

static ezb_af_ep_desc_t create_temperature_endpoint(uint8_t endpoint_id, int16_t max_measured_value)
{
    ezb_zha_temperature_sensor_config_t config = EZB_ZHA_TEMPERATURE_SENSOR_CONFIG();
    config.basic_cfg.power_source = EZB_ZCL_BASIC_POWER_SOURCE_BATTERY;
    config.temp_meas_cfg.min_measured_value = EPSOLAR_TEMPERATURE_MIN_cC;
    config.temp_meas_cfg.max_measured_value = max_measured_value;
    ezb_af_ep_desc_t endpoint = ezb_zha_create_temperature_sensor(endpoint_id, &config);
    add_basic_identity(endpoint);
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

    ezb_af_ep_desc_t battery = create_temperature_endpoint(
        EPSOLAR_BATTERY_ENDPOINT, EPSOLAR_BATTERY_TEMPERATURE_MAX_cC
    );
    add_dc_electrical_cluster(battery, false);
    add_analog_input_cluster(
        battery,
        battery_level_description,
        EZB_ZCL_ANALOG_INPUT_APPLICATION_TYPE_PERCENTAGE,
        0
    );
    add_battery_power_cluster(battery);
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(device, battery));

    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(
        device,
        create_temperature_endpoint(EPSOLAR_CONTROLLER_ENDPOINT, EPSOLAR_CONTROLLER_TEMPERATURE_MAX_cC)
    ));

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

/* Attributes that zigbee2mqtt binds and expects reports for. The stack only
 * emits periodic reports when a reporting configuration survives in NVS, so
 * every cycle also pushes an explicit Report Attributes command; that is what
 * the espressif sensor examples do and it is independent of the coordinator's
 * reporting setup. */
static const struct {
    uint8_t endpoint;
    uint16_t cluster;
    uint16_t attribute;
} reported_attributes[] = {
    {EPSOLAR_ARRAY_ENDPOINT, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
     EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID},
    {EPSOLAR_ARRAY_ENDPOINT, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
     EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID},
    {EPSOLAR_ARRAY_ENDPOINT, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
     EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_ID},
    {EPSOLAR_LOAD_ENDPOINT, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
     EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID},
    {EPSOLAR_LOAD_ENDPOINT, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
     EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID},
    {EPSOLAR_LOAD_ENDPOINT, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
     EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_POWER_ID},
    {EPSOLAR_BATTERY_ENDPOINT, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
     EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID},
    {EPSOLAR_BATTERY_ENDPOINT, EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
     EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_CURRENT_ID},
    {EPSOLAR_BATTERY_ENDPOINT, EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
     EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID},
    {EPSOLAR_CONTROLLER_ENDPOINT, EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
     EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID},
    {EPSOLAR_BATTERY_ENDPOINT, EZB_ZCL_CLUSTER_ID_POWER_CONFIG,
     EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID},
    {EPSOLAR_BATTERY_ENDPOINT, EZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
     EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID},
    {EPSOLAR_BATTERY_STATUS_ENDPOINT, EZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
     EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID},
    {EPSOLAR_CHARGING_STATUS_ENDPOINT, EZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
     EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID},
    {EPSOLAR_DISCHARGING_STATUS_ENDPOINT, EZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
     EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID},
};

#define EPSOLAR_REPORTED_ATTRIBUTE_COUNT \
    (sizeof(reported_attributes) / sizeof(reported_attributes[0]))

/* Caller holds the Zigbee stack lock. Returns true once every attribute is
 * armed, so the caller can keep retrying: the coordinator's Configure
 * Reporting is what creates the reporting records, and on a fresh join it can
 * land after our first telemetry cycle.
 *
 * zigbee2mqtt asks for min 10 s / max 300 s, but that configuration only
 * produces on-change reports here: the max-interval heartbeat never fires, so
 * a static solar array goes silent for as long as nothing moves. Re-arm every
 * reportable attribute locally with a zero reportable change, which makes the
 * stack emit on every telemetry cycle it sees a write, and re-assert the
 * intervals independently of whatever the coordinator configured. */
static bool configure_local_reporting(void)
{
    uint32_t present = 0;
    uint32_t armed = 0;
    for (size_t i = 0; i < EPSOLAR_REPORTED_ATTRIBUTE_COUNT; ++i) {
        ezb_zcl_reporting_info_t info = ezb_zcl_reporting_info_find(
            reported_attributes[i].endpoint,
            reported_attributes[i].cluster,
            EZB_ZCL_CLUSTER_SERVER,
            reported_attributes[i].attribute,
            EZB_ZCL_STD_MANUF_CODE
        );
        if (info == EZB_ZCL_INVALID_REPORTING_INFO) {
            continue;
        }
        present |= 1U << i;

        ezb_zcl_attr_variable_t delta = {0};
        ezb_err_t err = ezb_zcl_reporting_info_update(
            info,
            EPSOLAR_REPORT_MIN_INTERVAL_S,
            EPSOLAR_REPORT_MAX_INTERVAL_S,
            &delta
        );
        if (err != EZB_ERR_NONE) {
            ESP_LOGW(
                TAG,
                "Unable to arm reporting for endpoint=%u cluster=0x%04x attribute=0x%04x: 0x%04x",
                reported_attributes[i].endpoint,
                reported_attributes[i].cluster,
                reported_attributes[i].attribute,
                err
            );
            continue;
        }
        ezb_zcl_reporting_start_attr_report(info);
        armed |= 1U << i;
    }

    static uint32_t logged_armed = UINT32_MAX;
    if (armed != logged_armed) {
        logged_armed = armed;
        ESP_LOGW(
            TAG,
            "Reporting records present for %d/%d attributes (mask 0x%05" PRIx32 "); "
            "armed %d at %us/%us with zero reportable change",
            __builtin_popcount(present),
            (int)EPSOLAR_REPORTED_ATTRIBUTE_COUNT,
            present,
            __builtin_popcount(armed),
            EPSOLAR_REPORT_MIN_INTERVAL_S,
            EPSOLAR_REPORT_MAX_INTERVAL_S
        );
    }
    return __builtin_popcount(armed) == (int)EPSOLAR_REPORTED_ATTRIBUTE_COUNT;
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

static bool publish_dc_measurements(
    uint8_t endpoint,
    uint16_t voltage_cV,
    uint16_t current_cA,
    uint32_t power_cW
)
{
    int16_t voltage = clamp_zcl_measurement(voltage_cV);
    int16_t current = clamp_zcl_measurement(current_cA);
    int16_t power = clamp_zcl_measurement(power_cW / 10U);
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

static bool publish_telemetry(const epsolar_telemetry_t *telemetry)
{
    esp_zigbee_lock_acquire(portMAX_DELAY);

    bool success = true;
    if (telemetry->valid & EPSOLAR_VALID_ARRAY) {
        success &= publish_dc_measurements(
            EPSOLAR_ARRAY_ENDPOINT,
            telemetry->array_voltage_cV,
            telemetry->array_current_cA,
            telemetry->array_power_cW
        );
    }
    if (telemetry->valid & EPSOLAR_VALID_LOAD) {
        success &= publish_dc_measurements(
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
        static uint8_t reported_source_level = UINT8_MAX;
        uint8_t battery_percentage =
            clamp_battery_percentage(telemetry->battery_level_percent);
        uint8_t percentage_remaining = battery_percentage * 2U;
        success &= set_attribute(
            EPSOLAR_BATTERY_ENDPOINT,
            EZB_ZCL_CLUSTER_ID_POWER_CONFIG,
            EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
            &percentage_remaining
        );
        success &= set_analog_value(EPSOLAR_BATTERY_ENDPOINT, battery_percentage);
        uint8_t source_level = battery_power_source_level(battery_percentage);
        if (source_level != reported_source_level) {
            if (set_battery_power_descriptor(source_level)) {
                reported_source_level = source_level;
            } else {
                success = false;
            }
        }
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

    static bool reporting_configured;
    if (!reporting_configured) {
        reporting_configured = configure_local_reporting();
    }

    esp_zigbee_lock_release();
    return success;
}

static void schedule_commissioning_retry(uint8_t mode);

/* APSDE-DATA.confirm statuses a probe can plausibly see (Zigbee specification
 * table 2.27). "no APS ack" is the interesting one: the frame left the node and
 * the end-to-end acknowledgement never came back, so the uplink can still be
 * perfectly healthy while nothing addressed to this node arrives. */
static const char *aps_status_name(uint8_t status)
{
    switch (status) {
    case 0x00: return "success";
    case 0xa0: return "ASDU too long";
    case 0xa3: return "illegal request";
    case 0xa4: return "invalid binding";
    case 0xa6: return "invalid parameter";
    case 0xa7: return "no APS ack";
    case 0xa8: return "no bound device";
    case 0xa9: return "no short address";
    case 0xaa: return "not supported";
    case 0xad: return "security failure";
    case 0xae: return "table full";
    case 0xaf: return "unsecured";
    default: return "unknown";
    }
}

/* Progress through the repair ladder since the last confirmed delivery. */
static struct {
    uint32_t last_repair_uptime_s;
    uint32_t last_rejoin_uptime_s;
    uint8_t repair_stage;
} s_link;

/* Runs in the Zigbee stack task with the stack lock already held.
 *
 * The application confirm hook only fires for frames the application submitted
 * itself; reports the stack's reporting engine emits are confirmed internally
 * and are invisible here. That is why the probe below exists. */
static void link_probe_confirm(ezb_af_user_cnf_t *cnf, void *user_ctx)
{
    (void)user_ctx;
    /* The confirm is the end of the cycle's ack exchange whichever way it
     * went: success, or 0xa7 after the stack's APS retries have exhausted.
     * Either way there is nothing left to stay awake for. */
    close_report_window();
    s_diag.last_probe_status = cnf->status;
    if (cnf->status == 0) {
        s_diag.report_confirms++;
        s_diag.last_confirm_uptime_s = uptime_seconds();
        s_diag.probe_streak = 0;
        s_link.repair_stage = 0;
        return;
    }
    s_diag.report_failures++;
    /* One line per outage: the cycle line carries the streak, and each repair
     * attempt logs itself. */
    if (++s_diag.probe_streak == 1) {
        ESP_LOGW(
            TAG,
            "Link probe not delivered: cluster=0x%04x status=0x%02x (%s)",
            cnf->cluster_id,
            cnf->status,
            aps_status_name(cnf->status)
        );
    }
}

/* One acknowledged Report Attributes per telemetry cycle, sent straight to the
 * coordinator. It is the only end-to-end delivery evidence the application can
 * get: ezb_zcl_set_attr_value() succeeds against a dead radio path, and
 * ezb_bdb_dev_joined() keeps returning true with a dead parent.
 *
 * Exactly one frame per cycle, and the caller must not hold the stack lock:
 * bursting reports drains the fixed out-buffer pool and starves the mainloop
 * that would otherwise drain it. */
static void send_link_probe(void)
{
    ezb_zcl_report_attr_cmd_t command = {
        .cmd_ctrl = {
            .dst_addr = {
                .addr_mode = EZB_ADDR_MODE_SHORT,
                .u = {.short_addr = 0x0000},
            },
            .dst_ep = 1,
            .src_ep = EPSOLAR_ARRAY_ENDPOINT,
            .cluster_id = EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
            .manuf_code = EZB_ZCL_STD_MANUF_CODE,
            .fc = {.direction = 1, .dis_default_rsp = 1},
            .cnf_ctx = {.cb = link_probe_confirm},
        },
        .payload = {.attr_id = EZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_DC_VOLTAGE_ID},
    };

    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_err_t err = ezb_zcl_report_attr_cmd_req(&command);
    esp_zigbee_lock_release();

    if (err != EZB_ERR_NONE) {
        s_diag.report_failures++;
        s_diag.probe_streak++;
        ESP_LOGW(TAG, "Link probe rejected by the stack: 0x%04x", err);
        /* No frame in flight means no confirm will ever close the window;
         * the stack's own locks cover whatever is still draining. */
        close_report_window();
    }
}

static void device_annce_confirm(const ezb_zdo_device_annce_req_result_t *result, void *user_ctx)
{
    (void)user_ctx;
    if (result->error != EZB_ERR_NONE) {
        ESP_LOGW(TAG, "Device announcement not sent: 0x%04x", result->error);
    }
}

/* Device_annce, broadcast to every device whose receiver is on.
 *
 * A secure rejoin keeps the short address, and the stack only re-announces when
 * the address changed. Routers and the coordinator therefore keep forwarding
 * everything addressed to this node towards the router that used to be its
 * parent: uplink keeps working - reports still reach zigbee2mqtt and the values
 * in Home Assistant keep moving - while nothing comes back, APS
 * acknowledgements included. The announcement refreshes the address map and the
 * routes, and it is by far the cheapest repair for that half-dead state.
 *
 * Caller holds the Zigbee stack lock. */
static bool announce_presence_locked(void)
{
    ezb_zdo_device_annce_req_t request = {.cb = device_annce_confirm};
    ezb_err_t err = ezb_zdo_device_annce_req(&request);
    if (err != EZB_ERR_NONE) {
        ESP_LOGW(TAG, "Unable to announce this device: 0x%04x", err);
        return false;
    }
    s_diag.announces++;
    return true;
}

static bool announce_presence(void)
{
    esp_zigbee_lock_acquire(portMAX_DELAY);
    bool announced = announce_presence_locked();
    esp_zigbee_lock_release();
    return announced;
}

/* A rejoin, not a fresh join: BDB steering on a device that is not factory new
 * re-attaches to the stored network and picks a new parent if the old one is
 * gone. Returns whether the rejoin was actually started. */
static bool request_rejoin(const char *reason)
{
    uint32_t now = uptime_seconds();
    if (s_link.last_rejoin_uptime_s != 0
        && now - s_link.last_rejoin_uptime_s < EPSOLAR_REJOIN_BACKOFF_S) {
        return false;
    }
    s_link.last_rejoin_uptime_s = now;
    s_diag.rejoins++;
    ESP_LOGW(TAG, "Rejoining Zigbee network: %s", reason);
    schedule_commissioning_retry(EZB_BDB_MODE_NETWORK_STEERING);
    return true;
}

/* Nothing in the stack tells the application that its frames stopped being
 * acknowledged: ezb_bdb_dev_joined() keeps returning true with a dead parent
 * and the attribute writes keep succeeding, so the confirm clock is the only
 * liveness signal available.
 *
 * A rejoin alone does not repair a stale downlink path, which is why the ladder
 * announces the device first and only then reaches for heavier hammers. */
static void check_link_health(bool joined)
{
    if (!joined) {
        return;
    }
    uint32_t now = uptime_seconds();
    uint32_t silent_s = now - s_diag.last_confirm_uptime_s;
    if (silent_s < EPSOLAR_LINK_STALL_S
        || now - s_link.last_repair_uptime_s < EPSOLAR_LINK_STALL_S) {
        return;
    }

    if (s_link.repair_stage >= EPSOLAR_LINK_REPAIR_ATTEMPTS) {
        ESP_LOGE(
            TAG,
            "No report confirmation for %" PRIu32 "s after %u repairs; restarting",
            silent_s,
            s_link.repair_stage
        );
        save_diagnostics();
        esp_restart();
    }

    bool repaired;
    if (s_link.repair_stage == 0) {
        ESP_LOGW(
            TAG,
            "No report confirmation for %" PRIu32 "s (last status 0x%02x); announcing this device",
            silent_s,
            s_diag.last_probe_status
        );
        repaired = announce_presence();
    } else {
        repaired = request_rejoin("still no report confirmation after announcing this device");
    }
    if (repaired) {
        s_link.repair_stage++;
        s_link.last_repair_uptime_s = now;
    }
}

/* The neighbor table of an end device holds its parent, and the entry is gone
 * once the link to it is lost. Caller holds the Zigbee stack lock. */
static bool find_parent(ezb_nwk_neighbor_info_t *parent)
{
    ezb_nwk_info_iterator_t iterator = EZB_NWK_INFO_ITERATOR_INIT;
    while (ezb_nwk_get_next_neighbor(&iterator, parent) == EZB_ERR_NONE) {
        if (parent->relationship == EZB_NWK_RELATIONSHIP_PARENT) {
            return true;
        }
    }
    return false;
}

static esp_timer_handle_t s_stall_timer;

static void telemetry_stalled(void *arg)
{
    (void)arg;
    s_diag.stall_restarts++;
    ESP_LOGE(
        TAG,
        "No telemetry cycle for %" PRIu32 "s (cycles=%" PRIu32 " light_sleeps=%" PRIu32
        " wakeup_causes=0x%08" PRIx32 "); restarting",
        uptime_seconds() - s_diag.last_cycle_uptime_s,
        s_diag.cycles,
        s_diag.light_sleeps,
        s_diag.last_wakeup_causes
    );
    esp_restart();
}

/* Re-armed at the end of every cycle, so the deadline only expires when cycles
 * themselves stop. It is always further out than the next telemetry wake, so
 * it never wakes the chip on its own. */
static void arm_stall_watchdog(void)
{
    esp_timer_stop(s_stall_timer);
    ESP_ERROR_CHECK(esp_timer_start_once(
        s_stall_timer,
        (uint64_t)EPSOLAR_STALL_RESTART_CYCLES * CONFIG_EPSOLAR_UPDATE_INTERVAL_SECONDS
            * 1000000ULL
    ));
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
        vTaskDelay(pdMS_TO_TICKS(EPSOLAR_MODBUS_INIT_RETRY_MS));
    }

    const esp_timer_create_args_t stall_timer = {
        .callback = telemetry_stalled,
        .name = "epsolar_stall",
    };
    ESP_ERROR_CHECK(esp_timer_create(&stall_timer, &s_stall_timer));
    arm_stall_watchdog();

    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        epsolar_telemetry_t telemetry;
        open_report_window();
        err = epsolar_read_telemetry(&modbus, &telemetry);
        s_diag.cycles++;
        s_diag.last_cycle_uptime_s = uptime_seconds();
        if (err == ESP_OK) {
            if (publish_telemetry(&telemetry)) {
                s_diag.last_publish_uptime_s = s_diag.last_cycle_uptime_s;
            } else {
                s_diag.publish_failures++;
            }
        } else {
            s_diag.modbus_failures++;
            ESP_LOGW(TAG, "No EPSolar telemetry available");
        }

        send_link_probe();

        ezb_nwk_neighbor_info_t parent;
        esp_zigbee_lock_acquire(portMAX_DELAY);
        bool joined = ezb_bdb_dev_joined();
        uint16_t short_address = ezb_nwk_get_short_address();
        bool parented = find_parent(&parent);
        esp_zigbee_lock_release();

        uint32_t now = s_diag.last_cycle_uptime_s;
        ESP_LOGI(
            TAG,
            "cycle=%" PRIu32 " up=%" PRIu32 "s valid=0x%02" PRIx32
            " data_age=%" PRIu32 "s ack_age=%" PRIu32 "s"
            " modbus_fail=%" PRIu32 " publish_fail=%" PRIu32
            " probe_ok=%" PRIu32 " probe_fail=%" PRIu32 " rejoins=%" PRIu32
            " sleeps=%" PRIu32 " slept=%" PRIu32 "s wake=0x%" PRIx32
            " joined=%d addr=0x%04x parent=0x%04x lqi=%u heap=%" PRIu32 "/%" PRIu32,
            s_diag.cycles,
            now,
            err == ESP_OK ? telemetry.valid : 0U,
            now - s_diag.last_publish_uptime_s,
            now - s_diag.last_confirm_uptime_s,
            s_diag.modbus_failures,
            s_diag.publish_failures,
            s_diag.report_confirms,
            s_diag.report_failures,
            s_diag.rejoins,
            s_diag.light_sleeps,
            (uint32_t)(s_diag.light_sleep_us / 1000000U),
            s_diag.last_wakeup_causes,
            joined,
            short_address,
            parented ? parent.short_addr : 0xffffU,
            parented ? parent.lqi : 0U,
            esp_get_free_heap_size(),
            esp_get_minimum_free_heap_size()
        );
        check_link_health(joined);
        /* Dense while a run is young, sparse once it has proven itself. Every
         * failure this node has had arrived in the first few minutes, which is
         * exactly where a ten-cycle mirror is blind; past that the counters are
         * cumulative, so a later readout loses at most the last few minutes and
         * the flash stops being rewritten once a minute forever. */
        if (s_diag.cycles <= EPSOLAR_DIAG_DENSE_CYCLES
            || s_diag.cycles % EPSOLAR_DIAG_SAVE_CYCLES == 0) {
            save_diagnostics();
        }
        arm_stall_watchdog();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONFIG_EPSOLAR_UPDATE_INTERVAL_SECONDS * 1000U));
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

/* Every (re)join is a repair, whether this application asked for it or the
 * stack rejoined on its own: announce the node so the network stops routing to
 * the router that used to be its parent, and give the repair ladder a fresh
 * window in which a confirmation can arrive. Caller holds the stack lock. */
static void on_network_joined(void)
{
    announce_presence_locked();
    s_link.last_repair_uptime_s = uptime_seconds();
    start_telemetry_task();
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
        if (ezb_bdb_is_factory_new()) {
            ESP_LOGI(TAG, "Starting Zigbee network steering");
            ezb_err_t err = ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
            if (err != EZB_ERR_NONE) {
                ESP_LOGE(TAG, "Unable to start Zigbee network steering: 0x%04x", err);
                schedule_commissioning_retry(EZB_BDB_MODE_NETWORK_STEERING);
            }
        } else {
            ESP_LOGI(TAG, "Zigbee device resumed its saved network");
            on_network_joined();
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
            on_network_joined();
        } else {
            ESP_LOGW(TAG, "Zigbee network steering failed: 0x%02x", status);
            schedule_commissioning_retry(EZB_BDB_MODE_NETWORK_STEERING);
        }
        break;
    }

    case EZB_ZDO_SIGNAL_LEAVE: {
        const ezb_zdo_signal_leave_params_t *leave = ezb_app_signal_get_params(signal);
        if (leave->leave_type == EZB_ZDO_LEAVE_TYPE_RESET) {
            ESP_LOGW(TAG, "Removed from Zigbee network; restarting network steering");
            schedule_commissioning_retry(EZB_BDB_MODE_NETWORK_STEERING);
        }
        break;
    }

    case EZB_NWK_SIGNAL_NETWORK_STATUS: {
        const ezb_nwk_signal_network_status_params_t *nwk = ezb_app_signal_get_params(signal);
        /* First sync-loss report; informational. Polling continues, and a
         * parent that is truly gone escalates to NO_ACTIVE_LINKS_LEFT. */
        ESP_LOGW(
            TAG,
            "Zigbee network status 0x%02x (%s) reported by 0x%04x",
            nwk->status,
            ezb_nwk_network_status_to_string(nwk->status),
            nwk->network_addr
        );
        break;
    }

    case EZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
        request_rejoin("no active links left");
        break;

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
                /* The parent ages the child out after ed_timeout without a
                 * keepalive, and an end device that only polls every few
                 * minutes cannot receive anything: the parent holds indirect
                 * transactions for 7.68 s. Poll on the telemetry cadence, so
                 * downlink works and a dead parent is noticed in minutes. */
                .ed_timeout = EZB_NWK_ED_TIMEOUT_8MIN,
                .keep_alive = EPSOLAR_ZIGBEE_KEEP_ALIVE_MS,
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
    ESP_ERROR_CHECK(
        set_battery_power_descriptor(EZB_AF_NODE_POWER_SOURCE_LEVEL_100_PERCENT)
            ? ESP_OK
            : ESP_FAIL
    );
    ezb_nwk_set_rx_on_when_idle(false);
    ezb_aps_secur_enable_distributed_security(false);
    ezb_nwk_set_min_join_lqi(EPSOLAR_ZIGBEE_MIN_JOIN_LQI);
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
    ESP_LOGI(TAG, "Starting ESP32-C6 EPSolar Zigbee sensor");
    report_boot_diagnostics();
    configure_power_management();
    register_light_sleep_counters();
    ESP_ERROR_CHECK(
        xTaskCreate(zigbee_task, "zigbee_main", 6144, NULL, 5, NULL) == pdPASS
            ? ESP_OK
            : ESP_ERR_NO_MEM
    );
}