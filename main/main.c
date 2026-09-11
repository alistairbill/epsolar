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
#include "esp_zigbee.h"
#include "ezbee/platform/radio.h"
#include "ezbee/zha.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "solar.h"

#define EPSOLAR_STORAGE_PARTITION "zb_storage"
/* How long a cycle waits for the link probe's APSDE-DATA.confirm before giving
 * up on it - success, or 0xa7 once the APS retries exhaust, normally arrives
 * well inside this. On battery the node deep sleeps as soon as the wait ends
 * either way. */
#define EPSOLAR_CONFIRM_TIMEOUT_MS 15000
/* The long poll interval: how often the node polls its parent while awake.
 * It must stay well inside the ed_timeout, or the parent ages the node out. */
#define EPSOLAR_ZIGBEE_KEEP_ALIVE_MS 60000
/* Fast poll is the only receive window a sleepy end device has for a pending
 * downlink frame. Two deadlines bound it - association and rejoin responses
 * must be fetched inside macResponseWaitTime (~491 ms), and the parent
 * discards a queued APS ack after macTransactionPersistenceTime (7.68 s).
 * 200 ms is the only interval verified to meet both on this network. */
#define EPSOLAR_ZIGBEE_FAST_POLL_MS 200
#define EPSOLAR_MODBUS_INIT_RETRY_MS 5000
#define EPSOLAR_MODBUS_INIT_ATTEMPTS 3
/* Hold out for a parent with some link margin, not one at the edge of hearing. */
#define EPSOLAR_ZIGBEE_MIN_JOIN_LQI 40
/* Failsafe on battery: a wake that has not finished its cycle by this deadline
 * is forced into deep sleep whole, whatever it is stuck on - joining, Modbus,
 * a confirm that never comes. The next wake is a clean cold start. This is
 * what bounds every hang the run-24 class demonstrated (a boot that wedged in
 * stack bring-up and sat there for hours on battery). */
#define EPSOLAR_MAX_AWAKE_S 45
/* Never schedule a sleep so short the node thrashes. */
#define EPSOLAR_DEEP_SLEEP_MIN_S 5
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

/* RTC RAM survives deep sleep, so this is what stitches the once-a-minute
 * wakes into one continuous run: counters accumulate across wakes and only a
 * power loss zeroes them. NVS is the channel that survives the power loss -
 * a field readout usually starts by pulling the supply, which is the one
 * event RTC RAM does not survive.
 *
 * Every write refuses while a USB host is present, so the snapshot of the
 * failed run survives the boot that attaching the cable causes and every reset
 * the monitor does after it. Readout is idempotent: reset as often as you
 * like. That freeze is the only thing protecting the evidence, so anything
 * added to the save path has to go through save_diagnostics_to(). */
#define EPSOLAR_DIAG_MAGIC 0x45505336U /* "EPS6" */
#define EPSOLAR_DIAG_NAMESPACE "epsolar"
/* Two records, and they must not share a key. The run record is written only
 * from the cycle loop, so it always describes a run that produced telemetry;
 * the boot record is stamped as the current boot passes each milestone. Shared,
 * a boot that died early would overwrite the last good run with a cycles=0
 * stub. */
#define EPSOLAR_DIAG_RUN_KEY "diag"
#define EPSOLAR_DIAG_BOOT_KEY "boot"
#define EPSOLAR_DIAG_DENSE_CYCLES 30
#define EPSOLAR_DIAG_SAVE_CYCLES 10

/* How far the current boot got. The interesting failures are all in the stretch
 * between reset and the first completed cycle, which produces no counters at
 * all and, on external power, no console output either. */
enum {
    EPSOLAR_STAGE_RESET = 0,
    EPSOLAR_STAGE_POWER_CONFIGURED,
    EPSOLAR_STAGE_ZIGBEE_STARTED,
    EPSOLAR_STAGE_JOINED,
    EPSOLAR_STAGE_MODBUS_READY,
};

static const char *stage_name(uint8_t stage)
{
    switch (stage) {
    case EPSOLAR_STAGE_RESET: return "reset";
    case EPSOLAR_STAGE_POWER_CONFIGURED: return "power configured";
    case EPSOLAR_STAGE_ZIGBEE_STARTED: return "Zigbee started";
    case EPSOLAR_STAGE_JOINED: return "joined";
    case EPSOLAR_STAGE_MODBUS_READY: return "Modbus ready";
    default: return "unknown";
    }
}

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
    uint32_t stage_uptime_s;
    uint8_t last_probe_status;
    uint8_t stage;
    bool light_sleep_enabled;
} epsolar_diag_t;

static RTC_NOINIT_ATTR epsolar_diag_t s_diag;

static uint32_t uptime_seconds(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000);
}

static void save_diagnostics_to(const char *key)
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
    err = nvs_set_blob(nvs, key, &s_diag, sizeof(s_diag));
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Unable to store diagnostics: %s", esp_err_to_name(err));
    }
    nvs_close(nvs);
}

static void save_diagnostics(void)
{
    save_diagnostics_to(EPSOLAR_DIAG_RUN_KEY);
}

/* A timer wake from deep sleep, continuing the run the last cold boot began.
 * Decided in report_boot_diagnostics(), before anything consults it. */
static bool s_deep_sleep_wake;

/* Records how far this boot has got, in its own key, so it cannot overwrite the
 * last run that produced telemetry. Deep-sleep wakes stamp RTC RAM only: they
 * arrive once a minute for months, and NVS would wear out recording each one.
 * The boot record therefore describes cold boots, which is what it is for. */
static void record_stage(uint8_t stage)
{
    s_diag.stage = stage;
    s_diag.stage_uptime_s = uptime_seconds();
    if (!s_deep_sleep_wake) {
        save_diagnostics_to(EPSOLAR_DIAG_BOOT_KEY);
    }
}

static bool load_diagnostics(const char *key, epsolar_diag_t *diag)
{
    nvs_handle_t nvs;
    if (nvs_open(EPSOLAR_DIAG_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return false;
    }
    size_t length = sizeof(*diag);
    bool loaded = nvs_get_blob(nvs, key, diag, &length) == ESP_OK
        && length == sizeof(*diag)
        && diag->magic == EPSOLAR_DIAG_MAGIC;
    nvs_close(nvs);
    return loaded;
}

static void log_previous_run(const char *source, const epsolar_diag_t *previous)
{
    ESP_LOGW(
        TAG,
        "  run %" PRIu32 " (%s, history 0x%08" PRIx32
        "): stage=%u (%s) at %" PRIu32 "s cycles=%" PRIu32
        " modbus_failures=%" PRIu32 " publish_failures=%" PRIu32
        " report_confirms=%" PRIu32 " report_failures=%" PRIu32
        " last_probe_status=0x%02x announces=%" PRIu32 " rejoins=%" PRIu32
        " forced_sleeps=%" PRIu32 " last_cycle=%" PRIu32 "s last_publish=%" PRIu32
        "s last_confirm=%" PRIu32 "s deep_sleep=%d wakes=%" PRIu32
        " slept=%" PRIu32 "s longest_sleep=%" PRIu32 "ms wakeup_causes=0x%08" PRIx32,
        previous->boots,
        source,
        previous->reset_reasons,
        previous->stage,
        stage_name(previous->stage),
        previous->stage_uptime_s,
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

/* Report every source rather than whichever happens to be valid, because they
 * routinely describe different runs and each answers something the others
 * cannot. RTC RAM is the most recent and the only record of a run the node
 * ended itself with esp_restart(), but a USB reset - reset reason 11, which is
 * what idf.py monitor performs on opening the port - preserves it, so during a
 * readout it describes the readout. NVS is frozen while USB is attached and is
 * therefore the only record of a run on external power. */
static void report_boot_diagnostics(void)
{
    esp_reset_reason_t reason = esp_reset_reason();

    /* A timer wake from deep sleep is the next cycle of the same run, not a
     * new run: keep the RTC record accumulating and do not rotate the reset
     * history, or a day of field operation would shift every cold boot out of
     * it. A wake with a USB host attached instead falls through to the full
     * readout below - that is someone coming to collect the evidence. */
    if (reason == ESP_RST_DEEPSLEEP && s_diag.magic == EPSOLAR_DIAG_MAGIC
        && !usb_serial_jtag_is_connected()) {
        s_deep_sleep_wake = true;
        s_diag.last_wakeup_causes = esp_sleep_get_wakeup_causes();
        ESP_LOGI(
            TAG,
            "Wake %" PRIu32 " of run %" PRIu32 " (cycles=%" PRIu32
            " confirms=%" PRIu32 " failures=%" PRIu32 ")",
            s_diag.light_sleeps,
            s_diag.boots,
            s_diag.cycles,
            s_diag.report_confirms,
            s_diag.report_failures
        );
        return;
    }

    epsolar_diag_t retained = s_diag;
    epsolar_diag_t last_run;
    epsolar_diag_t last_boot;
    bool have_retained = retained.magic == EPSOLAR_DIAG_MAGIC;
    bool have_run = load_diagnostics(EPSOLAR_DIAG_RUN_KEY, &last_run);
    bool have_boot = load_diagnostics(EPSOLAR_DIAG_BOOT_KEY, &last_boot);

    /* Counters continue from the freshest source. RTC RAM outranks both stored
     * records when it survived, since NVS lags it by up to
     * EPSOLAR_DIAG_SAVE_CYCLES, and the boot record outranks the run record
     * because a boot that never cycled never touches the latter. */
    epsolar_diag_t previous = (epsolar_diag_t){0};
    if (have_retained) {
        previous = retained;
    } else if (have_boot && (!have_run || last_boot.boots >= last_run.boots)) {
        previous = last_boot;
    } else if (have_run) {
        previous = last_run;
    }

    if (!have_retained && !have_run && !have_boot) {
        ESP_LOGI(TAG, "Boot 1 (reset reason %d); no retained diagnostics", reason);
    } else {
        /* One boot number, stated once; each record then names the run it
         * describes, since they are routinely different runs. */
        ESP_LOGW(TAG, "Boot %" PRIu32 " (reset reason %d); retained diagnostics:", previous.boots + 1, reason);
        if (have_retained) {
            log_previous_run("RTC RAM", &retained);
        }
        if (have_boot) {
            log_previous_run("NVS boot record", &last_boot);
        }
        if (have_run) {
            log_previous_run("NVS run record, as of its last cycle", &last_run);
        }
    }

    s_diag = (epsolar_diag_t){
        .magic = EPSOLAR_DIAG_MAGIC,
        .boots = previous.boots + 1,
        .stall_restarts = previous.stall_restarts,
        .reset_reasons = (previous.reset_reasons << 8) | (uint8_t)reason,
    };
}

/* Decided once at boot: a USB host present means a debugging session, and
 * deep sleep would fight it - the USB Serial JTAG console drops on every
 * sleep, and each wake would rotate the records a readout came to collect.
 * With USB attached the node runs its cycles from a plain task loop with the
 * console alive. The field regime is one cycle per wake with deep sleep in
 * between - the model Espressif's own battery examples use - because on this
 * chip the 802.15.4 receive path does not survive light sleep (the radio
 * goes permanently deaf after the first one; esp-zigbee-sdk #775), and a
 * cold start per cycle is the regime that never depends on it. */
static bool s_deep_sleep_mode;

/* Sleeps whatever is left of the update interval, so wakes land on the
 * telemetry cadence rather than drifting by the awake time. Never returns. */
static void enter_deep_sleep(void)
{
    uint64_t awake_us = (uint64_t)esp_timer_get_time();
    uint64_t interval_us = (uint64_t)CONFIG_EPSOLAR_UPDATE_INTERVAL_SECONDS * 1000000ULL;
    uint64_t minimum_us = (uint64_t)EPSOLAR_DEEP_SLEEP_MIN_S * 1000000ULL;
    uint64_t sleep_us = interval_us > awake_us + minimum_us
        ? interval_us - awake_us
        : minimum_us;

    s_diag.light_sleeps++;
    s_diag.light_sleep_us += sleep_us;
    uint32_t sleep_ms = (uint32_t)(sleep_us / 1000U);
    if (sleep_ms > s_diag.longest_light_sleep_ms) {
        s_diag.longest_light_sleep_ms = sleep_ms;
    }
    ESP_LOGI(TAG, "Deep sleeping for %" PRIu32 " ms", sleep_ms);
    esp_deep_sleep(sleep_us);
}

/* Runs in the esp_timer task, which stays scheduled through everything short
 * of a total lockup - including a Zigbee bring-up that never completes. */
static void awake_deadline_expired(void *arg)
{
    (void)arg;
    s_diag.stall_restarts++;
    ESP_LOGE(
        TAG,
        "Awake %ds without finishing the cycle (stage=%u cycles=%" PRIu32 "); forcing deep sleep",
        EPSOLAR_MAX_AWAKE_S,
        s_diag.stage,
        s_diag.cycles
    );
    save_diagnostics();
    enter_deep_sleep();
}

static void choose_power_mode(void)
{
    s_deep_sleep_mode = !usb_serial_jtag_is_connected();
    /* The wire-format slot that used to record the light sleep regime; 1
     * still means "the battery regime", which is what a readout needs. */
    s_diag.light_sleep_enabled = s_deep_sleep_mode;
    if (!s_deep_sleep_mode) {
        ESP_LOGW(TAG, "USB host attached at boot; cycling without deep sleep for this run");
        return;
    }
    esp_timer_handle_t deadline;
    const esp_timer_create_args_t args = {
        .callback = awake_deadline_expired,
        .name = "epsolar_awake",
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &deadline));
    ESP_ERROR_CHECK(
        esp_timer_start_once(deadline, (uint64_t)EPSOLAR_MAX_AWAKE_S * 1000000ULL)
    );
}

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
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_err_t err = ezb_af_set_node_power_desc(&descriptor);
    esp_zigbee_lock_release();
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



/* Explicit Report Attributes for whichever attribute was just set, sent
 * unconditionally to the coordinator regardless of whatever autonomous
 * reporting configuration might or might not have survived from a previous
 * boot.
 *
 * Autonomous reporting - what Configure Reporting from the coordinator sets
 * up, and what mark_reportable() allocates a slot for - only lasts as long
 * as the in-memory data model it was configured against. That model is
 * rebuilt from scratch by register_device() on every cold boot, i.e. every
 * deep-sleep wake, so whatever the coordinator configured during the one
 * pairing session is gone by the very next wake. Sending explicitly here,
 * every cycle, sidesteps needing that configuration to survive at all -
 * cluster descriptors still mark attributes reportable so Configure
 * Reporting itself succeeds without error during interview, but nothing
 * downstream of that first cycle actually depends on it working. */
static void report_current_value(uint8_t endpoint, uint16_t cluster, uint16_t attribute)
{
    ezb_zcl_report_attr_cmd_t command = {
        .cmd_ctrl = {
            .dst_addr = {
                .addr_mode = EZB_ADDR_MODE_SHORT,
                .u = {.short_addr = 0x0000},
            },
            .dst_ep = 1,
            .src_ep = endpoint,
            .cluster_id = cluster,
            .manuf_code = EZB_ZCL_STD_MANUF_CODE,
            .fc = {.direction = 1, .dis_default_rsp = 1},
            /* No cnf_ctx.cb: fire-and-forget. A dropped report is corrected
             * by the same explicit send next cycle regardless, and waiting
             * on a confirm per attribute here would multiply a cycle's
             * awake-time cost many times over for no benefit worth that. */
        },
        .payload = {.attr_id = attribute},
    };

    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_err_t err = ezb_zcl_report_attr_cmd_req(&command);
    esp_zigbee_lock_release();

    if (err != EZB_ERR_NONE) {
        ESP_LOGW(
            TAG,
            "Explicit report send failed: endpoint=%u cluster=0x%04x attribute=0x%04x err=0x%04x",
            endpoint,
            cluster,
            attribute,
            err
        );
    }
}




/* Let the mainloop transmit the report the last write or arm fired before the
 * next one is queued. Unpaced, the cycle's ~16 frames leave the radio back to
 * back - the sharpest load the node puts on its supply. */
/* Takes and drops the stack lock itself, per write: the mainloop can only
 * transmit the report a write fires while the lock is free. Only the
 * telemetry task calls this. */
static bool set_attribute(uint8_t endpoint, uint16_t cluster, uint16_t attribute, void *value)
{
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_zcl_status_t status = ezb_zcl_set_attr_value(
        endpoint,
        cluster,
        EZB_ZCL_CLUSTER_SERVER,
        attribute,
        EZB_ZCL_STD_MANUF_CODE,
        value,
        false
    );
    esp_zigbee_lock_release();
    if (status == EZB_ZCL_STATUS_SUCCESS) {
        report_current_value(endpoint, cluster, attribute);
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

/* Runs unlocked: every stack call below takes and drops the lock itself, so
 * the mainloop can transmit each report as the write that fired it lands
 * rather than all at once when a publish-wide hold would end. */
static bool publish_telemetry(const epsolar_telemetry_t *telemetry)
{
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

/* Given by the confirm callback - either outcome - so the telemetry cycle can
 * wait for the ack exchange to finish before it ends, which on battery means
 * before the node deep sleeps. */
static SemaphoreHandle_t s_probe_confirmed;

/* Runs in the Zigbee stack task with the stack lock already held.
 *
 * The application confirm hook only fires for frames the application submitted
 * itself; reports the stack's reporting engine emits are confirmed internally
 * and are invisible here. That is why the probe below exists. */
static void link_probe_confirm(ezb_af_user_cnf_t *cnf, void *user_ctx)
{
    (void)user_ctx;
    s_diag.last_probe_status = cnf->status;
    if (cnf->status == 0) {
        s_diag.report_confirms++;
        s_diag.last_confirm_uptime_s = uptime_seconds();
        s_diag.probe_streak = 0;
    } else {
        s_diag.report_failures++;
        /* One line per outage: the cycle line carries the streak. */
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
    /* The confirm ends the cycle's ack exchange whichever way it went:
     * success, or 0xa7 once the stack's APS retries have exhausted. */
    xSemaphoreGive(s_probe_confirmed);
}

/* One acknowledged Report Attributes per telemetry cycle, sent straight to the
 * coordinator. It is the only end-to-end delivery evidence the application can
 * get: ezb_zcl_set_attr_value() succeeds against a dead radio path, and
 * ezb_bdb_dev_joined() keeps returning true with a dead parent.
 *
 * Returns whether a frame is in flight, i.e. whether a confirm will follow. */
static bool send_link_probe(void)
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
        return false;
    }
    return true;
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

static void telemetry_task(void *arg)
{
    /* Stamped here rather than in on_network_joined(), which holds the stack
     * lock - no place for an NVS commit. */
    record_stage(EPSOLAR_STAGE_JOINED);

    s_probe_confirmed = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(s_probe_confirmed != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    epsolar_modbus_t modbus = {0};
    esp_err_t err = ESP_FAIL;
    for (int attempt = 1; attempt <= EPSOLAR_MODBUS_INIT_ATTEMPTS; attempt++) {
        err = epsolar_modbus_init(&modbus);
        if (err == ESP_OK) {
            break;
        }
        ESP_LOGE(
            TAG,
            "Modbus initialization failed (%d/%d): %s",
            attempt,
            EPSOLAR_MODBUS_INIT_ATTEMPTS,
            esp_err_to_name(err)
        );
        vTaskDelay(pdMS_TO_TICKS(EPSOLAR_MODBUS_INIT_RETRY_MS));
    }
    /* A dead RS485 bus does not silence the node: the cycle below still runs,
     * still probes the coordinator, and on battery still sleeps on schedule. */
    bool modbus_ready = err == ESP_OK;
    if (modbus_ready) {
        record_stage(EPSOLAR_STAGE_MODBUS_READY);
    }

    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        epsolar_telemetry_t telemetry;
        s_diag.cycles++;
        s_diag.last_cycle_uptime_s = uptime_seconds();
        err = modbus_ready
            ? epsolar_read_telemetry(&modbus, &telemetry)
            : ESP_ERR_INVALID_STATE;
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

        /* Wait the ack exchange out: the coordinator's APS ack rides back as
         * an indirect transmission the stack fetches with its own fast polls,
         * and the confirm - either outcome - is the moment the cycle has
         * nothing left to do. On battery that is what gates the deep sleep. */
        xSemaphoreTake(s_probe_confirmed, 0);
        if (send_link_probe()) {
            xSemaphoreTake(s_probe_confirmed, pdMS_TO_TICKS(EPSOLAR_CONFIRM_TIMEOUT_MS));
        }

        ezb_nwk_neighbor_info_t parent;
        esp_zigbee_lock_acquire(portMAX_DELAY);
        bool joined = ezb_bdb_dev_joined();
        uint16_t short_address = ezb_nwk_get_short_address();
        bool parented = find_parent(&parent);
        esp_zigbee_lock_release();

        ESP_LOGI(
            TAG,
            "cycle=%" PRIu32 " wakes=%" PRIu32 " valid=0x%02" PRIx32
            " modbus_fail=%" PRIu32 " publish_fail=%" PRIu32
            " probe_ok=%" PRIu32 " probe_fail=%" PRIu32 " streak=%" PRIu32
            " joined=%d addr=0x%04x parent=0x%04x lqi=%u heap=%" PRIu32 "/%" PRIu32,
            s_diag.cycles,
            s_diag.light_sleeps,
            err == ESP_OK ? telemetry.valid : 0U,
            s_diag.modbus_failures,
            s_diag.publish_failures,
            s_diag.report_confirms,
            s_diag.report_failures,
            s_diag.probe_streak,
            joined,
            short_address,
            parented ? parent.short_addr : 0xffffU,
            parented ? parent.lqi : 0U,
            esp_get_free_heap_size(),
            esp_get_minimum_free_heap_size()
        );
        /* Dense while a run is young, sparse once it has proven itself. Every
         * failure this node has had arrived in the first few minutes, which is
         * exactly where a ten-cycle mirror is blind; past that the counters are
         * cumulative, so a later readout loses at most the last few minutes and
         * the flash stops being rewritten once a minute forever. */
        if (s_diag.cycles <= EPSOLAR_DIAG_DENSE_CYCLES
            || s_diag.cycles % EPSOLAR_DIAG_SAVE_CYCLES == 0) {
            save_diagnostics();
        }
        if (s_deep_sleep_mode) {
            enter_deep_sleep();
        }
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

/* Announce on every (re)join, cold boots and deep-sleep wakes alike: a resume
 * that kept the short address does not re-announce by itself, and the
 * broadcast is what stops the network routing downlink traffic towards a
 * router that is no longer the parent. One cheap frame per wake buys fresh
 * routes. Caller holds the stack lock. */
static void on_network_joined(void)
{
    announce_presence_locked();
    /* Read again after commissioning: an interval quietly reset during a join
     * would be indistinguishable from one that was never configured. */
    ESP_LOGW(
        TAG,
        "SED poll configuration after join: long poll %" PRIu32 "ms, fast poll %" PRIu32 "ms",
        ezb_nwk_get_keepalive_interval(),
        ezb_nwk_get_fast_poll_interval()
    );
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

/* BDB commissioning statuses this node can plausibly see. */
static const char *bdb_status_name(ezb_bdb_comm_status_t status)
{
    switch (status) {
    case EZB_BDB_STATUS_SUCCESS: return "success";
    case EZB_BDB_STATUS_IN_PROGRESS: return "in progress";
    case EZB_BDB_STATUS_NO_NETWORK: return "no network";
    case EZB_BDB_STATUS_TARGET_FAILURE: return "target failure";
    case EZB_BDB_STATUS_FORMATION_FAILURE: return "formation failure";
    case EZB_BDB_STATUS_NO_SCAN_RESPONSE: return "no scan response";
    case EZB_BDB_STATUS_NOT_PERMITTED: return "not permitted";
    case EZB_BDB_STATUS_TCLK_EX_FAILURE: return "trust centre key exchange failed";
    case EZB_BDB_STATUS_NOT_ON_A_NETWORK: return "not on a network";
    case EZB_BDB_STATUS_ON_A_NETWORK: return "already on a network";
    case EZB_BDB_STATUS_CANCELLED: return "cancelled";
    case EZB_BDB_STATUS_DEV_ANNCE_SEND_FAILURE: return "device announce not acked";
    default: return "unknown";
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
            /* Escalate to steering rather than retrying initialisation.
             * Initialisation reads stored network state; if that state does not
             * get the node back onto its network, reading it again will not
             * either, and retrying it is a livelock. Steering on a device that
             * is not factory new is a rejoin, which is the recovery this needs,
             * and a failed steering schedules its own retry below. */
            ESP_LOGW(
                TAG,
                "Zigbee initialization failed: 0x%02x (%s); rejoining instead",
                status,
                bdb_status_name(status)
            );
            schedule_commissioning_retry(EZB_BDB_MODE_NETWORK_STEERING);
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
            ESP_LOGW(
                TAG,
                "Zigbee network steering failed: 0x%02x (%s)",
                status,
                bdb_status_name(status)
            );
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
        s_diag.rejoins++;
        ESP_LOGW(TAG, "No active links left; rejoining Zigbee network");
        schedule_commissioning_retry(EZB_BDB_MODE_NETWORK_STEERING);
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
    /* On USB, power isn't a constraint, so behave as a non-sleepy device -
     * always listening - which sidesteps the SED poll-interval entirely and
     * lets Z2M's interview/bind requests through immediately instead of
     * racing the 60s long-poll window. The battery build still needs true
     * SED behaviour: rx_on_when_idle(true) would keep the radio powered
     * continuously and defeat the whole point of the deep sleep regime. */
    ezb_nwk_set_rx_on_when_idle(!s_deep_sleep_mode);
    /* Asserted rather than trusted: the 200 ms library default is
     * load-bearing, and a release that changed it would otherwise break
     * joining silently. The long poll interval is read, not set:
     * zed_config.keep_alive reaches the stack intact, and a wrong value here
     * would look identical to the fast poll defect from the outside. */
    ezb_nwk_set_fast_poll_interval(EPSOLAR_ZIGBEE_FAST_POLL_MS);
    ESP_LOGW(
        TAG,
        "SED poll configuration: long poll %" PRIu32 "ms, fast poll %" PRIu32 "ms",
        ezb_nwk_get_keepalive_interval(),
        ezb_nwk_get_fast_poll_interval()
    );
    ezb_aps_secur_enable_distributed_security(false);
    ezb_nwk_set_min_join_lqi(EPSOLAR_ZIGBEE_MIN_JOIN_LQI);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(EZB_RADIO_2P4GHZ_ALL_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(zigbee_signal_handler));
    register_device();
    ESP_ERROR_CHECK(esp_zigbee_start(false));
    record_stage(EPSOLAR_STAGE_ZIGBEE_STARTED);
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
    choose_power_mode();
    /* After choose_power_mode(), not before: it sets the regime flag, and an
     * earlier stamp would record deep_sleep=0 for every run regardless of
     * regime - the one field a readout uses to tell a USB run from a battery
     * one. */
    record_stage(EPSOLAR_STAGE_POWER_CONFIGURED);
    ESP_ERROR_CHECK(
        xTaskCreate(zigbee_task, "zigbee_main", 6144, NULL, 5, NULL) == pdPASS
            ? ESP_OK
            : ESP_ERR_NO_MEM
    );
}