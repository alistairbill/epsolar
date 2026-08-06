# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ESP-IDF firmware for a Seeed XIAO ESP32-C6 that reads an EPSolar/EPEver solar charge
controller over Modbus RTU and republishes the telemetry as a Zigbee sleepy end device
(ZED), consumed by zigbee2mqtt. `epsolar.mjs` is the matching zigbee2mqtt external
converter — it lives here but is deployed separately to the zigbee2mqtt host.

There is no test suite; verification is flash-and-observe. `DEBUGGING.md` is the
procedure for the light-sleep fault and should be read before touching power
management.

## Build and flash

The IDF environment must be sourced first — nothing works without it:

```sh
. ~/.espressif/tools/activate_idf_v6.0.2.sh
idf.py build
idf.py -p /dev/cu.usbmodem* build flash monitor   # Ctrl-] exits the monitor
idf.py menuconfig                                 # "EPSolar Zigbee sensor" menu
```

`sdkconfig` is gitignored; **`sdkconfig.defaults` is the source of truth**. After editing
`sdkconfig.defaults` or `main/Kconfig.projbuild`, run `idf.py reconfigure` (or delete
`sdkconfig`) — a stale `sdkconfig` silently keeps the old values.

`idf.py flash` does not erase NVS, so retained diagnostics survive a reflash. Bumping
`EPSOLAR_DIAG_MAGIC` invalidates them deliberately (first boot then logs
`no retained diagnostics`).

Target is `esp32c6`; IDF ≥ 6.0.0, esp-zigbee-lib ^2.0, esp-modbus ^2.1.3 pulled by the
component manager into `managed_components/` (gitignored).

## Zigbee API generation

esp-zigbee-lib 2.x exposes the **`ezb_*` API** (`ezbee/…` headers). The old `esp_zb_*`
names from 1.x only exist under `include/compat` and are not used here — do not copy
patterns from 1.x-era Espressif examples without translating them. Header reference:
`managed_components/espressif__esp-zigbee-lib/include/ezbee/`.

## Architecture

Three concurrent contexts, plus timers:

- `app_main` — antenna select, NVS, boot diagnostics, PM config, then spawns `zigbee_main`.
- `zigbee_main` (`zigbee_task`) — builds the endpoint/cluster data model, starts the
  stack, and runs `esp_zigbee_launch_mainloop()`. Signal handling and all APS confirm
  callbacks run here.
- `epsolar_read` (`telemetry_task`) — created only once the node has joined
  (`on_network_joined`). One cycle per `CONFIG_EPSOLAR_UPDATE_INTERVAL_SECONDS`:
  Modbus read → attribute writes → one link probe → link-health check → diagnostics save.

Data path: `modbus.c` (block descriptors, esp-modbus master) → `solar.c` (register decode
into `epsolar_telemetry_t`, per-block `valid` bitmask) → `main.c` (`publish_telemetry`
maps fields onto ZCL attributes).

Any call touching the stack must hold `esp_zigbee_lock_acquire()`/`_release()`. The
publish path (`set_attribute`, `configure_local_reporting`, `set_battery_power_descriptor`)
and `send_link_probe` deliberately take and drop the lock **per command**, with an
`EPSOLAR_REPORT_PACING_MS` pause after each write/arm — bursting reports under one hold
drains the fixed out-buffer pool and starves the mainloop that would drain it, and the
back-to-back radio burst released at the end of the hold is the sharpest supply load the
node generates.

## Endpoint map — keep in sync with `epsolar.mjs`

| EP | Clusters | Carries |
|----|----------|---------|
| 1 | Basic, ElectricalMeasurement (V/I/P) | solar array |
| 2 | TemperatureMeasurement, ElectricalMeasurement (V/I), AnalogInput, PowerConfig | battery |
| 3 | TemperatureMeasurement | controller temperature |
| 4 | ElectricalMeasurement (V/I/P) | load |
| 5 / 6 / 7 | AnalogInput | raw battery / charging / discharging status registers |

The converter decodes status-register bit fields (`decodeBatteryStatus` etc.) and its
endpoint-ID tables mirror these constants. Changing an endpoint, a divisor, or a status
bit mask on either side requires the same change on the other.

Encoding conventions that both sides depend on:

- Voltage/current are centi-units with ZCL divisor 100; **power uses divisor 10
  (deciwatts)** because an int16 with divisor 100 clamps at 327 W.
- `INT16_MIN` (-32768) is the "not yet measured" sentinel; the converter drops it.
- `batteryPercentageRemaining` is ZCL half-percent, so the firmware writes `percent * 2`
  and the converter divides by 2.
- Basic-cluster strings are ZCL length-prefixed literals (`"\x09" "DIY Solar"`); the
  prefix byte must match the string length, and the `fingerprint` in `epsolar.mjs` matches
  those exact values.

## Reporting

`mark_reportable()` must run on the cluster descriptor **before**
`ezb_af_device_desc_register()` — the stack allocates one reporting slot per reportable
attribute at registration, and most non-temperature attributes do not ship with
`EZB_ZCL_ATTR_ACCESS_REPORTING`. Without a slot, both Configure Reporting from the
coordinator and `ezb_zcl_report_attr_cmd_req()` fail with `EZB_ERR_FAIL`.

`configure_local_reporting()` then re-arms every entry in `reported_attributes[]` with a
zero reportable change and its own intervals, because zigbee2mqtt's configuration yields
on-change-only reports and a static array would otherwise go silent.

## Link liveness and the repair ladder

Nothing in the stack reports that frames stopped being delivered: `ezb_zcl_set_attr_value`
succeeds against a dead radio, and `ezb_bdb_dev_joined()` returns true with a dead parent.
So each cycle sends one APS-confirmed Report Attributes to the coordinator
(`send_link_probe`), and `link_probe_confirm` is the only liveness clock.

`arm_stall_watchdog()` is armed *before* `epsolar_modbus_init()`, not after. That retry
loop never gives up and logs to a console nobody reads on external power, so an init that
cannot succeed otherwise leaves the node joined, announced, online in zigbee2mqtt and
silent indefinitely with nothing watching it. It still does not cover a node that never
joins — the telemetry task is never created in that case.

`check_link_health()` escalates one step per `EPSOLAR_LINK_STALL_S` of silence:
Device_annce (repairs the common half-dead state where a secure rejoin kept the short
address and downlink still routes to the old parent) → rejoin via BDB network steering
(rate-limited by `EPSOLAR_REJOIN_BACKOFF_S`) → `esp_restart()`. Separately,
`arm_stall_watchdog()` restarts the node if telemetry cycles themselves stop.

## Power management

Light sleep is the fragile part; read `DEBUGGING.md` before changing anything here.

- `s_startup_lock` holds `ESP_PM_NO_LIGHT_SLEEP` from the moment light sleep is armed
  until the first telemetry cycle completes. Stack bring-up was otherwise entirely
  unguarded — `app_main` returns as soon as `zigbee_task` is created, the report window
  doesn't open until the first cycle, and the Modbus lock is per transaction — so the idle
  task could sleep the chip during `esp_zigbee_init()`. There is deliberately **no failsafe
  timeout**: a node that hasn't completed a cycle hasn't shown it survives a sleep.
- The whole telemetry cycle is bracketed by `open_report_window()` /
  `close_report_window()`, an `ESP_PM_NO_LIGHT_SLEEP` lock held from the Modbus read
  until the probe's APS confirm arrives (either outcome) or
  `EPSOLAR_REPORT_WINDOW_TIMEOUT_MS` fires. A sleepy ED fetches its APS ack via short
  polls; sleeping through that window was the observed field failure (confirm status
  `0xa7`).
- `modbus.c` holds its own `ESP_PM_NO_LIGHT_SLEEP` lock per transaction — esp-modbus
  takes none, and UART RX bytes are lost across a sleep.
- A USB host present at boot disables light sleep for that entire run: USB Serial JTAG
  does not survive a light sleep, so **the fault cannot reproduce over USB.**
- `select_external_antenna()` drives and `gpio_hold_en()`s GPIO3/GPIO14 — the RF switch is
  not exempt from the sleep GPIO isolation, so the hold is what keeps the external antenna
  selected across sleeps.

## Diagnostics black box

The node cannot be observed while failing (attaching USB requires pulling external power,
which is a power-on reset — the one event RTC RAM does not survive). So `s_diag`
(`RTC_NOINIT_ATTR`, mirrored to the `epsolar` NVS namespace) is the channel:
There are **two NVS records under separate keys**, and they must stay separate. The run
record (`diag`) is written only from the cycle loop, so it always describes a run that
produced telemetry; the boot record (`boot`) is stamped by `record_stage()` as each boot
passes a milestone. They shared a key once, and a boot that died early overwrote the last
good run with a `cycles=0` stub — destroying the exact snapshot the black box existed to
capture. Every write goes through `save_diagnostics_to()`, which **refuses while a USB host
is attached**; that freeze is the only thing protecting the evidence.

`record_stage()` runs after `configure_power_management()`, never before — `light_sleep_enabled`
is set there, and a stamp taken earlier records `light_sleep=0` for every run regardless of
regime, which is the field a readout uses to tell a USB run from a battery one.

`report_boot_diagnostics()` prints **all three** sources under one boot header: a USB reset
(reason 11 — what `idf.py monitor` performs) preserves RTC RAM, so the RTC RAM record
describes the readout session, not the field run. Reporting only whichever source happened
to be valid hid the field run behind the readout stub. Each record names the run it
describes, because they are routinely different runs.

**`epsolar_diag_t`'s layout and `EPSOLAR_DIAG_MAGIC` are a wire format.** `load_diagnostics()`
rejects on both a magic mismatch and a size mismatch, so changing either discards the
stored snapshot on the next boot. Never do it while a snapshot is waiting to be read.

Counters are incremented unlocked from both the telemetry task and the stack task — a lost
count is accepted deliberately.
