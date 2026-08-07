# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ESP-IDF firmware for a Seeed XIAO ESP32-C6 that reads an EPSolar/EPEver solar charge
controller over Modbus RTU and republishes the telemetry as a Zigbee sleepy end device
(ZED), consumed by zigbee2mqtt. `epsolar.mjs` is the matching zigbee2mqtt external
converter — it lives here but is deployed separately to the zigbee2mqtt host.

There is no test suite; verification is flash-and-observe. `DEBUGGING.md` is the
historical record of the light-sleep fault that forced the current deep-sleep
architecture; read it before considering light sleep again.

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

**On battery the node lives one telemetry cycle at a time**: cold boot → resume the
saved network → one cycle → deep sleep for the rest of
`CONFIG_EPSOLAR_UPDATE_INTERVAL_SECONDS` → repeat, exactly the shape of Espressif's own
battery examples. This is not an optimization but a correctness requirement: on the C6
the 802.15.4 receive path does not survive light sleep (the radio goes permanently deaf
after the first one — see `DEBUGGING.md` and esp-zigbee-sdk #775), so the only sleep
this firmware performs is the one a full reboot recovers from. With a USB host attached
at boot the node instead cycles in a plain 60 s task loop with the console alive.

Three concurrent contexts:

- `app_main` — antenna select, NVS, boot diagnostics, power-mode choice (battery mode
  arms `awake_deadline_expired`, an `EPSOLAR_MAX_AWAKE_S` failsafe that forces deep
  sleep if a wake wedges anywhere, stack bring-up included), then spawns `zigbee_main`.
- `zigbee_main` (`zigbee_task`) — builds the endpoint/cluster data model, starts the
  stack, and runs `esp_zigbee_launch_mainloop()`. Signal handling and all APS confirm
  callbacks run here. `ezb_nwk_set_rx_on_when_idle(false)` must run **before**
  `esp_zigbee_start()` and never change afterwards (esp-zigbee-sdk #879: changing it on
  a joined device silently kills the downlink).
- `epsolar_read` (`telemetry_task`) — created only once the node has joined
  (`on_network_joined`). One cycle: Modbus read → attribute writes → one APS-confirmed
  link probe → wait for its confirm (`s_probe_confirmed`, `EPSOLAR_CONFIRM_TIMEOUT_MS`)
  → diagnostics save → deep sleep (battery) or delay-until-next-minute (USB).

Data path: `modbus.c` (block descriptors, esp-modbus master) → `solar.c` (register decode
into `epsolar_telemetry_t`, per-block `valid` bitmask) → `main.c` (`publish_telemetry`
maps fields onto ZCL attributes).

Any call touching the stack must hold `esp_zigbee_lock_acquire()`/`_release()`. The
publish path and `send_link_probe` take and drop the lock **per command** so the
mainloop can transmit between writes instead of receiving the whole burst when a
publish-wide hold ends.

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

Report emission rides entirely on the coordinator's Configure Reporting (zigbee2mqtt asks
for min 10 s / max 300 s with deltas of 0.1 V / 0.1 A / 1 W-or-unit): an attribute write
only produces a report when it moves by more than the configured delta, so static values
go quiet in zigbee2mqtt and the link probe is the only guaranteed per-cycle frame. The
stack persists the coordinator's reporting configuration in `zb_storage`, which is what
lets delta reporting keep working across the once-a-minute deep-sleep reboots; if values
other than the probe's ever stop arriving after wakes, that persistence is the first
assumption to check.

## Link liveness

Nothing in the stack reports that frames stopped being delivered: `ezb_zcl_set_attr_value`
succeeds against a dead radio, and `ezb_bdb_dev_joined()` returns true with a dead parent.
So each cycle sends one APS-confirmed Report Attributes to the coordinator
(`send_link_probe`) and **waits for the confirm** (`s_probe_confirmed`, bounded by
`EPSOLAR_CONFIRM_TIMEOUT_MS`) before the cycle is allowed to end — on battery, before the
node deep sleeps. There is no repair ladder any more: every battery wake is a cold boot
that resumes the saved network (a failed resume escalates to BDB steering in the signal
handler), `on_network_joined()` broadcasts one Device_annce per join so downlink routes
never go stale, and a wake whose probe fails simply sleeps and tries again as a fresh
boot a minute later.

## Power / sleep regime

**Do not reintroduce light sleep.** Two independent field failure modes are on record
(`DEBUGGING.md`): the 802.15.4 RX path goes permanently deaf after the first automatic
light sleep (APS confirm `0xa7` forever while provably awake — runs 23/26), and the chip
can enter its first light sleep and never exit it (run 27: `cycles=1, light_sleeps=0`).
Espressif's own tracker documents the class (esp-zigbee-sdk #775, #787; IDF's
`IEEE802154_SLEEP_ENABLE` is default-off citing unfinished power-down support, IDF-7317).

The current regime:

- Battery: one telemetry cycle per deep-sleep wake (`enter_deep_sleep()` sleeps the
  remainder of the update interval). Every wake is a full reboot; the radio never has to
  survive a sleep. `awake_deadline_expired` (`EPSOLAR_MAX_AWAKE_S`) forces deep sleep if
  a wake wedges anywhere, bring-up included, so no hang can strand the node awake.
- USB host at boot: no sleep at all, cycles run in a task loop, console stays alive.
  **Battery-regime behaviour (reboot-per-cycle) therefore cannot be observed over USB.**
- `select_external_antenna()` drives and `gpio_hold_en()`s GPIO3/GPIO14; each wake is a
  boot, so the RF switch is reconfigured from scratch every cycle.

## Diagnostics black box

The node cannot be observed while failing in the field, so `s_diag`
(`RTC_NOINIT_ATTR`, mirrored to the `epsolar` NVS namespace) is the channel. RTC RAM
survives deep sleep, so on battery it stitches the once-a-minute wakes into one
continuous run: `report_boot_diagnostics()` treats a deep-sleep wake as a continuation
(no record rotation, no boots increment, no reset-history shift — a day of wakes must not
scroll the cold boots out of the history), and `light_sleeps`/`light_sleep_us` now count
deep-sleep wakes and time slept. Only a power loss zeroes RTC; NVS is what survives that.

There are **two NVS records under separate keys**, and they must stay separate. The run
record (`diag`) is written only from the cycle loop, so it always describes a run that
produced telemetry; the boot record (`boot`) is stamped by `record_stage()` as each boot
passes a milestone — but **only on cold boots**: deep-sleep wakes stamp RTC only, or the
once-a-minute wakes would wear out NVS. They shared a key once, and a boot that died
early overwrote the last good run with a `cycles=0` stub. Every write goes through
`save_diagnostics_to()`, which **refuses while a USB host is attached**; that freeze is
the only thing protecting the evidence.

Known anomaly, unexplained: on past battery runs the NVS boot record stayed at
`stage=1 (power configured)` even when the RTC record for the same boot reached stage 4
and the run record was being written from the same task seconds later (runs 24 and 27).
Treat a stage-1 boot record as "stage writes past 1 didn't stick", not as proof the boot
died in bring-up.

`record_stage()` runs after `choose_power_mode()`, never before — the regime flag is set
there, and a stamp taken earlier records `deep_sleep=0` for every run regardless of
regime, which is the field a readout uses to tell a USB run from a battery one.

`report_boot_diagnostics()` prints **all three** sources under one boot header: a USB reset
(reason 11 — what `idf.py monitor` performs) preserves RTC RAM, so the RTC RAM record
describes the readout session, not the field run. Reporting only whichever source happened
to be valid hid the field run behind the readout stub. Each record names the run it
describes, because they are routinely different runs. Boot numbers can collide across the
USB freeze (NVS doesn't advance while frozen), so match records by content, not number.

**`epsolar_diag_t`'s layout and `EPSOLAR_DIAG_MAGIC` are a wire format.** `load_diagnostics()`
rejects on both a magic mismatch and a size mismatch, so changing either discards the
stored snapshot on the next boot. Never do it while a snapshot is waiting to be read.

Counters are incremented unlocked from both the telemetry task and the stack task — a lost
count is accepted deliberately.
