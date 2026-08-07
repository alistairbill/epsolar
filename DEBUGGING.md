# Debugging the light sleep fault

> **Resolved — 2026-08-07.** The fault was never in this firmware's lock
> scheduling. Two independent failure modes were captured by the black box:
> the 802.15.4 RX path goes permanently deaf after the first automatic light
> sleep (runs 23/26: first cycle's probe confirms, every later confirm is
> `0xa7` while the report window provably held the chip awake), and the chip
> can enter its first light sleep and never exit (run 27: `cycles=1,
> light_sleeps=0` — the counter counts exits). Espressif's tracker documents
> the class: esp-zigbee-sdk #775/#787, and IDF's `IEEE802154_SLEEP_ENABLE` is
> default-off citing unfinished safe power-down (IDF-7317). The fix was
> architectural: light sleep was removed entirely and the battery regime is
> now one telemetry cycle per **deep-sleep** wake, a full reboot each cycle,
> as in Espressif's own battery examples. This document is kept as the record
> of the investigation and of the black-box readout procedure, which still
> applies.

This node cannot be observed while it is failing. USB has to be unplugged before
external power goes on, pulling external power is a power-on reset, and a
power-on reset is the one event RTC RAM does not survive. So the firmware writes
its counters to NVS as it runs and freezes them the moment a USB host appears.
The whole method is: let it fail unattended, then go and read the black box.

## 0. Before you start

Have the board on external power only, USB unplugged. Know which zigbee2mqtt
entity you are watching, so "it stopped" has a timestamp.

## 1. Flash

USB has to be in for this, so external power comes off first.

```sh
# external power OFF, USB IN
. ~/.espressif/tools/activate_idf_v6.0.2.sh
cd ~/esp/epsolar
idf.py reconfigure          # Kconfig and sdkconfig both changed
idf.py -p /dev/cu.usbmodem* build flash monitor
```

`idf.py flash` does not touch the NVS partition, so anything already stored
there survives. The diagnostics blob is versioned, though, and a build that
bumps `EPSOLAR_DIAG_MAGIC` or changes `epsolar_diag_t` discards every stored
record, so **the first boot after such a flash will say `no retained
diagnostics`**. That is correct, not a failure — but it also means never
changing either while a snapshot is still waiting to be read.

Stay in the monitor for two or three minutes and check the node joins and starts
reporting:

```
I epsolar_zigbee: Joined Zigbee network: PAN=0x… channel=… address=0x…
I epsolar_zigbee: cycle=1 up=…s valid=0x3f … probe_ok=1 … joined=1 parent=0x0000 lqi=…
```

You want `joined=1`, `probe_ok` incrementing, and `parent` not `0xffff`.
`sleeps=0` is expected here — USB Serial JTAG holds a no-light-sleep lock the
entire time it is plugged in, so **the fault cannot reproduce over USB.** You are
only checking that it works before you send it away.

Exit the monitor with `Ctrl-]`.

## 2. Run it on external power

```
USB OUT, then external power ON
```

Order matters. The board reboots; that boot is the run under test.

## 3. Wait

Watch zigbee2mqtt. Two outcomes:

- **Reports stop.** Note roughly when. Give it another 20 minutes before you
  touch it — the repair ladder announces after 3 minutes of silence and restarts
  the node after 12, and the stall watchdog restarts it after 3 missed cycles,
  so leaving it alone lets those attempts record themselves in `announces`,
  `rejoins` and `stall_restarts`.
- **Reports keep coming.** Leave it for **several days**, not ten minutes. The
  time constants here are long: `ed_timeout` is 8 minutes, rejoin backoff is 5
  minutes, the repair ladder is 40. A fix that survives one afternoon has not
  been tested.

## 4. Read it

```
external power OFF, then USB IN
```

```sh
idf.py -p /dev/cu.usbmodem* monitor
```

Diagnostics are frozen while USB is attached, so the boot you caused, and every
further reset, leaves the stored snapshot alone. **Reset as many times as you
like.**

Do not press RESET hoping for more: an EN reset is a power-on reset as far as
RTC RAM is concerned, and there is nothing extra there anyway.

You get a boot header and up to **three** records, each naming the run it
describes:

```
W epsolar_zigbee: Boot 12 (reset reason 11); retained diagnostics:
W epsolar_zigbee:   run 11 (RTC RAM, history 0x0b01010b): stage=4 (Modbus ready) at 4s cycles=1 …
W epsolar_zigbee:   run 10 (NVS boot record, history 0x010b0101): stage=3 (joined) at 3s cycles=0 …
W epsolar_zigbee:   run 10 (NVS run record, as of its last cycle, history 0x010b0101): stage=4 …
```

They are routinely *different runs*, and that is the whole point. Attaching the
cable is a power-on reset, so RTC RAM holds only the handful of seconds the
board has been up since — and the monitor's own reset is a *USB* reset, reset
reason 11, which preserves RTC RAM rather than clearing it. So the RTC RAM
record describes your readout, not the failure.

- **RTC RAM** — the most recent run, and the only record of a run the node ended
  itself with `esp_restart()`. During a readout this is the readout.
- **NVS boot record** — how far the last non-USB boot got, stamped at each
  milestone. This is the one that survives a run which never produced telemetry.
- **NVS run record** — the last run that completed at least one cycle. It is
  never overwritten by a boot that died early, so it can be much older than the
  other two. Check its `run` number before drawing conclusions from it.

Tell a readout apart from a field run by `light_sleep`: a readout run reports
`light_sleep=0`, because USB was attached when it booted.

## 5. Decode

If `cycles=0`, read `stage` first — the run never produced a counter worth
reading, and `stage` is the only thing that says where it stopped:

| `stage` | Reached | So it died in |
|---|---|---|
| 1 | power configured | the Zigbee stack init, or `register_device()` — see below |
| 2 | Zigbee started | commissioning — it never joined |
| 3 | joined | `epsolar_modbus_init()`, which retries forever on failure |
| 4 | Modbus ready | the first Modbus read or publish |

`stage=4` with `cycles=0` is the one that means the wiring is fine and the first
transaction is what hangs. `stage=3` that never advances means Modbus init is
failing in a loop — on external power, with no console to say so. The stall
watchdog covers that loop, so `stage=3` with `stall_restarts` climbing is the
signature: it is retrying, failing, and being restarted every three intervals.

`stage=1` that never advances means the node never got the Zigbee stack
started. This was observed on battery — stage 1 at 0s, stage 2 never recorded,
`light_sleep=1` — while the identical path on USB completes in under three
seconds. Bringing the stack up was the one stretch of a boot with no PM lock
held anywhere, so the idle task could sleep the chip mid-radio-init;
`s_startup_lock` now holds it awake from boot until the first completed cycle.
A `stage=1` after that fix means something other than sleep, and the supply is
the next suspect.

Note the one gap the watchdog still does not cover: a node that never joins
never starts the telemetry task, so nothing restarts it. That case shows as
`stage=2` and only the Zigbee stack's own commissioning retries are working on
it.

Otherwise read `cycles`, `light_sleeps`, `report_confirms`:

| What you see | What happened |
|---|---|
| `cycles` large, `light_sleeps` large, `report_confirms` frozen low | App alive and sleeping fine, **radio path died**. Downlink/ack problem. |
| `cycles` ≈ 3, `last_cycle` ≈ 180s, `light_sleeps` small but non-zero | **Chip stopped waking.** Wedged in or after a light sleep. |
| `light_sleeps=0` | Light sleep never engaged. The fault is something else entirely. |
| `stall_restarts` > 0 and the `run` number climbing | App was hanging; the watchdog kept recovering it. Read `stage` to see where. |
| `slept` ≈ `last_cycle` | Sleeping essentially all the time. Normal and healthy. |

Supporting fields:

- `last_probe_status` — APS confirm status of the last link probe. `0x00`
  success, `0xa7` no APS ack (frame left, nothing came back), `0xa9` no short
  address, `0xad` security failure.
- `last_confirm` vs `last_cycle` — how long the node kept cycling after its last
  acknowledged frame. A large gap is the radio-dead signature.
- `history` — reset reasons, newest in the low byte:
  `1` poweron · `3` `esp_restart` · `4` panic · `5` int wdt · `6` task wdt ·
  `7` other wdt · `9` **brownout** · `11` usb · `12` jtag.
  `11`/`12` are your own readout boots; ignore them. A `9` means the supply is
  collapsing on the wake-edge current step, which is a different bug.
- `wakeup_causes` — bitmap, bit N set for `esp_sleep_source_t` N. Timer wakeups
  are the normal case.

## 6. If it still fails

Establish the reference run — light sleep off, everything else identical:

```sh
idf.py menuconfig
#   EPSolar Zigbee sensor  ->  [ ] Enable automatic light sleep
idf.py build flash
```

If it runs clean for days with that unchecked and fails with it checked, the
fault is light sleep and nothing else. Then bisect one symbol per run:

1. `CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP=n`
2. `CONFIG_PM_POWER_DOWN_CPU_IN_LIGHT_SLEEP=n`
3. RF switch pins held through sleep: `gpio_sleep_sel_dis(GPIO_NUM_3)` and
   `gpio_sleep_sel_dis(GPIO_NUM_14)` in `select_external_antenna()`.
   `ESP_SLEEP_GPIO_RESET_WORKAROUND` force-selects `PM_SLP_DISABLE_GPIO`, and
   `esp_sleep_config_gpio_isolate()` floats every pin during sleep except the
   console UART and flash CS — the external antenna select is not exempt.

## 7. Watching it live, if it comes to that

The only way to see the failure as it happens is a console that is not USB.
Wire a 3.3 V USB-TTL adapter to D0/D1 (GPIO0/GPIO1), sharing ground with the
external supply, and:

```
CONFIG_ESP_CONSOLE_UART_CUSTOM=y
CONFIG_ESP_CONSOLE_UART_NUM=0
CONFIG_ESP_CONSOLE_UART_TX_GPIO=0
CONFIG_ESP_CONSOLE_UART_RX_GPIO=1
CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG=y
```

The adapter does not power the board, so it coexists with external power, and
console UART pins are the one exemption in `esp_sleep_enable_gpio_switch()` —
the console keeps working across light sleep, which USB Serial JTAG does not.
Keep the secondary as USB Serial JTAG so you still have a console when the
adapter is not plugged in.
