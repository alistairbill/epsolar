# Debugging the light sleep fault

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
there survives. The diagnostics blob is versioned, though, and this build bumped
the version, so **the first boot after this flash will say `no retained
diagnostics`**. That is correct, not a failure.

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

- **Reports stop.** Note roughly when. Give it another 45 minutes before you
  touch it — the repair ladder needs 40 minutes of silence before it restarts
  the node, and the stall watchdog restarts it after 3 missed cycles, so leaving
  it alone lets those attempts record themselves.
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

You get **two** lines, and you want the second one:

```
… previous run (RTC RAM): …
… previous run (NVS, as of its last cycle): …
```

They are different runs, and the difference is the whole point. Attaching the
cable is a power-on reset, so RTC RAM holds only the handful of seconds the
board has been up since — and the monitor's own reset is a *USB* reset, reset
reason 11, which preserves RTC RAM rather than clearing it. So the RTC RAM line
describes your readout, not the failure. Tell them apart by `light_sleep`: a
readout run reports `light_sleep=0`, because USB was attached when it booted.

The NVS line is the field run. It is the one frozen the moment you plugged in.

The line you want:

```
W epsolar_zigbee: Boot 7 (reset reason 1, history 0x00030301); previous run
(NVS, as of its last cycle): cycles=182 modbus_failures=0 publish_failures=0
report_confirms=3 report_failures=179 last_probe_status=0xa7 announces=4
rejoins=2 stall_restarts=0 last_cycle=10921s last_publish=0s last_confirm=10740s
light_sleep=1 light_sleeps=8934 slept=10402s longest_sleep=59713ms
wakeup_causes=0x00000010
```

## 5. Decode

Read three fields first: `cycles`, `light_sleeps`, `report_confirms`.

| What you see | What happened |
|---|---|
| `cycles` large, `light_sleeps` large, `report_confirms` frozen low | App alive and sleeping fine, **radio path died**. Downlink/ack problem. |
| `cycles` ≈ 3, `last_cycle` ≈ 180s, `light_sleeps` small but non-zero | **Chip stopped waking.** Wedged in or after a light sleep. |
| `light_sleeps=0` | Light sleep never engaged. The fault is something else entirely. |
| `cycles=0` with `boots` moved on | Booted and died before joining. Never reached a telemetry cycle. |
| `stall_restarts` > 0 and `boots` climbing | App was hanging; the watchdog kept recovering it. |
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
