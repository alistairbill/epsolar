# Investigation handover

ESP32-C6 (Seeed XIAO) Zigbee sleepy end device, esp-zigbee-lib 2.0.3, IDF 6.0.2.
Reads an EPSolar controller over Modbus RTU, reports to zigbee2mqtt. Branch
`zigbee-esp32c6-migration`, 15 commits since `c74cac1`.

**Goal: run reliably with automatic light sleep enabled.** Not achieved.

## Failure modes seen

Three distinct ones. Do not assume they share a cause.

| | Symptom | Light sleep on | Light sleep off |
|---|---|---|---|
| **A** | Wedges at `stage=1`, never starts the Zigbee stack | runs 8, 11, 16, 18 | run 14 |
| **B** | Cycles continue, APS acks stop with `0xa7`, node dies | runs 7, 10 (~2 cycles), run 20 (1 cycle) | run 13 (22 cycles); one transient on USB (boot 22) |
| **C** | Cannot join at all, `NO_NETWORK` on init and steering | resolved — node resumed its saved network at boot 22 | resolved |

**A and B both reproduce with light sleep disabled.** That is the single most
important fact in this investigation and it is why "turn light sleep off" is not
a fix — it is what run 13 and run 14 were running.

### The best data point (run 13, light sleep OFF, 200 ms fast poll)

```
stage=4 cycles=30 modbus_failures=0 publish_failures=0
report_confirms=22 report_failures=7 last_probe_status=0xa7
last_cycle=1744s last_publish=1744s last_confirm=1264s
```

22 cycles confirmed cleanly, then every ack lost from ~21 minutes onward. The
app stayed alive and kept cycling and publishing — so this is a link/network
failure, not a hang or a supply collapse.

### Run 20 and boot 22 (7 Aug): report pacing tested, three new facts

Run 20 was the first field test of the paced-burst build (`2e79e48`), external
power, light sleep on, with the fast-poll gearbox (200 ms active / 5000 ms idle):

```
stage=4 cycles=5 modbus_failures=0 publish_failures=0
report_confirms=1 report_failures=3 last_probe_status=0xa7
last_cycle=262s last_confirm=8s announces=2 rejoins=0
light_sleeps=2162 slept=209s longest_sleep=99ms wakeup_causes=0x10
```

**Pacing did not fix B.** Cycle 1 confirmed at 8 s, every later probe failed
`0xa7` — the same near-immediate onset as runs 7 and 10. Of the two announces,
one is the routine join announce; the ladder's repair announce fired at ~244 s
and the run ended ~18 s later, before the next probe — so *whether an announce
repairs the link is still untested*, and the rejoin rung was never reached.

**First `0xa7` ever seen on USB.** At boot 22 (USB, light sleep disabled), the
cycle-1 probe failed `0xa7` after the full APS retry window, then cycles 2–3
confirmed normally. So `0xa7` has at least one cause that is neither light sleep
nor a sagging supply. Cycle 1 is also the node's heaviest traffic burst (15
reporting re-arms + full publish + probe, immediately after resuming the saved
network), which is exactly the load lead 1 implicates. On USB it recovers by the
next cycle; in the field it never does. The supply hypothesis is weakened but
not dead — the *persistence* of the failure still tracks the power source.

**The gearbox buys no sleep.** With the idle poll at 5000 ms, run 20's sleeps
average 97 ms and cap at 99 ms (2162 sleeps, all timer wakes) — identical to the
old 200 ms-fast-poll pattern. Either the interval change never reaches the
running fast poll (consistent with upstream #782) or something else ticks at
~10 Hz. The app-side log "Fast poll interval now 5000ms" only proves the call
was made, not that the poll spacing changed; verifying needs a sniffer or
parent-side observation.

Run 21 (the field run after run 20) shows `stage=2, cycles=0, light_sleep=1` —
either failure A recurring one stage later than before, or simply the plug being
pulled seconds after boot when the node was brought inside (reset history
`0x01010101` is four straight power-on resets, consistent with plug-cycling).
Only the operator knows which.

## The comparison that matters most

| Run | Build | Power | Light sleep | Result |
|---|---|---|---|---|
| 1h45m | `c74cac1` | **USB** | off | 108 cycles, `probe_ok=107`, **zero** failures |
| run 13 | ~`99016aa` | **external** | off | 22 confirms, then 7x `0xa7`, dead at ~30 min |

Light sleep is off in both. The variable is the power source. Nothing touched
the radio path between those two builds — the changes were diagnostics, the
stall watchdog, and a startup lock that is not created when light sleep is off —
so on the RF side they are equivalent.

On USB it runs indefinitely; on external power the transmit path degrades after
~20 minutes while the CPU keeps cycling and publishing. That shape fits a supply
that sustains the CPU but sags on RF transmit bursts, and it fits the
unexplained brownout (reset reason `9`) in the reset history.

Not proven — one 1h45m run is not proof it would never fail on USB, and the
failure is not a clean brownout. The c74cac1-on-external test was attempted on
7 Aug but ran with light sleep on and died in its first light sleep (see lead
0) — the original never-wakes fault, not failure B — so it decided nothing
about the supply. Bulk capacitance at the board's supply pins (100-470 uF
electrolytic plus 100 nF ceramic) is the standard remedy and has never been
tried.

## Ruled out, with evidence

- **Light sleep as the cause of A or B** — run 14 (A) and run 13 (B) both have
  `light_sleep=0`.
- **Fast poll interval as the cause of B** — run 13 ran at the 200 ms default.
- **`zed_config.keep_alive` not reaching the stack** — reads back as 60000 ms
  before anything sets it. The long poll interval is correctly configured.
- **Modbus** — `modbus_failures=0` in every failing run.
- **Peripheral power-down in light sleep** — disabled since `498817b`; failures
  continued unchanged.
- **NVS/`zb_storage` exhaustion wiping credentials** — would make the node
  factory-new, which takes the `is_factory_new()` branch and logs "Starting
  Zigbee network steering". The logs show the init-failure branch instead, so
  credentials are intact.

## Changes that made things worse (do not repeat)

- **`nwk_pim_stop_fast_poll()` at the probe confirm** (`5db963f`, reverted in
  `8a63475`). Cycle 1 confirmed, the stop ran, all four subsequent probes failed
  with `0x01`, and the node then managed one 29 ms sleep in 243 s. Fast poll is
  the *only* receive window a SED has; without it the APS ack is discarded by
  the parent after `macTransactionPersistenceTime` (7.68 s) long before a 60 s
  long poll comes round.
- **Fast poll raised to 1000 ms** (`b3e55f3`, reverted in `eeff1b6`). Correlates
  exactly with the onset of C, though those builds were also flashed after the
  node had been wedged long enough to be aged out by its parent, which would
  produce the same symptom. Unresolved which it was.

## Established facts worth keeping

- **The SED never leaves fast poll.** Known upstream defect,
  espressif/esp-zigbee-sdk#782 (same board) and #215, both open, no maintainer
  response. Measured: 662 light sleeps in 65 idle seconds, ~74 ms asleep and
  ~24 ms awake each, `longest_sleep=99ms` — two wakes per 200 ms, sustained.
  So light sleep currently buys almost nothing (~24% duty cycle) and generates
  ~10 wake transients per second.
- Two different deadlines bound the poll interval: `macTransactionPersistenceTime`
  7.68 s for APS acks, `macResponseWaitTime` ~491 ms for association/rejoin
  responses. Sizing against the first breaks the second.
- `esp_zb_set_default_long_poll_interval()` is a macro over
  `ezb_nwk_set_keepalive_interval()` — same knob.
- `nwk_pim_is_fast_poll_running()` and `nwk_pim_stop_fast_poll()` are linkable
  but undeclared; signatures taken from disassembly.
  `nwk_pim_start_fast_poll()` takes one argument (`mv s0,a0`).
- Parent LQI is 47–48, against a join floor of 40. Thin.
- One brownout (reset reason `9`) appears in the reset history, never explained.
  Note the app stayed alive and cycling through run 13's failure, so whatever
  happens is not a whole-chip collapse — a rail that holds up the CPU but sags
  on transmit would look like this.
- Two known-good reference points, both on USB: `bb5778c` (Tue 4 Aug) and
  `c74cac1` (Wed 5 Aug).

## Open leads

0. **The supply.** The `c74cac1`-on-external test was run on 7 Aug, but not as
   prescribed: the sdkconfig had light sleep ON (`light_sleep=1` in the
   readout). Result — cycle 1 completed at 3 s, Modbus and publish fine, the
   probe's confirm never arrived (`report_confirms=0`), and the node froze at
   `cycles=1` with `light_sleeps=0`. That zero is meaningful: the counter
   increments on sleep *exit*, and c74cac1's own comment defines the signature
   — "cycles that stop with light_sleeps frozen means the chip never came back
   out of sleep." c74cac1 has neither the startup lock nor the report window,
   so nothing held the chip awake while the APS ack was in flight; it entered
   its first light sleep and never woke. So this run reproduced the *original*
   never-wakes fault, which the branch has since fixed (run 20 survived 2162
   sleeps), and it neither convicts nor exonerates the supply for failure B.
   The supply hypothesis still rests on run 13 vs the 1h45m USB run, both
   sleep-off. The `4504bf8` field run now supersedes a c74cac1 rerun: every
   failing external run to date carried the 15-report load, so if B persists
   without it the supply is effectively confirmed (bulk capacitance next), and
   if B disappears the traffic was the trigger.
1. **Traffic volume — being tested (commit `4504bf8`, 7 Aug).**
   `configure_local_reporting()` re-armed all 15 attributes with a **zero
   reportable change**, so every cycle emitted ~15 APS-acked reports plus the
   link probe. Each ack is an indirect transaction the parent must queue and the
   node must poll for — far outside normal SED behaviour, stressing everything
   implicated in B. `4504bf8` removes the re-arm (reporting now rides on
   zigbee2mqtt's min 10 s / max 300 s with real deltas, the probe is the only
   per-cycle frame) and also removes the fast-poll gearbox, pinning 200 ms.
   Caveat known going in: the re-arm predates `c74cac1`, and `c74cac1` ran
   108 clean cycles on USB with the full 15-report load — so traffic alone
   demonstrably does not kill a USB link inside two hours. The test is whether
   traffic × external power does.
2. **`ezb_config_memory()`** (`ezbee/core.h`) exposes `buffer_pool_size` and the
   APS/route table sizes. Raising the pool, or instrumenting it, would test the
   exhaustion half of lead 1 directly.
3. **`nwk_pim_start_fast_poll(arg)`** — a *bounded* fast poll started per cycle
   might expire naturally and let the node fall back to its 60 s long poll,
   which is the behaviour the upstream defect denies us. Argument semantics
   unknown.
4. **Failure A is completely unexplained.** Nothing instruments the gap between
   `record_stage(EPSOLAR_STAGE_POWER_CONFIGURED)` in `app_main` and
   `esp_zigbee_start()` in `zigbee_task` — roughly 2.8 s of stack and radio
   bring-up with no visibility inside it.

## Current state

Failure C is resolved: at boot 22 the node resumed its saved network
(addr 0x3a2d, parent 0xf380, LQI 59–79) and cycles confirm on USB. Failure B
reproduced unchanged on the paced-burst build in the field (run 20, above), so
pacing joins the list of changes that did not help. Commit `4504bf8` (7 Aug)
now tests lead 1: the zero-delta re-arm is gone, the probe is the only
APS-acked frame an idle cycle sends, and the gearbox is removed (fast poll
pinned at 200 ms). One reading to correct: the boot-22 `0xa7` was not caused
by the 200→5000 ms downshift — both log lines share timestamp 12644 because
`link_probe_confirm` shifted to idle and then logged the failure from the same
callback; the whole retry window ran at 200 ms.

## Diagnostics

`DEBUGGING.md` has the readout procedure. In short: the node cannot be observed
while failing, so it writes counters to NVS and freezes them the moment a USB
host appears. Read three records at boot — RTC RAM, NVS boot record, NVS run
record. They are routinely different runs; tell a readout apart from a field run
by `light_sleep=0`. `stage` says how far a boot got. Do not use
`idf.py monitor --no-reset`; it skips the boot lines, which are the most
valuable output the node produces.

Changing `epsolar_diag_t` or `EPSOLAR_DIAG_MAGIC` discards every stored record.

## Build trap

Editing `sdkconfig.defaults` or `main/Kconfig.projbuild` does **not** change the
build. An existing `sdkconfig` wins and `idf.py reconfigure` will not override
it. Delete `sdkconfig` and rebuild, then verify the value actually landed. This
has silently produced identical firmware more than once.
