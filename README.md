# Brick

## Companion controller and local dashboard

Flash both ESPs from this revision, then run:

```sh
python3 -m http.server 8000 --bind 127.0.0.1 --directory tools/console
```

Open <http://localhost:8000> in desktop Chrome/Chromium or Edge, connect the
sender by USB, and select a Joy-Con/gamepad exposed by your OS. The second pilot
can take over heading with a 150 ms timeout back to the main radio. The page also
provides manual translation flip, live telemetry, log commands, CSV copy/download,
and a run inspector with locally reconstructed correlation curves. Throttle,
translation, and arming remain on the physical radio.

Top/bottom LED data now use GPIO 1/4 with blue/red heading digits. Rewire the bottom
data line separately, or change those pins in `brick/config.h`. Control/telemetry
packets are versioned and require updating **both** ESPs; persisted dump format
remains v5. See [setup, mappings, wiring, and RF notes](tools/console/README.md).


## RSSI RPM estimation and delay-compensated heading

Flash the robot with `brick` and the controller ESP with `sender_test`. The
sender fills spare ESP-NOW transmit slots with two-byte RSSI probes and schedules
throttle packets every 20 ms. Heartbeat and throttle traffic have priority over
probes. The sender services the link every millisecond on ESP32-S3 and uses
6 Mbps for unicast. The actual receive rate depends on the radio link.

Robot telemetry is best effort, limited to 20 Hz, and yields its shared transmit
queue to recorder status and dump packets. It pauses while a radio export is
queued or active. Heartbeat, throttle and sender RSSI traffic continue. This
prevents per-loop telemetry from claiming every freed queue slot before an
export packet can enter the queue.

RSSI is the robot's RPM source. The current robot configuration disables the
accelerometer. Version-5 captures reuse its storage slots for heading diagnostics. Watch serial output for `rpm`, `fast_rpm`,
`long_rpm`, `throttle`, and `rssi_hz`. An RPM of zero means the signal has
not acquired a reliable period or has lost it.

The estimator retains about 406 ms of RSSI on a 1 ms grid and requires at least
500 delivered packets/sec across that history. It acquires a period using a
250 ms comparison window and three agreeing analyses spaced about 50 ms apart.
Once acquired, it searches near the current period every 10 ms of received
data. The tracking comparison window is three periods, bounded to 60–250 ms.
A motor-throttle change of 0.01 or disagreement between the fast and long
windows enables a 300 ms transient mode with a two-period window bounded to
40–250 ms and a wider local period search. Long-window analyses continue at
50 ms intervals. Short-window confidence controls acceptance. A brief invalid
tracking result can hold the previous RPM for up to 150 ms, while a packet
outage resets it after 50 ms.

Global peak selection prefers the earliest credible period. Normally its score
must be within 0.08 of the strongest peak and at least 0.55. An earlier peak
with score at least 0.75 can also qualify when peaks near twice and three times
its interpolated period support the same cycle. Alternating weak/strong lobes
are excluded from this extra rule to avoid selecting half a rotation.

Tracking rejects implausible speed steps instead of slewing toward them. The
allowed RPM difference is the smaller of 20% of the last accepted RPM and
2.5% plus 6000 RPM/s over elapsed time, capped at 50 ms. This bounds a single
update without estimating acceleration. A rejected candidate holds the last
valid estimate briefly, then expires to zero if no credible estimate arrives.

The global analysis independently checks for a persistent integer-repeat lock.
Three strong, consistent global results indicating a different 2–8-times
period invalidate the local lock and start confirmed reacquisition. This can
recover even if feedforward is unavailable. During reacquisition RPM is zero,
meaning unavailable, until the new period has been confirmed.

The steady RPM expected from the calibrated feedforward is a soft period hint.
It is enabled after at least 600 ms of stable throttle, several stable long
estimates, and signal evidence. It can resolve two-lobe ambiguity
among near-equal correlation peaks within 20% of the calibrated RPM. Signal
evidence is required. A hint cannot bypass continuity/confirmation checks or
directly replace measured RPM. Disagreement with a wrong local lock no longer
disables the global hint indefinitely.

The implementation estimates speed from RSSI periodicity. Its sampling and
autocorrelation approach was inspired by
[TGower/rotation_sensing](https://github.com/TGower/rotation_sensing).

## RPM timing and heading

`RpmHeadingTracker` uses RPM dead reckoning plus a correction for delayed speed
measurements. No RSSI waveform phase, antenna orientation, or feedforward target
is used to correct heading. The default mode corrects spin/acceleration while
translation uses plain dead reckoning.

The RPM estimator now publishes an effective measurement epoch for each accepted
period. Correlation compares the latest N samples with a segment one period
older, so a linear-ramp approximation puts its measurement at:

```
epoch = window_end - ((N - 1) * 1 ms + measured_period) / 2
```

The Wi-Fi/local-clock bridge maps that epoch to `micros()`, using timestamps
captured before application queueing. Cached long-window results retain their
original epoch; repeated, backward, future and over-age observations cannot
masquerade as fresh measurements.

The heading tracker integrates RPM between these past epochs with a trapezoidal
rule. This revises the predicted history as measured speeds become available.
It projects that integral to the present using a fitted RPM trend from at most
12 observations spanning 150 ms. Weak/noisy trends become a local average rather
than an acceleration extrapolation. Significant trends are limited to 6000 RPM/s,
a 120 ms prediction ramp and a speed change of at most 20% / 600 RPM.

The difference between that revised heading and the live heading becomes a
bounded angular correction rate (at most 4 rad/s, nominal 100 ms response).
It does **not** step the heading angle. The fast task extrapolates the published
total heading rate; driver turns remain an independent offset. Initial lock and
reacquisition after a long outage preserve the current heading. Complete loss of
valid RPM coasts for at most 300 ms, then freezes prediction.

### Correction modes

The robot boots in **acceleration-only** mode by default. This keeps the existing
compensator for non-translation operation, including spin acceleration/braking;
it does not introduce new throttle-edge gating. While translation is commanded,
heading follows raw-RPM dead reckoning, without prediction or error correction.
The former translation correction remains available for comparison.

Send a standalone command plus Enter to either ESP (Newline or Both NL & CR):

| Command | Mode |
| --- | --- |
| `a` | Acceleration-only; translation correction off |
| `t` | Correction on, including the guarded translation profile |
| `n` | All correction off; plain dead reckoning |

These live selections last until reboot. Set `RPM_HEADING_COMPENSATION` and
`RPM_HEADING_TRANSLATION_COMPENSATION` in `brick/config.h` for boot defaults:
`true,false` selects acceleration-only; `true,true` enables translation;
`false` for the first flag disables all correction. Update the sender firmware
as well as the robot to forward the new live commands; the dump format remains v5.

Changing modes preserves the heading angle and driver-turn offset. Disabling
translation discards its correction history rather than banking error. Correction
stays off for 500 ms after neutral, then waits for a measurement whose effective
epoch is after that interval. It reacquires against the current heading, so it
cannot replay an earlier movement error after the driver manually corrects it.
Spin correction is temporarily unavailable during this settling/fresh-data wait.

The following guarded translation behavior applies only when enabled with `t`
or the corresponding boot flag:

When the translation stick exceeds the same deadzone used for motor modulation,
a separate translation profile engages over 100 ms. A five-observation median
checks suspicious heading-RPM innovations before they can enter the retrospective
integral; normal smooth RPM changes pass immediately. The fallback median is
timestamped at its center sample, so confirmation does not turn filter delay
into additional heading error. Large isolated one/two-observation spikes are withheld from the integral,
while sustained speed changes can be confirmed. Smaller changes below the
plausibility threshold still follow the measured RPM.
This filter affects heading only; motor RPM estimation/control is unchanged.

Weak trends use a 200 ms error response, a 2 rad/s correction cap, bounded
correction-rate slew, and smoothing/slew limits on the total heading rate.
Prediction increments are limited to 8% / 180 RPM and 1800 RPM/s while this guard
is fully engaged. Coherent acceleration trends progressively regain the existing
faster response; a suspect observation keeps the guard engaged for 300 ms, then blends its
influence out over 200 ms.
The guard holds for 500 ms after neutral, then blends out over 300 ms. Mode
changes do not re-zero heading or reinterpret the measurement clock/epoch.
The non-translation acceleration gains remain unchanged.

Tune `RPM_HEADING_TRANSLATION_RESPONSE_MS`,
`RPM_HEADING_TRANSLATION_MAX_CORRECTION_RAD_S`, and
`RPM_HEADING_TRANSLATION_PREDICTION_MS` independently in `brick/config.h`.
The prediction horizon defaults to the existing 120 ms: translation intent alone
is not evidence that the physical measurement delay changed. The slower guarded
response deliberately trades some transient load-change error for smoother
recovery; large sustained RF-induced RPM errors and steady RPM bias remain
possible. Updated robot captures are needed to assess these gains on hardware.

Use `n` (or boot with `RPM_HEADING_COMPENSATION=false`) for plain RPM integration.
`RPM_HEADING_DELAY_SCALE` defaults to 1 and scales the inferred measurement age
for calibration. This effective-epoch model is approximate, particularly under
rapid/nonlinear acceleration or changing RF conditions. It cannot remove steady
RPM bias or infer unseen speed changes before delayed measurements arrive.
Use actual robot runs to calibrate it; captures have no independent heading truth.

The former RSSI phase-lock implementation has been removed. The continuous clock,
packet timestamps, driver-turn offset and filesystem park/RMT-drain handshake
remain. The tracker uses a small RPM history instead of the 4 KiB RF phase ring.

### Build and test

The sketches were built for ESP32-S3 with Arduino-ESP32 3.3.7. Update both
robot and sender firmware for version-5 heading dump exports. Earlier sender
firmware rejects v5 dumps. The desktop plotter accepts version-1 through
version-5 dumps.

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc brick
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc sender_test

c++ -std=c++17 -O2 -Ibrick/src/subsystems tests/rssi_rpm_estimator_test.cpp \
    brick/src/subsystems/rssi_rpm_estimator.cpp -o /tmp/brick-rssi-test
/tmp/brick-rssi-test
c++ -std=c++17 -O2 -Ibrick/src/subsystems tests/rssi_tracking_test.cpp \
    brick/src/subsystems/rssi_rpm_estimator.cpp -o /tmp/brick-tracking-test
/tmp/brick-tracking-test
c++ -std=c++17 -O2 -Ibrick/src/subsystems tests/rssi_continuity_test.cpp \
    brick/src/subsystems/rssi_rpm_estimator.cpp -o /tmp/brick-continuity-test
/tmp/brick-continuity-test
c++ -std=c++17 -O2 -Itests/host -Ibrick tests/run_recorder_test.cpp \
    brick/src/subsystems/run_recorder.cpp -o /tmp/brick-recorder-test
/tmp/brick-recorder-test
c++ -std=c++17 -O2 -Itests/host tests/melt_idle_test.cpp \
    brick/src/subsystems/melt.cpp brick/src/subsystems/pid.cpp \
    brick/src/subsystems/led_display.cpp -o /tmp/brick-idle-test
/tmp/brick-idle-test
c++ -std=c++17 -O2 -Itests/host tests/led_display_test.cpp \
    brick/src/subsystems/melt.cpp brick/src/subsystems/pid.cpp \
    brick/src/subsystems/led_display.cpp brick/src/subsystems/led.cpp \
    -o /tmp/brick-display-test
/tmp/brick-display-test
c++ -std=c++17 -O2 -Itests/host -Ibrick tests/comms_export_test.cpp \
    brick/src/subsystems/comms.cpp brick/src/subsystems/run_recorder.cpp \
    brick/src/subsystems/rssi_rpm_estimator.cpp brick/src/subsystems/rpm_heading_tracker.cpp \
    brick/src/lib/NowPair.cpp \
    -o /tmp/brick-comms-export-test
/tmp/brick-comms-export-test
/tmp/brick-comms-export-test 20
c++ -std=c++17 -O2 tests/storage_gate_test.cpp -o /tmp/brick-storage-gate-test
/tmp/brick-storage-gate-test
c++ -std=c++17 -O2 -Itests/host tests/led_buffer_test.cpp \
    brick/src/subsystems/led.cpp -o /tmp/brick-led-test
/tmp/brick-led-test
c++ -std=c++17 -O2 -Ibrick/src/subsystems tests/rpm_heading_test.cpp \
    brick/src/subsystems/rpm_heading_tracker.cpp -o /tmp/brick-rpm-heading-test
/tmp/brick-rpm-heading-test
c++ -std=c++17 -O2 -Ibrick/src/subsystems tests/heading_mode_test.cpp \
    brick/src/subsystems/rpm_heading_tracker.cpp -o /tmp/brick-heading-mode-test
/tmp/brick-heading-mode-test
c++ -std=c++17 -O2 -Ibrick/src/subsystems tests/translation_heading_test.cpp \
    brick/src/subsystems/rpm_heading_tracker.cpp -o /tmp/brick-translation-test
/tmp/brick-translation-test
c++ -std=c++17 -O2 -Ibrick/src/subsystems tests/rpm_epoch_test.cpp \
    brick/src/subsystems/rssi_rpm_estimator.cpp -o /tmp/brick-epoch-test
/tmp/brick-epoch-test
c++ -std=c++17 -O2 tests/dump_command_test.cpp -o /tmp/brick-command-test
/tmp/brick-command-test
c++ -std=c++17 -O2 -Ibrick/src/subsystems tests/heading_clock_test.cpp -o /tmp/brick-heading-test
/tmp/brick-heading-test
c++ -std=c++17 -O2 -Itests/host tests/now_pair_timing_test.cpp \
    brick/src/lib/NowPair.cpp -o /tmp/brick-timing-test
/tmp/brick-timing-test
c++ -std=c++17 -O2 tests/run_dump_phase_test.cpp -o /tmp/brick-phase-dump-test
/tmp/brick-phase-dump-test
c++ -std=c++17 -O2 tests/companion_packet_test.cpp -o /tmp/brick-companion-test
/tmp/brick-companion-test
c++ -std=c++17 -O2 -Itests/host tests/radio_resilience_test.cpp \
    brick/src/lib/NowPair.cpp -o /tmp/brick-radio-test
/tmp/brick-radio-test
c++ -std=c++17 -O2 -Itests/host tests/dual_led_test.cpp \
    brick/src/subsystems/led.cpp -o /tmp/brick-dual-led-test
/tmp/brick-dual-led-test
c++ -std=c++17 -O2 -Itests/host tests/sender_console_test.cpp \
    brick/src/lib/NowPair.cpp -o /tmp/brick-sender-console-test
/tmp/brick-sender-console-test
c++ -std=c++17 -O2 -Itests/host -Ibrick tests/control_timeout_test.cpp \
    brick/src/subsystems/comms.cpp brick/src/subsystems/run_recorder.cpp \
    brick/src/subsystems/rssi_rpm_estimator.cpp brick/src/subsystems/rpm_heading_tracker.cpp \
    brick/src/lib/NowPair.cpp -o /tmp/brick-control-timeout-test
/tmp/brick-control-timeout-test
node --test tests/console_test.mjs
python3 -m unittest discover -s tests -p 'test_*.py'
```

The communication regression compiles the real Comms, recorder and NowPair
scheduler against bounded host queues. It checks full RAM and saved-file
exports under telemetry/heartbeat contention at 10 ms and 20 ms loop periods,
including reconnection, retransmissions and payload CRC. The sender-side CSV
and loss/backpressure tests run separately in `run_recorder_test`.

## Benchmark on an ESP32

Build and flash the separate `rpm_benchmark` sketch. It runs the same estimator
implementation used by the robot. Open its serial port at 115200 baud.

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc rpm_benchmark
arduino-cli upload --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc --port /dev/ttyACM0 rpm_benchmark
```

Replace `/dev/ttyACM0` with your board's port.

Send `b` to run deterministic noisy simulations at 400, 600, 1111, 2275 and
3000 RPM. Each result includes correctness checks after warm-up, RMS RPM error,
mean and maximum combined RPM/heading sample/update/analysis times, an estimated compute load at the
simulated packet rate, and free heap and stack bytes. The matches fraction
counts post-warm-up estimates within 2.5% of the expected RPM. Sample generation,
serial output and deliberate yields are outside the timed estimator calls.
This sketch initializes the estimator and serial port. The robot's live dumps
include `estimator_us` to measure combined update time with normal robot/Wi-Fi activity.

Send `p` for a six-second ideal delayed-RPM test: a 1200-to-3000 RPM ramp with
100 ms measurement delay and known measurement epochs. It compares compensated
heading with plain dead reckoning and reports settled RMS, transient peak and
CPU/heap use. This isolates the heading model; `b`/`r` and the host simulator use
the actual noisy-RSSI estimator. Ideal-measurement results are not robot accuracy.

Send `d` to print a two-second 3000 RPM simulation as CSV with a `true_rpm`
and `true_phase_rad` columns and device timings. Send `r` for a continuous-phase 1200-to-3000 RPM
ramp with a step in throttle before the mechanical speed settles. Save the
serial output and use the plotter below.

## Save a robot run

Serial dump commands on **both** ESPs require a standalone letter and Enter:
`s`, `p`, `e`, `f`, or `x`. The heading controls `a`, `t`, `n` use the same format. Set Serial Monitor line endings to Newline or Both NL & CR.
Boot and link connection do not request an export. Partial input or embedded
letters in startup/echo text cannot trigger one. Controller capture still uses
an actual SWB high-to-low edge; its initial position is not a trigger.

The robot keeps the latest 2048 events in RAM, including every delivered RSSI
sample and each control/heading update. At about 1000 RSSI packets/sec
this covers roughly 1.8 seconds before the trigger. The version-5 buffer uses
168 KiB RAM.

Toggle controller SWB from high to low to capture the recent history. Flash
persistence follows automatically after motor commands and outputs are zero
and the fast task has parked its peripheral work.
Alternatively send `s` over either board's serial connection. An optional
active-low GPIO button can be selected with `DUMP_BUTTON_PIN` in
`brick/config.h`. Leave it at `-1` to use SWB and serial commands.

A partition scheme divides the ESP32's flash into regions for firmware and
persistent data. Select a scheme with at least 512 KiB reserved for a filesystem.
In Arduino IDE, select your board and matching **Tools > Flash Size**, then
choose an entry containing **SPIFFS** under **Tools > Partition Scheme**.
For example, the ESP32-S3 Dev Module's default 4 MB scheme includes a filesystem
partition. Upload the robot sketch again to install the selected partition table.
With Arduino CLI, the corresponding board option is `PartitionScheme=default`,
for example `esp32:esp32:esp32s3:CDCOnBoot=cdc,PartitionScheme=default`.

The recorder mounts that region as LittleFS using the standard `spiffs`
partition label. The **SPIFFS** menu label reserves a suitable region that this
sketch uses for LittleFS. The firmware distinguishes a missing partition from an
existing partition that fails to mount. Formatting initializes an existing
partition. It cannot create a missing entry in the partition table.

On a new or incompatible filesystem, turn the spin switch off and send lowercase
`f` through either board's serial monitor at 115200 baud. In Arduino IDE,
enter `f` in the Serial Monitor input and press Send/Enter. Formatting erases
that filesystem's contents. The robot reports `format queued until motor/LED task idle`
and then `format complete; filesystem ready`, or an error. The sender forwards
the final result. On startup, `filesystem ready` confirms a successful mount. If a saved file
exists, its generation and record count are reported. The firmware continues operating after a failed mount.

Firmware versions before this fix only accept `f` on the robot's serial port
and give no confirmation after a successful format. Update both sketches to
use the sender's format command and the new diagnostics.

The trigger freezes the pre-trigger history in RAM. CRC preparation runs in
small RAM-only steps while motor and radio updates continue. The snapshot can
be exported using `p` or `e` while the robot is driving. Repeated `s` commands
report that the same snapshot is retained.

Flash access requires a fast-task parking handshake, after arming finishes and
current drive commands are zero. The motor task sends zero, drains both DShot
channels and the LED RMT channel, and acknowledges the current parking request.
It then sleeps without submitting motor/LED transactions. Only after that ACK
has remained eligible for 200 ms can the recorder access the filesystem. A new
drive command revokes parking and pauses filesystem work. Delayed ACKs from an
earlier request cannot authorize a later save.
This includes a neutral controller with SWA still enabled. Zero spin throttle
can select tank mode, so center the movement stick too, or turn SWA off.
Every runtime LED transmission now belongs to the motor task, and its encoded
buffer is retained until the previous transaction completes. LEDs hold their
last frame while the task is parked. File-backed exports and formatting use
the same handshake. RAM export can run without parking the fast task. Internal flash
operations can suspend both ESP32 cores, so writing flash while driving can interrupt DShot output even when
writes are split into small pieces. RAM exports avoid those operations.

A `saved generation=... records=...` message confirms a complete validated file.
After persistence succeeds, rolling capture resumes. The `paused` count reports
events excluded while the snapshot is frozen. If power is removed before the
deferred save finishes, that RAM snapshot is lost. RAM export provides a way to
retrieve it first. A failed write retains the RAM snapshot, and `s` retries it.
A restart before persistence loses the RAM snapshot, but the previous validated
flash file remains available. Re-exporting that older generation does not
recover the lost capture. Boot prints `# boot reset_reason=...` with a readable
watchdog/panic/brownout reason to distinguish a reboot from a link outage.
Two files alternate between saves. A versioned header, exact length and CRC
identify a complete snapshot, allowing the previous complete file to be
recovered after an interrupted write. Version-1 through version-4 snapshots remain
readable and exportable. The `overwritten` count reports events evicted from the
rolling buffer before capture. A repeated generation identifies repeated export
of the same snapshot.


| Command | Robot serial | Sender serial |
| --- | --- | --- |
| `s` | Freeze recent history, queue flash save | Request robot capture |
| `p` | Print saved CSV here | Request CSV on robot serial |
| `e` | Export saved data over ESP-NOW | Request export, reconstruct and print CSV here |
| `f` | Format filesystem while idle | Request robot format |
| `x` | Cancel queued or active export | Cancel receiver and request robot cancellation |

Both CSV routes delimit a complete export with `# dump_begin` and
`# dump_end`. Save that serial output as `run.csv`. Normal boot/status text can
be included in the file. ESP-NOW export uses session IDs, record indices,
acknowledgements, retries and a final CRC check. Duplicate records are printed
once. Throttle and heartbeat processing continues during export. The sender
continues RSSI probes while receiving the dump. Serial output is drained
without blocking the sender's IBus/radio loop. Request
`e` again after an interrupted transfer. The robot reports why a request is
waiting, when radio transfer starts, when its header is queued and acknowledged,
and progress every 128 records. **Radio
CSV appears on the sender serial port**, even if `e` was entered on the robot.
Open the sender serial monitor to drain CSV and let it acknowledge records.
Use `p` for CSV on the robot instead. Radio export does not require the robot
serial port to be open.

Queued and active exports expire after 10 seconds without progress, including
a full transmit queue or unavailable serial output. The existing ACK retry
limit can fail sooner. A `radio TX queue blocked` failure means no retry of the
current packet could be queued; an ACK timeout means retries were queued but
not acknowledged. No-progress failures and `x` release the busy state
without deleting the snapshot. File cleanup waits for idle if drive resumes.
Additional capture/format requests during an export report busy. Repeated
capture requests retain the frozen snapshot until it is saved.

The CSV contains raw RSSI, measured/predicted heading RPM, heading, pending
heading error, effective RPM measurement age, feedforward steady RPM, throttle,
fast/long estimates, analysis parameters, runtime and queue drop counts. State
records also include controller/connection flags and the maximum motor-loop gap.

Version-5 keeps the 84-byte record and 168 KiB ring. Its three reused float slots
hold `heading_rad`, `heading_error_rad` (pending error, not an angle step), and
`rpm_measurement_age_ms`. `heading_rpm` aliases `control_rpm`: this includes
bounded acceleration prediction and the angular error correction rate. Motor
speed control still uses raw `rssi_rpm`. The upper bits of `hint_used` encode
`heading_status`: 8 unavailable, 9 plain dead reckoning, 10 delay compensation,
11 coasting, 12 translation noise guard, 13 translation coherent-trend tracking,
14 translation correction off/settling, 15 waiting for a fresh RPM epoch.
Bit 0 retains the feedforward-hint flag.

V4 RF phase/error/confidence and v1–v3 acceleration slots remain separately
readable/exportable. Latest CSVs append heading fields; unavailable fields are
NaN. Older RF phase and current heading diagnostics are not mixed on the plot.
Angles have an arbitrary initial offset; they do not define magnetic north.

`rpm_status` values are 0 warm-up, 1 low packet rate, 2 low RSSI variance,
3 missing sample coverage, 4 no credible period, 5 confirming a period,
6 tracking, 7 holding the previous estimate, 8 stale and 9 rejected speed jump. Older exports use -1
for unavailable status fields. The plotter shows the reason at the time cursor.
A zero RPM with low RSSI variance is an unavailable measurement. Raw RSSI still
needs visible periodic variation while the robot is spinning.

`connection_state` values are 0 network disconnected, 1 controller disconnected,
2 spin switch off, 3 accelerometer disconnected and 4 ready. The separate
`controller_connected`, `spin_enabled` and `capture_switch` columns show the
received control flags.
`kind=1` identifies RSSI events and `kind=2` identifies control/heading
events. Fields belonging to the other event type are ignored by the plotter.
RSSI `timestamp_us` comes from the Wi-Fi receive clock. State timestamps use
the local microsecond clock at the heading snapshot. In v4+ RSSI rows, `received_us`
is the Wi-Fi callback time; in state rows it is task logging time. Versions 1–3
only retain task delivery times. The plotter preserves RSSI packet intervals and
aligns their first sample to its local timestamp. Older captures cannot validate
the exact radio-to-control latency and use an approximate bridge in timing replay tests.

## Plot simulations and captures

Install Python 3.9+ and the plotting dependency.

```sh
python3 -m pip install -r tools/requirements.txt
python3 tools/plot_dump.py run.csv
python3 tools/plot_dump.py run.csv --start 0.2 --end 1.2 --save run.png
python3 tools/plot_dump.py simulation.csv run.csv --save comparison.pdf
```

The graph includes RSSI, measured/fast/long RPM, feedforward RPM, throttle,
runtime and packet rate. V5 adds wrapped heading, pending heading error,
measurement age and total heading prediction rate. V4 captures retain their
legacy RF phase diagnostics. The cursor identifies the applicable tracking mode.
Acceleration fields remain readable in old dumps and are omitted from plots.
The target is a dashed step trace. Measured `rssi_rpm` is drawn above the fast
estimate so identical values remain visible. The time cursor shows numeric
`target_rpm` and `rssi_rpm` from the latest control record. Zero placeholders in
RSSI-only rows are excluded from those traces and readouts.
Move the time slider or click a time plot to inspect normalized correlation
against lag on the right. The tracking and long comparison curves are
reconstructed from raw packets on the host, including the firmware's grid
alignment, interpolation gaps, detrending and paired variance normalization.
The used RPM and steady feedforward prior are marked as lag lines. The
long curve is recomputed at the selected analysis time, so it can differ from
the stored long estimate, which updates less frequently. Earlier samples
needed by a curve can be outside the beginning of a rolling snapshot.

`--time 1.2` selects an initial correlation cursor for interactive viewing or
a saved image. Multiple inputs overlay on the same time axes. The plotter also
accepts the binary `run0.bin` / `run1.bin` files when extracted from LittleFS,
and raw `Timestamp_US,RSSI` CSV captures from the reference project. It checks
binary CRC/length and serial export framing/counts. `--run 0` selects the first
complete export in a serial log. The default selects the last complete export.
Incomplete transfers are reported and skipped when a later complete export is
available. `--allow-incomplete` permits inspecting a partial export.

To produce a simulation on the host with the same estimator and signal
generator used by the device benchmark, run the following from the repository
root. Arguments select RPM, duration in seconds, and packets/sec before
simulated drops. An optional fourth argument supplies the ending RPM. The
throttle steps after 30% of the run and the simulated speed ramps continuously
until 70% of the run, allowing transient response to be inspected. Host runtime
values reflect the host CPU.

```sh
c++ -std=c++17 -O2 tools/rssi_simulator.cpp \
    brick/src/subsystems/rssi_rpm_estimator.cpp brick/src/subsystems/rpm_heading_tracker.cpp \
    -o /tmp/brick-sim
/tmp/brick-sim 1200 6 1000 3000 1 > simulation.csv
# Set the final argument to 0 for an identical RPM-only comparison run.
/tmp/brick-sim 1200 6 1000 3000 0 > dead.csv
python3 tools/plot_dump.py simulation.csv dead.csv --time 3
python3 -m unittest discover -s tests -p 'test_*.py'
```
