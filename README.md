# Brick

## RSSI RPM estimation

Flash the robot with `brick` and the controller ESP with `sender_test`. The
sender fills spare ESP-NOW transmit slots with two-byte RSSI probes and schedules
throttle packets every 20 ms. Heartbeat and throttle traffic have priority over
probes. The sender services the link every millisecond on ESP32-S3 and uses
6 Mbps for unicast. The actual receive rate depends on the radio link.

RSSI is the robot's RPM source. The accelerometer still supplies orientation
and raw acceleration logging. Watch serial output for `rpm`, `fast_rpm`,
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

The steady RPM expected from the calibrated feedforward is a soft period hint.
It is enabled after at least 600 ms of stable throttle, several stable long
estimates, and agreement between the windows. It can resolve two-lobe ambiguity
among near-equal correlation peaks within 20% of the calibrated RPM. Signal
evidence is required. Throttle does not directly replace measured RPM.

The implementation estimates speed from RSSI periodicity. Its sampling and
autocorrelation approach was inspired by
[TGower/rotation_sensing](https://github.com/TGower/rotation_sensing).

### Build and test

The sketches were built for ESP32-S3 with Arduino-ESP32 3.3.7. Update both
robot and sender firmware for version-2 dump exports. The desktop plotter
accepts both version-1 and version-2 dumps.

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc brick
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc sender_test

c++ -std=c++17 -O2 -Ibrick/src/subsystems tests/rssi_rpm_estimator_test.cpp \
    brick/src/subsystems/rssi_rpm_estimator.cpp -o /tmp/brick-rssi-test
/tmp/brick-rssi-test
c++ -std=c++17 -O2 -Ibrick/src/subsystems tests/rssi_tracking_test.cpp \
    brick/src/subsystems/rssi_rpm_estimator.cpp -o /tmp/brick-tracking-test
/tmp/brick-tracking-test
```

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
mean and maximum sample/update/analysis times, an estimated compute load at the
simulated packet rate, and free heap and stack bytes. The matches fraction
counts post-warm-up estimates within 2.5% of the expected RPM. Sample generation,
serial output and deliberate yields are outside the timed estimator calls.
This sketch initializes the estimator and serial port. The robot's live dumps
include `estimator_us` to measure update time with normal robot/Wi-Fi activity.

Send `d` to print a two-second 3000 RPM simulation as CSV with a `true_rpm`
column and device timings. Send `r` for a continuous-phase 1200-to-3000 RPM
ramp with a step in throttle before the mechanical speed settles. Its
acceleration columns are an ideal centrifugal reference at a 4.846 cm radius. Save the serial output and use the plotter below.

## Save a robot run

The robot keeps the latest 2048 events in RAM, including every delivered RSSI
sample and each accelerometer/control update. At about 1000 RSSI packets/sec
this covers roughly 1.8 seconds before the trigger. The version-2 buffer uses
160 KiB RAM.

Toggle controller SWB from high to low to save the recent history to flash.
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
that filesystem's contents. The robot reports `formatting filesystem; please wait`
and then `format complete; filesystem ready`, or an error. The sender forwards
the final result. On startup, `filesystem ready; s saves recent history` confirms
a successful mount. The firmware continues operating after a failed mount.

Firmware versions before this fix only accept `f` on the robot's serial port
and give no confirmation after a successful format. Update both sketches to
use the sender's format command and the new diagnostics.

Saving freezes capture and copies up to 16 events per general-update loop.
Capture resumes after the save finishes, usually over a second for a full
buffer. The `paused` count reports events omitted while saving. Flash operations
can add latency to control and radio tasks, so compare live timing and drop
counts during a bench run. Two files alternate between saves. A versioned
header, exact length and CRC identify a complete snapshot, allowing the previous
complete file to be recovered after an interrupted write. The latest complete
snapshot is selected after reboot. Version-1 snapshots can still be read and
exported by the updated robot and sender. The `overwritten` count reports older events
evicted from the rolling buffer.

| Command | Robot serial | Sender serial |
| --- | --- | --- |
| `s` | Save recent history | Request robot save |
| `p` | Print saved CSV here | Request CSV on robot serial |
| `e` | Export saved data over ESP-NOW | Request export, reconstruct and print CSV here |
| `f` | Format filesystem with spin switch off | Request robot format |

Both CSV routes delimit a complete export with `# dump_begin` and
`# dump_end`. Save that serial output as `run.csv`. Normal boot/status text can
be included in the file. ESP-NOW export uses session IDs, record indices,
acknowledgements, retries and a final CRC check. Duplicate records are printed
once. Throttle and heartbeat processing continues during export. The sender
pauses RSSI probes while receiving the dump and resumes afterward. Request
`e` again after an interrupted transfer. Additional save/export requests while
a transfer or save is running report busy.

The CSV contains raw RSSI, accelerometer values in g, RSSI/control RPM, fast
and long estimates, steady feedforward RPM, packet rate, commanded motor
throttle, per-update estimator runtime, and queue drop counts. Version-2 records
also identify the selected comparison length, lag search limits, analysis clock,
mode and hint state. Full correlation arrays are calculated on the host.
`kind=1` identifies RSSI events and `kind=2` identifies accelerometer/control
events. Fields belonging to the other event type are ignored by the plotter.
RSSI `timestamp_us` comes from the Wi-Fi receive clock. State timestamps use
the local microsecond clock. `received_us` records task delivery time. The
plotter preserves RSSI packet intervals and aligns the first RSSI event with
its delivery time, so the cross-sensor alignment includes queue delivery delay.

## Plot simulations and captures

Install Python 3.9+ and the plotting dependency.

```sh
python3 -m pip install -r tools/requirements.txt
python3 tools/plot_dump.py run.csv
python3 tools/plot_dump.py run.csv --start 0.2 --end 1.2 --save run.png
python3 tools/plot_dump.py simulation.csv run.csv --save comparison.pdf
```

The graph includes raw RSSI, calculated/fast/long RPM, the feedforward steady
RPM, motor throttle, raw acceleration, estimator update time and packet rate.
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
    brick/src/subsystems/rssi_rpm_estimator.cpp -o /tmp/brick-sim
/tmp/brick-sim 1200 3 1000 3000 > simulation.csv
python3 tools/plot_dump.py simulation.csv --time 1.5
python3 -m unittest discover -s tests -p 'test_*.py'
```
