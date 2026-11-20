# Brick pilot console

A local, dependency-free page for a second heading pilot, telemetry, and run logs.
The computer connects by USB to **the sender**, not directly to the robot. Pair a
Joy-Con with your computer's Bluetooth settings (or attach a USB gamepad), then
select it in the page. The OS/browser must expose it through the Gamepad API;
this page does not implement a Bluetooth HID driver or pair devices itself.
Raw Joy-Con axis/button numbers vary by OS. Check the live input preview first.

## Start

1. Flash **both** `brick` and `sender_test` from this revision. The control and
   telemetry application packets changed; old/new firmware cannot drive together.
2. From the repository root, run:

   ```sh
   python3 -m http.server 8000 --bind 127.0.0.1 --directory tools/console
   ```

3. Open <http://localhost:8000> in desktop Chrome/Chromium or Edge. Web Serial
   needs a supporting browser and a secure context (localhost qualifies). Firefox
   can inspect imported CSV but does not currently provide this Web Serial path.
4. Close Arduino Serial Monitor and other programs using the sender's port.
   Click **Connect sender**, select its USB serial port, and wait for telemetry.
   Opening USB may reset some boards; wait for the link and main radio to recover.
5. Press a controller button if the browser hasn't exposed it yet. Select the
   heading axis/direction/gain/deadzone, then click **Enable companion heading**.
   The stick commands a heading **turn rate**, matching the existing radio turn
   input. It does not command an absolute compass heading.

The physical radio remains responsible for spin throttle, translation, SWA
arming, and its existing failsafe detection. The computer cannot enable the
motors or raise throttle. Heading has one owner: while enabled, the companion
replaces the radio's turn channel; it does not add a second correction.

A **150 ms** serial lease expires back to radio heading. Releasing the enable
button in the page, losing window focus, hiding the tab, disconnecting the
controller, or losing serial releases heading. Gamepad reconnection requires an
explicit re-enable. You may configure a hold-to-steer button as well. Leave the
page focused for the second pilot. Mapping changes release heading; only mapping
preferences are saved locally, never control enable state.

Button mappings are configurable: flip defaults to button 1, while capture/export
are disabled until assigned. Actions trigger once per press and only when
companion control is enabled. The on-screen flip/log buttons also work without a
gamepad. There are no external scripts, uploads, or cloud services.

## Flipping and separate LED faces

**Flip translation** sends an absolute orientation, repeated in fresh control
packets. Repeated packets cannot toggle it twice. It reverses the motor translation
multiplier without rotating heading, changing spin direction, or shifting the
LED vector. Tank drive retains the main radio's usual controls.

Orientation is **latched on the sender**, including after a browser disconnect or
an RF outage; it defaults to upright on sender reboot. A new page connection
starts with "leave unchanged" and reads the sender's current orientation. Check
the actual robot orientation after either board reboots. Robot telemetry confirms
the received orientation separately from the sender's requested orientation.

Separate the two strips' data wires before using two colors. By default:

| Output | Data GPIO | Heading digits | Translation vector |
| --- | --- | --- | --- |
| Top | 1 | Blue | Yellow |
| Bottom | 4 | Red | Cyan |

Edit `NEOPIXEL_PIN` and `BOTTOM_NEOPIXEL_PIN` in `brick/config.h` if your board
needs different pins. Each output is `NUM_LEDS` pixels (currently 8); retain common
ground and the existing power arrangement. The row order retains the merged
body/display configuration. Status patterns remain shared; ready idle is white on top and magenta below. The two LED
instances have independent RMT buffers, and flash saving parks/drains **both**
outputs plus both motors before writing flash. Each output uses one RMT memory
block so the ESP32-S3 has room for all four TX channels.

## Telemetry and logs

The robot sends versioned 64-byte telemetry at up to 20 Hz; the sender emits JSON
lines prefixed `@T ` at up to 10 Hz when it isn't exporting a dump. The dashboard
shows measured/target RPM, throttle, packet rate, RSSI, correlation, effective RPM
age, control freshness/owner, orientation, connection state, software queue drops,
MAC send failures, maximum send completion time, and the configured RF channel.
Stale/disconnected telemetry is explicitly marked; displayed RPM is not an
independent speed sensor. The new telemetry contains no acceleration/radius data.

Use **Capture RAM + save** (`s`) to freeze history, then stop motor outputs so
flash saving can proceed. **Export to console** (`e`) transfers the retained run
via the sender. CSV is isolated from telemetry/status output even under serial
backpressure. The page enables Copy/Download CSV only after `dump_end` and a
matching record count. Interrupted captures remain marked incomplete. Up to six
captures are retained in page memory (4 MiB each); download before closing or
refreshing the page.

Import existing robot or simulator CSV with **Import CSV**. The log inspector
shows RPM, RSSI, throttle, and a time cursor. The normalized lag correlation is
reconstructed locally with the firmware's grid/gap masking, detrending, variance
thresholds, and logged comparison/search parameters. JavaScript double precision
can differ slightly from firmware float rounding. The existing Python plotter is
still available, and dump binary/CSV format remains v5 (legacy acceleration
columns are retained only as compatibility placeholders in old dump tools).

The Serial terminal offers `s/p/e/f/x/a/t/n`, with descriptions. `p` prints CSV on
the **robot's own USB**, so use `e` to collect it here. Formatting (`f`) requires an
explicit confirmation because it deletes saved captures. Terminal history is
bounded to the latest 800 non-telemetry lines; the capture's CSV is stored
separately and is not truncated with that terminal history.

## Competition RF behavior

- The sender wakes its application task on send completion rather than always
  losing the rest of a 1 ms polling tick. Wi-Fi callbacks only update counters and
  notify; they never send recursively or run estimation. An explicit periodic
  yield still gives the idle task time.
- Controls have a dedicated latest-value TX/RX path, ahead of bulk telemetry/log
  data. There is no FIFO of stale joystick positions. Unsent controls expire after
  100 ms; heartbeat traffic retains its priority.
- Boot session and increasing sequence numbers reject duplicate/out-of-order
  controls. Receiver queue age counts against the **100 ms** control watchdog.
  The motor task independently checks that deadline even if the general loop
  stalls. Old controls/telemetry from earlier firmware are rejected.
- Loss of usable RPM suspends melty translation below the existing minimum
  melting RPM. Feedforward spin remains available while the estimator reacquires;
  a lost primary control stream stops the motors. RPM estimation, heading
  compensation gains, and feedforward tuning are unchanged.
- Both peers use the conservative 6 Mbps rate and disable Wi-Fi sleep. Discovery
  uses the default rate. No live scans or automatic channel hops interrupt an
  armed robot.

Before an event, choose a tested clear channel in `common/radio_config.h` and
flash **both** ESPs. Channels 1/6/11 are useful starting candidates, but use the
actual venue conditions. Optionally set `ROBOT_MAC`/`SENDER_MAC` there to avoid
accidental discovery pairing with another NowPair device. All-zero MACs retain
automatic discovery. These address filters are **not encryption/authentication**;
they prevent accidental cross-pairing, not deliberate spoofing.

Test packet rate, long gaps, fail counts, and control loss with the robot rotating
and translating in its competition body. A faster PHY rate can reduce airtime but
also lose weak-angle samples; don't use throughput alone to select it. This PR
provides bounded scheduling and diagnostics, not a guarantee of operation under
arbitrary interference. The RPM estimator's 500 packets/s gate remains unchanged.

## Development checks

Run the root README's native and Python checks, plus:

```sh
node --test tests/console_test.mjs
# Optional browser test (fake USB/gamepad, no robot required):
python3 -m pip install playwright
python3 -m playwright install chromium
python3 tests/console_browser_smoke.py
```

The browser test exercises USB framing, heading ownership, held-button flip
edges, capture, telemetry display, complete-only CSV downloads, log import,
controller/focus/serial disconnects, and mobile layout. Browser correlation is
cross-checked against the existing NumPy reference on recorded logs by
`tests/test_console_correlation.py`.

Browser API references:
[Web Serial](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API),
[Gamepad API](https://developer.mozilla.org/en-US/docs/Web/API/Gamepad_API).
