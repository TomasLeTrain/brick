# Brick

## RSSI RPM estimation

Flash the robot with `brick` and the controller ESP with `sender_test`. The
sender fills spare ESP-NOW transmit slots with two-byte RSSI probes and schedules
throttle packets every 20 ms. Heartbeat and throttle traffic have priority over
probes. The sender services the link every millisecond on ESP32-S3 and uses
6 Mbps for unicast. The actual receive rate depends on the radio link.

Watch the robot's serial output for `accel_rpm`, `rssi_rpm`, and `rssi_hz`.
The RSSI estimate covers roughly 400 to 3,300 RPM. It uses about 400 ms of
receive history, requires at least 500 packets/sec across that history, and
checks for agreement across three updates about 50 ms apart before reporting
an estimate. `rssi_rpm:0` means the estimate is unavailable. `rssi_hz` is
reported once the sample window fills. The sampling and autocorrelation
approach was inspired by [TGower/rotation_sensing](https://github.com/TGower/rotation_sensing).

To try RSSI during drive control, set `USE_RSSI_RPM_FOR_CONTROL` in
`brick/config.h` to `true` after comparing both readings on the robot. Drive
control falls back to accelerometer RPM when RSSI is unavailable. The
accelerometer also provides the upright reading. The estimator assumes the
RSSI waveform repeats once per revolution. An antenna setup with multiple
identical lobes per turn can produce a multiple of the true RPM. The history
window adds lag during spin-up and spin-down.

### Build and test

The sketches were built for ESP32-S3 with Arduino-ESP32 3.3.7.

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc brick
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc sender_test

c++ -std=c++17 -O2 -Ibrick/src/subsystems tests/rssi_rpm_estimator_test.cpp \
    brick/src/subsystems/rssi_rpm_estimator.cpp -o /tmp/brick-rssi-test
/tmp/brick-rssi-test
```
