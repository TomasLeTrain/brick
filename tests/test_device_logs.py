"""Replay real captures and adversarial periods through the firmware estimator."""

from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import math
import random

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import plot_dump


class DeviceLogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.program = Path(cls.directory.name) / "replay"
        subprocess.run([
            "c++", "-std=c++17", "-O2", "-I" + str(ROOT / "brick/src/subsystems"),
            str(ROOT / "tests/replay_log.cpp"),
            str(ROOT / "brick/src/subsystems/rssi_rpm_estimator.cpp"),
            "-o", str(cls.program),
        ], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def replay(self, source):
        result = subprocess.check_output([str(self.program)], input="".join(source), text=True)
        return [(float(line.split()[0]), int(line.split()[1])) for line in result.splitlines()]

    def replay_capture(self, name, restore_at=None, hint=True):
        dump = plot_dump.load(ROOT / "logs" / name)[-1]
        source, states = [], []
        restored = False
        for t, row in zip(plot_dump.times(dump.rows), dump.rows):
            if row["kind"] == 1:
                source.append(f"r {int(row['timestamp_us'])} {int(row['rssi_dbm'])}\n")
            else:
                if restore_at is not None and t >= restore_at and not restored:
                    # Rolling logs omit the earlier history that produced this
                    # lock. Restore its recorded fast estimate after warming up
                    # the real sample history, rather than assuming cold start
                    # can reproduce an event that is outside the capture.
                    source.append(f"l {int(row['received_us'])} {row['rssi_rpm']}\n")
                    restored = True
                source.append(f"u {int(row['received_us'])} "
                              f"{row['throttle']} {row['target_rpm'] if hint else 0}\n")
                states.append((t, row))
        return states, self.replay(source)

    def test_flat_capture_reports_low_variance(self):
        self.assertEqual((ROOT / "logs/log3.csv").read_bytes(),
                         (ROOT / "logs/log4.csv").read_bytes())
        states, output = self.replay_capture("log3.csv")
        self.assertEqual(sum(row["connected"] == 1 for _, row in states), 2)
        self.assertEqual(len(output), len(states))
        self.assertTrue(all(rpm == 0 for rpm, _ in output))
        self.assertTrue(all(status == 2 for _, status in output[50:]))

    def test_log8_acceleration_remains_responsive(self):
        states, output = self.replay_capture("log8.csv")
        errors = [abs(rpm - row["rssi_rpm"]) for (t, row), (rpm, _) in zip(states, output)
                  if t >= .65]
        self.assertLess(sum(errors) / len(errors), 5)
        self.assertLess(max(errors), 25)
        self.assertGreater(output[-1][0], 1750)
        self.assertTrue(any(.9 <= t <= 1.3 and rpm < row["target_rpm"] - 60
                            for (t, row), (rpm, _) in zip(states, output)))

    def test_log9_recovers_recorded_fifth_repeat_lock(self):
        for hint in (True, False):
            with self.subTest(hint=hint):
                states, output = self.replay_capture("log9.csv", restore_at=.7, hint=hint)
                injected = next(t for t, _ in states if t >= .7)
                recovered = next(t for (t, _), (rpm, _) in zip(states, output)
                                 if t > injected and rpm > 1800)
                self.assertLess(recovered - injected, .4)
                self.assertTrue(any(rpm == 0 for (t, _), (rpm, _) in zip(states, output)
                                    if injected < t < recovered))
                self.assertTrue(all(1900 < rpm < 2300 for (t, _), (rpm, _) in zip(states, output)
                                    if t >= recovered))

    def test_first_supported_peak_precedes_stronger_fifth_repeat(self):
        source = []
        for t in range(0, 2500000, 1000):
            turns = 2100 * t / 60000000
            amplitude = 16 if int(turns) % 5 == 4 else 8
            source.append(f"r {t} {round(-64 + amplitude * math.sin(2 * math.pi * turns))}\n")
            if t % 10000 == 0:
                source.append(f"u {t} 0 0\n")
        output = self.replay(source)
        self.assertTrue(all(abs(rpm - 2100) < 105 for rpm, _ in output[80:]))

    def test_unequal_half_rotation_lobes_keep_full_period(self):
        source = []
        for t in range(0, 2000000, 1000):
            phase = 2 * math.pi * 1500 * t / 60000000
            source.append(f"r {t} {round(-64 + 3 * math.sin(phase) + 10.5 * math.cos(2 * phase))}\n")
            if t % 10000 == 0:
                source.append(f"u {t} 0 0\n")
        self.assertTrue(all(abs(rpm - 1500) < 40 for rpm, _ in self.replay(source)[80:]))

    def test_abrupt_stop_expires_instead_of_holding_stale_speed(self):
        source = []
        for t in range(0, 2000000, 1000):
            value = round(-64 + 10 * math.sin(2 * math.pi * 2100 * t / 60000000)) if t < 1000000 else -64
            source.append(f"r {t} {value}\n")
            if t % 10000 == 0:
                source.append(f"u {t} 0.3 2100\n")
        output = self.replay(source)
        self.assertGreater(output[90][0], 1900)
        self.assertTrue(all(rpm == 0 for rpm, _ in output[150:]))

    def test_log22_recovers_despite_packet_loss_without_a_hint(self):
        for hint in (True, False):
            with self.subTest(hint=hint):
                states, output = self.replay_capture("logs22.csv", hint=hint)
                self.assertTrue(all(row["rssi_rpm"] == 0 for _, row in states))
                # Most recorded updates were rejected by the old 500 Hz gate.
                self.assertGreater(sum(row["rpm_status"] == 1 for _, row in states), 300)
                settled = [rpm for (t, _), (rpm, _) in zip(states, output) if t >= .6]
                self.assertGreater(sum(rpm > 0 for rpm in settled), .6 * len(settled))
                recovered = [rpm for rpm in settled if rpm > 0]
                # Signal-supported band, not independently measured RPM truth.
                self.assertTrue(all(1550 < rpm < 2750 for rpm in recovered))
                self.assertGreater(output[-1][0], 2300)

    def test_log22_packet_timing_with_known_periods_and_no_period(self):
        rows = plot_dump.load(ROOT / "logs/logs22.csv")[-1].rows
        packets = [int(row["timestamp_us"]) for row in rows if row["kind"] == 1]
        times = [t - packets[0] for t in packets]
        for rpm in (0, 1200, 2400, 3000):
            with self.subTest(rpm=rpm):
                rng = random.Random(111)
                events = []
                for t in times:
                    value = round(-64 + 8 * math.sin(math.tau * rpm * t / 60000000)
                                  + rng.randint(-2, 2))
                    events.append((t, 0, f"r {t} {value}\n"))
                for t in range(0, times[-1] + 1, 10000):
                    # Wrong hint must not manufacture its requested 3300 RPM.
                    events.append((t, 1, f"u {t} .3 3300\n"))
                output = self.replay(e[2] for e in sorted(events))[100:]
                recovered = [value for value, _ in output if value > 0]
                if rpm:
                    self.assertGreater(len(recovered), .75 * len(output))
                    self.assertTrue(all(abs(value - rpm) < .05 * rpm for value in recovered))
                else:
                    self.assertFalse(recovered)


if __name__ == "__main__":
    unittest.main()
