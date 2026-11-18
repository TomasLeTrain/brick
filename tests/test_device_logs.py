"""Replay real captures and adversarial periods through the firmware estimator."""

from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import math

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


if __name__ == "__main__":
    unittest.main()
