"""Real RSSI replay: waveform monitoring, acceleration assist, steady DR."""

from pathlib import Path
import math
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import plot_dump


class PhaseLogTests(unittest.TestCase):
    def test_real_waveforms_support_phase_observations_and_fallback(self):
        with tempfile.TemporaryDirectory() as directory:
            program = Path(directory) / "phase"
            subsystem = ROOT / "brick/src/subsystems"
            subprocess.run(["c++", "-std=c++17", "-O2", "-I" + str(subsystem),
                            str(ROOT / "tests/phase_log_probe.cpp"),
                            str(subsystem / "rssi_rpm_estimator.cpp"),
                            str(subsystem / "rssi_phase_tracker.cpp"), "-o", str(program)], check=True)
            for name in ("log7", "log8", "log9", "log10"):
                with self.subTest(log=name):
                    rows = plot_dump.load(ROOT / "logs" / (name + ".csv"))[-1].rows
                    raw = [r for r in rows if r["kind"] == 1]
                    # Estimate one constant bridge from the least-delayed old
                    # task delivery. New v4 logs retain the actual callback time.
                    offset = min((int(r["received_us"]) - int(r["timestamp_us"]) + 2**31) %
                                 2**32 - 2**31 for r in raw)
                    source, state_times = [], []
                    for t, row in zip(plot_dump.times(rows), rows):
                        if row["kind"] == 1:
                            radio = int(row["timestamp_us"])
                            source.append(f"r {radio} {(radio + offset) & 0xffffffff} {int(row['rssi_dbm'])}\n")
                        else:
                            source.append(f"u {int(row['received_us'])} {row['throttle']} {row['target_rpm']}\n")
                            state_times.append(t)
                    output = subprocess.check_output([str(program)], input="".join(source), text=True)
                    values = [list(map(float, line.split())) for line in output.splitlines()]
                    self.assertTrue(all(math.isfinite(v) for row in values for v in row))
                    tail = [v for t, v in zip(state_times, values) if t >= .8]
                    if name == "log8":
                        self.assertGreater(sum(v[3] == 5 for v in tail), 0)
                        self.assertGreater(sum(v[3] == 2 for v in tail), 10)
                    else:
                        self.assertGreater(sum(v[3] == 5 for v in tail), len(tail) * .1)
                    self.assertTrue(all(-math.pi <= v[0] <= math.pi for v in tail))
                    self.assertTrue(all(0 <= v[2] <= 1 for v in tail))
                    self.assertTrue(all(v[2] >= .45 for v in tail if v[3] == 2))

    def test_logs_12_to_14_cannot_steer_heading_at_steady_throttle(self):
        with tempfile.TemporaryDirectory() as directory:
            program = Path(directory) / "phase"
            subsystem = ROOT / "brick/src/subsystems"
            subprocess.run(["c++", "-std=c++17", "-O2", "-I" + str(subsystem),
                            str(ROOT / "tests/phase_log_probe.cpp"),
                            str(subsystem / "rssi_rpm_estimator.cpp"),
                            str(subsystem / "rssi_phase_tracker.cpp"), "-o", str(program)], check=True)
            for name in ("log12", "log13", "log14"):
                with self.subTest(log=name):
                    rows = plot_dump.load(ROOT / "logs" / (name + ".csv"))[-1].rows
                    source, states = [], []
                    for row in rows:
                        if row["kind"] == 1:
                            source.append(f"r {int(row['timestamp_us'])} {int(row['received_us'])} {int(row['rssi_dbm'])}\n")
                        else:
                            # Replay recorded RPM to isolate the phase policy
                            # from RPM warmup and absent pre-capture history.
                            source.append(f"p {int(row['timestamp_us'])} {row['rssi_rpm']} {row['throttle']} {row['target_rpm']} 0\n")
                            states.append(row)
                    output = subprocess.check_output([str(program)], input="".join(source), text=True)
                    values = [list(map(float, line.split())) for line in output.splitlines()]
                    self.assertEqual(len(values), len(states))
                    self.assertTrue(all(math.isfinite(x) for v in values for x in v))
                    self.assertTrue(all(v[3] != 2 for v in values))
                    self.assertTrue(all(abs(v[0] - v[5]) < 1e-6 for v in values))
                    self.assertTrue(all(abs(v[1] - r['rssi_rpm']) < .001
                                        for v, r in zip(values, states)))
                    # These captures include appreciable observer corrections
                    # with the previous firmware, even at steady throttle.
                    old_steps = []
                    for a, b in zip(states, states[1:]):
                        dt = (int(b['timestamp_us']) - int(a['timestamp_us'])) % 2**32 / 1e6
                        error = b['phase_rad'] - a['phase_rad'] - a['phase_rpm'] * math.tau / 60 * dt
                        old_steps.append(abs((error + math.pi) % math.tau - math.pi))
                    self.assertGreater(max(old_steps), .1)


if __name__ == "__main__":
    unittest.main()
