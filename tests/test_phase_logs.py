"""Real RSSI shape/quality checks; old logs lack exact Wi-Fi callback timing."""

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
                    self.assertGreater(sum(v[3] == 2 for v in tail), len(tail) * .25)
                    self.assertTrue(all(-math.pi <= v[0] <= math.pi for v in tail))
                    self.assertTrue(all(0 <= v[2] <= 1 for v in tail))
                    self.assertTrue(all(v[2] >= .35 for v in tail if v[3] == 2))


if __name__ == "__main__":
    unittest.main()
