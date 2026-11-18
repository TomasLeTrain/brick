"""Replay the real all-zero captures through the firmware estimator."""

from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import plot_dump


class DeviceLogTests(unittest.TestCase):
    def test_flat_capture_reports_low_variance(self):
        with tempfile.TemporaryDirectory() as directory:
            program = Path(directory) / "replay"
            subprocess.run([
                "c++", "-std=c++17", "-O2", "-I" + str(ROOT / "brick/src/subsystems"),
                str(ROOT / "tests/replay_log.cpp"),
                str(ROOT / "brick/src/subsystems/rssi_rpm_estimator.cpp"),
                "-o", str(program),
            ], check=True)
            first = ROOT / "logs/log3.csv"
            self.assertEqual(first.read_bytes(), (ROOT / "logs/log4.csv").read_bytes())
            rows = plot_dump.load(first)[0].rows
            states = [r for r in rows if r["kind"] == 2]
            self.assertEqual(sum(r["connected"] == 1 for r in states), 2)
            source = []
            for row in rows:
                if row["kind"] == 1:
                    source.append(f"r {int(row['timestamp_us'])} {int(row['rssi_dbm'])}\n")
                else:
                    source.append(f"u {int(row['received_us'])} "
                                  f"{row['throttle']} {row['target_rpm']}\n")
            result = subprocess.check_output([str(program)], input="".join(source), text=True)
            output = [line.split() for line in result.splitlines()]
            self.assertEqual(len(output), len(states))
            self.assertTrue(all(float(rpm) == 0 for rpm, _ in output))
            self.assertTrue(all(int(status) == 2 for _, status in output[50:]))


if __name__ == "__main__":
    unittest.main()
