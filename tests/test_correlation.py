"""Compare desktop correlation curves against the firmware C++ implementation."""

import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import rssi_correlation


class CorrelationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.directory.cleanup)
        cls.probe = Path(cls.directory.name) / "probe"
        subprocess.run([
            "c++", "-std=c++17", "-O2", "-I" + str(ROOT / "brick/src/subsystems"),
            str(ROOT / "tests/correlation_probe.cpp"),
            str(ROOT / "brick/src/subsystems/rssi_rpm_estimator.cpp"),
            "-o", str(cls.probe),
        ], check=True)

    def test_curves_match_native_windows_and_gap_masks(self):
        for rpm in (1200, 3000):
            for gaps in (False, True):
                rows = []
                origin = 0xFFFF0000
                for i in range(2000):
                    if gaps and (1750 <= i < 1762 or 1950 <= i < 1962):
                        continue
                    t = i * 1000 + i % 7 * 31
                    value = round(-64 + 8 * math.sin(2 * math.pi * rpm * t / 60000000)
                                  + 2 * math.cos(i * 1.7))
                    rows.append({"kind": 1, "timestamp_us": (origin + t) & 0xFFFFFFFF,
                                 "rssi_dbm": value})
                source = "".join(f"{r['timestamp_us']} {r['rssi_dbm']}\n" for r in rows)
                output = subprocess.check_output([str(self.probe)], input=source, text=True)
                lines = output.splitlines()
                timestamp, samples, first, last = map(int, lines[0].split())
                rows.append({"kind": 2, "analysis_timestamp_us": timestamp})
                grid = rssi_correlation.grid_at(rows, len(rows) - 1)
                native = np.array([[float(x) for x in line.split()] for line in lines[1:]])
                for column, shape in ((1, (samples, first, last)), (2, (250, 9, 155))):
                    curve = rssi_correlation.correlation_curve(grid, *shape)
                    expected = native[:, column]
                    np.testing.assert_array_equal(np.isfinite(curve), expected != -2)
                    np.testing.assert_allclose(curve[np.isfinite(curve)],
                                               expected[np.isfinite(curve)],
                                               atol=.002, rtol=.002)


if __name__ == "__main__":
    unittest.main()
