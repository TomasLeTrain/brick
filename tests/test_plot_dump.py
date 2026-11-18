"""Regressions for binary integrity, serial framing and independent clock wrap."""

from pathlib import Path
import struct
import sys
import tempfile
import unittest
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import plot_dump


class PlotDumpTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "run.csv"

    def test_serial_runs_and_record_count(self):
        self.path.write_text(
            "booting\n# dump_begin count=2\nkind,timestamp_us,rssi_dbm\n"
            "1,10,-61\n# dump saving\n1,20,-62\n# dump_end\n"
            "# dump_begin count=1\nTimestamp_US,RSSI\n30,-63\n# dump_end\n"
        )
        runs = plot_dump.load(self.path)
        self.assertEqual(len(runs), 2)
        self.assertEqual(len(runs[0].rows), 2)
        self.assertEqual(runs[1].rows[0]["rssi_dbm"], -63)
        self.path.write_text(self.path.read_text().replace("count=2", "count=3"))
        with self.assertWarnsRegex(UserWarning, "expected 3"):
            recovered = plot_dump.load(self.path)
        self.assertEqual(len(recovered), 1)
        self.path.write_text(
            "# dump_begin count=3\nTimestamp_US,RSSI\n10,-60\n# dump_end\n"
        )
        with self.assertRaisesRegex(ValueError, "expected 3"):
            plot_dump.load(self.path)

    def test_partial_export_requires_opt_in(self):
        self.path.write_text(
            "# dump_begin count=2\nkind,timestamp_us,rssi_dbm\n1,10,-61\n"
        )
        with self.assertRaisesRegex(ValueError, "incomplete"):
            plot_dump.load(self.path)
        self.assertEqual(len(plot_dump.load(self.path, True)[0].rows), 1)
        self.path.write_text(self.path.read_text() +
                             "# dump_begin count=1\nTimestamp_US,RSSI\n20,-62\n# dump_end\n")
        with self.assertWarnsRegex(UserWarning, "incomplete export"):
            recovered = plot_dump.load(self.path)
        self.assertEqual(recovered[0].rows[0]["rssi_dbm"], -62)

    def test_binary_integrity(self):
        payload = struct.pack("<II8fIIIhBB", 0xFFFFFFF0, 12000,
                              1, 2, 3, 1500, 1499, 1499, 900, .3, 400, 0, 2, -61, 2, 1)
        header = struct.pack("<8I", 0x31524442, 1, 12, 1, 56,
                             zlib.crc32(payload), 5, 20)
        self.path.write_bytes(header + payload)
        dump = plot_dump.load(self.path)[0]
        self.assertEqual(dump.rows[0]["estimator_us"], 400)
        self.assertEqual(dump.rows[0]["rssi_rpm"], 1499)
        self.path.write_bytes(header + payload[:-1] + bytes([payload[-1] ^ 1]))
        with self.assertRaisesRegex(ValueError, "CRC mismatch"):
            plot_dump.load(self.path)
        self.path.write_bytes(header + payload[:-2])
        with self.assertRaisesRegex(ValueError, "truncated"):
            plot_dump.load(self.path)

    def test_two_clock_rollover_and_batch_delivery(self):
        rows = [
            {"kind": 1, "timestamp_us": 0xFFFFFF00, "received_us": 0xFFFFFF00},
            {"kind": 1, "timestamp_us": 0x000002E8, "received_us": 0xFFFFFF00},
            {"kind": 2, "timestamp_us": 0x000006D0, "received_us": 0x00000734},
        ]
        self.assertEqual(plot_dump.times(rows), [0, .001, .002])

    def test_numeric_truncation_rejected(self):
        self.path.write_text("timestamp_us,rssi\n0,-60\n1\n")
        with self.assertRaisesRegex(ValueError, "cells"):
            plot_dump.load(self.path)


    def test_version_two_binary_layout(self):
        values = dict.fromkeys(plot_dump.BINARY_FIELDS, 0)
        values.update(timestamp_us=10000, received_us=10050, kind=2,
                      target_rpm=2500, rssi_rpm=2450, fast_rpm=2450, long_rpm=2400,
                      correlation=.9, comparison_samples=72, min_lag=18, max_lag=31)
        payload = plot_dump.RECORD_V2.pack(*list(values.values())[:25])
        self.assertEqual(len(payload), 80)
        self.path.write_bytes(struct.pack("<8I", 0x31524442, 2, 1, 1, 80,
                                          zlib.crc32(payload), 0, 0) + payload)
        row = plot_dump.load(self.path)[0].rows[0]
        self.assertEqual(row["target_rpm"], 2500)
        self.assertEqual(row["fast_rpm"], 2450)
        self.assertEqual(row["comparison_samples"], 72)
        self.assertNotIn("accel_rpm", row)


    def test_version_three_control_and_rpm_diagnostics(self):
        values = dict.fromkeys(plot_dump.BINARY_FIELDS, 0)
        values.update(timestamp_us=10000, received_us=10050, kind=2,
                      connected=0x80 | 1 | (4 << 1) | 0x10 | 0x20,
                      tracking_mode=2 << 2, motor_gap_us=1200)
        payload = plot_dump.RECORD.pack(*values.values())
        self.path.write_bytes(struct.pack("<8I", 0x31524442, 3, 1, 1, 84,
                                          zlib.crc32(payload), 0, 0) + payload)
        row = plot_dump.load(self.path)[0].rows[0]
        self.assertEqual(row["connected"], 1)
        self.assertEqual(row["connection_state"], 4)
        self.assertTrue(row["controller_connected"])
        self.assertTrue(row["spin_enabled"])
        self.assertFalse(row["capture_switch"])
        self.assertEqual(row["rpm_status"], 2)
        self.assertEqual(row["motor_gap_us"], 1200)

    def test_version_four_phase_reuses_slots_and_decodes_status(self):
        values = dict.fromkeys(plot_dump.BINARY_FIELDS, 0)
        values.update(timestamp_us=10000, received_us=10050, kind=2,
                      connected=0x89, ax_g=.6, ay_g=.03, az_g=.8,
                      control_rpm=2101, hint_used=1 | (2 << 1))
        payload = plot_dump.RECORD.pack(*values.values())
        self.path.write_bytes(struct.pack("<8I", 0x31524442, 4, 1, 1, 84,
                                          zlib.crc32(payload), 0, 0) + payload)
        row = plot_dump.load(self.path)[0].rows[0]
        self.assertAlmostEqual(row["phase_rad"], .6, places=5)
        self.assertEqual(row["phase_rpm"], 2101)
        self.assertEqual(row["phase_status"], 2)
        self.assertEqual(row["hint_used"], 1)
        self.assertNotIn("ax_g", row)

    def test_interactive_cursor_updates_correlation(self):
        import math
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        rows = []
        for t in range(0, 1000000, 1000):
            rows.append(dict(kind=1, timestamp_us=t, received_us=t,
                             rssi_dbm=round(-64 + 10 * math.sin(t * 2 * math.pi / 20000))))
            if t % 10000 == 0:
                rows.append(dict(kind=2, timestamp_us=t, received_us=t, rssi_rpm=3000,
                                 throttle=.44, analysis_timestamp_us=t,
                                 comparison_samples=60, min_lag=15, max_lag=25))
        figure = plot_dump.plot([("capture", plot_dump.Dump(rows))], selected_time=.8)
        self.addCleanup(plt.close, figure)
        inspector = figure._dump_inspector
        inspector.slider.set_val(.9)
        self.assertEqual(inspector.cursors[0].get_xdata(), [.9, .9])
        self.assertTrue(any("tracking" in line.get_label()
                            for line in inspector.curve_axis.lines))

    def test_log7_target_uses_control_rows_and_hides_acceleration(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        path = Path(__file__).resolve().parents[1] / "logs/log7.csv"
        dump = plot_dump.load(path)[-1]
        states = [row for row in dump.rows if row["kind"] == 2]
        figure = plot_dump.plot([("log7", dump)], selected_time=.8)
        self.addCleanup(plt.close, figure)
        inspector = figure._dump_inspector
        rpm_axis = next(axis for axis in inspector.axes if axis.get_ylabel() == "RPM")
        series = {line.get_label(): line for line in rpm_axis.lines}
        self.assertEqual(list(series["target_rpm"].get_ydata()), [1332] * len(states))
        self.assertEqual(list(series["rssi_rpm"].get_ydata()),
                         list(series["fast_rpm"].get_ydata()))
        self.assertFalse(any("Acceleration" in axis.get_ylabel() for axis in inspector.axes))
        self.assertTrue(any("target_rpm=1332.0 RPM" in text.get_text()
                            for text in inspector.curve_axis.texts))


if __name__ == "__main__":
    unittest.main()
