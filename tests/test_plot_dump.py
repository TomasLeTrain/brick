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


if __name__ == "__main__":
    unittest.main()
