"""Production RPM/heading replay: continuity without independent heading truth."""
from pathlib import Path
import math
import subprocess
import sys
import tempfile
import itertools
import unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import plot_dump

class HeadingLogs(unittest.TestCase):
    def test_captures_keep_heading_continuous(self):
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / 'replay'
            src = ROOT / 'brick/src/subsystems'
            subprocess.run(['c++', '-std=c++17', '-O2', '-I' + str(src),
                            str(ROOT / 'tests/heading_log_probe.cpp'),
                            str(src / 'rssi_rpm_estimator.cpp'),
                            str(src / 'rpm_heading_tracker.cpp'), '-o', str(binary)], check=True)
            for name, motion in itertools.product(('log7', 'log8', 'log9', 'log10', 'log12', 'log13', 'log14'), (False, True)):
                with self.subTest(log=name, forced_motion=motion):
                    rows = plot_dump.load(ROOT / 'logs' / (name + '.csv'))[-1].rows
                    legacy = name in ('log7', 'log8', 'log9', 'log10')
                    offset = min((int(r['received_us']) - int(r['timestamp_us']) + 2**31) % 2**32 - 2**31
                                 for r in rows if r['kind'] == 1)
                    source, times = [], []
                    for r in rows:
                        if r['kind'] == 1:
                            radio = int(r['timestamp_us'])
                            callback = (radio + offset) % 2**32 if legacy else int(r['received_us'])
                            source.append(f"r {radio} {callback} {int(r['rssi_dbm'])}\n")
                        else:
                            t = int(r['timestamp_us'])
                            source.append(f"{'t' if motion else 'u'} {t} {r['throttle']} {r['target_rpm']}\n")
                            times.append(t)
                    values = [list(map(float, x.split())) for x in
                              subprocess.check_output([str(binary)], input=''.join(source), text=True).splitlines()]
                    self.assertEqual(len(values), len(times))
                    self.assertTrue(all(math.isfinite(x) for row in values for x in row))
                    # These captures do not record translation commands. Exercise both
                    # profiles for numerical stability, without claiming heading accuracy.
                    self.assertTrue(any(v[4] in ((12, 13) if motion else (10,)) for v in values))
                    for i in range(1, len(values)):
                        dt = (times[i] - times[i-1]) % 2**32 / 1e6
                        if dt > .05 or values[i-1][4] == 8:
                            continue
                        phase_step = values[i][0] - values[i-1][0] - values[i-1][1] * math.tau / 60 * dt
                        phase_step = (phase_step + math.pi) % math.tau - math.pi
                        self.assertLess(abs(phase_step), .001)
                        self.assertLessEqual(abs(values[i][5]), 4 * dt + .0001)

    def test_noisy_rssi_pipeline_improves_ramp_over_plain_integration(self):
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / 'simulation'
            src = ROOT / 'brick/src/subsystems'
            subprocess.run(['c++', '-std=c++17', '-O2',
                            str(ROOT / 'tools/rssi_simulator.cpp'),
                            str(src / 'rssi_rpm_estimator.cpp'),
                            str(src / 'rpm_heading_tracker.cpp'), '-o', str(binary)], check=True)
            results = []
            raw = []
            for compensate in (1, 0):
                path = Path(directory) / f'{compensate}.csv'
                path.write_text(subprocess.check_output([str(binary), '1200', '6', '1000', '3000',
                                                        str(compensate)], text=True))
                rows = plot_dump.load(path)[0].rows
                raw.append([r['rssi_dbm'] for r in rows if r['kind'] == 1])
                states = [r for r in rows if r['kind'] == 2]
                reference = next(r['heading_rad'] - r['true_phase_rad'] for r in states
                                 if r['timestamp_us'] >= 1000000)
                errors = [(r['heading_rad'] - r['true_phase_rad'] - reference + math.pi) % math.tau - math.pi
                          for r in states if r['timestamp_us'] >= 2000000]
                results.append(sum(e * e for e in errors) / len(errors))
            self.assertEqual(raw[0], raw[1])
            self.assertLess(results[0], results[1] * .5)
