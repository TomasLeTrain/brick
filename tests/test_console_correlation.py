"""Cross-check browser reconstruction against the existing NumPy reference."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import unittest
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import plot_dump
from rssi_correlation import grid_at, correlation_curve

@unittest.skipUnless(shutil.which('node'), 'Node is required for the browser curve check')
class BrowserCorrelationTest(unittest.TestCase):
    def test_logged_curves_match_reference(self):
        script = """
import fs from 'node:fs';
import {parseCsv,correlationAt} from './tools/console/core.mjs';
const rows=parseCsv(fs.readFileSync(process.argv[1],'utf8'));
console.log(JSON.stringify(JSON.parse(process.argv[2]).map(i=>correlationAt(rows,i))));
"""
        for name in ('log18.csv', 'log19.csv', 'log20.csv', 'logs22.csv'):
            path = ROOT/'logs'/name
            rows = plot_dump.load(path)[0].rows
            states = [i for i, row in enumerate(rows) if row['kind'] == 2]
            indices = [states[len(states)//2], states[-1]]
            curves = json.loads(subprocess.check_output(['node','--input-type=module','-e',script,str(path),json.dumps(indices)],cwd=ROOT,text=True))
            for index, curve in zip(indices, curves):
                r = rows[index]
                expected = correlation_curve(grid_at(rows,index), int(r.get('comparison_samples') or 250),
                                              int(r.get('min_lag') or 9), int(r.get('max_lag') or 155))
                actual = np.array([np.nan if v is None else v for v in curve])
                np.testing.assert_allclose(actual, expected, atol=.0005, rtol=.0005, equal_nan=True,
                                           err_msg=f'{name} index={index}')
