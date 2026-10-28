#!/usr/bin/env python3
"""Regression checks for matrix drift and link-map contamination detection."""
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from check_avr_matrix import PROFILES, ROOT, check_artifacts, devices


class MatrixChecks(unittest.TestCase):
    def test_supported_devices_and_both_profiles(self):
        """The emitted CI entries cover the complete documented device/profile product."""
        rows = devices()
        self.assertEqual(len(rows), 13)
        expected = {(row['mcu'], row['f_cpu'], profile) for row in rows for profile in PROFILES}
        result = subprocess.run([sys.executable, str(ROOT / 'tools/check_avr_matrix.py'), '--github'],
                                check=True, capture_output=True, text=True)
        matrix = json.loads(result.stdout)['include']
        actual = {(row['mcu'], row['f_cpu'], row['profile']) for row in matrix}
        self.assertEqual(len(matrix), 26)
        self.assertEqual(actual, expected)

    def test_drift(self):
        """Missing/duplicate devices, clock drift and documentation drift must fail."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name in ('tools/avr_matrix.json', 'include/etl/ioports.h', 'README.md'):
                destination = root / name
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(ROOT / name, destination)
            path = root / 'tools/avr_matrix.json'
            original = path.read_text()
            rows = json.loads(original)
            for bad in (rows[:-1], rows + [rows[0]], [dict(row, f_cpu='1UL') for row in rows]):
                path.write_text(json.dumps(bad))
                with self.assertRaises(ValueError):
                    devices(root)
            path.write_text(original)
            readme = root / 'README.md'
            readme.write_text(readme.read_text().replace('| `attiny84`', '| `attiny85`'))
            with self.assertRaisesRegex(ValueError, 'README'):
                devices(root)

    def test_artifact_checks(self):
        """Missing outputs, missing profile identity and contamination must fail."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for profile in PROFILES:
                name = f'etl_avr_std_{profile}_smoke'
                for suffix in ('elf', 'hex', 'map'):
                    (root / f'{name}.{suffix}').write_text('artifact')
                link_map = root / f'{name}.map'
                clean = f'libetlstd_{profile}.a __etl_profile_{profile}'
                link_map.write_text(clean)
                check_artifacts(root, profile)
                opposite = 'full' if profile == 'minimal' else 'minimal'
                for bad in ('libstdc++.a', 'libstdc++.so', 'GLIBCXX_3.4', f'libetlstd_{opposite}.a'):
                    link_map.write_text(clean + '\n' + bad)
                    with self.assertRaisesRegex(ValueError, 'Contaminated'):
                        check_artifacts(root, profile)
                link_map.write_text('unrelated archive')
                with self.assertRaisesRegex(ValueError, 'Missing ETLStd'):
                    check_artifacts(root, profile)
                link_map.write_text(clean)
                (root / f'{name}.hex').unlink()
                with self.assertRaisesRegex(ValueError, 'Missing or empty'):
                    check_artifacts(root, profile)

    def test_cli_modes_and_standalone_artifacts(self):
        """Artifact checking works without sources; ambiguous/irrelevant CLI options fail."""
        script = ROOT / 'tools/check_avr_matrix.py'
        for args in (('--profile', 'full'), ('--artifacts', '.', '--github'),
                     ('--artifacts', '.', '--devices'), ('--artifacts', '.', '--github-regression')):
            result = subprocess.run([sys.executable, str(script), *args], capture_output=True, text=True)
            self.assertEqual(result.returncode, 2, result.stderr)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            standalone = root / 'checker.py'
            shutil.copyfile(script, standalone)
            for profile in PROFILES:
                for suffix in ('elf', 'hex', 'map'):
                    (root / f'etl_avr_std_{profile}_smoke.{suffix}').write_text(
                        f'libetlstd_{profile}.a __etl_profile_{profile}')
            result = subprocess.run([sys.executable, str(standalone), '--artifacts', str(root)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('artifacts: OK', result.stdout)


if __name__ == '__main__':
    unittest.main()
