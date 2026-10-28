#!/usr/bin/env python3
"""Regression tests for MCU-derived F_CPU defaults across cached CMake reconfiguration."""
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


class AVRDefaultsChecks(unittest.TestCase):
    def test_defaults_follow_mcu_and_preserve_explicit_overrides(self):
        """Automatic clocks follow the MCU; user cache overrides persist until explicitly cleared."""
        with tempfile.TemporaryDirectory(prefix="etl-avr-defaults-") as temporary:
            root = Path(temporary)
            source = root / "source"
            source.mkdir()
            (source / "CMakeLists.txt").write_text(f'''cmake_minimum_required(VERSION 3.25)
project(ClockDefaults LANGUAGES NONE)
include("{ROOT / 'cmake/AVRMatrixDefaults.cmake'}")
file(WRITE "${{CMAKE_BINARY_DIR}}/frequency.txt" "${{ETL_AVR_F_CPU}}")
''')
            build = root / "build"

            def configure(mcu, expected, *extra):
                """Reconfigure one persistent build directory and read its resolved clock."""
                result = subprocess.run(["cmake", "-S", str(source), "-B", str(build),
                                         f"-DETL_AVR_MCU={mcu}", *extra], capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertEqual((build / "frequency.txt").read_text(), expected)

            # The defaults module uses PROJECT_SOURCE_DIR to find its manifest.
            (source / "tools").mkdir()
            (source / "tools/avr_matrix.json").write_bytes((ROOT / "tools/avr_matrix.json").read_bytes())
            configure("atmega328p", "16000000UL")
            self.assertNotIn("ETL_AVR_F_CPU:", (build / "CMakeCache.txt").read_text())
            configure("attiny84", "8000000UL")
            configure("atmega328p", "16000000UL")
            configure("atmega328p", "12000000UL", "-DETL_AVR_F_CPU=12000000UL")
            configure("attiny84", "12000000UL")
            configure("attiny84", "8000000UL", "-UETL_AVR_F_CPU")
            result = subprocess.run(["cmake", "-S", str(source), "-B", str(build),
                                     "-DETL_AVR_MCU=atmega2560"], capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("is not in tools/avr_matrix.json", result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
