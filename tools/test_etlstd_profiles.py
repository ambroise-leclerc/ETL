#!/usr/bin/env python3
"""AVR configuration/link checks for the exported ETLStd package (no execution)."""
import argparse
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def run(*args, success=True, diagnostic=None):
    result = subprocess.run([str(arg) for arg in args], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if (result.returncode == 0) != success or (diagnostic and diagnostic not in result.stdout):
        raise RuntimeError(f"Unexpected result: {' '.join(map(str, args))}\n{result.stdout}")
    return result.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--compiler", required=True)
    parser.add_argument("--mcu", required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="etlstd-profiles-") as temporary:
        temp = Path(temporary)
        prefix = temp / "prefix"
        relocated = temp / "relocated"
        run("cmake", "--install", args.build, "--prefix", prefix, "--component", "ETLStd")
        prefix.rename(relocated)
        for path in relocated.rglob("*.cmake"):
            if str(ROOT) in path.read_text() or str(args.build) in path.read_text():
                raise RuntimeError(f"Non-relocatable package: {path}")
        config = next(relocated.rglob("ETLStdConfig.cmake")).parent

        def configure(source, name, package, *extra, success=True, diagnostic=None):
            build = temp / name
            run("cmake", "-S", source, "-B", build,
                f"-DCMAKE_TOOLCHAIN_FILE={ROOT / 'cmake/avr-gcc-toolchain.cmake'}",
                f"-DCMAKE_CXX_COMPILER={args.compiler}", f"-DETL_AVR_MCU={args.mcu}",
                f"-DETLStd_DIR={package}", *extra, success=success, diagnostic=diagnostic)
            return build

        for location, package in (("tree", args.build), ("installed", config)):
            for profile in ("Minimal", "Full"):
                build = configure(ROOT / "test/package_consumer", f"{location}-{profile}", package,
                                  f"-DETLSTD_TEST_TARGET=ETL::Std{profile}")
                run("cmake", "--build", build)

        source = temp / "negative"
        source.mkdir()
        (source / "main.cpp").write_text("int main() { return 0; }\n")
        (source / "other.cpp").write_text("int other() { return 1; }\n")
        (source / "CMakeLists.txt").write_text('''cmake_minimum_required(VERSION 3.25)
project(ProfileProbes LANGUAGES CXX)
find_package(ETLStd CONFIG REQUIRED)
add_library(minimal OBJECT main.cpp)
target_link_libraries(minimal PRIVATE ETL::StdMinimal)
add_library(full OBJECT other.cpp)
target_link_libraries(full PRIVATE ETL::StdFull)
add_executable(mixed $<TARGET_OBJECTS:minimal> $<TARGET_OBJECTS:full>)
target_link_libraries(mixed PRIVATE ETL::StdMinimal ETL::StdFull)
add_executable(bad_flags main.cpp)
target_link_libraries(bad_flags PRIVATE ${PROFILE})
target_compile_options(bad_flags PRIVATE ${BAD_FLAG})
''')
        build = configure(source, "mixed", config, "-DPROFILE=ETL::StdMinimal", "-DBAD_FLAG=-fno-exceptions")
        run("cmake", "--build", build, "--target", "mixed", success=False,
            diagnostic="__etl_profile_mismatch_guard")
        for profile, flag in (("Minimal", "-fexceptions"), ("Minimal", "-frtti"),
                              ("Full", "-fno-exceptions"), ("Full", "-fno-rtti")):
            build = configure(source, f"bad-{profile}{flag}", config,
                              f"-DPROFILE=ETL::Std{profile}", f"-DBAD_FLAG={flag}", success=False,
                              diagnostic="ETLStd profile conflicts with the compiler")
        (source / "CMakeLists.txt").write_text('''cmake_minimum_required(VERSION 3.25)
project(MissingMCU LANGUAGES CXX)
unset(ETL_AVR_MCU CACHE)
unset(ETL_AVR_MCU)
find_package(ETLStd CONFIG REQUIRED)
''')
        configure(source, "missing-mcu", config,
                  success=False, diagnostic="ETLStd requires ETL_AVR_MCU")
        # ATmega2560 uses a different multilib from every ETL-modeled AVR.
        configure(ROOT / "test/package_consumer", "wrong-multilib", config, "-DETL_AVR_MCU=atmega2560",
                  success=False, diagnostic="ETLStd has no runtime for MCU")
        archive = next(relocated.rglob("libetlstd_minimal.a"))
        archive.unlink()
        configure(ROOT / "test/package_consumer", "missing-archive", config,
                  success=False, diagnostic="does not exist")
    print("ETLStd package, relocation, profile mismatch and incompatible flags: OK")


if __name__ == "__main__":
    main()
