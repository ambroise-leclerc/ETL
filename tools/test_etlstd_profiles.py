#!/usr/bin/env python3
"""AVR configuration/link checks for the exported ETLStd package (no execution)."""
import argparse
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def run(*args, success=True, diagnostic=None):
    """Run a configuration/build probe and require its expected status and diagnostic."""
    result = subprocess.run([str(arg) for arg in args], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if (result.returncode == 0) != success or (diagnostic and diagnostic not in result.stdout):
        raise RuntimeError(f"Unexpected result: {' '.join(map(str, args))}\n{result.stdout}")
    return result.stdout


def main():
    """Exercise profile identity, installed consumption, flags and package failure modes."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--compiler", required=True)
    parser.add_argument("--mcu", required=True)
    args = parser.parse_args()
    compiler_bin = Path(args.compiler).resolve().parent

    def avr_tool(name):
        """Prefer inspection tools from the tested compiler installation."""
        return shutil.which(name, path=str(compiler_bin)) or shutil.which(name) or name

    def loaded_sizes(binary):
        """Measure flash/SRAM sections while excluding non-allocated profile metadata."""
        sizes = run(avr_tool("avr-size"), "-A", binary)
        return {name: int(size) for name, size in re.findall(r"^(\.text|\.data|\.bss)\s+(\d+)", sizes, re.MULTILINE)}

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
            """Configure a separate consumer/probe with the selected AVR toolchain and package."""
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
                binary = build / "consumer"
                if loaded_sizes(binary) != loaded_sizes(build / "unprofiled"):
                    raise RuntimeError(f"Profile identity adds flash/SRAM cost: {binary}")
                sections = run(avr_tool("avr-objdump"), "-h", binary)
                section = re.search(r"^\s*\d+ \.etl\.profile\s+[^\n]+\n([^\n]+)", sections, re.MULTILINE)
                if not section or any(flag in section[1] for flag in ("ALLOC", "LOAD", "CODE")):
                    raise RuntimeError(f"Profile identity is missing or allocated/executable: {binary}")

        version_probe = temp / "versions.cmake"
        version_probe.write_text('''foreach(minor 1 2 3 9)
    set(PACKAGE_FIND_VERSION "0.${minor}")
    set(PACKAGE_FIND_VERSION_MAJOR 0)
    set(PACKAGE_FIND_VERSION_MINOR ${minor})
    set(PACKAGE_FIND_VERSION_PATCH 0)
    set(PACKAGE_FIND_VERSION_TWEAK 0)
    include("${VERSION_FILE}")
    if (minor EQUAL 2)
        if (NOT PACKAGE_VERSION_COMPATIBLE)
            message(FATAL_ERROR "0.2 must accept the installed 0.2.0 package")
        endif()
    elseif(PACKAGE_VERSION_COMPATIBLE)
        message(FATAL_ERROR "0.${minor} must reject the installed 0.2.0 package")
    endif()
endforeach()
''')
        subprocess.run(["cmake", f"-DVERSION_FILE={config / 'ETLStdConfigVersion.cmake'}",
                        "-P", str(version_probe)], cwd=temp, check=True)

        wrapper = temp / "wrapper"
        wrapper.mkdir()
        (wrapper / "CMakeLists.txt").write_text(f'''cmake_minimum_required(VERSION 3.25)
project(Parent LANGUAGES CXX)
add_subdirectory("{ROOT}" etl)
if (DEFINED BUILD_TESTING)
    message(FATAL_ERROR "ETL unexpectedly enabled CTest in its parent")
endif()
''')
        configure(wrapper, "subdirectory", config, "-DETL_BUILD_HOST_TESTS=OFF", "-DETL_BUILD_STD=ON")

        source = temp / "negative"
        source.mkdir()
        (source / "main.cpp").write_text("int main() { return 0; }\n")
        (source / "other.cpp").write_text("int other() { return 1; }\n")
        (source / "full_options.cpp").write_text((ROOT / "examples/avr/std_full_options_smoke.cpp").read_text())
        (source / "CMakeLists.txt").write_text('''cmake_minimum_required(VERSION 3.25)
project(ProfileProbes LANGUAGES CXX)
find_package(ETLStd CONFIG REQUIRED)
add_library(full_options OBJECT full_options.cpp)
target_link_libraries(full_options PRIVATE ETL::StdFull)
add_library(minimal_options OBJECT full_options.cpp)
target_link_libraries(minimal_options PRIVATE ETL::StdMinimal)
add_library(minimal OBJECT main.cpp)
target_link_libraries(minimal PRIVATE ETL::StdMinimal)
add_library(full OBJECT other.cpp)
target_link_libraries(full PRIVATE ETL::StdFull)
add_executable(mixed $<TARGET_OBJECTS:minimal> $<TARGET_OBJECTS:full>)
target_link_libraries(mixed PRIVATE ETL::StdMinimal ETL::StdFull)
target_link_options(mixed PRIVATE -Wl,--gc-sections)
add_executable(bad_flags main.cpp)
target_link_libraries(bad_flags PRIVATE ${PROFILE})
target_compile_options(bad_flags PRIVATE ${BAD_FLAG})
if (DEFINED TRANSITIVE_BAD_FLAG)
    add_library(profile_interface INTERFACE)
    target_link_libraries(profile_interface INTERFACE ${PROFILE})
    add_library(profile_bridge INTERFACE)
    target_link_libraries(profile_bridge INTERFACE profile_interface)
    add_library(Profile::Bridge ALIAS profile_bridge)
    add_executable(bad_transitive main.cpp)
    target_link_libraries(bad_transitive PRIVATE Profile::Bridge)
    target_compile_options(bad_transitive PRIVATE ${TRANSITIVE_BAD_FLAG})
endif()
''')
        build = configure(source, "mixed", config, "-DPROFILE=ETL::StdMinimal", "-DBAD_FLAG=-fno-exceptions")
        run("cmake", "--build", build, "--target", "mixed", success=False,
            diagnostic="__etl_profile_mismatch_guard")
        run("cmake", "--build", build, "--target", "full_options")
        errors = run("cmake", "--build", build, "--target", "minimal_options", success=False,
                     diagnostic="exception handling disabled")
        if "dynamic_cast" not in errors:
            raise RuntimeError(f"Minimal did not reject the RTTI probe: {errors}")
        for profile, flag in (("Minimal", "-fexceptions"), ("Minimal", "-frtti"),
                              ("Full", "-fno-exceptions"), ("Full", "-fno-rtti")):
            build = configure(source, f"bad-{profile}{flag}", config,
                              f"-DPROFILE=ETL::Std{profile}", f"-DBAD_FLAG={flag}", success=False,
                              diagnostic="ETLStd profile conflicts with the compiler")
        for profile, valid_flag, bad_flag in (("Minimal", "-fno-exceptions", "-fexceptions"),
                                               ("Minimal", "-fno-rtti", "-frtti"),
                                               ("Full", "-fexceptions", "-fno-exceptions"),
                                               ("Full", "-frtti", "-fno-rtti")):
            configure(source, f"transitive-{profile}{bad_flag}", config,
                      f"-DPROFILE=ETL::Std{profile}", f"-DBAD_FLAG={valid_flag}",
                      f"-DTRANSITIVE_BAD_FLAG={bad_flag}", success=False,
                      diagnostic="compiler flags on bad_transitive")
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
        incomplete = temp / "incomplete-build-tree"
        incomplete.mkdir()
        for name in ("ETLStdConfig.cmake", "ETLStdConfigVersion.cmake", "ETLStdTargets.cmake", "ETLStdValidate.cmake"):
            shutil.copyfile(args.build / name, incomplete / name)
        targets = incomplete / "ETLStdTargets.cmake"
        rewritten, count = re.subn(r'(IMPORTED_LOCATION_\w+ ")[^"]*libetlstd_minimal\.a(")',
                                   lambda match: match[1] + str(incomplete / "missing.a") + match[2],
                                   targets.read_text())
        if not count:
            raise RuntimeError("Build-tree export contains no minimal runtime location")
        targets.write_text(rewritten)
        configure(ROOT / "test/package_consumer", "missing-build-archive", incomplete,
                  success=False, diagnostic="ETLStd runtime archive is missing")
        archive = next(relocated.rglob("libetlstd_minimal.a"))
        archive.unlink()
        configure(ROOT / "test/package_consumer", "missing-archive", config,
                  success=False, diagnostic="does not exist")
    print("ETLStd package, relocation, profile mismatch and incompatible flags: OK")


if __name__ == "__main__":
    main()
