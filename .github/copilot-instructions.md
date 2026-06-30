# Copilot instructions for ETL

## Build and test

- The repository is header-only; the default CMake flow builds the host-side Catch test runner:
  - `cmake -S . -B build`
  - `cmake --build build`
- Run the full suite with `ctest --test-dir build --output-on-failure` or `./build/TestsETL`
- List available tests with `./build/TestsETL --list-tests`
- Run a single test with a Catch filter, for example:
  - `./build/TestsETL '[GPIO]'`
  - `./build/TestsETL 'Scenario: vector'`
- AVR smoke builds use the dedicated toolchain file:
  - `cmake -S . -B build-avr -DCMAKE_TOOLCHAIN_FILE=cmake/avr-gcc-toolchain.cmake -DETL_BUILD_HOST_TESTS=OFF -DETL_BUILD_AVR_SMOKE=ON -DETL_AVR_MCU=atmega328p -DETL_AVR_F_CPU=16000000UL`
  - `cmake --build build-avr`
- The repository now targets **C++23 everywhere**. Current documented floors are **CMake 3.25+**, **host GCC 13+**, and **modern-avr AVR GCC 14.2+** for the AVR smoke path.
- CI is moving from historical Travis config to GitHub Actions in `.github/workflows/ci.yml`
- There is no dedicated lint target in the repository. Formatting is controlled by `.clang-format`.

## High-level architecture

- `include/etl/` is the main product. `include/etl.h` is the umbrella header, and the library is designed as a header-only embedded template library for AVR-class microcontrollers.
- `include/etl/ioports.h` is the target-selection layer. It chooses an implementation from `include/etl/architecture/` based on preprocessor macros such as `__AVR_*__`, `__ESP_*__`, or `__Mock_Mock__`.
- The core API is built around static port/pin types instead of runtime objects. `PortN` and `PinN` classes expose operations like `set`, `clear`, `setOutput`, and `changeBits`, plus compile-time metadata like `Pin::Port`, `Pin::bit()`, and `Pin::bitmask()`. The README's RGB LED examples reflect this compile-time composition model.
- `libstd/include/` is ETL's bundled standard-library subset for targets that do not provide a usable C++ standard library. ETL code uses the `ETLSTD` namespace alias so the same headers can work with host `std` or bundled `etlstd`.
- Treat `libstd/include/random` as a placeholder roadmap surface, not as a supported embedded header, until the repository defines and tests a real random subset.
- `CMakeLists.txt` now separates the host test runner from the AVR smoke build. The AVR path is a compile/link validation target, not a flashed firmware application.
- Host-side tests run against the mock architecture in `include/etl/architecture/MockCore.h` and `test/MockDevice.h`. `Device::yield()` is the central simulation step: it flushes staged register writes, applies `Device::pragma("BitLink" ...)` wiring, and dispatches pin-change callbacks.

## Key repository conventions

- New test files are not auto-discovered. Add them explicitly to `ETL_TESTS` or `LIBSTD_TESTS` in `CMakeLists.txt`, or they will not be compiled into `TestsETL`.
- `test/test.cpp` owns both `CATCH_CONFIG_MAIN` and `CATCH_CONFIG_NO_POSIX_SIGNALS`. Other test translation units should only add `SCENARIO` blocks.
- Tests that exercise GPIO, ports, or interrupts usually define `__Mock_Mock__` before including ETL headers so `ioports.h` resolves to the mock backend.
- Tests that are meant to cover the bundled standard-library subset often define `ETLSTD etlstd` before ETL headers. When changing portable ETL code, prefer `ETLSTD::` and `libstd/include/...` over hard-coding `std::` and host STL headers.
- Consumers need both the repository root and `include/` on their include path because ETL headers refer to bundled headers with `libstd/include/...` paths.
- Because the repository baseline is now C++23, update docs, tests, and examples consistently when changing standard requirements; avoid leaving mixed C++14/C++17/C++23 claims behind.
- In the mock backend, many port and pin helpers call `Device::yield()` internally. Preserve that yield-driven flow in tests and mock-oriented code instead of bypassing it with direct register mutation unless the test is intentionally inspecting raw register state.
