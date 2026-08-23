ETLStd migration guide
======================

This guide describes how to consume the bundled freestanding standard-library
subset (`libstd`) and how source and build usage changes between the 0.2
transitional surface and the 0.3 cleanup. Hosted (desktop) ETL applications
keep using the **host** C++ standard library; they must **not** link or include
ETLStd as a replacement runtime.

Status of the CMake package
---------------------------

Issue #129 introduces `find_package(ETLStd CONFIG REQUIRED)` and the exported
targets `ETL::StdMinimal` and `ETL::StdFull`. Issue #130 attaches immutable
minimal/full profiles and link-time mismatch rejection to those targets.

Until those land, this repository still exposes a single header-only `etl`
INTERFACE library. Consumers reach libstd headers through include paths (see
`libstd/readme.md`), not through an installed `ETLStdConfig.cmake`. The
**intended** installed-consumer contract after #129/#130 is documented below so
migration and smoke work stay aligned; do not invent interim CMake package
names or ambiguous `ETL::Std` targets.

Namespace and include migration
-------------------------------

| 0.2 / transitional | 0.3 / target |
| --- | --- |
| `ETLSTD::X` (often via `#define ETLSTD etlstd`) | `std::X` |
| `#include <libstd/include/X>` | `#include <X>` after the ETLStd include root is on the path |
| Ad-hoc `-I` to the repo root + `include/` | `find_package(ETLStd CONFIG REQUIRED)` + one profile target |

Example before (matches `examples/avr/libstd_smoke.cpp` today):

```cpp
#define ETLSTD etlstd
#include <libstd/include/array>
#include <libstd/include/string_view>

int main() {
    constexpr ETLSTD::string_view text("avr-libstd");
    ETLSTD::array<unsigned char, 3> data{{1, 2, 3}};
    return text.starts_with("avr") ? 0 : 1;
}
```

Example after package + canonical headers:

```cpp
#include <array>
#include <string_view>

int main() {
    constexpr std::string_view text("avr-libstd");
    std::array<unsigned char, 3> data{{1, 2, 3}};
    return text.starts_with("avr") ? 0 : 1;
}
```

Final application code must not keep legacy `ETLSTD::` or `<libstd/include/...>`
spellings once the package profiles are the supported path.

0.2 compatibility warnings and 0.3 removal
------------------------------------------

- **0.2**: transitional aliases and path forms may emit compatibility warnings
  when both legacy and canonical spellings are visible. Treat warnings as a
  migration checklist, not as noise to silence globally.
- **0.3**: legacy `ETLSTD` macros and `<libstd/include/X>` consumer paths are
  removed from the supported surface. Only canonical `<X>` includes against the
  ETLStd include root remain.

Exact diagnostic text is owned by the package/profile work in #129/#130; this
guide only records the timeline.

Profiles: minimal and full
--------------------------

After #130:

| Profile | Exported target | Intended flags (AVR / freestanding) |
| --- | --- | --- |
| Minimal | `ETL::StdMinimal` | `-std=c++23 -ffreestanding -fno-exceptions -fno-rtti -nostdinc++ -nostdlib++` |
| Full | `ETL::StdFull` | `-std=c++23 -ffreestanding -fexceptions -frtti -nostdinc++ -nostdlib++` |

Rules:

- Link **exactly one** profile target per application.
- Do not select profiles with ad-hoc preprocessor defines; configuration comes
  from the target's generated `__etl/config` include directory.
- Mixing minimal and full archives must fail at link time (duplicate or missing
  profile marker symbols).

Host applications must not link ETLStd
--------------------------------------

Desktop and CI host builds of ETL continue to use the toolchain's hosted
libstdc++/libc++. Linking `ETL::StdMinimal` or `ETL::StdFull` into a hosted
ETL test or application is unsupported and will contaminate include order and
ABI. Use ETLStd only for freestanding AVR (or similarly configured) images.

Supported MCUs
--------------

Device headers under `include/etl/architecture/` currently cover:

- ATmega48A, ATmega48PA
- ATmega88A, ATmega88PA
- ATmega168A, ATmega168P, ATmega168PA
- ATmega328, ATmega328P
- ATmega32U4
- ATmega644A, ATmega644P
- ATtiny84
- ESP07, ESP8266
- Mock (host unit tests)

Package multilib resolution (after #129) keys off `ETL_AVR_MCU` and
`avr-g++ -print-multi-directory`, not marketing names alone. Unsupported MCU
values must fail configuration with an actionable diagnostic.

Installed consumer contract (after #129)
----------------------------------------

One executable contract shared with
`docs/installed-consumer-smoke-tests.md`:

```cmake
cmake_minimum_required(VERSION 3.25)
project(etlstd_consumer LANGUAGES CXX)
find_package(ETLStd CONFIG REQUIRED)
add_executable(consumer main.cpp)
# Choose exactly one:
target_link_libraries(consumer PRIVATE ETL::StdMinimal)
# target_link_libraries(consumer PRIVATE ETL::StdFull)
```

```cpp
// main.cpp — public headers only; no repository-relative includes
#include <array>
#include <type_traits>

int main() {
    constexpr std::array<int, 2> a{{1, 2}};
    return a.size() == 2 ? 0 : 1;
}
```

Configure the consumer with **only** `CMAKE_PREFIX_PATH` pointing at a clean
install prefix (no source-tree `-I`).

Direct avr-g++ usage
--------------------

Until the package wires flags transitively, a freestanding compile that mirrors
the AVR smoke intent looks like:

```sh
avr-g++ -std=c++23 -mmcu=atmega328p -DF_CPU=16000000UL \
  -Os -ffreestanding -fno-exceptions -fno-rtti \
  -nostdinc++ -isystem /path/to/prefix/include \
  -I/path/to/etl/include -I/path/to/etl \
  -c examples/avr/libstd_smoke.cpp -o libstd_smoke.o
```

With ETLStd archives installed (#129), replace manual `-isystem` / archive
paths with the multilib directory reported by
`avr-g++ -print-multi-directory` and link the matching minimal or full archive
plus avr-libc as required by the profile. Prefer the CMake package targets over
hand-rolled command lines for anything beyond debugging include order.

CMake AVR smoke in this repository
----------------------------------

```sh
cmake -S . -B build-avr \
  -DCMAKE_TOOLCHAIN_FILE=cmake/avr-gcc-toolchain.cmake \
  -DETL_BUILD_HOST_TESTS=OFF \
  -DETL_BUILD_AVR_SMOKE=ON \
  -DETL_AVR_MCU=atmega328p
cmake --build build-avr
```

This builds `etl_avr_smoke` and `etl_avr_libstd_smoke` against the in-tree
`etl` INTERFACE target. It is **not** a substitute for the clean-prefix
package consumers described in the smoke-test guide.

Troubleshooting
---------------

| Symptom | Likely cause |
| --- | --- |
| Hosted headers found instead of ETLStd | Include order / missing `-nostdinc++` / host app incorrectly linking ETLStd |
| `ETLSTD` undeclared after migration | Left-over macro use; switch to `std::` |
| Duplicate profile marker at link | Linked both `ETL::StdMinimal` and `ETL::StdFull` |
| Unknown MCU at configure | `ETL_AVR_MCU` not in the supported set / multilib map |
| Consumer needs repo-relative includes | Not using install prefix + `find_package`; blocked until #129 |

Compatibility notes
-------------------

Breaking changes land here as package work merges. Until `ETLStdConfig.cmake`
exists on `master`, treat the in-tree AVR smoke targets and `libstd/readme.md`
support table as the executable source of truth for headers, and treat the
`find_package` contract above as the industrialization target.