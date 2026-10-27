C++ Standard Library for ETL
============================


A bundled standard-library subset for ETL.

**ETL** is a header only template library geared towards the size and performance constraints of embedded applications. The repository now targets a **C++23** baseline, but AVR environments still benefit from a bundled standard-library subset that covers the facilities ETL actually uses on embedded targets.

When ETL is consumed from another build system on AVR, add `include/` to the compiler include path and `libstd/include/c++/v1` as a system include directory (`-isystem`), so that it is searched before the compiler's own C++ directories. ETL headers include standard headers by their standard names; on a host build, do not add the canonical root and ETL uses the compiler's standard library.

Layout
======

The canonical standard headers live in `libstd/include/c++/v1/` and their reserved implementation headers in `libstd/include/c++/v1/__etl/`. The former `libstd/include/<header>` paths remain for release 0.2 as generated wrappers that emit a deprecation warning and include the canonical header; they are removed in release 0.3. Regenerate them with `python3 tools/generate_libstd_compat_headers.py`; the `libstd-compat-headers` CTest runs the same script with `--check` and fails on missing, extra or stale wrappers.

The canonical headers declare their facilities directly in `std` (the former `ETLSTD` macro survives only as a deprecated alias set by the legacy wrappers, through `libstd/include/c++/compat/etlstd.h`). They therefore cannot be compiled next to the host standard library, which Catch loads. The Catch suites under `test/libstd/` and `test/CircularBuffer.cpp` are disabled by default (`-DETL_BUILD_LIBSTD_HOST_TESTS=ON` re-enables them, but they do not compile) until they are converted to isolated test executables. `array`, `type_traits` and `string_view` are already converted (see below). `libstd-source-policy` (`tools/check_libstd_namespace.py`, self-tested by `libstd-source-policy-self-test`) rejects `ETLSTD`/`etlstd`, `namespace etlstd` and `<libstd/include/...>` includes in `libstd/include/c++`, `include/`, `examples/` and `test/`, except for the 0.2 compatibility files listed in `tools/libstd_source_policy_allowlist.txt` (stale entries fail; the allowlist becomes empty when 0.3 removes the bridge), and `libstd-header-*` compile `<array>`, `<memory>` and `<tuple>` with `-nostdinc++` against the canonical include root only.

Support policy
==============

The bundled `libstd` surface is intentionally smaller than the hosted standard library. Headers in this directory fall into three categories:

| Status | Meaning |
| --- | --- |
| **Supported** | ETL relies on this header directly, it is part of the maintained embedded subset, and it should have active tests or compile validation. |
| **Experimental** | The header is being developed as an embedded-oriented subset, but the API and behavioral contract are still narrower or less stable than the hosted standard library. |
| **Placeholder** | The header exists for roadmap or compatibility reasons but should not be treated as part of the supported ETL surface yet. |

Current classification:

| Header family | Status | Notes |
| --- | --- | --- |
| `memory`, `utility`, `type_traits`, `cstddef`, `cstdint`, `limits`, `ratio`, `chrono`, `iterator`, `array`, `bitset`, `functional`, `queue`, `string_view`, `exception`, `stdexcept`, `initializer_list`, `new` | Supported | These headers are part of the maintained embedded subset and are expected to stay usable under the current C++23 baseline. |
| `thread`, `vector`, `random`, `string`, `tuple` | Experimental | Present and exercised, but still evolving toward a clearer embedded contract and deeper validation. `random` currently offers only `xorshift32_engine` and `uniform_int_distribution`, a small modulo-biased embedded subset, not a full `<random>` implementation. `string` and `tuple` have real, tested implementations (see below for `string`'s documented scope) but are narrower than the hosted standard library. |

Any header not present in the classification table above should be treated as non-contractual until it is added and given a status.

This bundled library is intentionally partial. It primarily covers the following families of facilities that ETL relies on:

- `new`, `delete`, `new[]`, `delete[]`, placement new and delete operators
- a Freestore implementation used by the new and delete operators.
- `std::move` and `std::forward` needed for xvalue support, move semantics and perfect forwarding.
- `std::unique_ptr` and `std::make_unique`
- `<type_traits>`
- `<cstddef>`
- `<functional>`
- `<iterator>`
- `<initializer_list>`
- `<utility>`
- `<array>`
- `<exception>`
- `<string_view>`
- a deliberately small `<string>` subset for owning, NUL-terminated dynamic text buffers

`<string>` support is intentionally embedded-oriented rather than a promise of full `std::basic_string` coverage. The maintained subset currently includes:

- construction from C strings, counted ranges, `string_view`, copies, and moves
- contiguous mutable storage via `data()` / `c_str()`
- iteration with pointer iterators
- `size()`, `length()`, `empty()`, `capacity()`, `reserve()`, `resize()`, and `clear()`
- `append(...)`, `push_back(...)`, `operator+=`, `copy(...)`, `compare(...)`, `substr(...)`, and `starts_with(...)`
- cheap interop with `string_view`

The following are intentionally **not** part of the promised surface today:

- small-string optimization or copy-on-write
- the full search / replace / insert / erase family from hosted `std::basic_string`
- allocator-heavy API parity work beyond ETL's current embedded needs

`<thread>` is part of the evolving embedded subset: `this_thread::sleep_for` is supported as a timing helper when the target `Device` exposes `delayTicks`, and mock validation should treat it as a cycle-accounted delay rather than as a high-precision wall-clock guarantee.


Isolated test executables
=========================

Suites that exercise the canonical headers are built as standalone executables with `-std=c++23 -ffreestanding -nostdinc++`, the canonical include root `libstd/include/c++/v1` and no Catch, Threads or host C++ headers. They are named `libstd-isolated-<name>` in CTest (`ctest -R libstd-isolated`).

To add a suite:

1. Write `test/libstd/<name>.cpp`. Include `"../libstd_harness/harness.h"`, then the standard headers by their standard names (`<array>`, ...), and use `using namespace std;`.
2. Use `TEST_CASE("title")`, `CHECK(expr)`, `CHECK_EQ(a, b)` and `CHECK_THROWS_AS(expr, Type)`. Failures print `file:line` on stderr and the executable exits non-zero. An unexpected exception fails the current test case at its declaration location; the remaining test cases still run and the final summary is printed.
3. Register it in `CMakeLists.txt` with `etl_add_libstd_isolated_test(<name> test/libstd/<name>.cpp)` and remove the former Catch file from `LIBSTD_TESTS`.

`libstd-isolated-failing-fixture` fails on purpose (`WILL_FAIL`) to check that failures reach the exit status. `test/libstd_harness/stubs/etl/ioports.h` replaces the Mock device for `<string_view>`; the executables still link the host runtime (exceptions) until runtime archives land separately.
