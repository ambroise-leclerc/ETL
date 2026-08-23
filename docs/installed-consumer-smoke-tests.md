Installed ETLStd consumer smoke tests
=====================================

These checks validate **installed** trees (clean prefix) and, secondarily,
build-tree package exports. They share one consumer contract with
`docs/migration-guide.md`.

Prerequisite
------------

- #129 — `ETLStdConfig.cmake`, `ETL::StdMinimal`, `ETL::StdFull`, install layout
- #130 — profile flags and link-time mismatch markers

If those exports are missing, do not paper over the gap with repo-relative
include hacks in the package-consumer fixtures. Report the missing target and
stop (per issue #135).

Shared consumer contract
------------------------

Every positive consumer must:

1. Call `find_package(ETLStd CONFIG REQUIRED)`.
2. Link **exactly one** of `ETL::StdMinimal` or `ETL::StdFull`.
3. Compile a single translation unit that includes only standard headers such as
   `<array>` and `<type_traits>` (no `libstd/include/...`, no source-tree paths).
4. Produce a runnable/return-0 program for host mock builds, or a successful
   link for AVR freestanding images.

Layout (intended)
-----------------

```text
test/package_consumer/
  build_tree_minimal/
  build_tree_full/
  clean_prefix_minimal/
  clean_prefix_full/
  negative_both_profiles/
  negative_missing_runtime/
```

Four positive combinations cover build-tree vs clean-prefix x minimal vs full.

Clean-prefix CTest fixture (intended)
-------------------------------------

```sh
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/tmp/etlstd-prefix
cmake --build build
cmake --install build
ctest --test-dir build -R etlstd-package-consumer --output-on-failure
```

The fixture must:

1. Install into a **fresh** temporary prefix.
2. Configure each clean-prefix consumer with **only**
   `-DCMAKE_PREFIX_PATH=<prefix>` (no source-tree include flags).
3. Build the consumer executable.
4. Fail the test if configuration or build succeeds when the prefix is wrong.

Build-tree consumers may use the build-tree package export directory instead of
the install prefix, but still must not add repository include paths by hand.

Expected failures (negative consumers)
--------------------------------------

| Fixture | Action | Expected failure mode |
| --- | --- | --- |
| `negative_both_profiles` | Link both `ETL::StdMinimal` and `ETL::StdFull` | Link error: duplicate or conflicting profile marker |
| `negative_missing_runtime` | Omit the selected profile target / archive | Link error: missing profile marker or unresolved freestanding symbols |
| Wrong `CMAKE_PREFIX_PATH` | Point at empty directory | Configure error from `find_package(ETLStd CONFIG REQUIRED)` |

Checklist
---------

- [ ] `build_tree_minimal` configures and builds with package export only
- [ ] `build_tree_full` configures and builds with package export only
- [ ] `clean_prefix_minimal` configures and builds with install prefix only
- [ ] `clean_prefix_full` configures and builds with install prefix only
- [ ] Negative consumers fail for the intended reason
- [ ] CTest names match `-R etlstd-package-consumer`

Interim validation (before #129)
--------------------------------

Until the package exists, keep exercising the in-tree path:

```sh
cmake -S . -B build-avr \
  -DCMAKE_TOOLCHAIN_FILE=cmake/avr-gcc-toolchain.cmake \
  -DETL_BUILD_HOST_TESTS=OFF \
  -DETL_BUILD_AVR_SMOKE=ON
cmake --build build-avr
```

That confirms `examples/avr/libstd_smoke.cpp` against the `etl` INTERFACE
target. It does **not** satisfy the installed-consumer acceptance criteria of
issue #135; those remain gated on the package export.