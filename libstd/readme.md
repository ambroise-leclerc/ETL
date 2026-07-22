C++ Standard Library for ETL
============================


A bundled standard-library subset for ETL.

**ETL** is a header only template library geared towards the size and performance constraints of embedded applications. The repository now targets a **C++23** baseline, but AVR environments still benefit from a bundled standard-library subset that covers the facilities ETL actually uses on embedded targets.

When ETL is consumed from another build system, add both the repository root and `include/` to the compiler include path. ETL headers refer to bundled headers through paths such as `libstd/include/memory`, so `include/` alone is not enough.

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
| `thread`, `vector`, `random` | Experimental | Present and exercised, but still evolving toward a clearer embedded contract and deeper validation. `random` currently offers only `xorshift32_engine` and `uniform_int_distribution`, a small modulo-biased embedded subset, not a full `<random>` implementation. |
| `string`, `tuple` | Placeholder | Present in the tree, but not yet strong enough to be treated as part of the supported subset. |

If a header is not explicitly listed as supported, prefer to treat it as non-contractual until its status is promoted.

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
