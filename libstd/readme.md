C++ Standard Library for ETL
============================


A bundled standard-library subset for ETL.

**ETL** is a header only template library geared towards the size and performance constraints of embedded applications. The repository now targets a **C++23** baseline, but AVR environments still benefit from a bundled standard-library subset that covers the facilities ETL actually uses on embedded targets.

When ETL is consumed from another build system, add both the repository root and `include/` to the compiler include path. ETL headers refer to bundled headers through paths such as `libstd/include/memory`, so `include/` alone is not enough.

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
