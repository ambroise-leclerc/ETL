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

`<thread>` is part of the evolving embedded subset: `this_thread::sleep_for` is supported as a timing helper when the target `Device` exposes `delayTicks`, and mock validation should treat it as a cycle-accounted delay rather than as a high-precision wall-clock guarantee.
