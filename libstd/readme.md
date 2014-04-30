C++ Standard Library for ETL
============================


A C++14 Standard Library implementation for ETL.

**ETL** is an header only template library geared towards the size and performance constraints of embedded applications. It relies heavily on the C++ Standard Library. But since the avr 8-bit toolchain does not provide a standard C++ standard library, this header-only std-lib provides all elements needed by ETL to be compiled as a standalone product.


So far, **C++ Standard Library for ETL** covers the following aspects of C++14 Standard Library :

- `new`, `delete`, `new[]`, `delete[]`, placement new and delete operators
- a Freestore implementation used by the new and delete operators.
- `std::move` and `std::forward` needed for xvalue support, move semantics and perfect forwading.
- `std::unique_ptr` and `std::make_unique`
- `<type_traits>`
- `<cstddef>`
- `<functional>`
- `<iterator>`
- `<initializer_list>`
- `<utility>`
- `<array>`
- `<exception>`

