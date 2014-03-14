C++ Standard Library for ETL
============================


A C++14 Standard Library implementation for ETL.

**ETL** is an header only template library geared towards the sizesize and performance constraints of embedded applications. It relies heavily on the C++ Standard Library. But since the avr 8-bit toolchain does not provide a standard C++ standard library, this header-only std-lib provides all elements needed by ETL to be compiled as a standalone product.


So far, **C++ Standard Library for ETL* covers the following aspects of C++11 Standard Library :

- new, delete, new[], delete[], placement new and delete operators
- a Freestore implementation used by the new and delete opearators.
- std::move and std::forward needed for xvalue support, move semantics and perferct forwading.
- <type_traits>
- <cstddef>
- std::unique_ptr
- std::make_unique

