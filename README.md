ETL
===

C++(11/14) Embedded Template Library for AVR 8-bit microcontrollers.

**ETL** is a header only template library geared towards the size and performance constraints of embedded applications.


###Using ETL in an Atmel Studio project

1. Create a new C++ project :
  - 'File' -> 'New Project' -> 'GCC C++ Executable Project'
  - Select your target device

2. Configure the project to use C++11 :
  In 'Project' -> 'Project properties' -> 'Toolchain' -> 'AVR/GNU C++ Compiler'
  ->  'Miscellaneous'
  add "-std=c++11" in the 'Other flags:' field.

3. Add ETL include path in your configuration :
  In 'Project' -> 'Project properties' -> 'Toolchain' -> 'AVR/GNU C++ Compiler'
  -> 'General'
   - add **"C:\\*your_path*\ETL\include"** in the 'Default Include Paths:' field.*
   - Add **"C:\\*your_path*\ETL\libstd\include"** unless your toolchain has already a C++ standard library.

      \*(where ***your_path*** is the actual location of the ETL library).

4. Add `#include <etl.h>` in your cpp file.
  You can now use new, delete, new[], delete[] and placement new operators.
  The free store manager provides you additional functions :
    - `etl::FreeStore::GetMemorySize()`
    - `etl::FreeStore::GetMemoryFragmentation()`
    - `etl::Freestore::GetFreeMemory()`
