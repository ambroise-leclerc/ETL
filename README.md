ETL
===

C++11 Embedded Template Library for AVR 8-bit microcontrollers.

ETL is a header only template library geared towards the size and performance
constraints of embedded applications.


Using ETL in an Atmel Studio project :

1) Create a new C++ project :
  - 'File' -> 'New Project' -> 'GCC C++ Executable Project'
  - Select your target device

2) Configure the project to use C++11 :
  In 'Project' -> 'Project properties' -> 'Toolchain' -> 'AVR/GNU C++ Compiler'
  ->  'Miscellaneous'
  add "-std=c++11" in the 'Other flags:' field.

3) Add ETL include path in your configuration :
  In 'Project' -> 'Project properties' -> 'Toolchain' -> 'AVR/GNU C++ Compiler'
  -> 'General'
  add "C:\your_path\ETL\include" in the 'Default Include Paths:' field (where
  'your_path' is the actual location of the ETL library).