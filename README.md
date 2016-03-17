ETL
===

C++14 Embedded Template Library for AVR 8-bit microcontroller.

**ETL** is a header only template library geared towards the size and performance constraints of embedded applications.
Its main objective is to take advantage of the efficiency of generic programming to produce the fastest possible binaries from an elegant and expressive code.

**ETL** provides compile-time abstractions whose strongly typed definitions can leverage high-level information and relations about the mcu's peripherals.

For example, type information about the PORT to which belongs a given PIN makes compile-time optimization of PORT manipulation possible.

```C++
template <typename PinRed, typename PinGreen, typename PinBlue>
class RGBLed {
  enum bitColor : uint8_t { Red=1<<0, Green=1<<1, Blue=1<<2 };
public:
  void SetColor(uint8_t color) const {
     if (color & Red) pinRed::Set(); else pinRed::Clear();
     if (color & Green) pinGreen::Set(); else pinGreen::Clear();
     if (color & Blue) pinBlue::Set(); else pinBlue::Clear();
    }    
};
```
If we create an RGBLed object connected to pins D7, B0 and B1 and call SetColor :

```C++
RGBLed<PinD7, PinB0, PinB1> myLed;

myLed.SetColor(0b101);
```
the compiler will produce the following assembly code :

```Assembly
5f 9a    sbi 0x0b, 7         ; 2 cycles
28 98    cbi 0x05, 0         ; 2 cycles
29 9a    sbi 0x05, 1         ; 2 cycles
```

And if another RGBLed is connected to pins B2, B3 and B4 :

```C++
RGBLed<PinB2, PinB3, PinB4> myLed2;

myLed2.SetColor(0b101);
```
almost the same output will be produced :

```Assembly
sbi 0x05, 2         ; 2 cycles
cbi 0x05, 3         ; 2 cycles
sbi 0x05, 4         ; 2 cycles
```

For this second RGBLed, a good hand-optimization would have used a global PORTB manipulation considering the fact that all pins belong to the same PORTB.
This optimization can easily be performed programmatically by testing at compile-time if the pins belong to a same port, using the std::is_same traits class.


```C++
template <typename PinRed, typename PinGreen, typename PinBlue>
class RGBLed {
public:
  void SetColor(uint8_t color) const {
    if (std::is_same<pinRed::Port, pinGreen::Port>::value &&
        std::is_same<pinRed::Port, pinBlue::Port>::value) {
      pinRed::Port::ChangeBits(pinRed::bitmask() | pinGreen::bitmask() | pinBlue::bitmask(), 
                                (color & Red ? pinRed::bitmask() : 0) + 
                                (color & Green ? pinGreen::bitmask() : 0) +
                                (color & Blue ? pinBlue::bitmask() : 0));
    }      
    else {
     if (color & Red)   pinRed::Set();    else pinRed::Clear();
     if (color & Green) pinGreen::Set();  else pinGreen::Clear();
     if (color & Blue)  pinBlue::Set();   else pinBlue::Clear();
    }      
  }
};
```

which now produces this code when all the pins belong to the same PORT, providing a +50% acceleration :

```Assembly
85 b1    in r24, 0x05        ; 1 cycle
8c 77    andi r24, 0x7c      ; 1 cycle
86 60    ori r24, 0x06       ; 1 cycle 
85 b9    out 0x05, r24       ; 1 cycle
```


###Using ETL in an Atmel Studio 7 project

1. Create a new C++ project :
  - 'File' -> 'New Project' -> 'GCC C++ Executable Project'
  - Select your target device

2. Configure the project to use C++14 :
  In 'Project' -> 'Project properties' -> 'Toolchain' -> 'AVR/GNU C++ Compiler'
  ->  'Miscellaneous'
  add `-std=c++14` in the 'Other flags:' field.

3. Add ETL include path in your configuration :
  In 'Project' -> 'Project properties' -> 'Toolchain' -> 'AVR/GNU C++ Compiler'
  -> 'General'
   - add **"C:\\*your_path*\ETL\include"** in the 'Default Include Paths:' field.*
   - Add **"C:\\*your_path*\ETL\libstd\include"** unless your toolchain has already a C++ standard library.

      \*(where ***your_path*** is the actual location of the ETL library).

4. Add `#include <etl.h>` in your cpp file.
  You can now use new, delete, new[], delete[] and placement new operators.
  The free store manager provides you additional functions :

```
  etl::FreeStore::GetMemorySize()  
  etl::FreeStore::GetMemoryFragmentation()  
  etl::Freestore::GetFreeMemory()
```
  
  An optional freestore trace log can be activated by defining ETL_FREESTORE_LOG_DEPTH with the requested log depth (3 bytes are used per log so a 64 log depth will consume 192 bytes of SRAM) :
  
```
  #define ETL_FREESTORE_LOG_DEPTH 64
  #include <etl.h>
```

  This defines three public variables in :
  
```
  uint8_t etl::FreeStore::FreeStoreDebugTrace::log_counter;       // number of logged operations
  Operation etl::FreeStore::FreeStoreDebugTrace::log_operation[]  // logged operations
  void* etl::FreeStore::FreeStoreDebugTrace::log_address[]        // logged addresses
```
  
