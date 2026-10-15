ETL
===

[![CI](https://github.com/ambroise-leclerc/ETL/actions/workflows/ci.yml/badge.svg)](https://github.com/ambroise-leclerc/ETL/actions/workflows/ci.yml)

*C++23 Embedded Template Library for AVR 8-bit microcontrollers.*

**ETL** is a header only template library geared towards the size and performance constraints of embedded applications.
Its main objective is to take advantage of the efficiency of generic programming and to produce the fastest possible binaries.

**ETL** provides compile-time abstractions whose strongly typed definitions can leverage high-level information and relations about the mcu's peripherals.

For example, type information about the PORT to which belongs a given PIN makes compile-time optimization of PORT manipulation possible.

```C++
template <typename PinRed, typename PinGreen, typename PinBlue>
class RGBLed {
  enum class bitColor : uint8_t { Red=1<<0, Green=1<<1, Blue=1<<2 };
public:
  void SetColor(uint8_t color) const {
     if (color & bitColor::Red) pinRed::Set(); else pinRed::Clear();
     if (color & bitColor::Green) pinGreen::Set(); else pinGreen::Clear();
     if (color & bitColor::Blue) pinBlue::Set(); else pinBlue::Clear();
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
This optimization can easily be performed programmatically by testing at compile-time if the pins belong to a same port, using `std::is_same_v`.


```C++
template <typename PinRed, typename PinGreen, typename PinBlue>
class RGBLed {
public:
  void SetColor(uint8_t color) const {
    if (std::is_same_v<pinRed::Port, pinGreen::Port> &&
        std::is_same_v<pinRed::Port, pinBlue::Port>) {
      pinRed::Port::ChangeBits(pinRed::bitmask() | pinGreen::bitmask() | pinBlue::bitmask(), 
                                (color & bitColor::Red ? pinRed::bitmask() : 0) + 
                                (color & bitColor::Green ? pinGreen::bitmask() : 0) +
                                (color & bitColor::Blue ? pinBlue::bitmask() : 0));
    }      
    else {
     if (color & bitColor::Red)   pinRed::Set();    else pinRed::Clear();
     if (color & bitColor::Green) pinGreen::Set();  else pinGreen::Clear();
     if (color & bitColor::Blue)  pinBlue::Set();   else pinBlue::Clear();
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

Toolchain baseline
==================

ETL now targets a single **C++23** baseline across the repository:

- public headers require a C++23-capable compiler
- host tests build with **GCC 13+** and are validated in CI with GCC 13, 14, 15, 16 and Clang 18
- AVR smoke builds target **avr-gcc 14.2+** and are validated in CI with GCC 14.2 (modern-avr), 15.2 and 16.1 (ZakKemble)
- the AVR smoke targets build without libstdc++, relying only on avr-libc and the bundled `libstd`; the 15.2 and 16.1 CI jobs check this, as their toolchains ship no libstdc++
- the default build system baseline is **CMake 3.25+**

The bundled `libstd` directory remains a focused compatibility layer for the subset of standard-library facilities ETL uses on embedded targets; it is not a full reimplementation of the entire C++23 standard library. In particular, `<string>` now means a small owning string subset with `string_view` interop, append/resize/reserve support, and explicit non-goals around full hosted `std::string` parity.

Within `libstd`, ETL now distinguishes between **supported**, **experimental**, and **placeholder** headers. The detailed support matrix lives in `libstd/readme.md`; in short, low-level headers ETL already depends on are maintained, `vector`/`random`/`thread` are experimental, and `string`/`tuple` are placeholders not yet fit for use.


Host-side build and tests
=========================

The repository is header-only, but it ships a host-side Catch test runner for validating the mock architecture and bundled `libstd` implementation.

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Useful test runner commands:

```sh
./build/TestsETL --list-tests
./build/TestsETL '[GPIO]'
./build/TestsETL '[libstd]'
./build/TestsETL '[thread]'
./build/TestsETL 'Scenario: std::tuple'
```

Open AVR workflow
=================

The primary workflow is now an open, cross-platform AVR toolchain rather than Atmel Studio / Microchip Studio.

Recommended tools:

- host GCC 13+ (or Clang 18+) for local host builds and tests
- CMake 3.25+
- `avr-g++` 14.2+ for AVR validation, for example:
  - [ZakKemble/avr-gcc-build](https://github.com/ZakKemble/avr-gcc-build/releases) (GCC 15.2 or 16.1, no libstdc++)
  - [modern-avr/toolchain](https://github.com/modern-avr/toolchain/releases) (GCC 14.2, ships libstdc++)
  - [Microchip AVR 8-bit Toolchain](https://www.microchip.com/en-us/tools-resources/develop/microchip-studio/gcc-compilers) 4.x (GCC 15.1)
- `avrdude` for flashing

This repository includes an AVR smoke target that validates the headers with a real `avr-g++` build:

```sh
cmake -S . -B build-avr \
  -DCMAKE_TOOLCHAIN_FILE=cmake/avr-gcc-toolchain.cmake \
  -DETL_BUILD_HOST_TESTS=OFF \
  -DETL_BUILD_AVR_SMOKE=ON \
  -DETL_AVR_MCU=atmega328p \
  -DETL_AVR_F_CPU=16000000UL
cmake --build build-avr
```

The repository CI builds that path with three pinned toolchains (archives verified by SHA-256): `modern-avr/toolchain` `v14.2.0` and ZakKemble `avr-gcc` `15.2.0` and `16.1.0`. Each job reports the `avr-size` of the smoke binaries in its summary.

That produces:

- `build-avr/etl_avr_smoke.elf`
- `build-avr/etl_avr_smoke.hex`
- `build-avr/etl_avr_libstd_smoke.elf`
- `build-avr/etl_avr_libstd_smoke.hex`
- `build-avr/etl_avr_user_isr_smoke.elf` / `.hex` (user-defined USART ISR next to ETL)
- `build-avr/etl_avr_uart0_smoke.elf` / `.hex` (`Uart0` with the ETL interrupt handlers)
- `build-avr/etl_avr_multi_tu_smoke.elf` / `.hex` (ETL included from two translation units)
- `build-avr/etl_avr_custom_new_smoke.elf` / `.hex` (user replacement of `operator new` / `operator delete`)
- `build-avr/etl_avr_interrupt_smoke.elf` / `.hex` (handlers attached with `etl::Interrupt`)
- `build-avr/etl_avr_isr_equivalence_etl.elf` / `etl_avr_isr_equivalence_libc.elf` (same handlers with `etl::Interrupt` and avr-libc `ISR()`, compared by CI)

You can override `ETL_AVR_MCU` to validate another supported MCU such as `attiny84`.

Using ETL in your own AVR project
=================================

1. Configure your project to compile with C++23.
2. Add **two** include paths:
   - `.../ETL/include`
   - `.../ETL`

   The repository root is required because ETL headers include bundled standard-library headers with paths such as `libstd/include/memory`.

3. Add `#include <etl.h>` in your source file.

Interrupts
----------

Interrupt handlers are attached to the vectors that the device headers declare on each peripheral, without macros:

```cpp
static volatile uint8_t ticks;

static void onOverflow() { ticks = ticks + 1; }

[[gnu::naked]] void onCompareMatchB() {        // saves what it uses and ends with reti
    asm volatile("sbi %0, 0\n\treti" :: "I"(_SFR_IO_ADDR(GPIOR0)));
}

int main() {
    etl::Interrupt<etl::Timer0::Overflow>::attach<onOverflow>();
    etl::Interrupt<etl::Timer0::CompareMatchA>::attach<[] { /* ... */ }, etl::IsrKind::NonBlocking>();
    etl::Interrupt<etl::Timer0::CompareMatchB>::attach<onCompareMatchB, etl::IsrKind::Naked>();
    etl::Interrupt<etl::PinChangeIRQ0::Trigger>::attach<[] { /* ... */ }>();
    // ... enable the interrupt sources, then sei()
}
```

- `attach()` binds the handler at link time and generates no code: the vector is installed as soon as the program contains the call. It does not enable the interrupt source.
- `IsrKind::Blocking` (default) and `IsrKind::NonBlocking` match avr-libc `ISR()` and `ISR_NOBLOCK`. A lambda, or a `static` or `inline` function, is inlined into the interrupt code, which is then identical to an `ISR()` with the same body (checked by CI). A function with external linkage is called, as from an `ISR()`.
- `IsrKind::Naked` reaches a `[[gnu::naked]]` function through one jump from the vector.
- Attaching the same handler from several source files is fine. Attaching another handler, or defining an avr-libc `ISR()`, for the same vector is a build error (`symbol '__vector_16' is already defined` or `multiple definition of '__vector_16'`).
- Vector names: `TimerN::Overflow`, `TimerN::CompareMatchA`/`B`/`C`/`D`, `TimerN::Capture`, `PinChangeIRQN::Trigger`. Any other vector can be named with `etl::InterruptVector<USART_RX_vect_num>`.
- On the host, `Interrupt<V>::raise()` runs the attached handler, so that handlers can be unit-tested.

The hardware UART drivers (`Uart0`, and `Uart1` on the ATmega644) use the same mechanism: `start()` attaches their handlers, so a program that does not start a UART keeps the USART vectors free for its own handlers.

ETL is header-only and can be included from any number of translation units.

`etl.h` provides the global `operator new` / `operator delete` (including the sized and array forms) on top of `etl::FreeStore`. They are weak definitions: to use your own allocator, define these functions in a source file that does not include `etl.h`, and they take precedence. The free store is set up on its first use, so a program that never allocates pays nothing for it.

If your project uses the freestore helpers, ETL provides:

```
etl::FreeStore::GetMemorySize()
etl::FreeStore::GetMemoryFragmentation()
etl::FreeStore::GetFreeMemory()
```

An optional freestore trace log can be activated by defining `ETL_FREESTORE_LOG_DEPTH` before including `etl.h`:

```
#define ETL_FREESTORE_LOG_DEPTH 64
#include <etl.h>
```

This exposes:

```
uint8_t etl::FreeStore::FreeStoreDebugTrace::log_counter;
Operation etl::FreeStore::FreeStoreDebugTrace::log_operation[];
void* etl::FreeStore::FreeStoreDebugTrace::log_address[];
```

Flashing with avrdude
=====================

Once you have built a `.hex` file, a typical flashing command looks like:

```sh
avrdude -c <programmer> -p <mcu> -U flash:w:build-avr/etl_avr_smoke.hex:i
```
  
