/// @file ETLDevice_ATmega88PA.h
/// @date 11/05/2014 19:25:16
/// @author Ambroise Leclerc and Cecile Thiebaut
/// @brief Atmel AVR 8-bit microcontrollers architecture specifications and low level functions.
//
// Copyright (c) 2017, Ambroise Leclerc and Cecile Thiebaut
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in
//     the documentation and/or other materials provided with the
//     distribution.
//   * Neither the name of the copyright holders nor the names of
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS' 
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
#pragma once


extern void __builtin_avr_delay_cycles(unsigned long);

namespace etl {
class Device {
public:
    static void delayTicks(uint32_t ticks)            { __builtin_avr_delay_cycles(ticks); }
    static const size_t flashSize = 8192;
    static const size_t eepromSize = 512;
    static const size_t sramSize = 1024;
    static const size_t architectureWidth = 8;
    static const size_t defaultBufferSize = 8;
    using OffType = uint16_t;
    static const uint32_t McuFrequency = F_CPU;

    /// Enables interrupts by setting the global interrupt mask.
    /// This function generates a single 'sei' instruction with
    /// no overhead.
    static void enableInterrupts() { asm volatile("sei" ::: "memory"); }

    /// Disables interrupts by clearing the global interrupt mask.
    /// This function generates a single 'cli' instruction with
    /// no overhead.
    static void disableInterrupts() { asm volatile("cli" ::: "memory"); }
};

} // namespace etl
