/// @file interrupts.h
/// @data 04/05/2014 18:15:46
/// @author Ambroise Leclerc
/// @brief Interrupts handling
//
// Copyright (c) 2014, Ambroise Leclerc
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

#ifndef ETL_INTERRUPTS_H_
#define ETL_INTERRUPTS_H_
#include <cstdint>

namespace etl {
  

#ifdef AVR
class Interrupts {
 public:
  /// Enables interrupts by setting the global interrupt mask.
  /// This function generates a single 'sei' instruction with
  /// no overhead.
  static void enable() { asm volatile("sei" ::: "memory"); }
    
  /// Disables interrupts by clearing the global interrupt mask.
  /// This function generates a single 'cli' instruction with
  /// no overhead.
  static void disable() { asm volatile("cli" ::: "memory"); }

  static uint8_t saveStatus() { return SREG; }

  static void restoreStatus(uint8_t statusSave) { SREG = statusSave; }
};

#endif AVR


#ifdef ESP
#endif ESP


#define MOCK
#ifdef MOCK
class Interrupts {
public:
    static bool enabled;
    static void enable()    { enabled = true; }
    static void disable()   { enabled = false; }
};

bool Interrupts::enabled = true;

#endif MOCK


  
} // namespace etl  

#endif // ETL_INTERRUPTS_H_