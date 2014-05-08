/// @file watchdog.h
/// @data 03/05/2014 18:06:55
/// @author Ambroise Leclerc
/// @brief Watchdog timer. This file declares the etl::Watchdog class.
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

#ifndef ETL_WATCHDOG_H_
#define ETL_WATCHDOG_H_

#include <avr/wdt.h>  // Watchdog timer handling

namespace etl {

template<uint16_t TIMEOUT_MS>
class Watchdog {
 public:
  static const uint16_t kWdtValue = (TIMEOUT_MS<23)?0:(TIMEOUT_MS<45)?1:(TIMEOUT_MS<90)?
                                    2:(TIMEOUT_MS<185)?3:(TIMEOUT_MS<375)?4:(TIMEOUT_MS<750)?
                                    5:(TIMEOUT_MS<1500)?6:(TIMEOUT_MS<3000)?7:(TIMEOUT_MS<6000)?8:9;
  //static const uint16_t kTimeoutValues[];
 
  Watchdog() { wdt_enable(kWdtValue); }
  ~Watchdog() { wdt_disable(); }  
  static void Rearm() { wdt_reset(); }
    
  static constexpr uint16_t kTimeoutValues[] = { 15, 30, 60, 120, 250, 500, 1000, 2000, 4000, 8000 };
  static constexpr uint16_t kActualTimeoutMs = kTimeoutValues[kWdtValue];
  

}; 

//template<uint16_t T> const uint16_t Watchdog<T>::kTimeoutValues[] = { 15, 30, 60, 120, 250, 500, 1000, 2000, 4000, 8000 };
  
} // namespace etl  


#endif // ETL_WATCHDOG_H_