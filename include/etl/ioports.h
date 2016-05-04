/// @file ioports_ATmega328P.h
/// @date 12/05/2014 09:34:16
/// @author Ambroise Leclerc and Cécile Gomes
/// @brief Microcontrollers peripherals handling classes
//
// Copyright (c) 2016, Ambroise Leclerc and Cécile Gomes
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


template<typename ParentPort>
struct Pin {
  /// Port is defined as the Port object to which this pin belongs.
  using Port = ParentPort;
  using PinChangeIRQ = typename ParentPort::PinChangeIRQ;
};


#if defined (__AVR_Dummy__)
#elif defined (__ESP_ESP8266__)
#include "architecture/ioports_ESP8266.h"
#elif defined (__ESP_ESP07__)
#include "architecture/ioports_ESP07.h"
#elif defined (__Mock_Mock__)
#include "architecture/ioports_Mock.h"
#elif defined (__AVR_ATmega32U4__)
#include "architecture/ioports_ATmega32U4.h"
#elif defined (__AVR_ATmega48A__)
#include "architecture/ioports_ATmega48A.h"
#elif defined (__AVR_ATmega48PA__)
#include "architecture/ioports_ATmega48PA.h"
#elif defined (__AVR_ATmega88A__)
#include "architecture/ioports_ATmega88A.h"
#elif defined (__AVR_ATmega88PA__)
#include "architecture/ioports_ATmega88PA.h"
#elif defined (__AVR_ATmega168A__)
#include "architecture/ioports_ATmega168A.h"
#elif defined (__AVR_ATmega168PA__)
#include "architecture/ioports_ATmega168PA.h"
#elif defined (__AVR_ATmega168P__)
#include "architecture/ioports_ATmega168P.h"
#elif defined (__AVR_ATmega328__)
#include "architecture/ioports_ATmega328.h"
#elif defined (__AVR_ATmega328P__)
#include "architecture/ioports_ATmega328P.h"
#endif

