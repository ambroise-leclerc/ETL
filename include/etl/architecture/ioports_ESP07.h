/// @file ioports_ESP07.h
/// @date 5/4/16 11:13 PM
/// @author Ambroise Leclerc and Cécile Gomes
/// @brief Espressif ESP 32-bit microcontrollers peripherals handling classes
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

#include <ioports_esp.h>

namespace etl {

    struct Pin15 : public Pin<Port0> {

       static void Set() {
            AbstractPin<15>::Set();
        }

        static void Clear() {
            AbstractPin<15>::Clear();
        }

        static void Toggle() {
            AbstractPin<15>::Toggle();
        }

        static void SetOutput() {
            AbstractPin<15>::SetOutput();
        }

        static void SetInput() {
            AbstractPin<15>::SetInput();
        }

        static uint16_t  Read() {
           return AbstractPin<15>::Read();
        }

        static void PulseHigh() {
            AbstractPin<15>::PulseHigh();
        }

        static void PulseLow() {
           AbstractPin<15>::PulseLow();
         }

        static constexpr uint8_t bitmask() {
            return AbstractPin<15>::bitmask();
        }

         // Attache le handler sur un interrput
        static void AttachHandler(voidFuncPtr userFunc) {
            AbstractPin<15>::AttachHandler(userFunc);
        }

         // Fonction d interruption que j'aurais bien mise prive mais je sais pas comment on fait!
         static void Interrupt() {
              AbstractPin<15>::Interrupt();
         }
    };

    struct Pin14 : public Pin<Port0> {

       static void Set() {
            AbstractPin<14>::Set();
        }

        static void Clear() {
            AbstractPin<14>::Clear();
        }

        static void Toggle() {
            AbstractPin<14>::Toggle();
        }

        static void SetOutput() {
            AbstractPin<14>::SetOutput();
        }

        static void SetInput() {
            AbstractPin<14>::SetInput();
        }

        static uint16_t  Read() {
           return AbstractPin<14>::Read();
        }

        static void PulseHigh() {
            AbstractPin<14>::PulseHigh();
        }

        static void PulseLow() {
           AbstractPin<14>::PulseLow();
         }

        static constexpr uint8_t bitmask() {
            return AbstractPin<14>::bitmask();
        }

         // Attache le handler sur un interrput
        static void AttachHandler(voidFuncPtr userFunc) {
            AbstractPin<14>::AttachHandler(userFunc);
        }

         // Fonction d interruption que j'aurais bien mise prive mais je sais pas comment on fait!
         static void Interrupt() {
              AbstractPin<14>::Interrupt();
         }
    };

    struct Pin13 : public Pin<Port0> {

       static void Set() {
            AbstractPin<13>::Set();
        }

        static void Clear() {
            AbstractPin<13>::Clear();
        }

        static void Toggle() {
            AbstractPin<13>::Toggle();
        }

        static void SetOutput() {
            AbstractPin<13>::SetOutput();
        }

        static void SetInput() {
            AbstractPin<13>::SetInput();
        }

        static uint16_t  Read() {
           return AbstractPin<13>::Read();
        }

        static void PulseHigh() {
            AbstractPin<13>::PulseHigh();
        }

        static void PulseLow() {
           AbstractPin<13>::PulseLow();
         }

        static constexpr uint8_t bitmask() {
            return AbstractPin<13>::bitmask();
        }

         // Attache le handler sur un interrput
        static void AttachHandler(voidFuncPtr userFunc) {
            AbstractPin<13>::AttachHandler(userFunc);
        }

         // Fonction d interruption que j'aurais bien mise prive mais je sais pas comment on fait!
         static void Interrupt() {
              AbstractPin<13>::Interrupt();
         }
    };

    struct Pin12 : public Pin<Port0> {

       static void Set() {
            AbstractPin<12>::Set();
        }

        static void Clear() {
            AbstractPin<12>::Clear();
        }

        static void Toggle() {
            AbstractPin<12>::Toggle();
        }

        static void SetOutput() {
            AbstractPin<12>::SetOutput();
        }

        static void SetInput() {
            AbstractPin<12>::SetInput();
        }

        static uint16_t  Read() {
           return AbstractPin<12>::Read();
        }

        static void PulseHigh() {
            AbstractPin<12>::PulseHigh();
        }

        static void PulseLow() {
           AbstractPin<12>::PulseLow();
         }

        static constexpr uint8_t bitmask() {
            return AbstractPin<12>::bitmask();
        }

         // Attache le handler sur un interrput
        static void AttachHandler(voidFuncPtr userFunc) {
            AbstractPin<12>::AttachHandler(userFunc);
        }

         // Fonction d interruption que j'aurais bien mise prive mais je sais pas comment on fait!
         static void Interrupt() {
              AbstractPin<12>::Interrupt();
         }
    };

    struct Pin5 : public Pin<Port0> {

       static void Set() {
            AbstractPin<5>::Set();
        }

        static void Clear() {
            AbstractPin<5>::Clear();
        }

        static void Toggle() {
            AbstractPin<5>::Toggle();
        }

        static void SetOutput() {
            AbstractPin<5>::SetOutput();
        }

        static void SetInput() {
            AbstractPin<5>::SetInput();
        }

        static uint16_t  Read() {
           return AbstractPin<5>::Read();
        }

        static void PulseHigh() {
            AbstractPin<5>::PulseHigh();
        }

        static void PulseLow() {
           AbstractPin<5>::PulseLow();
         }

        static constexpr uint8_t bitmask() {
            return AbstractPin<5>::bitmask();
        }

         // Attache le handler sur un interrput
        static void AttachHandler(voidFuncPtr userFunc) {
            AbstractPin<5>::AttachHandler(userFunc);
        }

         // Fonction d interruption que j'aurais bien mise prive mais je sais pas comment on fait!
         static void Interrupt() {
              AbstractPin<5>::Interrupt();
         }
    };

    struct Pin4 : public Pin<Port0> {

       static void Set() {
            AbstractPin<4>::Set();
        }

        static void Clear() {
            AbstractPin<4>::Clear();
        }

        static void Toggle() {
            AbstractPin<4>::Toggle();
        }

        static void SetOutput() {
            AbstractPin<4>::SetOutput();
        }

        static void SetInput() {
            AbstractPin<4>::SetInput();
        }

        static uint16_t  Read() {
           return AbstractPin<4>::Read();
        }

        static void PulseHigh() {
            AbstractPin<4>::PulseHigh();
        }

        static void PulseLow() {
           AbstractPin<4>::PulseLow();
         }

        static constexpr uint8_t bitmask() {
            return AbstractPin<4>::bitmask();
        }

         // Attache le handler sur un interrput
        static void AttachHandler(voidFuncPtr userFunc) {
            AbstractPin<4>::AttachHandler(userFunc);
        }

         // Fonction d interruption que j'aurais bien mise prive mais je sais pas comment on fait!
         static void Interrupt() {
              AbstractPin<4>::Interrupt();
         }
    };

    struct Pin2 : public Pin<Port0> {

       static void Set() {
            AbstractPin<2>::Set();
        }

        static void Clear() {
            AbstractPin<2>::Clear();
        }

        static void Toggle() {
            AbstractPin<2>::Toggle();
        }

        static void SetOutput() {
            AbstractPin<2>::SetOutput();
        }

        static void SetInput() {
            AbstractPin<2>::SetInput();
        }

        static uint16_t  Read() {
           return AbstractPin<2>::Read();
        }

        static void PulseHigh() {
            AbstractPin<2>::PulseHigh();
        }

        static void PulseLow() {
           AbstractPin<2>::PulseLow();
         }

        static constexpr uint8_t bitmask() {
            return AbstractPin<2>::bitmask();
        }

         // Attache le handler sur un interrput
        static void AttachHandler(voidFuncPtr userFunc) {
            AbstractPin<2>::AttachHandler(userFunc);
        }

         // Fonction d interruption que j'aurais bien mise prive mais je sais pas comment on fait!
         static void Interrupt() {
              AbstractPin<2>::Interrupt();
         }
    };

    struct Pin0 : public Pin<Port0> {

       static void Set() {
            AbstractPin<0>::Set();
        }

        static void Clear() {
            AbstractPin<0>::Clear();
        }

        static void Toggle() {
            AbstractPin<0>::Toggle();
        }

        static void SetOutput() {
            AbstractPin<0>::SetOutput();
        }

        static void SetInput() {
            AbstractPin<0>::SetInput();
        }

        static uint16_t  Read() {
           return AbstractPin<0>::Read();
        }

        static void PulseHigh() {
            AbstractPin<0>::PulseHigh();
        }

        static void PulseLow() {
           AbstractPin<0>::PulseLow();
         }

        static constexpr uint8_t bitmask() {
            return AbstractPin<0>::bitmask();
        }

         // Attache le handler sur un interrput
        static void AttachHandler(voidFuncPtr userFunc) {
            AbstractPin<0>::AttachHandler(userFunc);
        }

         // Fonction d interruption que j'aurais bien mise prive mais je sais pas comment on fait!
         static void Interrupt() {
              AbstractPin<0>::Interrupt();
         }
    };
} // namespace etl
