/// @file ioports_ESP07.h
/// @date 01/03/2016 12:31:01
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

    struct Pin0 : public Pin<Port0> {

       static void set() {
            AbstractPin<0>::Set();
        }


       static void set(bool value) {
            AbstractPin<0>::Set(value);
        }

        static void clear() {
            AbstractPin<0>::Clear();
        }

        static void toggle() {
            AbstractPin<0>::Toggle();
        }

        static void setOutput() {
            AbstractPin<0>::SetOutput();
        }

        static void setInput() {
            AbstractPin<0>::SetInput();
        }

        static uint16_t  read() {
           return AbstractPin<0>::Read();
        }

        static void pulseHigh() {
            AbstractPin<0>::PulseHigh();
        }

        static void pulseLow() {
           AbstractPin<0>::PulseLow();
         }

        static constexpr uint8_t bitmask() {
            return AbstractPin<0>::bitmask();
        }

         // Attache le handler sur un interrput
        static void attachHandler(voidFuncPtr userFunc) {
            AbstractPin<0>::AttachHandler(userFunc);
        }

         // Fonction d interruption que j'aurais bien mise prive mais je sais pas comment on fait!
         static void interrupt() {
              AbstractPin<0>::Interrupt();
         }
    };

    struct Pin2 : public Pin<Port0> {

       static void set() {
            AbstractPin<2>::Set();
        }


       static void set(bool value) {
            AbstractPin<2>::Set(value);
        }

        static void clear() {
            AbstractPin<2>::Clear();
        }

        static void toggle() {
            AbstractPin<2>::Toggle();
        }

        static void setOutput() {
            AbstractPin<2>::SetOutput();
        }

        static void setInput() {
            AbstractPin<2>::SetInput();
        }

        static uint16_t  read() {
           return AbstractPin<2>::Read();
        }

        static void pulseHigh() {
            AbstractPin<2>::PulseHigh();
        }

        static void pulseLow() {
           AbstractPin<2>::PulseLow();
         }

        static constexpr uint8_t bitmask() {
            return AbstractPin<2>::bitmask();
        }

         // Attache le handler sur un interrput
        static void attachHandler(voidFuncPtr userFunc) {
            AbstractPin<2>::AttachHandler(userFunc);
        }

         // Fonction d interruption que j'aurais bien mise prive mais je sais pas comment on fait!
         static void interrupt() {
              AbstractPin<2>::Interrupt();
         }
    };

    struct Pin4 : public Pin<Port0> {

       static void set() {
            AbstractPin<4>::Set();
        }


       static void set(bool value) {
            AbstractPin<4>::Set(value);
        }

        static void clear() {
            AbstractPin<4>::Clear();
        }

        static void toggle() {
            AbstractPin<4>::Toggle();
        }

        static void setOutput() {
            AbstractPin<4>::SetOutput();
        }

        static void setInput() {
            AbstractPin<4>::SetInput();
        }

        static uint16_t  read() {
           return AbstractPin<4>::Read();
        }

        static void pulseHigh() {
            AbstractPin<4>::PulseHigh();
        }

        static void pulseLow() {
           AbstractPin<4>::PulseLow();
         }

        static constexpr uint8_t bitmask() {
            return AbstractPin<4>::bitmask();
        }

         // Attache le handler sur un interrput
        static void attachHandler(voidFuncPtr userFunc) {
            AbstractPin<4>::AttachHandler(userFunc);
        }

         // Fonction d interruption que j'aurais bien mise prive mais je sais pas comment on fait!
         static void interrupt() {
              AbstractPin<4>::Interrupt();
         }
    };

    struct Pin5 : public Pin<Port0> {

       static void set() {
            AbstractPin<5>::Set();
        }


       static void set(bool value) {
            AbstractPin<5>::Set(value);
        }

        static void clear() {
            AbstractPin<5>::Clear();
        }

        static void toggle() {
            AbstractPin<5>::Toggle();
        }

        static void setOutput() {
            AbstractPin<5>::SetOutput();
        }

        static void setInput() {
            AbstractPin<5>::SetInput();
        }

        static uint16_t  read() {
           return AbstractPin<5>::Read();
        }

        static void pulseHigh() {
            AbstractPin<5>::PulseHigh();
        }

        static void pulseLow() {
           AbstractPin<5>::PulseLow();
         }

        static constexpr uint8_t bitmask() {
            return AbstractPin<5>::bitmask();
        }

         // Attache le handler sur un interrput
        static void attachHandler(voidFuncPtr userFunc) {
            AbstractPin<5>::AttachHandler(userFunc);
        }

         // Fonction d interruption que j'aurais bien mise prive mais je sais pas comment on fait!
         static void interrupt() {
              AbstractPin<5>::Interrupt();
         }
    };

    struct Pin12 : public Pin<Port0> {

       static void set() {
            AbstractPin<12>::Set();
        }


       static void set(bool value) {
            AbstractPin<12>::Set(value);
        }

        static void clear() {
            AbstractPin<12>::Clear();
        }

        static void toggle() {
            AbstractPin<12>::Toggle();
        }

        static void setOutput() {
            AbstractPin<12>::SetOutput();
        }

        static void setInput() {
            AbstractPin<12>::SetInput();
        }

        static uint16_t  read() {
           return AbstractPin<12>::Read();
        }

        static void pulseHigh() {
            AbstractPin<12>::PulseHigh();
        }

        static void pulseLow() {
           AbstractPin<12>::PulseLow();
         }

        static constexpr uint8_t bitmask() {
            return AbstractPin<12>::bitmask();
        }

         // Attache le handler sur un interrput
        static void attachHandler(voidFuncPtr userFunc) {
            AbstractPin<12>::AttachHandler(userFunc);
        }

         // Fonction d interruption que j'aurais bien mise prive mais je sais pas comment on fait!
         static void interrupt() {
              AbstractPin<12>::Interrupt();
         }
    };

    struct Pin13 : public Pin<Port0> {

       static void set() {
            AbstractPin<13>::Set();
        }


       static void set(bool value) {
            AbstractPin<13>::Set(value);
        }

        static void clear() {
            AbstractPin<13>::Clear();
        }

        static void toggle() {
            AbstractPin<13>::Toggle();
        }

        static void setOutput() {
            AbstractPin<13>::SetOutput();
        }

        static void setInput() {
            AbstractPin<13>::SetInput();
        }

        static uint16_t  read() {
           return AbstractPin<13>::Read();
        }

        static void pulseHigh() {
            AbstractPin<13>::PulseHigh();
        }

        static void pulseLow() {
           AbstractPin<13>::PulseLow();
         }

        static constexpr uint8_t bitmask() {
            return AbstractPin<13>::bitmask();
        }

         // Attache le handler sur un interrput
        static void attachHandler(voidFuncPtr userFunc) {
            AbstractPin<13>::AttachHandler(userFunc);
        }

         // Fonction d interruption que j'aurais bien mise prive mais je sais pas comment on fait!
         static void interrupt() {
              AbstractPin<13>::Interrupt();
         }
    };

    struct Pin14 : public Pin<Port0> {

       static void set() {
            AbstractPin<14>::Set();
        }


       static void set(bool value) {
            AbstractPin<14>::Set(value);
        }

        static void clear() {
            AbstractPin<14>::Clear();
        }

        static void toggle() {
            AbstractPin<14>::Toggle();
        }

        static void setOutput() {
            AbstractPin<14>::SetOutput();
        }

        static void setInput() {
            AbstractPin<14>::SetInput();
        }

        static uint16_t  read() {
           return AbstractPin<14>::Read();
        }

        static void pulseHigh() {
            AbstractPin<14>::PulseHigh();
        }

        static void pulseLow() {
           AbstractPin<14>::PulseLow();
         }

        static constexpr uint8_t bitmask() {
            return AbstractPin<14>::bitmask();
        }

         // Attache le handler sur un interrput
        static void attachHandler(voidFuncPtr userFunc) {
            AbstractPin<14>::AttachHandler(userFunc);
        }

         // Fonction d interruption que j'aurais bien mise prive mais je sais pas comment on fait!
         static void interrupt() {
              AbstractPin<14>::Interrupt();
         }
    };

    struct Pin15 : public Pin<Port0> {

       static void set() {
            AbstractPin<15>::Set();
        }


       static void set(bool value) {
            AbstractPin<15>::Set(value);
        }

        static void clear() {
            AbstractPin<15>::Clear();
        }

        static void toggle() {
            AbstractPin<15>::Toggle();
        }

        static void setOutput() {
            AbstractPin<15>::SetOutput();
        }

        static void setInput() {
            AbstractPin<15>::SetInput();
        }

        static uint16_t  read() {
           return AbstractPin<15>::Read();
        }

        static void pulseHigh() {
            AbstractPin<15>::PulseHigh();
        }

        static void pulseLow() {
           AbstractPin<15>::PulseLow();
         }

        static constexpr uint8_t bitmask() {
            return AbstractPin<15>::bitmask();
        }

         // Attache le handler sur un interrput
        static void attachHandler(voidFuncPtr userFunc) {
            AbstractPin<15>::AttachHandler(userFunc);
        }

         // Fonction d interruption que j'aurais bien mise prive mais je sais pas comment on fait!
         static void interrupt() {
              AbstractPin<15>::Interrupt();
         }
    };
} // namespace etl
