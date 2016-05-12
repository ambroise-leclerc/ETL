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

#pragma once
#include <cstdint>
#include <map>
#include <list>
#include <tuple>
#include <functional>

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

#endif // AVR


#ifdef ESP
#endif // ESP


#define MOCK
#ifdef MOCK

class Interrupts {
public:
    static bool enabled(bool change = false, bool value = true) {
        static bool val = true;
        if (change) {
            val = value;
        }
        return val;
    }

    static void enable()    { enabled(true, true); }
    static void disable() { enabled(true, false); }
};


template <typename Register, typename RegisterType>
class InterruptsDispatcher {
    using Callback = std::function<void()>;
    enum triggerCallbacksFields { TriggerMask, CallbackFunc };
    using TriggerCallbacks = std::list<std::tuple<RegisterType, Callback>>;

    std::map<Register, TriggerCallbacks>  interruptRegisters;       ///< which registers can trigger interrupts ?

public:
    /// Registers a callback that will be called when register regId status matches (&) triggerMask
    /// @param callback a callable object
    /// @param regId id of register 
    void addCallback(std::function<void()> callback, Register regId, RegisterType triggerMask) {
        interruptRegisters[regId].push_back(std::make_tuple(triggerMask, callback));
    }

    /// Unregisters all callbacks associated to triggerMask on register regId.
    void removeCallback(Register regId, RegisterType triggerMask) {
        interruptRegisters[regId].remove_if([&triggerMask](auto& trigCallback) { return std::get<TriggerMask>(trigCallback) == triggerMask; });
    }

    void signalPortsPinsChange(Register regId, RegisterType changedPins) {
        for (auto& trigCallback : interruptRegisters[regId]) {
            if (changedPins & std::get<TriggerMask>(trigCallback)) {
                std::get<CallbackFunc>(trigCallback)();
            }
        }
    }

    void clear() {
        interruptRegisters.clear();
    }
};



#endif // MOCK


  
} // namespace etl  
