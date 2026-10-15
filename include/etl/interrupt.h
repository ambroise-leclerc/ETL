/// @file interrupt.h
/// @date 15/10/2026
/// @author Ambroise Leclerc
/// @brief Binding of interrupt handlers to interrupt vectors, without macros.
//
// Copyright (c) 2026, Ambroise Leclerc
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

#include <libstd/include/cstdint>

namespace etl {

/// How an interrupt handler runs.
enum class IsrKind : uint8_t {
    Blocking,     ///< Interrupts stay disabled while the handler runs (avr-libc ISR_BLOCK).
    NonBlocking,  ///< Interrupts are enabled again on entry, so the handler can be interrupted (ISR_NOBLOCK).
    Naked,        ///< No prologue or epilogue: the handler is a [[gnu::naked]] function that saves what it
                  ///< uses and ends with reti. It is reached through one jump from the vector.
};

/// Interrupt vector number N of the target MCU. The device headers name each vector of their
/// peripherals with it, e.g. Timer0::Overflow or PinChangeIRQ0::Trigger.
template <uint8_t N> struct InterruptVector {
    static constexpr uint8_t number = N;
};

/// T, made dependent on Args. Calling Interrupt<...>::attach() from a template with a vector or a
/// handler that does not depend on its parameters would install the handler as soon as the template
/// is defined: making the vector dependent delays it until the template is instantiated.
template <typename T, auto... Args> struct dependent {
    using type = T;
};
template <typename T, auto... Args> using dependent_t = typename dependent<T, Args...>::type;

#if defined(__AVR__)

namespace detail {

// Interrupt entry points, one instance per vector, handler and kind. Template instances are weak
// symbols and would lose against the weak __bad_interrupt aliases of the startup code: each entry
// binds its vector with a strong alias, defined in the entry's COMDAT section so that several
// translation units can attach the same handler. With .equiv, attaching another handler (or defining
// an avr-libc ISR) for the same vector is a build error instead of a silent override.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmisspelled-isr"

template <uint8_t N, auto handler> [[gnu::signal, gnu::used, gnu::externally_visible]] void blockingEntry() {
    asm(".global __vector_%1\n.equiv __vector_%1, %c0" ::"i"(&blockingEntry<N, handler>), "n"(N));
    handler();
}

template <uint8_t N, auto handler> [[gnu::interrupt, gnu::used, gnu::externally_visible]] void nonBlockingEntry() {
    asm(".global __vector_%1\n.equiv __vector_%1, %c0" ::"i"(&nonBlockingEntry<N, handler>), "n"(N));
    handler();
}

template <uint8_t N, auto handler> [[gnu::naked, gnu::used, gnu::externally_visible]] void nakedEntry() {
    asm(".global __vector_%1\n.equiv __vector_%1, %c0\n\t%~jmp %x2" ::"i"(&nakedEntry<N, handler>), "n"(N), "i"(handler));
}

#pragma GCC diagnostic pop

} // namespace detail

/// Interrupt handling for a vector, e.g. Interrupt<Timer0::Overflow>.
template <typename Vector> struct Interrupt {
    /// Installs handler on the vector: a function or a capture-less lambda, called with no argument.
    /// The binding is resolved at link time and attach() generates no code: the vector is installed
    /// as soon as the program contains this call, even one that never runs. It does not enable the
    /// interrupt source, which is the peripheral's job.
    /// Attaching the same handler from several places is fine; attaching another handler, or defining
    /// an ISR for the same vector, is a build error.
    /// A lambda, or a static or inline function, is inlined into the interrupt entry, which is then
    /// identical to an avr-libc ISR() with the same body. A function with external linkage is called
    /// instead, with the register saves that implies, as from an ISR() calling it.
    /// @tparam handler function or capture-less lambda; with IsrKind::Naked, a [[gnu::naked]] function
    /// @tparam kind how the handler runs (blocking by default, like avr-libc ISR())
    template <auto handler, IsrKind kind = IsrKind::Blocking> static void attach() noexcept {
        if constexpr (kind == IsrKind::Blocking) {
            static_cast<void>(&detail::blockingEntry<Vector::number, handler>);
        } else if constexpr (kind == IsrKind::NonBlocking) {
            static_cast<void>(&detail::nonBlockingEntry<Vector::number, handler>);
        } else {
            static_cast<void>(&detail::nakedEntry<Vector::number, handler>);
        }
    }
};

#else // Host: the attached handler is recorded so that tests can raise the interrupt.

template <typename Vector> struct Interrupt {
    template <auto handler, IsrKind kind = IsrKind::Blocking> static void attach() noexcept {
        installed = [] { handler(); };
    }

    /// Simulates the interrupt: runs the attached handler, if any.
    static void raise() {
        if (installed != nullptr) {
            installed();
        }
    }

    /// Removes the attached handler.
    static void detach() noexcept { installed = nullptr; }

private:
    inline static void (*installed)() = nullptr;
};

#endif

} // namespace etl
