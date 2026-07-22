/// @file functional_reference_wrapper.h
/// @date 06/06/2014 18:36:55
/// @author Ambroise Leclerc
/// @brief reference_wrapper : wraps a reference in an assignable and copyable object.
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

#include <libstd/include/h/functional.h>

namespace ETLSTD {

namespace etlHelper {

template <typename T> constexpr T *reference_wrapper_addressof(T &value) noexcept {
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_addressof(value);
#else
    return &value;
#endif
}

} // namespace etlHelper

template <typename T> class reference_wrapper {
public:
    using type = T;

    constexpr reference_wrapper(T &value) noexcept : data_(etlHelper::reference_wrapper_addressof(value)) {}
    reference_wrapper(T &&) = delete;
    constexpr reference_wrapper(const reference_wrapper &) noexcept = default;
    constexpr reference_wrapper &operator=(const reference_wrapper &) noexcept = default;

    constexpr operator T &() const noexcept { return *data_; }
    constexpr T &get() const noexcept { return *data_; }

    template <typename... Args>
    constexpr auto operator()(Args &&...args) const noexcept(noexcept(ETLSTD::invoke(get(), forward<Args>(args)...)))
        -> decltype(ETLSTD::invoke(get(), forward<Args>(args)...)) {
        return ETLSTD::invoke(get(), forward<Args>(args)...);
    }

private:
    T *data_;
};

template <typename T> constexpr reference_wrapper<T> ref(T &value) noexcept { return reference_wrapper<T>(value); }

template <typename T> constexpr reference_wrapper<T> ref(reference_wrapper<T> value) noexcept {
    return reference_wrapper<T>(value.get());
}

template <typename T> void ref(const T &&) = delete;

template <typename T> constexpr reference_wrapper<const T> cref(const T &value) noexcept {
    return reference_wrapper<const T>(value);
}

template <typename T> constexpr reference_wrapper<const T> cref(reference_wrapper<T> value) noexcept {
    return cref(value.get());
}

template <typename T> void cref(const T &&) = delete;

} // namespace ETLSTD