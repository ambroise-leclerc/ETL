/// @file functional.h
/// @date 14/03/2014 17:02:55
/// @author Ambroise Leclerc
/// @brief Function objects.
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

#include <libstd/include/type_traits>
#include <libstd/include/utility>

namespace ETLSTD {

template <typename T> class reference_wrapper;

namespace etlHelper {

template <typename T> struct is_reference_wrapper : false_type {};

template <typename T> struct is_reference_wrapper<reference_wrapper<T>> : true_type {};

template <typename T> inline constexpr bool is_reference_wrapper_v = is_reference_wrapper<T>::value;

template <typename T> constexpr T &&unwrap_reference(T &&value) noexcept { return forward<T>(value); }

template <typename T> constexpr T &unwrap_reference(reference_wrapper<T> value) noexcept { return value.get(); }

template <typename MemPtr, typename Obj>
constexpr auto invoke_member_object(MemPtr &&mem_ptr, Obj &&obj) -> decltype(unwrap_reference(forward<Obj>(obj)).*mem_ptr) {
    return unwrap_reference(forward<Obj>(obj)).*mem_ptr;
}

template <typename MemPtr, typename Ptr>
constexpr auto invoke_member_object(MemPtr &&mem_ptr, Ptr &&ptr) -> decltype((*forward<Ptr>(ptr)).*mem_ptr) {
    return (*forward<Ptr>(ptr)).*mem_ptr;
}

template <typename MemFn, typename Obj, typename... Args>
constexpr auto invoke_member_function(MemFn &&mem_fn, Obj &&obj, Args &&...args)
    -> decltype((unwrap_reference(forward<Obj>(obj)).*mem_fn)(forward<Args>(args)...)) {
    return (unwrap_reference(forward<Obj>(obj)).*mem_fn)(forward<Args>(args)...);
}

template <typename MemFn, typename Ptr, typename... Args>
constexpr auto invoke_member_function(MemFn &&mem_fn, Ptr &&ptr, Args &&...args)
    -> decltype(((*forward<Ptr>(ptr)).*mem_fn)(forward<Args>(args)...)) {
    return ((*forward<Ptr>(ptr)).*mem_fn)(forward<Args>(args)...);
}

template <typename F, typename... Args, enable_if_t<!is_member_pointer<decay_t<F>>::value, int> = 0>
constexpr auto invoke(F &&f, Args &&...args) -> decltype(forward<F>(f)(forward<Args>(args)...)) {
    return forward<F>(f)(forward<Args>(args)...);
}

template <typename MemPtr, typename Obj, typename... Args,
          enable_if_t<is_member_function_pointer<decay_t<MemPtr>>::value, int> = 0>
constexpr auto invoke(MemPtr &&mem_ptr, Obj &&obj, Args &&...args)
    -> decltype(invoke_member_function(forward<MemPtr>(mem_ptr), forward<Obj>(obj), forward<Args>(args)...)) {
    return invoke_member_function(forward<MemPtr>(mem_ptr), forward<Obj>(obj), forward<Args>(args)...);
}

template <typename MemPtr, typename Obj, enable_if_t<is_member_object_pointer<decay_t<MemPtr>>::value, int> = 0>
constexpr auto invoke(MemPtr &&mem_ptr, Obj &&obj)
    -> decltype(invoke_member_object(forward<MemPtr>(mem_ptr), forward<Obj>(obj))) {
    return invoke_member_object(forward<MemPtr>(mem_ptr), forward<Obj>(obj));
}

template <typename, typename = void> struct result_of {};

template <typename F, typename... Args> struct result_of<F(Args...), decltype(void(invoke(declval<F>(), declval<Args>()...)))> {
    using type = decltype(invoke(declval<F>(), declval<Args>()...));
};

} // namespace etlHelper

template <typename F, typename... ArgTypes>
constexpr auto invoke(F &&f,
                      ArgTypes &&...args) noexcept(noexcept(etlHelper::invoke(forward<F>(f), forward<ArgTypes>(args)...)))
    -> decltype(etlHelper::invoke(forward<F>(f), forward<ArgTypes>(args)...)) {
    return etlHelper::invoke(forward<F>(f), forward<ArgTypes>(args)...);
}

template <typename F> struct result_of {};

template <typename F, typename... ArgTypes> struct result_of<F(ArgTypes...)> : etlHelper::result_of<F(ArgTypes...)> {};

} // namespace ETLSTD
