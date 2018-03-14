/// @file traits_operations.h
/// @data 07/03/2014 16:55:53
/// @author Ambroise Leclerc
/// @brief Type traits operations.
//
// Embedded Template Library
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

namespace ETLSTD {

template<typename T, T v>
struct integral_constant {
    static constexpr T value = v;
    using value_type = T;
    using type = integral_constant;
    constexpr operator value_type() const { return value; }
    constexpr value_type operator()() const { return value; }
};

template<bool B>
using bool_constant = integral_constant<bool, B>;
using true_type =   bool_constant<true>;
using false_type =  bool_constant<false>;

template<typename T, typename U> struct is_same       : false_type {};
template<typename T>             struct is_same<T, T> : true_type {};
template<typename T, typename U> constexpr bool is_same_v = is_same<T, U>::value;
 
template<bool, typename T, typename F>    struct conditional { using type = T; };
template<typename T, typename F>          struct conditional<false, T, F> { using type = F; };
template<bool B, typename T, typename F > using conditional_t = typename conditional<B, T, F>::type;

template<typename T> struct remove_const          { using type = T; };
template<typename T> struct remove_const<const T> { using type = T; };
template<typename T> using remove_const_t = typename remove_const<T>::type;

template<typename T> struct remove_volatile             { using type = T; };
template<typename T> struct remove_volatile<volatile T> { using type = T; };
template<typename T> using remove_volatile_t = typename remove_volatile<T>::type;

template<typename T> struct remove_cv {
    using type = typename remove_volatile<typename remove_const<T>::type>::type;
};
template<typename T> using remove_cv_t = typename remove_cv<T>::type;

template<typename T> struct remove_reference      { using type = T; };
template<typename T> struct remove_reference<T&>  { using type = T; };
template<typename T> struct remove_reference<T&&> { using type = T; };
template<typename T> using remove_reference_t = typename remove_reference<T>::type;
  
template<typename T> struct remove_extent { using type = T; };
template<typename T> struct remove_extent<T[]> { using type = T; };
template<typename T, size_t N> struct remove_extent<T[N]> { using type = T; };
template<typename T> using remove_extent_t = typename remove_extent<T>::type;

} // namespace ETLSTD
