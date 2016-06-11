/// @file traits_primary_types.h
/// @data 07/03/2014 16:55:53
/// @author Ambroise Leclerc
/// @brief Type traits primary types.
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
#include <../libstd/include/cstdint>

namespace std {
namespace etlHelper {
  template<typename> struct is_type_entier         : std::false_type { };
  template <> struct is_type_entier<bool>          : std::true_type { };
  template <> struct is_type_entier<int8_t>        : std::true_type { };
  template <> struct is_type_entier<uint8_t>       : std::true_type { };
  template <> struct is_type_entier<int16_t>       : std::true_type { };
  template <> struct is_type_entier<uint16_t>      : std::true_type { };
  template <> struct is_type_entier<int32_t>       : std::true_type { };
  template <> struct is_type_entier<uint32_t>      : std::true_type { };
  template <> struct is_type_entier<int64_t>       : std::true_type { };
  template <> struct is_type_entier<uint64_t>      : std::true_type { };
  template<typename> struct is_type_flottant       : std::false_type { };
  template <> struct is_type_flottant<float>       : std::true_type { };
  template <> struct is_type_flottant<double>      : std::true_type { };
  template <> struct is_type_flottant<long double> : std::true_type { };
  template<typename T> struct is_pointeur          : std::false_type { };
  template<typename T> struct is_pointeur<T*>      : std::true_type { };
  template<typename T> struct is_mb_pointeur       : std::false_type { };
  template<typename T, typename U> struct is_mb_pointeur<T U::*>     : std::true_type { };
} // namespace etlHelper 

/// Checks whether T is a void type. is_void< >::value equals true if T is of
/// type void, const void, volatile void, or const volatile void.
template<typename T>
struct is_void
 : std::integral_constant<bool, std::is_same<void, typename std::remove_cv<T>::type>::value>
{};

/// Checks if T is an union.
/// is_union::value is true if T is an union, false otherwise.                         
template<typename T> struct is_union : std::integral_constant<bool, __is_union(T)> { };
template<typename T> struct is_enum : std::integral_constant<bool, __is_enum(T)> { };
template<typename T> struct is_class : std::integral_constant<bool, __is_class(T)> { };
template<typename T> struct is_integral : etlHelper::is_type_entier<typename std::remove_cv<T>::type>::type { };
template<typename T> struct is_floating_point : etlHelper::is_type_flottant<typename std::remove_cv<T>::type>::type { };
template<typename T> struct is_pointer : etlHelper::is_pointeur<typename std::remove_cv<T>::type> {};
template<typename T> struct is_member_pointer : etlHelper::is_mb_pointeur<typename std::remove_cv<T>::type> {};
  
  
template<typename T> 
struct is_arithmetic : std::integral_constant<bool,
                         std::is_integral<T>::value ||
                         std::is_floating_point<T>::value> { };
                           
/// Checks if T is a scalar type.
/// is_scalar::value is true if T is a scalar type, false otherwise.  
template<typename T>
struct is_scalar : std::integral_constant<bool,
                     std::is_arithmetic<T>::value     ||
                     std::is_enum<T>::value           ||
                     std::is_pointer<T>::value        ||
                     std::is_member_pointer<T>::value ||
                     std::is_same<std::nullptr_t, typename std::remove_cv<T>::type>::value> { };

/// Checks if T is an array.
/// is_array::value is true if T is an array, false otherwise.                         
template<typename T> struct is_array                       : std::false_type { };
template<typename T> struct is_array<T[]>                  : std::true_type { };
template<typename T, size_t N> struct is_array<T[N]>  : std::true_type { };



/// Checks if T is an object type (scalar, array, class, union).
/// is_object::value is true if T is an object, false otherwise.
template<typename T>
struct is_object : std::integral_constant<bool,
                     std::is_scalar<T>::value ||
                     std::is_array<T>::value  ||
                     std::is_union<T>::value  ||
                     std::is_class<T>::value> { };

/// Checks if T is a function type.
/// is_function::value is true if T is a function type, false otherwise
template<typename> struct is_function : std::false_type { };
template<typename Ret, typename... Args> struct is_function<Ret(Args...)> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args..., ...)> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args...) const> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args...) volatile> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args...) const volatile> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args..., ...) const> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args..., ...) volatile> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args..., ...) const volatile> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args...) &> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args...) const &> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args...) volatile &> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args...) const volatile &> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args..., ...) &> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args..., ...) const &> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args..., ...) volatile &> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args..., ...) const volatile &> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args...) && > : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args...) const &&> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args...) volatile &&> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args...) const volatile &&> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args..., ...) && > : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args..., ...) const &&> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args..., ...) volatile &&> : std::true_type {};
template<typename Ret, typename... Args> struct is_function<Ret(Args..., ...) const volatile &&> : std::true_type {};

template<typename T> using is_function_v = typename is_function<T>::value;
} // namespace std
