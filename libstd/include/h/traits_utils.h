/// @file traits_utils.h
/// @data 13/03/2014 22:55:53
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

#ifndef ETL_LIBSTD_TRAITS_UTILS_H
#define ETL_LIBSTD_TRAITS_UTILS_H

namespace std {

template<typename ...T> struct common_type;
template<typename T> struct common_type<T> {
  using type = std::decay_t<T>;
};
template<typename... T >
using common_type_t = typename common_type<T...>::type;
 
template<typename T1, typename T2>
struct common_type<T1, T2> {
    using type = std::decay_t<decltype(true ? std::declval<T1>() : std::declval<T2>())>;
};
 
template <typename T1, typename T2, typename... T3>
struct common_type<T1, T2, T3...> {
    using type = std::common_type_t<std::common_type_t<T1, T2>, T3...>;
};

  
}  

#endif // ETL_LIBSTD_TRAITS_UTILS_H
