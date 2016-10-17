/// @file traits_add.h
/// @data 13/03/2014 22:50:53
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

#include "traits_primary_types.h"
#include "traits_references.h"

namespace ETLSTD {

namespace etlHelper {
  template<typename T, bool b> struct rvalue_reference_type { using type = T; };
  template<typename T> struct rvalue_reference_type<T, true> { using type = T&&; }; 
} // namespace etlHelper

// Obtains a pointer on type T or on the referred type.
template<typename T> struct add_pointer {
  using type = typename remove_reference<T>::type*;
};
template<typename T> using add_pointer_t = typename add_pointer<T>::type;

  
template<typename T> struct add_rvalue_reference {
  typedef typename etlHelper::rvalue_reference_type< 
    T, (is_void<T>::value == false && is_reference<T>::value == false)>::type type;
};
template<typename T> struct add_lvalue_reference   { using type = T; };
    
template<typename T> using add_rvalue_reference_t = typename add_rvalue_reference<T>::type;
template<typename T> using add_lvalue_reference_t = typename add_lvalue_reference<T>::type;

/// Handles array decay and reference deferencing.
template<typename T> struct decay {
  using U = typename remove_reference<T>::type;
  using type =  typename conditional<is_array<U>::value,
                                     typename remove_extent<U>::type*,
                                     typename conditional<  is_function<U>::value,
                                                            typename add_pointer<U>::type,
                                                            typename remove_cv<U>::type     >::type
                                    >::type;
};

template<typename T> using decay_t = typename decay<T>::type;
template<typename T> using add_pointer_t = typename add_pointer<T>::type;

template<typename T> struct add_const     { using type = const T; };
template<typename T> struct add_volatile  { using type = volatile T; };
template<typename T> struct add_cv        { using type = typename add_volatile<typename add_const<T>::type>::type; };

} // namespace ETLSTD

