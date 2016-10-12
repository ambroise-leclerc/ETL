/// @file traits_references.h
/// @data 07/03/2014 11:35:53
/// @author Ambroise Leclerc
/// @brief Type traits references operations.
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
template<typename T> struct is_reference             : false_type { };
template<typename T> struct is_reference<T&>         : true_type  { };
template<typename T> struct is_reference<T&&>        : true_type  { };

namespace etlHelper {
    template<typename T> struct is_referenceable
    : integral_constant<bool, (is_object<T>::value == true || is_reference<T>::value == true)>::type { };     
} // namespace etlHelper

template<typename T> struct is_rvalue_reference      : false_type { };
template<typename T> struct is_rvalue_reference<T&&> : true_type  { };
template<typename T> struct is_lvalue_reference      : false_type { };
template<typename T> struct is_lvalue_reference<T&>  : true_type  { };
  
} // namespace ETLSTD 
