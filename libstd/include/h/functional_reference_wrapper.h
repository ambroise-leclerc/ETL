/// @file functional.h
/// @data 06/06/2014 18:36:55
/// @author Ambroise Leclerc
/// @brief reference_wrapper : wraps a reference in an assignable and copyable object..
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
#include <libstd/include/memory>

namespace ETLSTD {

template<typename T>
class reference_wrapper /*: public etlHelper::reference_wrapper_base<typename std::remove_cv<T>::type> */{
 public:
  using type = T;
  
  /// Constructor : stores a reference to x.
  reference_wrapper(T& x) noexcept : data_(std::addressof(x)) {}
  
  /// Move constructor : deleted, construction from a temporary object is not allowed.
  reference_wrapper(T&&) = delete;
  
  /// Copy constructor : stores a reference to other.get().
  reference_wrapper(const std::reference_wrapper<T>& other) noexcept : data_(std::addressof(other.get())) {}
 private:
  T* data_;  
};  
  
  
} // namespace ETLSTD  


#endif // ETL_LIBSTD_FUNCTIONAL_REFERENCE_WRAPPER_H_