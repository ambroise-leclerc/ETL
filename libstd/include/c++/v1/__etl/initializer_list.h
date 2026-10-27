/// @file initializer_list.h
/// @data 19/04/2014 22:24:53
/// @author Ambroise Leclerc
/// @brief Initializer list.
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

namespace ETLSTD {
template<typename T>
class initializer_list {
 public:
  using value_type      = T;
  using reference       = const T&;
  using const_reference = const T&;
  using size_type       = size_t;
  using iterator        = const T*;
  using const_iterator  = const T*;
  
  
  constexpr initializer_list() noexcept : begin_(nullptr), size_(0) {}

  constexpr size_t   size()  const noexcept { return size_; }
  constexpr const T* begin() const noexcept { return begin_; }
  constexpr const T* end()   const noexcept { return begin_+size_; }
    
 private:
  iterator begin_;
  size_type size_;
};

template<typename T> 
constexpr const T* begin(initializer_list<T> list) noexcept { return list.begin(); }
  
template<typename T>
constexpr const T* end(initializer_list<T> list) noexcept { return list.end(); }


  
} // namespace ETLSTD  
