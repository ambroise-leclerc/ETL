/// @file iterator_traits.h
/// @data 16/04/2014 12:19:53
/// @author Ambroise Leclerc
/// @brief Provides definition for iterator traits.
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

// Iterator_tags : define the category of an iterator.
struct input_iterator_tag { };
struct output_iterator_tag { };
struct forward_iterator_tag : public input_iterator_tag { };
struct bidirectional_iterator_tag : public forward_iterator_tag { };
struct random_access_iterator_tag : public bidirectional_iterator_tag { };

template<typename Iterator>
struct iterator_traits {
  using iterator_category = typename Iterator::iterator_category;
  using value_type        = typename Iterator::value_type;
  using difference_type   = typename Iterator::difference_type;
  using pointer           = typename Iterator::pointer;
  using reference         = typename Iterator::reference;
};


/// iterator_traits specialization for raw pointers.
template<typename T>
struct iterator_traits<T*> {
  using iterator_category = random_access_iterator_tag;
  using value_type        = T;
  using difference_type   = ptrdiff_t;
  using pointer           = T*;
  using reference         = T&;
};

/// iterator_traits specialization for const raw pointers.
template<typename T>
struct iterator_traits<const T*> {
  using iterator_category = random_access_iterator_tag;
  using value_type        = T;
  using difference_type   = ptrdiff_t;
  using pointer           = const T*;
  using reference         = const T&;
};
  
} // namespace ETLSTD  
