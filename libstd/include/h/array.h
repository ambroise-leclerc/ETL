/// @file array.h
/// @data 15/04/2014 22:30:53
/// @author Ambroise Leclerc
/// @brief Array standard container for storing a fixed size sequence of elements.
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

#ifndef ETL_LIBSTD_ARRAY_H_
#define ETL_LIBSTD_ARRAY_H_

namespace std {

template<typename T, std::size_t N>
struct array {
  using value_type        = T;
  using pointer           = T*;
  using const_pointer     = const T;
  using reference         = T&;
  using const_reference   = const T&;
  using iterator          = T*;
  using const_iterator    = const T*;
  using size_type         = std::size_t;
  using difference_type   = std::ptrdiff_t;
  using reverse_iterator  = std::reverse_iterator<iterator>;
  using const_reverse_iterator  = std::reverse_iterator<const_iterator>;
  
  // Access
  reference operator[](size_type i) noexcept              { return elems_[i]; }
  const_reference operator[](size_type i) noexcept const  { return elems_[i]; }
    
  reference at(size_type i)       { CheckRange(i); return elems_[i]; }
  const_reference at(size_type i) { CheckRange(i); return elems_[i]; }
   
  reference front() noexcept                        { return elems_[0]; }
  constexpr const_reference front() const noexcept  { return elems_[0]; }
  reference back() noexcept                         { return elems_[N - 1]; }
  constexpr const_reference back() const noexcept   { return elems_[N - 1]; }
  pointer data() noexcept                           { return elems_; }
  constexpr const_pointer data() const noexcept     { return elems_; }
    
  // Size
  constexpr size_type size() const noexcept { return N; }
  constexpr size_type max_size() const noexcept { return N; }
  constexpr bool empty() const noexcept { return size() == 0; }
    
  // Iterators
  iterator begin() noexcept { return elems_; }
  iterator end() noexcept   { return elems_ + N; }
    
  const_iterator begin() const noexcept { return elems_; }
  const_iterator end() const noexcept   { return elems_ + N; }
  
  reverse_iterator rbegin() noexcept { return reverse_iterator(end()); }
  reverse_iterator rend() noexcept   { return reverse_iterator(begin()); }
    
  const_reverse_iterator rbegin() const noexcept { return const_reverse_iterator(end()); }
  const_reverse_iterator rend() const noexcept   { return const_reverse_iterator(begin()); }
   
  const_iterator cbegin() const noexcept { return elems_; }
  const_iterator cend() const noexcept   { return elems_ + N); }
  
  const_reverse_iterator crbegin() const noexcept { return const_reverse_iterator(end()); }
  const_reverse_iterator crend() const noexcept { return const_reverse_iterator(begin()); }
    
  // Operations
  void fill(const T& value) { std::fill_n(begin(), size(), value); }    
  void swap(array& other) noexcept { std::swap_ranges(begin(), end(), other.begin()); }
    
 private:
  T elems_[N];
  
 private:
  static void CheckRange(size_type i) {
    if (i >= size()) {
      std::out_of_range exception("std::array : index out of range");
      throw(exception);
    }      
  }    
};

// Zero-sized array specialization
template<typename T, 0>
struct array {
  using value_type        = T;
  using pointer           = T*;
  using const_pointer     = const T;
  using reference         = T&;
  using const_reference   = const T&;
  using iterator          = T*;
  using const_iterator    = const T*;
  using size_type         = std::size_t;
  using difference_type   = std::ptrdiff_t;
  using reverse_iterator  = std::reverse_iterator<iterator>;
  using const_reverse_iterator  = std::reverse_iterator<const_iterator>;
  
  // Access
  constexpr reference operator[](size_type) noexcept              { RangeError(); return elems_; }
  constexpr const_reference operator[](size_type) noexcept const  { RangeError(); return elems_; }   
  constexpr reference at(size_type)                               { RangeError(); return elems_; }
  constexpr const_reference at(size_type)                         { RangeError(); return elems_; }
  constexpr reference front() noexcept                            { RangeError(); return elems_; }
  constexpr const_reference front() const noexcept                { RangeError(); return elems_; }
  constexpr reference back() noexcept                             { RangeError(); return elems_; }
  constexpr const_reference back() const noexcept                 { RangeError(); return elems_; }
  constexpr pointer data() noexcept                               { return null_ptr; }
  constexpr const_pointer data() const noexcept                   { return null_ptr; }
    
  // Size
  constexpr size_type size() const noexcept { return 0; }
  constexpr size_type max_size() const noexcept { return 0; }
  constexpr bool empty() const noexcept { return true; }
    
  // Iterators
  iterator begin() noexcept { return iterator(reinterpret_cast<T*>(this)); }
  iterator end() noexcept   { return begin(); }
    
  const_iterator begin() const noexcept { return const_iterator(reinterpret_cast<const T*>(this)); }
  const_iterator end() const noexcept   { return begin(); }
  
  reverse_iterator rbegin() noexcept { return reverse_iterator(end()); }
  reverse_iterator rend() noexcept   { return reverse_iterator(begin()); }
    
  const_reverse_iterator rbegin() const noexcept { return const_reverse_iterator(end()); }
  const_reverse_iterator rend() const noexcept   { return const_reverse_iterator(begin()); }
   
  const_iterator cbegin() const noexcept { return const_iterator(reinterpret_cast<const T*>(this)); }
  const_iterator cend() const noexcept   { return begin(); }
  
  const_reverse_iterator crbegin() const noexcept { return const_reverse_iterator(end()); }
  const_reverse_iterator crend() const noexcept { return const_reverse_iterator(begin()); }
    
  // Operations
  void fill(const T&) { }    
  void swap(array&) noexcept { }

 private:
  T elems_[0];

 private:
  static void RangeError() {
      std::out_of_range exception("zero-sized std::array : index out of range");
      throw(exception);
    }      
  }    
};

} // namespace std
#endif // ETL_LIBSTD_ARRAY_H_