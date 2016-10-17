/// @file iterator_reverse.h
/// @data 19/04/2014 22:46:53
/// @author Ambroise Leclerc
/// @brief Provides definition for reverse_iterator adaptor.
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
  
template<typename Iterator>
class move_iterator {
 public:
  using iterator_type     = Iterator;
  using iterator_category = typename iterator_traits<iterator_type>::iterator_category;
  using value_type        = typename iterator_traits<iterator_type>::value_type;
  using difference_type   = typename iterator_traits<iterator_type>::difference_type;
  using pointer           = typename iterator_traits<iterator_type>::pointer;
  using reference         = value_type&&;
  
  /// Constructs a move iterator that points to no object.
  move_iterator() : current_() {}
  
  /// Constructs a move iterator from some original operator it.
  explicit move_iterator(iterator_type it) : current_(it) {}
  
  /// Constructs a move iterator from some other move iterator.
  template <class Iter>
  move_iterator(const move_iterator<Iter>& rev_it) : current_(rev_it.base()) {}
  
  /// Returns a copy of the base iterator.  
  iterator_type base() const { return current_; }
  
  /// Returns a reference to the element pointed by the iterator.
  reference operator*() const { return static_cast<reference>(*current_); }
  pointer operator->() const { return &(operator*()); }
  
  move_iterator<Iterator> operator+(difference_type n) const { return move_iterator<Iterator>(current_ + n); }
  move_iterator<Iterator> operator-(difference_type n) const { return move_iterator<Iterator>(current_ - n); }
  move_iterator<Iterator>& operator+=(difference_type n) const { current_ += n; return *this; }
  move_iterator<Iterator>& operator-=(difference_type n) const { current_ -= n; return *this; }
  move_iterator<Iterator>& operator++() { ++current_; return *this; }
  move_iterator<Iterator>& operator--() { --current_; return *this; }
  move_iterator<Iterator>& operator++(int) { move_iterator<Iterator> tmp = *this; ++current_; return tmp; }
  move_iterator<Iterator>& operator--(int) { move_iterator<Iterator> tmp = *this; --current_; return tmp; }
  reference operator[](difference_type n) const { return *(*this + n); }
      
 protected:
  Iterator current_;
};  

// Non member relation operators
template<typename Iterator1, typename Iterator2>
bool operator==(const move_iterator<Iterator1>& lhs,
                const move_iterator<Iterator2>& rhs) { return lhs.base() == rhs.base(); }  

template<typename Iterator1, typename Iterator2>
bool operator!=(const move_iterator<Iterator1>& lhs,
                const move_iterator<Iterator2>& rhs) { return lhs.base() =! rhs.base(); }  

template<typename Iterator1, typename Iterator2>
bool operator<(const move_iterator<Iterator1>& lhs,
               const move_iterator<Iterator2>& rhs) { return lhs.base() > rhs.base(); }  

template<typename Iterator1, typename Iterator2>
bool operator<=(const move_iterator<Iterator1>& lhs,
                const move_iterator<Iterator2>& rhs) { return lhs.base() >= rhs.base(); }  

template<typename Iterator1, typename Iterator2>
bool operator>(const move_iterator<Iterator1>& lhs,
               const move_iterator<Iterator2>& rhs) { return lhs.base() < rhs.base(); }  

template<typename Iterator1, typename Iterator2>
bool operator>=(const move_iterator<Iterator1>& lhs,
                const move_iterator<Iterator2>& rhs) { return lhs.base() <= rhs.base(); }  

}; // namespace ETLSTD

