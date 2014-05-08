/// @file algorithm_minmax.h
/// @data 18/04/2014 14:24:53
/// @author Ambroise Leclerc
/// @brief A collection of functions especially designed to be used on ranges of elements.
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

#ifndef ETL_LIBSTD_ALGORITHM_MINMAX_H_
#define ETL_LIBSTD_ALGORITHM_MINMAX_H_

namespace std {
  
template<typename ForwardIt>
ForwardIt max_element(ForwardIt first, ForwardIt last) {
  if (first == last) { return last; }
  ForwardIt largest = first;
  ++first;
  for (; first != last; ++first) {
    if (*largest < *first) { largest = first; }
  }
  return largest;
}

template<typename ForwardIt, typename Compare>
ForwardIt max_element(ForwardIt first, ForwardIt last, Compare comp) {
  if (first == last) { return last; }
  ForwardIt largest = first;
  ++first;
  for (; first != last; ++first) {
    if (comp(*largest, *first)) { largest = first; }
  }
  return largest;
}

template<typename ForwardIt>
ForwardIt min_element(ForwardIt first, ForwardIt last) {
  if (first == last) { return last; }
  ForwardIt smallest = first;
  ++first;
  for (; first != last; ++first) {
      if (*first < *smallest) { smallest = first; }
  }
  return smallest;
}

template<typename ForwardIt, typename Compare>
ForwardIt min_element(ForwardIt first, ForwardIt last, Compare comp) {
  if (first == last) { return last; } 
  ForwardIt smallest = first;
  ++first;
  for (; first != last; ++first) {
    if (comp(*first, *smallest)) { smallest = first; }
  }
  return smallest;
}

template<typename T> 
constexpr const T& max(const T& a, const T& b ) {
  return (a < b) ? b : a;
}

template<typename T, typename Compare>
constexpr const T& max(const T& a, const T& b, Compare comp) {
  return (comp(a, b)) ? b : a;
}  

template<typename T>
constexpr T max( std::initializer_list<T> ilist ) {
  return *std::max_element(ilist.begin(), ilist.end());
}  

template<typename T, typename Compare>
constexpr T max( std::initializer_list<T> ilist, Compare comp) {
  return *std::max_element(ilist.begin(), ilist.end(), comp);
}

template<typename  T> 
constexpr const T& min(const T& a, const T& b) {
  return (b < a) ? b : a;
}
  
template<typename  T, typename Compare>
constexpr const T& min(const T& a, const T& b, Compare comp) {
  return (comp(b, a)) ? b : a;
}
  
template<typename T>
constexpr T min(std::initializer_list<T> ilist) {
  return *std::min_element(ilist.begin(), ilist.end());
}
  
template<typename T, typename Compare>
constexpr T min(std::initializer_list<T> ilist, Compare comp) {
  return *std::min_element(ilist.begin(), ilist.end(), comp);
}  

template<typename InputIt1, typename InputIt2>
bool lexicographical_compare(InputIt1 first1, InputIt1 last1,
                             InputIt2 first2, InputIt2 last2) {
    for (; (first1 != last1) && (first2 != last2); first1++, first2++ ) {
        if (*first1 < *first2) { return true; }
        if (*first2 < *first1) { return false; }
    }
    return (first1 == last1) && (first2 != last2);
}

template<typename InputIt1, typename InputIt2, typename Compare>
bool lexicographical_compare(InputIt1 first1, InputIt1 last1,
                             InputIt2 first2, InputIt2 last2,
                             Compare comp) {
    for ( ; (first1 != last1) && (first2 != last2); first1++, first2++ ) {
        if (comp(*first1, *first2)) { return true; }
        if (comp(*first2, *first1)) { return false; }
    }
    return (first1 == last1) && (first2 != last2);
}

} // namespace std  

#endif // ETL_LIBSTD_ALGORITHM_MINMAX_H_