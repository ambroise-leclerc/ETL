/// @file algorithm_sequence.h
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
#pragma once
#include <libstd/include/utility>
#include <libstd/include/iterator>

namespace ETLSTD {

template<typename ForwardIterator1, typename ForwardIterator2>
ForwardIterator1 search(ForwardIterator1 first, ForwardIterator1 last, ForwardIterator2 s_first, ForwardIterator2 s_last) {
  for (; ; ++first) {
    ForwardIterator1 it = first;
    for (ForwardIterator2 s_it = s_first; ; ++it, ++s_it) {
      if (s_it == s_last) return first;
      if (it == last) return last;
      if (!(*it == *s_it)) {
        break;
      }
    }
  }
}

template<typename ForwardIterator1, typename ForwardIterator2, typename BinaryPredicate>
ForwardIterator1 search(ForwardIterator1 first, ForwardIterator1 last, ForwardIterator2 s_first, ForwardIterator2 s_last, BinaryPredicate p) {
  for (; ; ++first) {
    ForwardIterator1 it = first;
    for (ForwardIterator2 s_it = s_first; ; ++it, ++s_it) {
      if (s_it == s_last) return first;
      if (it == last) return last;
      if (!p(*it, *s_it)) {
        break;
      }
    }
  }
}

template<typename ForwardIterator, typename Size, typename T>
ForwardIterator search_n(ForwardIterator first, ForwardIterator last, Size count, const T& value) {
  for(; first != last; ++first) {
    if (!(*first == value)) {
      continue;
    }
 
    ForwardIterator candidate = first;
    Size cur_count = 0;
 
    while (true) {
      if (++cur_count == count) return candidate;
      if (++first == last) return last;
      if (!(*first == value)) break;
    }
  }
  return last;
}

template<typename ForwardIterator, typename Size, typename T, typename BinaryPredicate>
ForwardIterator search_n(ForwardIterator first, ForwardIterator last, Size count, const T& value, BinaryPredicate p) {
  for(; first != last; ++first)
    if (!p(*first, value)) continue;
    
  ForwardIterator candidate = first;
  Size cur_count = 0;
 
  while (true) {
    if (++cur_count == count) return candidate;
    if (++first == last) return last;
    if (!p(*first, value)) break;
  }
  return last;
}

template<typename InputIterator, typename T>
InputIterator find(InputIterator first, InputIterator last, const T& value) {
  for (; first != last; ++first) {
    if (*first == value) {
      return first;
    }
  }
  return last;
}

template<typename InputIterator, typename UnaryPredicate>
InputIterator find_if(InputIterator first, InputIterator last, UnaryPredicate p) {
  for (; first != last; ++first) {
    if (p(*first)) {
      return first;
    }
  }
  return last;
}

template<typename InputIterator, typename UnaryPredicate>
InputIterator find_if_not(InputIterator first, InputIterator last, UnaryPredicate q) {
  for (; first != last; ++first) {
    if (!q(*first)) {
      return first;
    }
  }
  return last;
}

template<typename InputIterator, typename UnaryPredicate>
bool all_of(InputIterator first, InputIterator last, UnaryPredicate p) {
  return find_if_not(first, last, p) == last;
}

template<typename InputIterator, typename UnaryPredicate>
bool any_of(InputIterator first, InputIterator last, UnaryPredicate p) {
  return find_if(first, last, p) != last;
}

template<typename InputIterator, typename UnaryFunction>
UnaryFunction for_each(InputIterator first, InputIterator last, UnaryFunction f) {
  for (; first != last; ++first) {
    f(*first);
  }
  return f;
}

template<typename InputIterator, typename T>
typename iterator_traits<InputIterator>::difference_type count(InputIterator first, InputIterator last, const T& value) {
  typename iterator_traits<InputIterator>::difference_type ret = 0;
  for (; first != last; ++first) {
    if (*first == value) {
      ret++;
    }
  }
  return ret;
}

template<typename InputIterator, typename UnaryPredicate>
typename iterator_traits<InputIterator>::difference_type count_if(InputIterator first, InputIterator last, UnaryPredicate p) {
  typename iterator_traits<InputIterator>::difference_type ret = 0;
  for (; first != last; ++first) {
    if (p(*first)) {
      ret++;
    }
  }
  return ret;
}

template<typename InputIterator1, typename InputIterator2>
pair<InputIterator1, InputIterator2> mismatch(InputIterator1 first1, InputIterator1 last1, InputIterator2 first2) {
  while (first1 != last1 && *first1 == *first2) {
    ++first1, ++first2;
  }
  return make_pair(first1, first2);
}

template<typename InputIterator1, typename InputIterator2, typename BinaryPredicate>
pair<InputIterator1, InputIterator2> mismatch(InputIterator1 first1, InputIterator1 last1, InputIterator2 first2, BinaryPredicate p) {
  while (first1 != last1 && p(*first1, *first2)) {
    ++first1, ++first2;
  }
  return make_pair(first1, first2);
}

template<typename InputIterator1, typename InputIterator2>
bool equal(InputIterator1 first1, InputIterator1 last1, InputIterator2 first2) {
  for (; first1 != last1; ++first1, ++first2) {
    if (!(*first1 == *first2)) {
      return false;
    }
  }
  return true;
}

template<typename InputIterator1, typename InputIterator2, typename BinaryPredicate>
bool equal(InputIterator1 first1, InputIterator1 last1, InputIterator2 first2, BinaryPredicate p) {
  for (; first1 != last1; ++first1, ++first2) {
    if (!p(*first1, *first2)) {
      return false;
    }
  }
  return true;
}


template<typename ForwardIterator1, typename ForwardIterator2>
ForwardIterator1 find_end(ForwardIterator1 first, ForwardIterator1 last, ForwardIterator2 s_first, ForwardIterator2 s_last) {
  if (s_first == s_last) return last;
  ForwardIterator1 result = last;
  while (true) {
    ForwardIterator1 new_result = search(first, last, s_first, s_last);
    if (new_result == last) return result;
    else {
      result = new_result;
      first = result;
      ++first;
    }
  }
  return result;
}

template<typename ForwardIterator1, typename ForwardIterator2, typename BinaryPredicate>
ForwardIterator1 find_end(ForwardIterator1 first, ForwardIterator1 last, ForwardIterator2 s_first, ForwardIterator2 s_last, BinaryPredicate p) {
  if (s_first == s_last) return last;
  ForwardIterator1 result = last;
  while (true) {
    ForwardIterator1 new_result = search(first, last, s_first, s_last, p);
    if (new_result == last) return result;
    else {
      result = new_result;
      first = result;
      ++first;
    }
  }
  return result;
}

template<typename InputIterator, typename ForwardIterator>
InputIterator find_first_of(InputIterator first, InputIterator last, ForwardIterator s_first, ForwardIterator s_last) {
  for (; first != last; ++first) {
    for (auto it = s_first; it != s_last; ++it) {
      if (*first == *it) return first;
    }
  }
  return last;
}

template<typename InputIterator, typename ForwardIterator, typename BinaryPredicate>
InputIterator find_first_of(InputIterator first, InputIterator last, ForwardIterator s_first, ForwardIterator s_last, BinaryPredicate p) {
  for (; first != last; ++first) {
    for (auto it = s_first; it != s_last; ++it) {
      if (p(*first, *it)) return first;
    }
  }
  return last;
}

template<typename ForwardIterator>
ForwardIterator adjacent_find(ForwardIterator first, ForwardIterator last) {
  if (first == last) return last;
  ForwardIterator next = first;
  ++next;
  for (; next != last; ++next, ++first) {
    if (*first == *next) return first;
  }
  return last;
}

template<typename ForwardIterator, typename BinaryPredicate>
ForwardIterator adjacent_find(ForwardIterator first, ForwardIterator last, BinaryPredicate p) {
  if (first == last) return last;
  ForwardIterator next = first;
  ++next;
  for (; next != last; ++next, ++first) {
    if (p(*first, *next)) return first;
  }
  return last;
}

} // namespace ETLSTD
