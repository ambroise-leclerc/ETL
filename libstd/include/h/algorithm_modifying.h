/// @file algorithm_modifying.h
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

#ifndef ETL_LIBSTD_ALGORITHM_MODIFYING_H_
#define ETL_LIBSTD_ALGORITHM_MODIFYING_H_

namespace std {

template<typename InputIterator, typename OutputIterator>
OutputIterator copy(InputIterator first, InputIterator last, OutputIterator d_first) {
  while (first != last) {
    *d_first++ = *first++;
  }
  return d_first;
}

template<typename InputIterator, typename OutputIterator, typename UnaryPredicate>
OutputIterator copy_if(InputIterator first, InputIterator last, OutputIterator d_first, UnaryPredicate pred) {
  while (first != last) {
    if (pred(*first))
      *d_first++ = *first;
    first++;
  }
  return d_first;
}

template<typename InputIterator, typename Size, typename OutputIterator>
OutputIterator copy_n(InputIterator first, Size count, OutputIterator result) {
  if (count > 0) {
    *result++ = *first;
    for (Size i = 1; i < count; ++i) {
      *result++ = *++first;
    }
  }
  return result;
}

template<typename BidirectionalIterator1, typename BidirectionalIterator2>
BidirectionalIterator2 copy_backward(BidirectionalIterator1 first, BidirectionalIterator1 last, BidirectionalIterator2 d_last)
{
  while (first != last) {
    *(--d_last) = *(--last);
  }
  return d_last;
}

template<typename InputIterator, typename OutputIterator>
OutputIterator move(InputIterator first, InputIterator last, OutputIterator d_first) {
  while (first != last) {
    *d_first++ = std::move(*first++);
  }
  return d_first;
}

template<typename BidirectionalIterator1, typename BidirectionalIterator2>
BidirectionalIterator2 move_backward(BidirectionalIterator1 first, BidirectionalIterator1 last, BidirectionalIterator2 d_last) {
  while (first != last) {
    *(--d_last) = std::move(*(--last));
  }
  return d_last;
}

template<typename ForwardIterator, typename T>
void fill(ForwardIterator first, ForwardIterator last, const T& value) {
  for (; first != last; ++first) {
    *first = value;
  }
}

template<typename OutputIterator, typename Size, typename T>
OutputIterator fill_n(OutputIterator first, Size count, const T& value) {
  for (Size i = 0; i < count; i++) {
    *first++ = value;
  }
  return first;
}

template<typename InputIterator, typename OutputIterator, typename UnaryOperation>
OutputIterator transform(InputIterator first1, InputIterator last1, OutputIterator d_first, UnaryOperation unary_op) {
  while (first1 != last1) {
    *d_first++ = unary_op(*first1++);
  }
  return d_first;
}

template<typename InputIterator1, typename InputIterator2, typename OutputIterator, typename BinaryOperation>
OutputIterator transform(InputIterator1 first1, InputIterator1 last1, InputIterator2 first2, OutputIterator d_first, BinaryOperation binary_op) {
  while (first1 != last1) {
    *d_first++ = binary_op(*first1++, *first2++);
  }
  return d_first;
}

template<typename ForwardIterator, typename Generator>
void generate(ForwardIterator first, ForwardIterator last, Generator g) {
  while (first != last) {
    *first++ = g();
  }
}

template<typename OutputIterator, typename Size, typename Generator>
OutputIterator generate_n(OutputIterator first, Size count, Generator g) {
  for( Size i = 0; i < count; i++ ) {
    *first++ = g();
  }
  return first;
}

template<typename ForwardIterator, typename T >
ForwardIterator remove(ForwardIterator first, ForwardIterator last, const T& value) {
  first = std::find(first, last, value);
  if (first != last)
    for(ForwardIterator i = first; ++i != last; )
      if (!(*i == value))
        *first++ = std::move(*i);
  return first;
}

template<typename ForwardIterator, typename UnaryPredicate>
ForwardIterator remove_if(ForwardIterator first, ForwardIterator last, UnaryPredicate p) {
  first = std::find_if(first, last, p);
  if (first != last)
    for (ForwardIterator i = first; ++i != last; )
      if (!p(*i))
        *first++ = std::move(*i);
  return first;
}

template<typename InputIterator, typename OutputIterator, typename T>
OutputIterator remove_copy(InputIterator first, InputIterator last, OutputIterator d_first, const T& value) {
  for (; first != last; ++first) {
    if (!(*first == value)) {
      *d_first++ = *first;
    }
  }
  return d_first;
}

template<typename InputIterator, typename OutputIterator, typename UnaryPredicate>
OutputIterator remove_copy_if(InputIterator first, InputIterator last, OutputIterator d_first, UnaryPredicate p) {
  for (; first != last; ++first) {
    if (!p(*first)) {
      *d_first++ = *first;
    }
  }
  return d_first;
}

template<typename ForwardIterator, typename T>
void replace(ForwardIterator first, ForwardIterator last, const T& old_value, const T& new_value) {
  for (; first != last; ++first) {
    if (*first == old_value) {
      *first = new_value;
    }
  }
}

template<typename ForwardIterator, typename UnaryPredicate, typename T>
void replace_if(ForwardIterator first, ForwardIterator last, UnaryPredicate p, const T& new_value) {
  for (; first != last; ++first) {
    if (p(*first)) {
      *first = new_value;
    }
  }
}

template<typename InputIterator, typename OutputIterator, typename T>
OutputIterator replace_copy(InputIterator first, InputIterator last, OutputIterator d_first, const T& old_value, const T& new_value) {
  for (; first != last; ++first) {
    *d_first++ = (*first == old_value) ? new_value : *first;
  }
  return d_first;
}

template<typename InputIterator, typename OutputIterator, typename UnaryPredicate, typename T>
OutputIterator replace_copy_if(InputIterator first, InputIterator last, OutputIterator d_first, UnaryPredicate p, const T& new_value) {
  for (; first != last; ++first) {
    *d_first++ = p( *first ) ? new_value : *first;
  }
  return d_first;
}

template<typename ForwardIterator1, typename ForwardIterator2>
void iter_swap(ForwardIterator1 a, ForwardIterator2 b) { std::swap(*a, *b); }

template<typename ForwardIterator1, class ForwardIterator2>
ForwardIterator2 swap_ranges(ForwardIterator1 first1, ForwardIterator1 last1, ForwardIterator2 first2) {
  while (first1 != last1) {
    std::iter_swap(first1++, first2++);
  }
  return first2;
}

template<typename BidirectionalIterator>
void reverse(BidirectionalIterator first, BidirectionalIterator last) {
  while ((first != last) && (first != --last)) {
    std::swap(*first++, *last);
  }
}

template<typename BidirectionalIterator, typename OutputIterator>
OutputIterator reverse_copy(BidirectionalIterator first, BidirectionalIterator last, OutputIterator d_first) {
  while (first != last) {
    *(d_first++) = *(--last);
  }
  return d_first;
}

template<typename ForwardIterator>
void rotate(ForwardIterator first, ForwardIterator n_first, ForwardIterator last) {
  ForwardIterator next = n_first;
  while (first != next) {
    std::iter_swap(first++, next++);
    if (next == last) {
      next = n_first;
    }
    else
    if (first == n_first) {
      n_first = next;
    }
  }
}

template<typename ForwardIterator, typename OutputIterator>
OutputIterator rotate_copy(ForwardIterator first, ForwardIterator n_first, ForwardIterator last, OutputIterator d_first) {
  d_first = std::copy(n_first, last, d_first);
  return std::copy(first, n_first, d_first);
}

template<typename RandomIterator, typename RandomFunc>
void random_shuffle(RandomIterator first, RandomIterator last, RandomFunc&& r) {
  typename std::iterator_traits<RandomIterator>::difference_type i, n;
  n = last - first;
  for (i = n-1; i > 0; --i) {
    std::swap(first[i], first[r(i+1)]);
  }
}
/*
template<typename RandomAccessIterator, typename UniformRandomNumberGenerator>
void shuffle(RandomAccessIterator first, RandomAccessIterator last, UniformRandomNumberGenerator&& g) {
  for (auto i = (last-first)-1; i>0; --i) {
    std::uniform_int_distribution<decltype(i)> distrib(0,i);
    std::swap(first[i], first[distrib(g)]);
  }
}
*/
template<typename ForwardIterator>
ForwardIterator unique(ForwardIterator first, ForwardIterator last) {
  if (first==last) return last;
 
  ForwardIterator result = first;
  while (++first!=last) {
    if (!(*result==*first)) {
      *(++result) = *first;
    }
  }
  return ++result;
}

template<typename ForwardIterator, typename BinaryPredicate>
ForwardIterator unique(ForwardIterator first, ForwardIterator last, BinaryPredicate p) {
  if (first==last) return last;
 
  ForwardIterator result = first;
  while (++first!=last) {
    if (!p(*result, *first)) {
      *(++result) = *first;
    }
  }
  return ++result;
}

template<typename ForwardIterator, typename OutputIterator>
ForwardIterator unique_copy(ForwardIterator first, ForwardIterator last, OutputIterator d_first) {
  if (first==last) return d_first;
 
  *d_first = *first;
  while (++first != last) {
    if (!(*d_first == *first)) {
        *(++d_first) = *first;
    }
  }
  return ++d_first;
}

template<typename ForwardIterator, typename OutputIterator, typename BinaryPredicate>
ForwardIterator unique_copy(ForwardIterator first, ForwardIterator last, OutputIterator d_first, BinaryPredicate p) { 
  if (first==last) return d_first;
 
  *d_first = *first;
  while (++first != last) {
    if (!p(*d_first, *first)) {
      *(++d_first) = *first;
    }
  }
  return ++d_first;
}

} // namespace std
#endif // ETL_LIBSTD_ALGORITHM_MODIFYING_H_