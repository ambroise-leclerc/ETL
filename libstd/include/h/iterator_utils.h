/// @file iterator_utils.h
/// @data 16/04/2014 15:46:53
/// @author Ambroise Leclerc
/// @brief Provides utilities for iterators.
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

#ifndef ETL_LIBSTD_ITERATOR_UTILS_H_
#define ETL_LIBSTD_ITERATOR_UTILS_H_
namespace std {
  
/// Convenience function template that constructs a std::reverse_iterator from
/// the given iterator iter.
/// @param iter iterator to be converter to reverse_iterator
/// @return a std::reverse_iterator constructed from iter
template<typename Iterator>
reverse_iterator<Iterator> make_reverse_iterator(Iterator iter) {
  return reverse_iterator<Iterator>(iter);
}

/// Convenience function template that constructs a std::move_iterator from
/// the given iterator iter.
/// @param iter iterator to be converter to move_iterator
/// @return a std::move_iterator constructed from iter
template<typename Iterator >
move_iterator<Iterator> make_move_iterator(Iterator iter) {
  return move_iterator<Iterator>(iter);
}
  
} // namespace std  

#endif // ETL_LIBSTD_ITERATOR_UTILS_H_