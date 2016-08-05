/// @file utility_pair.h
/// @data 19/04/2014 16:02:55
/// @author Ambroise Leclerc
/// @brief
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

#ifndef ETL_LIBSTD_UTILITY_PAIR_H
#define ETL_LIBSTD_UTILITY_PAIR_H

namespace std {
  
template<typename T1, typename T2>
struct pair {
  using first_type   = T1;
  using second_type  = T2;
  
  constexpr pair() : first(T1()), second(T2()) {}
  constexpr pair(const T1& x, const T2& y) : first(x), second(y) {}
    
  template<typename U1, typename U2>
  constexpr pair(U1&& x, U2&& y) : first(forward<U1>(x)), second(forward<U2>(y)){}
    
  template<typename U1, typename U2>
  constexpr pair( const pair<U1, U2>& p) : first(p.first), second(p.second) {}
    
  template<typename U1, typename U2>
  constexpr pair(pair<U1, U2>&& p) : first(move<U1>(p.first)), second(move<U2>(p.second)) {} 
  
  /*template<typename... Args1, typename... Args2 >
  pair( std::piecewise_construct_t, std::tuple<Args1...> first_args, std::tuple<Args2...> second_args );*/
      
  pair(const pair& p) = default;
  
  pair(pair&& p) = default;
  
  T1 first;
  T2 second;  
};

namespace etlHelper {
  
  // make_pair : The deduced types V1 and V2 are std::decay<T1>::type and std::decay<T2>::type (the
  // usual type transformations applied to arguments of functions passed by value) unless application
  // of std::decay results in std::reference_wrapper<X> for some type X, in which case the deduced type is X&.
template<typename T>
struct make_pair_decay {
  using type = T;
};
/*
template<typename T>
struct make_pair_decay<std::reference_wrapper<T>> {
  using type = T&;
};
*/
template<typename T>
struct make_pair_return_type {
  using type = typename make_pair_decay<typename std::decay<T>::type>::type;
};
  
} // namespace etlHelper

template<typename T1, typename T2>
constexpr auto make_pair(T1&& x, T2&& y)
 -> pair<typename etlHelper::make_pair_return_type<T1>::type, typename etlHelper::make_pair_return_type<T2>::type> {
  return pair<typename etlHelper::make_pair_return_type<T1>::type,
              typename etlHelper::make_pair_return_type<T2>::type>(forward<T1>(x), forward<T2>(y));
}

} // namespace std  

#endif // ETL_LIBSTD_UTILITY_PAIR_H