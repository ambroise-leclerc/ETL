/// @file functional_operators.h
/// @data 14/03/2014 17:02:55
/// @author Ambroise Leclerc
/// @brief Function objects.
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
  
namespace etlHelper {
	struct unspecified; // is_transparent type member is specified as *unspecified* in C++14 standard.
}

/// Addition.
/// @param[in] x,y arguments to be sumed
/// @return the sum of the two arguments.
template<typename T = void> struct plus {
  constexpr T operator()(const T& x, const T& y) const {
    return x + y;
  };    
  using first_argument_type = T;
  using second_argument_type = T;
  using result_type = T;
};  

/// std::plus<void> specialization with member type is_transparent
template<> struct plus<void> {
  template<typename T1, typename T2>
  auto operator()(T1&& x, T2&& y) const -> decltype(std::forward<T1>(x) + std::forward<T2>(y)) {
    return std::forward<T1>(x) + std::forward<T2>(y);
    typedef etlHelper::unspecified is_transparent __attribute__((__unused__));
  }    
};

/// Substraction.
/// @param[in] x,y arguments to be substracted
/// @return the difference of the two arguments.
template<typename T = void> struct minus {
  constexpr T operator()(const T& x, const T& y) const {
    return x - y;
  };    
  using first_argument_type = T;
  using second_argument_type = T;
  using result_type = T;
};  

/// std::minus<void> specialization with member type is_transparent
template<> struct minus<void> {
  template<typename T1, typename T2>
  auto operator()(T1&& x, T2&& y) const -> decltype(std::forward<T1>(x) - std::forward<T2>(y)) {
    return std::forward<T1>(x) - std::forward<T2>(y);
    typedef etlHelper::unspecified is_transparent __attribute__((__unused__));
  }    
};

/// Division.
/// @param[in] x,y arguments to divide
/// @return the result of the division of the two arguments.
template<typename T = void> struct divides {
  constexpr T operator()(const T& x, const T& y) const {
    return x / y;
  };    
  using first_argument_type = T;
  using second_argument_type = T;
  using result_type = T;
};  

/// std::divides<void> specialization with member type is_transparent
template<> struct divides<void> {
  template<typename T1, typename T2>
  auto operator()(T1&& x, T2&& y) const -> decltype(std::forward<T1>(x) / std::forward<T2>(y)) {
    return std::forward<T1>(x) / std::forward<T2>(y);
    typedef etlHelper::unspecified is_transparent  __attribute__((__unused__));
  }    
};

/// Multiplication.
/// @param[in] x,y arguments to multiply
/// @return the product of the two arguments.
template<typename T = void> struct multiplies{
  constexpr T operator()(const T& x, const T& y) const {
    return x * y;
  };    
  using first_argument_type = T;
  using second_argument_type = T;
  using result_type = T;
};  

/// std::<void> specialization with member type is_transparent
template<> struct multiplies<void> {
  template<typename T1, typename T2>
  auto operator()(T1&& x, T2&& y) const -> decltype(std::forward<T1>(x) * std::forward<T2>(y)) {
    return std::forward<T1>(x) * std::forward<T2>(y);
    typedef etlHelper::unspecified is_transparent  __attribute__((__unused__));
  }    
};

/// Modulus.
/// @param[in] x,y arguments
/// @return remainder of division of the two arguments.
template<typename T = void> struct modulus {
  constexpr T operator()(const T& x, const T& y) const {
    return x % y;
  };    
  using first_argument_type = T;
  using second_argument_type = T;
  using result_type = T;
};  

/// std::<void> specialization with member type is_transparent
template<> struct modulus<void> {
  template<typename T1, typename T2>
  auto operator()(T1&& x, T2&& y) const -> decltype(std::forward<T1>(x) % std::forward<T2>(y)) {
    return std::forward<T1>(x) % std::forward<T2>(y);
    typedef etlHelper::unspecified is_transparent  __attribute__((__unused__));
  }    
};

template<typename T = void> struct negate {
  constexpr T operator()(const T& x) const {
    return -x;
  };    
};  

template<> struct negate<void> {
  template<typename T>
  auto operator()(T&& x) const -> decltype(-std::forward<T>(x)) {
    return -std::forward<T>(x);
    typedef etlHelper::unspecified is_transparent  __attribute__((__unused__));
  }    
};

template<typename T = void> struct less {
  constexpr bool operator()(const T& x, const T& y) const {
    return x < y;
  };    
  using first_argument_type = T;
  using second_argument_type = T;
  using result_type = bool;
};  

/// std::less<void> specialization with member type is_transparent
template<> struct less<void> {
  template<typename T1, typename T2>
  auto operator()(T1&& x, T2&& y) const -> decltype(std::forward<T1>(x) < std::forward<T2>(y)) {
    return std::forward<T1>(x) < std::forward<T2>(y);
    typedef etlHelper::unspecified is_transparent  __attribute__((__unused__));
  }    
};


/// Checks whether x is equal to y.
/// @param[in] x,y values to compare
/// @return true if x == y, false otherwise.
template<typename T = void> struct equal_to {
  constexpr bool operator()(const T& x, const T& y) const {
    return x == y;
  };    
  using first_argument_type = T;
  using second_argument_type = T;
  using result_type = bool;
};  

/// std::equal_to<void> specialization with member type is_transparent
template<> struct equal_to<void> {
  template<typename T1, typename T2>
  auto operator()(T1&& x, T2&& y) const -> decltype(std::forward<T1>(x) == std::forward<T2>(y)) {
    return std::forward<T1>(x) == std::forward<T2>(y);
    typedef etlHelper::unspecified is_transparent  __attribute__((__unused__));
  }    
};

/// Checks whether x is different from y.
/// @param[in] x,y values to compare
/// @return true if x != y, false otherwise.
template<typename T = void> struct not_equal_to {
  constexpr bool operator()(const T& x, const T& y) const {
    return x != y;
  };    
  using first_argument_type = T;
  using second_argument_type = T;
  using result_type = bool;
};  

/// std::not_equal_to<void> specialization with member type is_transparent
template<> struct not_equal_to<void> {
  template<typename T1, typename T2>
  auto operator()(T1&& x, T2&& y) const -> decltype(std::forward<T1>(x) != std::forward<T2>(y)) {
    return std::forward<T1>(x) != std::forward<T2>(y);
    typedef etlHelper::unspecified is_transparent  __attribute__((__unused__));
  }    
};

/// Checks whether x is greater than y.
/// @param[in] x,y values to compare
/// @return true if x > y, false otherwise.
template<typename T = void> struct greater {
  constexpr bool operator()(const T& x, const T& y) const {
    return x > y;
  };    
  using first_argument_type = T;
  using second_argument_type = T;
  using result_type = bool;
};  

/// std::greater<void> specialization with member type is_transparent
template<> struct greater<void> {
  template<typename T1, typename T2>
  auto operator()(T1&& x, T2&& y) const -> decltype(std::forward<T1>(x) > std::forward<T2>(y)) {
    return std::forward<T1>(x) > std::forward<T2>(y);
    typedef etlHelper::unspecified is_transparent  __attribute__((__unused__));
  }    
};

/// Checks whether x is greater than or equal to y.
/// @param[in] x,y values to compare
/// @return true if x >= y, false otherwise.
template<typename T = void> struct greater_equal {
  constexpr bool operator()(const T& x, const T& y) const {
    return x >= y;
  };    
  using first_argument_type = T;
  using second_argument_type = T;
  using result_type = bool;
};  

/// std::greater_equal<void> specialization with member type is_transparent
template<> struct greater_equal<void> {
  template<typename T1, typename T2>
  auto operator()(T1&& x, T2&& y) const -> decltype(std::forward<T1>(x) >= std::forward<T2>(y)) {
    return std::forward<T1>(x) >= std::forward<T2>(y);
    typedef etlHelper::unspecified is_transparent  __attribute__((__unused__));
  }    
};

/// Checks whether x is less than or equal to y.
/// @param[in] x,y values to compare
/// @return true if x <= y, false otherwise.
template<typename T = void> struct less_equal {
  constexpr bool operator()(const T& x, const T& y) const {
    return x <= y;
  };    
  using first_argument_type = T;
  using second_argument_type = T;
  using result_type = bool;
};  

/// std::less_equal<void> specialization with member type is_transparent
template<> struct less_equal<void> {
  template<typename T1, typename T2>
  auto operator()(T1&& x, T2&& y) const -> decltype(std::forward<T1>(x) <= std::forward<T2>(y)) {
    return std::forward<T1>(x) <= std::forward<T2>(y);
    typedef etlHelper::unspecified is_transparent  __attribute__((__unused__));
  }    
};

template<typename T = void>
struct bit_not {
  using result_type   = T;
  using argument_type = T;
  constexpr T operator()(const T& arg) const { return ~arg; }
};

#ifdef __GNUG__
template<> struct bit_not<void> {
  template<typename T>
  constexpr auto operator()(T&& arg) const -> decltype(~std::forward<T>(arg));
  typedef etlHelper::unspecified is_transparent  __attribute__((__unused__));
};
#endif

} // namespace ETLSTD
