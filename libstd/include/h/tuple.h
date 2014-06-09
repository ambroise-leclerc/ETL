/// @file tuple
/// @data 06/06/2014 16:15:53
/// @author Ambroise Leclerc
/// @brief Tuple : a fixed-size collection of heterogeneous values.
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

#ifndef ETL_LIBSTD_TUPLE_H_
#define ETL_LIBSTD_TUPLE_H_

// "Some implementations of std::tuple use recursive inheritance. But the good ones don't. ;-)"
// So this is a quick std::tuple implementation with huge compile times and wrong memory order.
#include <functional>
#include <type_traits>

namespace etlHelper {
  
template<typename T>
struct deep_decay {
  using type = typename std::decay<T>::type;
};
 
template<typename T>
struct deep_decay<std::reference_wrapper<T>> {
    using type = T&;
};
 
template<typename T>
using deep_decay_t = typename deep_decay<T>::type;

} // namespace etlHelper  

namespace std {
 
// empty tuple
template<> class tuple<> {};
  
// recursive tuple definition
template<typename First, typename... Rest>
struct tuple<First, Rest...>: private tuple<Rest...> {
 // tuple(First first, Rest... rest): tuple<Rest...>(rest...), first_(first) {}

  First first_;
};

// tuple_value, an helper accessor class for std::tuple values.
template<std::size_t Index, typename Tuple> struct tuple_element;

// tuple_value<0...> access to  first value
template<typename First, typename... Rest>
struct tuple_element<0, std::tuple<First, Rest...>> {
  using type = First&;
  using TupleType = std::tuple<First, Rest...>;
};

// recursive tuple_value definition
template<size_t Index, typename First, typename... Rest>
struct tuple_element<Index, std::tuple<First, Rest...>>
 : public tuple_element<Index - 1, std::tuple<Rest...>> {
 };   
  


/// Extract the Ith element from the tuple.
/// @param[in] tuple std::tuple whose contents will be extracted
/// @return Reference to the selected element of tuple
template<std::size_t I, typename... Types>
auto get(std::tuple<Types...>& tuple) -> typename std::tuple_element<I, std::tuple<Types...>>::type {
  using ElementTuple = typename std::tuple_element<I, std::tuple<Types...>>::TupleType;
  return reinterpret_cast<ElementTuple&>(tuple).first_;
}  


/// Creates a tuple object from the arguments.
/// @param[in] args... arguments to construct the tuple from
/// @return a std::stuple object.
template<typename... Types>
constexpr std::tuple<typename ::etlHelper::deep_decay_t<Types>...> make_tuple(Types&&... args)  {
  return std::tuple<::etlHelper::deep_decay_t<Types>...>(std::forward<Types>(args)...);
}



} // namespace std

#endif // ETL_LIBSTD_TUPLE_H_