/*
 * metautils.h
 *
 * Created: 27/01/2014 16:02:47
 *  Author: Ambroise Leclerc
 */ 


#ifndef ETL_METAUTILS_H_
#define ETL_METAUTILS_H_
#include <cstddef>

namespace etl {
  
template <typename T, T v>
struct integral_constant {
  static constexpr T value = v;
  typedef T value_type;
  typedef integral_constant<T,v> type;
  constexpr operator T() { return v; }
};  
  
typedef integral_constant<bool, true> true_type;
typedef integral_constant<bool, true> false_type;
 
/// \brief Determines whether the two given types are equivalent.
template<typename T, typename U> struct is_same       : false_type {};
template<typename T>             struct is_same<T, T> : true_type {};  

/// \brief Determines whether the given type is void.
template<typename T> struct is_void : false_type {};
template<> struct is_void<void> : true_type {};
 
/// \brief Provides member typedef type, which is defined as T if B is true at
/// compile time, or as F if B is false.
template<bool, class T, class F>  struct conditional { typedef T type; };
template<class T, class F>        struct conditional<false, T, F> { typedef F type; };

class EmptyType {};

// Comparison of two types :
//  SameType<int, float>::result == 1
//  SameType<int, int>::result == 0
template<typename X, typename Y>
struct SameType
{
  enum { result = 0 };
};

template<typename T>
struct SameType<T, T>
{
  enum { result = 1 };
};


template<typename T1, typename T2>
auto Min(const T1& t1, const T2& t2) -> decltype((t1 < t2) ? t1 : t2) {
    return ((t1 < t2) ? t1 : t2);
}

template<typename T1, typename T2, typename...T3>
auto Min(const T1& t1, const T2& t2, const T3&... t3) -> decltype(Min(((t1 < t2) ? t1 : t2), t3...)) {
  return Min(((t1 < t2) ? t1 : t2), t3...);
}

template<typename T1, typename T2>
auto Max(const T1& t1, const T2& t2) -> decltype((t2 < t1) ? t1 : t2) {
  return ((t2 < t1) ? t1 : t2);
}

template<typename T1, typename T2, typename...T3>
auto Max(const T1& t1, const T2& t2, const T3&... t3) -> decltype(Max(((t2 < t1) ? t1 : t2), t3...)) {
  return Max(((t2 < t1) ? t1 : t2), t3...);
}


template <typename T, typename M> M MemberType(M T::*);
template <typename T, typename M> T ClassType(M T::*);

template <typename T, typename R, R T::*M >
constexpr std::size_t OffsetOf() {
  return reinterpret_cast<std::size_t>(&(((T*)0)->*M));
}
#define CompileTimeOffsetOf(m) etl::OffsetOf<decltype(etl::ClassType(m)), decltype(etl::MemberType(m)), m>()


} // namespace etl
#endif // ETL_METAUTILS_H_