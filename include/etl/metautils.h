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


template<typename T, typename M> M MemberType(M T::*);
template<typename T, typename M> T ClassType(M T::*);

template<typename T, typename R, R T::*M >
constexpr std::size_t OffsetOf() {
  return reinterpret_cast<std::size_t>(&(((T*)0)->*M));
}
#define CompileTimeOffsetOf(m) etl::OffsetOf<decltype(etl::ClassType(m)), decltype(etl::MemberType(m)), m>()


} // namespace etl
#endif // ETL_METAUTILS_H_