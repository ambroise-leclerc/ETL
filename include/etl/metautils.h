/*
 * metautils.h
 *
 * Created: 27/01/2014 16:02:47
 *  Author: Ambroise Leclerc
 */ 


#ifndef ETL_METAUTILS_H_
#define ETL_METAUTILS_H_
#include <cstdint>



namespace etl {

/// Produces an uint32_t hashcode of ztrings. Compile-time version of the algorithm can be
/// used to generate hashcodes from litterals using the "_hash" suffix.
/// Example with switch on strings :
///
/// void switchOnStrings(std::string command) {
///     switch (StringHash.calc(command.c_str())) {
///     case "Start"_hash:
///        ...
///     case "Stop"_hash:
///        ...
///     case "DoSomething"_hash:
///        ...
///     }
/// }
class StringHash {
public:
    static uint32_t calc(const char* zStr, uint32_t seed = 0) {
        while (*zStr) {
            seed = seed * 101 + *zStr++;
        }
        return seed;
    }

private:
    static constexpr uint32_t compileTime(const char* zStr, uint32_t seed = 0) {
        return *zStr ? (compileTime(zStr + 1, (seed * 101ull) + *zStr)) : seed;
    }

    friend constexpr uint32_t operator""_hash(char const* zStr, size_t);
};

constexpr uint32_t operator""_hash(char const* zStr, size_t) {
    return etl::StringHash::compileTime(zStr);
}

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
constexpr auto OffsetOf() {
  return reinterpret_cast<size_t>(&(((T*)0)->*M));
}
#define CompileTimeOffsetOf(m) etl::OffsetOf<decltype(etl::ClassType(m)), decltype(etl::MemberType(m)), m>()


} // namespace etl
#endif // ETL_METAUTILS_H_