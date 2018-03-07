/// @file metautils.h
/// @date 27/01/2014 16:02:23
/// @author Ambroise Leclerc and Cecile Thiebaut
/// @brief Embedded Template Library
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
//  POSSIBILITY OF SUCH DAMAGE./*

#pragma once 
#include <../libstd/include/cstdint>



namespace etl {

/// Produces an uint32_t hashcode of ztrings. Compile-time version of the algorithm can be
/// used to generate hashcodes from litterals using the "_hash" suffix.
/// Example with switch on strings :
///
/// using namespace etl;
///
/// void switchOnStrings(std::string command) {
///     switch (StringHash::calc(command.c_str())) {
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

    template<typename Iter>
    static uint32_t calc(Iter first, Iter last, uint32_t seed = 0) {
        while (first != last) {
            seed = seed * 101 + *first++;
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
