/// @file test/libstd/memory.cpp
/// @data 06/06/2016 22:23:53
/// @author Ambroise Leclerc
/// @brief BDD tests for <limits>
//
// Copyright (c) 2016, Ambroise Leclerc
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
#include <catch.hpp>

namespace etlTest {
//#include <libstd/include/limits>
} // namespace etlTest

namespace etlTest {
namespace std {


enum float_round_style {
    round_indeterminate = -1,
    round_toward_zero = 0,
    round_to_nearest = 1,
    round_toward_infinity = 2,
    round_toward_neg_infinity = 3
};

enum float_denorm_style {
    denorm_indeterminate = -1,
    denorm_absent = 0,
    denorm_present = 1
};

/// Base class for type traits. Wraps a static constant of specified type.
template<typename T, T v>
struct integral_constant {
    static constexpr T value = v;
    using value_type = T;
    using type = integral_constant;
    constexpr operator value_type() const { return value; }
    constexpr value_type operator()() const { return value; }
};

using true_type = integral_constant<bool, true>;	///> Specialization for type bool
using false_type = integral_constant<bool, false>;  ///> Specialization for type bool


namespace etlHelper {
    template<typename T, bool = true> struct type_is_signed : integral_constant<bool, T(-1) < T(0)> {};
    template<typename T> struct type_is_signed<T, false> : false_type {};
} // namespace etlHelper

namespace etlHelper {
    template<typename T> constexpr auto typeDigits() { return (sizeof(T) * 8 - type_is_signed<T>()); }
    template<typename T> constexpr auto typeDigits10() { return typeDigits<T>() * 643L / 2136; }
    template<typename T> constexpr T typeMax() {
        return type_is_signed<T>() ?
            ((((static_cast<T>(1) << (typeDigits<T>() - 1)) - 1) << 1) + 1) :
            ~static_cast<T>(0);
    }
    template<typename T> constexpr T typeMin() { return type_is_signed<T>() ? -typeMax<T>() - 1 : static_cast<T>(0); }

    template<typename T> class numericLimitsBase {
    protected:
        using type = T;

    public:
        static constexpr bool is_specialized = true;
        static constexpr type min() noexcept { return typeMin<type>(); }
        static constexpr type max() noexcept { return typeMax<type>(); }
        static constexpr type lowest() noexcept { return min(); }

        static constexpr int digits = typeDigits<type>();
        static constexpr int digits10 = typeDigits10<type>();;
        static constexpr int max_digits10 = 0;
        static constexpr bool is_signed = type_is_signed<type>::value;
        static constexpr bool is_integer = true;
        static constexpr bool is_exact = true;
        static constexpr int radix = 2;
        static constexpr type epsilon() noexcept { return 0; }
        static constexpr type round_error() noexcept { return 0; }
        static constexpr int min_exponent = 0;
        static constexpr int min_exponent10 = 0;
        static constexpr int max_exponent = 0;
        static constexpr int max_exponent10 = 0;
        static constexpr bool has_infinity = false;
        static constexpr bool has_quiet_NaN = false;
        static constexpr bool has_signaling_NaN = false;
        static constexpr float_denorm_style has_denorm = denorm_absent;
        static constexpr bool has_denorm_loss = false;
        static constexpr type infinity() noexcept { return type(); }
        static constexpr type quiet_NaN() noexcept { return type(); }
        static constexpr type signaling_NaN() noexcept { return type(); }
        static constexpr type denorm_min() noexcept { return static_cast<type>(0); }
        static constexpr bool is_iec559 = false;
        static constexpr bool is_bounded = true;
        static constexpr bool is_modulo = false;
        static constexpr bool traps = false;
        static constexpr bool tinyness_before = false;
        static constexpr float_round_style round_style = round_toward_zero;
    };

    // template class numericLimitsBase<unsigned char>;

} // namespace etlHelper


template<typename T> struct numeric_limits : public etlHelper::numericLimitsBase<T> {};

template<> struct numeric_limits<bool> : public etlHelper::numericLimitsBase<bool> {
    static constexpr bool min() noexcept { return false; }
    static constexpr bool max() noexcept { return true; }
    static constexpr bool lowest() noexcept { return min(); }
    static constexpr int digits = 1;
    static constexpr int digits10 = 0;
};

}

} // namespace etlTest

using namespace etlTest::std;

#include <iostream>
SCENARIO("std::numeric_limits") {
    std::cout << numeric_limits<unsigned char>::digits10 << "== 2\n";
    std::cout << numeric_limits<uint8_t>::is_signed << "== false\n";
    std::cout << numeric_limits<uint8_t>::is_exact << "== true\n";
    std::cout << numeric_limits<uint8_t>::is_integer << "== true\n";
    std::cout << numeric_limits<uint8_t>::digits << "== 8\n";
    std::cout << numeric_limits<uint8_t>::digits10 << "== 2\n";
    std::cout << numeric_limits<uint8_t>::max_digits10 << "== 0\n";

    std::cout << numeric_limits<uint8_t>::min() << "== 0\n";
    std::cout << numeric_limits<uint8_t>::max() << "== 255\n";

    std::cout << numeric_limits<int8_t>::is_signed << "== true\n";
    /*
    REQUIRE(numeric_limits<uint32_t>::is_signed == false);
    REQUIRE(numeric_limits<uint32_t>::is_exact == true);
    REQUIRE(numeric_limits<uint32_t>::is_integer == true);
    REQUIRE(numeric_limits<uint32_t>::digits == 32);
    REQUIRE(numeric_limits<uint32_t>::digits10 == 9);
    REQUIRE(numeric_limits<uint32_t>::max_digits10 == 0);

    REQUIRE(numeric_limits<uint8_t>::is_signed == false);
    REQUIRE(numeric_limits<uint8_t>::is_exact == true);
    REQUIRE(numeric_limits<uint8_t>::is_integer == true);
    REQUIRE(numeric_limits<uint8_t>::digits == 8);
    REQUIRE(numeric_limits<uint8_t>::digits10 == 2);
    REQUIRE(numeric_limits<uint8_t>::max_digits10 == 0);

    REQUIRE(numeric_limits<uint8_t>::min() == 0);
    REQUIRE(numeric_limits<uint8_t>::max() == 255);

    REQUIRE(numeric_limits<int8_t>::is_signed == true);
    //REQUIRE(is_signed<int32_t>::value);
   */ 
}
