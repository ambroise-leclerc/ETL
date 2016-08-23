/// @file test/libstd/type_traits.cpp
/// @data 19/08/2016 17:40:53
/// @author Ambroise Leclerc
/// @brief BDD tests for <type_traits>
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
#include <libstd/include/type_traits>
} // namespace etlTest

using namespace etlTest::std;

SCENARIO("std::signed") {
    auto s1 = is_signed<uint8_t>::value;    REQUIRE(s1 == false);
    auto s2 = is_signed<char>::value;       REQUIRE(s2 == true);
    auto s3 = is_signed<int32_t>::value;    REQUIRE(s3 == true);
    auto s4 = is_signed<uint64_t>::value;   REQUIRE(s4 == false);
    REQUIRE(is_signed<uint8_t>::value == false);

#if (__GNUC__ > 4) 
    REQUIRE(is_signed_v<int16_t> == true);
    REQUIRE(is_signed_v<uint32_t> == false);
#endif
}

SCENARIO("std::common_type, std::same_type") {
    class Base {};
    class Incarnation1 : Base {};
    class Incarnation2 : Base {};

    //using CommonType = common_type<Incarnation1, Incarnation2>::type;
#if defined(__GNU_G__)
    auto test = is_same<Base, common_type<Incarnation1, Incarnation2>::type >::value == true;
#endif
}