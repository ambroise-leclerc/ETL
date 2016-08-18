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
#include <libstd/include/limits>
} // namespace etlTest

using namespace etlTest::std;

SCENARIO("std::numeric_limits") {
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
    
}
