/// @file test/libstd/limits.cpp
/// @data 10/06/2016 22:23:53
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

#define ETLSTD etlstd

#include <libstd/include/limits>

using namespace ETLSTD;
using ETLSTD::size_t;



SCENARIO("std::numeric_limits") {
  
    auto r1 = numeric_limits<uint32_t>::is_signed;      REQUIRE(r1 == false);
    auto r2 = numeric_limits<uint32_t>::is_exact;       REQUIRE(r2 == true);
    auto r3 = numeric_limits<uint32_t>::is_integer;     REQUIRE(r3 == true);
    auto r4 = numeric_limits<uint32_t>::digits;         REQUIRE(r4 == 32);
    auto r5 = numeric_limits<uint32_t>::digits10;       REQUIRE(r5 == 9);
    auto r6 = numeric_limits<uint32_t>::max_digits10;   REQUIRE(r6 == 0);
  
    auto s1 = numeric_limits<uint8_t>::is_signed;       REQUIRE(s1 == false);
    auto s2 = numeric_limits<uint8_t>::is_exact;        REQUIRE(s2 == true);
    auto s3 = numeric_limits<uint8_t>::is_integer;      REQUIRE(s3 == true);
    auto s4 = numeric_limits<uint8_t>::digits;          REQUIRE(s4 == 8);
    auto s5 = numeric_limits<uint8_t>::digits10;        REQUIRE(s5 == 2);
    auto s6 = numeric_limits<uint8_t>::max_digits10;    REQUIRE(s6 == 0);
    auto s7 = numeric_limits<uint8_t>::min();           REQUIRE(s7 == 0);
    auto s8 = numeric_limits<uint8_t>::max();           REQUIRE(s8 == 255);
    auto s9 = numeric_limits<int8_t>::is_signed;        REQUIRE(s9 == true);

    auto t1 = numeric_limits<int64_t>::is_signed;       REQUIRE(t1 == true);
    auto t2 = numeric_limits<int64_t>::digits10;        REQUIRE(t2 == 18);
}
