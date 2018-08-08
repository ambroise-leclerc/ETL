/// @file test/libstd/memory.cpp
/// @data 06/06/2016 22:23:53
/// @author Ambroise Leclerc
/// @brief BDD tests for <tuple>
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
#include <string>

#define __Mock_Mock__
#define ETLSTD etlstd
#include <libstd/include/tuple>

using namespace ETLSTD;
/*
class TupleTest {
public:
    using Etq = const tuple<const char*, int, bool>;

    Etq static findEtiquette(uint8_t id) {
        switch (id) {
        case 0: return make_tuple("ADSC", 12, false);
        case 1: return make_tuple("VTIC", 2, false);
        }
        return make_tuple("UNDE", 0, false);
    }
};
*/

#include <iostream>


class Serializer {
public:
    ::std::string output;

    void f(ETLSTD::size_t s) { output += ::std::to_string(s); }

    template<typename T, T... Indices>
    void transform(integer_sequence<T, Indices...>) {
        int ignore[]{ (f(Indices), 0)... };
        (void)ignore;                       // avoid 'unused' warning
    }

};

SCENARIO("std::integer_sequence") {
    using seq5 = make_integer_sequence<int, 5>;
    using seq10 = make_index_sequence<10>;
    using seq18 = make_integer_sequence<char, 18>;
    using seq140 = make_integer_sequence<uint64_t, 140>;
    REQUIRE(seq5::size() == 5);
    REQUIRE(seq10::size() == 10);
    REQUIRE(seq18::size() == 18);
    REQUIRE(seq140::size() == 140);


    Serializer s;
    s.transform(index_sequence<4, 2, 3, 1, 5>{});
    REQUIRE(s.output == "42315");

    s.transform(make_integer_sequence<uint8_t, 12>{});
    REQUIRE(s.output == "4231501234567891011");

    s.transform(make_index_sequence<13>{});
    REQUIRE(s.output == "42315012345678910110123456789101112");
}

SCENARIO("std::tuple") {
    GIVEN("0 class instances") {

    }
}

