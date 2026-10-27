/// @file test/libstd/array.cpp
/// @date 01/07/2026
/// @author Ambroise Leclerc
/// @brief Tests for <array>
//
// Copyright (c) 2026, Ambroise Leclerc
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
#include "../libstd_harness/harness.h"

#include <array>
#include <stdexcept>
#include <type_traits>

using namespace std;

TEST_CASE("std::array const accessors") {
    array<int, 3> values;
    values[0] = 1;
    values[1] = 2;
    values[2] = 3;
    const array<int, 3> &constValues = values;
    using const_pointer = typename array<int, 3>::const_pointer;
    using const_data_type = decltype(constValues.data());

    CHECK((is_same_v<const_pointer, const int *>));
    CHECK((is_same_v<const_data_type, const int *>));

    CHECK(constValues.data() == values.data());
    CHECK(constValues.front() == 1);
    CHECK(constValues.back() == 3);
    CHECK(constValues[1] == 2);
    CHECK(constValues.at(2) == 3);

    CHECK(constValues.rbegin().base() == constValues.end());
    CHECK(*constValues.rbegin() == 3);
    CHECK(constValues.rend().base() == constValues.begin());

    CHECK(constValues.crbegin().base() == constValues.cend());
    CHECK(*constValues.crbegin() == 3);
    CHECK(constValues.crend().base() == constValues.cbegin());
}

TEST_CASE("std::array zero-sized specialization") {
    array<int, 0> values{};
    const array<int, 0> &constValues = values;

    CHECK(values.empty());
    CHECK(values.size() == 0);
    CHECK(values.max_size() == 0);

    CHECK(values.data() == nullptr);
    CHECK(constValues.data() == nullptr);
    CHECK(values.begin() == values.end());
    CHECK(constValues.begin() == constValues.end());
    CHECK(values.cbegin() == values.cend());
    CHECK(constValues.cbegin() == constValues.cend());
    CHECK(values.rbegin().base() == values.end());
    CHECK(constValues.rbegin().base() == constValues.end());
    CHECK(constValues.crbegin().base() == constValues.cend());

    CHECK_THROWS_AS(values[0], out_of_range);
    CHECK_THROWS_AS(constValues[0], out_of_range);
    CHECK_THROWS_AS(values.at(0), out_of_range);
    CHECK_THROWS_AS(constValues.at(0), out_of_range);
    CHECK_THROWS_AS(values.front(), out_of_range);
    CHECK_THROWS_AS(constValues.front(), out_of_range);
    CHECK_THROWS_AS(values.back(), out_of_range);
    CHECK_THROWS_AS(constValues.back(), out_of_range);
}
