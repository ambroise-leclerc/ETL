/// @file test/libstd/string.cpp
/// @data 28/09/2016 22:23:53
/// @author Ambroise Leclerc
/// @brief BDD tests for <string>
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

#define __Mock_Mock__
#define ETLSTD etlstd
#include <libstd/include/string>

using namespace ETLSTD;

SCENARIO("std::string subset owns null-terminated storage", "[libstd][string]") {
    string text("ETL");

    REQUIRE(text.size() == 3);
    REQUIRE(text.length() == 3);
    REQUIRE_FALSE(text.empty());
    REQUIRE(text.front() == 'E');
    REQUIRE(text.back() == 'L');
    REQUIRE(text[1] == 'T');
    REQUIRE(text.c_str()[3] == '\0');
    REQUIRE(text.view() == "ETL"sv);
}

SCENARIO("std::string subset supports embedded-friendly growth", "[libstd][string]") {
    string text("ET");

    text.reserve(8);
    REQUIRE(text.capacity() >= 8);
    REQUIRE(text.view() == "ET"sv);

    text += 'L';
    text.append(" std");
    text.push_back('!');
    REQUIRE(text.view() == "ETL std!"sv);

    text.append(text.data(), 3);
    REQUIRE(text.view() == "ETL std!ETL"sv);

    text.resize(3);
    REQUIRE(text.view() == "ETL"sv);
    REQUIRE(text.c_str()[3] == '\0');

    text.resize(5, '?');
    REQUIRE(text.view() == "ETL??"sv);

    text.clear();
    REQUIRE(text.empty());
    REQUIRE(text.size() == 0);
    REQUIRE(text.c_str()[0] == '\0');
}

SCENARIO("std::string subset interoperates with string_view and move semantics", "[libstd][string]") {
    string source("embedded");
    string copy(source);
    string_view prefix("embed");

    REQUIRE(copy == source);
    REQUIRE(copy.starts_with(prefix));

    char buffer[5] = {};
    REQUIRE(copy.copy(buffer, 4) == 4);
    REQUIRE(string_view(buffer, 4) == "embe"sv);

    string tail = copy.substr(3, 5);
    REQUIRE(tail.view() == "edded"sv);

    string moved(ETLSTD::move(copy));
    REQUIRE(moved.view() == "embedded"sv);
    REQUIRE(copy.empty());
}
