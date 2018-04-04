/// @file test/libstd/string_view.cpp
/// @data 26/03/2018 18:47:10
/// @author Ambroise Leclerc
/// @brief BDD tests for <string_view>
//
// Copyright (c) 2018, Ambroise Leclerc
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
#include <libstd/include/string_view>

using namespace ETLSTD;

SCENARIO("Test of std::string_view") {
    const char* str = "std::string_view";
    string_view v = str;
    REQUIRE(v.size() == 16);
    for (size_t index = 0; index < v.size(); ++index)
        REQUIRE(v[index] == str[index]);
    REQUIRE(v.compare("std::string_view") == 0);

    REQUIRE(v.starts_with('s'));
    REQUIRE(v.starts_with("std::s"));

    string_view v2("std::string");
    REQUIRE(v.starts_with(v2));

}

SCENARIO("std::string_view find functions") {
    string_view v("original string with five words.");

    REQUIRE(v.find("string") == 9);
    REQUIRE(v.find("words.") == 26);
    REQUIRE(v.find("banana") == string_view::npos);

    string_view with("with");
    REQUIRE(v.find(with) == 16);

    REQUIRE(v.find(with, 10) == 16);

    string_view v2("string with four words.");
    REQUIRE(v.find(v2.substr(7, 4)) == 16);
}


SCENARIO("std::string_view rfind functions") {
    string_view v("String with multiple 'with' words within.");

    REQUIRE(v.find("with") == 7);
    REQUIRE(v.rfind("with") == 34);
    REQUIRE(v.substr(0, 34).rfind("with") == 22);
    REQUIRE(v.substr(0, 22).rfind("with") == 7);
    REQUIRE(v.find("banana") == string_view::npos);
    REQUIRE(v.find("String") == 0);

    string_view with("'with'");
    REQUIRE(v.find(with) == 21);
}

SCENARIO("std::string_view find_first_of functions") {
    string_view v("String with multiple 'with' words within.");

    REQUIRE(v.find_first_of("mdkz") == 12);
    REQUIRE(v.find_first_of('.') == v.size() - 1);

    string_view tiple("tiple");
    REQUIRE(v.find_first_of(tiple, 2) == 3);
}

SCENARIO("std::string_view find_last_of functions") {
    string_view v("String with multiple 'with' words within.");

    REQUIRE(v.find_last_of("mdkz") == 31);
    REQUIRE(v.find_last_of('.') == v.size() - 1);

    string_view tiple("tiple");
    REQUIRE(v.find_last_of(tiple, 34) == 24);
}

SCENARIO("std::string_view find_first_not_of functions") {
    string_view v("String with multiple 'with' words within.");

    REQUIRE(v.find_first_not_of("String wh") == 12);
    REQUIRE(v.find_first_not_of("'String whmulpeordswn") == v.size() - 1);
    REQUIRE(v.find_first_not_of('S') == 1);
}

SCENARIO("std::string_view find_last_not_of functions") {
    string_view v("String with multiple 'with' words within.");

    REQUIRE(v.find_last_not_of(". withn") == 32);
    REQUIRE(v.find_last_not_of('.') == v.size() - 2);
}

SCENARIO("std::string_view comparisons") {
    auto s1 { "String1"sv };
    auto s2 = "String2"sv;
    string_view s3("String1 longer");
    string_view s4("String1");


    REQUIRE(s1 == s4);
    REQUIRE(s2 > s1);
    REQUIRE(s2 > s4);
    REQUIRE(s1 != s3);
    REQUIRE(s2 >= s1);
    REQUIRE(s4 >= s1);
    REQUIRE(s1 <= s2);
    REQUIRE(s1 <= s4);
}


SCENARIO("std::string trim functions") {
    const char *str = "   trim me";
    string_view v = str;
    REQUIRE(v.compare("   trim me") == 0);
    v.remove_prefix(std::min(v.find_first_not_of(" "), v.size()));

    REQUIRE(v.compare("trim me") == 0);
}