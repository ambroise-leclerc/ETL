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
#include "../libstd_harness/harness.h"

#include <string_view>

using namespace std;

TEST_CASE("Test of std::string_view", "[libstd][string_view]") {
    const char *str = "std::string_view";
    string_view v = str;
    CHECK(v.size() == 16);
    CHECK(v.length() == 16);
    CHECK(v.llength() == 16);
    for (size_t index = 0; index < v.size(); ++index)
        CHECK(v[index] == str[index]);
    CHECK(v.compare("std::string_view") == 0);

    CHECK(v.starts_with('s'));
    CHECK(v.starts_with("std::s"));

    string_view v2("std::string");
    CHECK(v.starts_with(v2));
}

TEST_CASE("std::string_view element access and copy", "[libstd][string_view]") {
    string_view v("string_view");

    CHECK(v.at(0) == 's');
    CHECK(v.at(v.size() - 1) == 'w');
    CHECK_THROWS_AS(v.at(v.size()), out_of_range);
    CHECK(v.back() == 'w');

    char copied[8] = {};
    CHECK(v.copy(copied, 6) == 6);
    CHECK(string_view(copied, 6) == "string"sv);
    CHECK(v.copy(copied, sizeof(copied), 7) == 4);
    CHECK(string_view(copied, 4) == "view"sv);
    CHECK_THROWS_AS(v.copy(copied, sizeof(copied), v.size() + 1), out_of_range);

    CHECK(v.substr(7) == "view"sv);
    CHECK_THROWS_AS(v.substr(v.size() + 1), out_of_range);
}

TEST_CASE("std::string_view reverse iteration", "[libstd][string_view]") {
    string_view v("abc");
    char reversed[4] = {};
    size_t index = 0;

    for (auto it = v.rbegin(); it != v.rend(); ++it)
        reversed[index++] = *it;

    CHECK(index == 3);
    CHECK(string_view(reversed, 3) == "cba"sv);
    CHECK(*v.rbegin() == 'c');

    string_view empty;
    CHECK(empty.rbegin() == empty.rend());
    CHECK(empty.crbegin() == empty.crend());
}

TEST_CASE("std::string_view empty boundary behavior", "[libstd][string_view]") {
    string_view empty;
    CHECK(empty.empty());
    CHECK(empty.size() == 0);
    CHECK(empty.begin() == empty.end());
    CHECK(empty.cbegin() == empty.cend());
    CHECK_THROWS_AS(empty.at(0), out_of_range);
    CHECK(empty.substr(0).empty());
    CHECK_THROWS_AS(empty.substr(1), out_of_range);

    char untouched[2] = {'x', 'y'};
    CHECK(empty.copy(untouched, 2) == 0);
    CHECK(untouched[0] == 'x');
    CHECK(untouched[1] == 'y');

    CHECK(empty.find(""sv) == 0);
    CHECK(empty.rfind(""sv) == 0);
    CHECK(empty.find('x') == string_view::npos);
    CHECK(empty.find("x") == string_view::npos);
    CHECK(empty.find_first_of("x") == string_view::npos);
    CHECK(empty.find_last_of("x") == string_view::npos);
    CHECK(empty.find_first_not_of("") == string_view::npos);
    CHECK(empty.find_last_not_of("") == string_view::npos);

    string_view v("abc");
    CHECK(v.find(""sv, v.size()) == v.size());
    CHECK(v.find(""sv, v.size() + 1) == string_view::npos);
    CHECK(v.rfind(""sv) == v.size());
    CHECK(v.rfind("bc"sv, 99) == 1);
    CHECK(v.find_first_not_of("") == 0);
    CHECK(v.find_last_not_of("") == v.size() - 1);
}

TEST_CASE("std::string_view find functions", "[libstd][string_view]") {
    string_view v("original string with five words.");

    CHECK(v.find("string") == 9);
    CHECK(v.find("words.") == 26);
    CHECK(v.find("banana") == string_view::npos);

    string_view with("with");
    CHECK(v.find(with) == 16);

    CHECK(v.find(with, 10) == 16);

    string_view v2("string with four words.");
    CHECK(v.find(v2.substr(7, 4)) == 16);
}

TEST_CASE("std::string_view rfind functions", "[libstd][string_view]") {
    string_view v("String with multiple 'with' words within.");

    CHECK(v.find("with") == 7);
    CHECK(v.rfind("with") == 34);
    CHECK(v.substr(0, 34).rfind("with") == 22);
    CHECK(v.substr(0, 22).rfind("with") == 7);
    CHECK(v.find("banana") == string_view::npos);
    CHECK(v.find("String") == 0);

    string_view with("'with'");
    CHECK(v.find(with) == 21);
}

TEST_CASE("std::string_view find_first_of functions", "[libstd][string_view]") {
    string_view v("String with multiple 'with' words within.");

    CHECK(v.find_first_of("mdkz") == 12);
    CHECK(v.find_first_of('.') == v.size() - 1);

    string_view tiple("tiple");
    CHECK(v.find_first_of(tiple, 2) == 3);
}

TEST_CASE("std::string_view find_last_of functions", "[libstd][string_view]") {
    string_view v("String with multiple 'with' words within.");

    CHECK(v.find_last_of("mdkz") == 31);
    CHECK(v.find_last_of('.') == v.size() - 1);

    string_view tiple("tiple");
    CHECK(v.find_last_of(tiple, 34) == 24);
}

TEST_CASE("std::string_view find_first_not_of functions", "[libstd][string_view]") {
    string_view v("String with multiple 'with' words within.");

    CHECK(v.find_first_not_of("String wh") == 12);
    CHECK(v.find_first_not_of("'String whmulpeordswn") == v.size() - 1);
    CHECK(v.find_first_not_of('S') == 1);
}

TEST_CASE("std::string_view find_last_not_of functions", "[libstd][string_view]") {
    string_view v("String with multiple 'with' words within.");

    CHECK(v.find_last_not_of(". withn") == 32);
    CHECK(v.find_last_not_of('.') == v.size() - 2);
}

TEST_CASE("std::string_view comparisons", "[libstd][string_view]") {
    auto s1{"String1"sv};
    auto s2 = "String2"sv;
    string_view s3("String1 longer");
    string_view s4("String1");

    CHECK(s1 == s4);
    CHECK(s2 > s1);
    CHECK(s2 > s4);
    CHECK(s1 != s3);
    CHECK(s2 >= s1);
    CHECK(s4 >= s1);
    CHECK(s1 <= s2);
    CHECK(s1 <= s4);
}

TEST_CASE("std::string trim functions", "[libstd][string_view]") {
    const char *str = "   trim me";
    string_view v = str;
    CHECK(v.compare("   trim me") == 0);
    v.remove_prefix(std::min(v.find_first_not_of(" "), v.size()));

    CHECK(v.compare("trim me") == 0);
}
