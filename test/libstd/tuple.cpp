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
#include <tuple>

#define __Mock_Mock__
#define ETLSTD etlstd
#include <libstd/include/tuple>

// Structured bindings look up tuple_size/tuple_element in ::std regardless of the type's own
// namespace, so forward the real specializations to etlstd's to exercise `auto [a, b] = t;`.
namespace std {
template <typename... T> struct tuple_size<etlstd::tuple<T...>> : etlstd::tuple_size<etlstd::tuple<T...>> {};
template <size_t I, typename... T>
struct tuple_element<I, etlstd::tuple<T...>> : etlstd::tuple_element<I, etlstd::tuple<T...>> {};
} // namespace std

using namespace ETLSTD;

namespace {

struct Empty {};

struct MoveOnly {
    int value;

    constexpr explicit MoveOnly(int v) : value(v) {}
    MoveOnly(const MoveOnly &) = delete;
    MoveOnly &operator=(const MoveOnly &) = delete;
    constexpr MoveOnly(MoveOnly &&other) noexcept : value(other.value) { other.value = -1; }
    constexpr MoveOnly &operator=(MoveOnly &&other) noexcept {
        value = other.value;
        other.value = -1;
        return *this;
    }
};

class Serializer {
public:
    ::std::string output;

    void f(ETLSTD::size_t s) { output += ::std::to_string(s); }

    template <typename T, T... Indices> void transform(integer_sequence<T, Indices...>) {
        int ignore[]{(f(Indices), 0)...};
        (void)ignore;
    }
};

static_assert(tuple_size<tuple<>>::value == 0);
static_assert(tuple_size_v<tuple<int, long>> == 2);
static_assert(is_same_v<tuple_element_t<1, tuple<char, int, bool>>, int>);
static_assert(is_same_v<decltype(get<0>(declval<tuple<int, bool> &>())), int &>);
static_assert(is_same_v<decltype(get<1>(declval<const tuple<int, bool> &>())), const bool &>);
static_assert(is_same_v<decltype(get<0>(declval<tuple<int> &&>())), int &&>);
static_assert(is_same_v<decltype(get<0>(declval<const tuple<int> &&>())), const int &&>);
static_assert(sizeof(tuple<Empty, uint8_t>) == sizeof(uint8_t));

} // namespace

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
    GIVEN("heterogeneous values constructed in place") {
        tuple<int, bool, ::std::string> values(42, true, "embedded");

        THEN("get exposes the stored elements") {
            REQUIRE(get<0>(values) == 42);
            REQUIRE(get<1>(values));
            REQUIRE(get<2>(values) == "embedded");
        }
    }

    GIVEN("a tuple with references produced by tie") {
        int number = 3;
        bool enabled = false;
        tuple<int &, bool &> refs = tie(number, enabled);

        WHEN("assigning from another tuple") {
            refs = make_tuple(9, true);

            THEN("the referenced objects are updated") {
                REQUIRE(number == 9);
                REQUIRE(enabled);
            }
        }

        WHEN("ignore is used during assignment") {
            tie(number, ignore) = make_tuple(12, "unused");

            THEN("only selected elements are updated") {
                REQUIRE(number == 12);
                REQUIRE_FALSE(enabled);
            }
        }
    }

    GIVEN("a tuple built with make_tuple") {
        int counter = 7;
        auto values = make_tuple("ticks", ref(counter), 4u);

        THEN("reference_wrapper arguments remain references") {
            static_assert(is_same_v<decltype(values), tuple<const char *, int &, unsigned int>>);
            get<1>(values) = 11;
            REQUIRE(counter == 11);
            REQUIRE(get<0>(values) == "ticks");
            REQUIRE(get<2>(values) == 4u);
        }
    }

    GIVEN("a tuple converted from another tuple type") {
        tuple<short, uint8_t> small_values(5, 1);

        WHEN("constructing and assigning a wider tuple") {
            tuple<int, bool> converted(small_values);
            tuple<long, int> assigned(0, 0);
            assigned = converted;

            THEN("element-wise conversions are preserved") {
                REQUIRE(get<0>(converted) == 5);
                REQUIRE(get<1>(converted));
                REQUIRE(get<0>(assigned) == 5);
                REQUIRE(get<1>(assigned) == 1);
            }
        }
    }

    GIVEN("a tuple containing a move-only value") {
        tuple<MoveOnly, int> moved(MoveOnly(17), 5);

        THEN("move construction stores the value once") {
            REQUIRE(get<0>(moved).value == 17);
            REQUIRE(get<1>(moved) == 5);
        }
    }

    GIVEN("a tuple of empty and non-empty types") {
        tuple<Empty, uint8_t> compact(Empty{}, 9);

        THEN("empty elements do not add payload storage") {
            REQUIRE(get<1>(compact) == 9);
            REQUIRE(sizeof(compact) == sizeof(uint8_t));
        }
    }

    GIVEN("a tuple destructured with structured bindings") {
        tuple<int, ::std::string, bool> values(3, "abc", true);

        WHEN("binding by value") {
            auto [number, text, flag] = values;

            THEN("each binding reads the matching element") {
                REQUIRE(number == 3);
                REQUIRE(text == "abc");
                REQUIRE(flag);
            }
        }

        WHEN("binding by reference") {
            auto &[number, text, flag] = values;
            number = 9;

            THEN("the binding aliases the tuple's storage") { REQUIRE(get<0>(values) == 9); }
        }
    }
}
