/// @file test/libstd/functional.cpp
/// @date 01/07/2026
/// @author Ambroise Leclerc
/// @brief BDD tests for <functional>
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

#include <catch.hpp>

#define __Mock_Mock__
#define ETLSTD etlstd
#include <libstd/include/functional>

using namespace ETLSTD;

namespace {

int add_pair(int lhs, int rhs) { return lhs + rhs; }

struct Accumulator {
    int bias = 0;

    int add(int value) {
        bias += value;
        return bias;
    }

    int add_const(int value) const { return bias + value; }
};

struct Callable {
    int operator()(int lhs, int rhs) const { return lhs * 10 + rhs; }
};

static_assert(is_same<decltype(invoke(add_pair, 1, 2)), int>::value, "");
static_assert(is_same<decltype(invoke(&Accumulator::add, declval<Accumulator &>(), 1)), int>::value, "");
static_assert(is_same<decltype(invoke(&Accumulator::bias, declval<Accumulator &>())), int &>::value, "");

} // namespace

SCENARIO("std::invoke supports direct callables", "[functional]") {
    REQUIRE(invoke(add_pair, 2, 5) == 7);
    REQUIRE(invoke(Callable{}, 3, 4) == 34);

    Callable callable;
    auto wrapped = ref(callable);
    REQUIRE(invoke(wrapped, 4, 2) == 42);
    REQUIRE(wrapped(1, 9) == 19);
}

SCENARIO("std::invoke supports member pointers", "[functional]") {
    Accumulator accumulator{3};

    REQUIRE(invoke(&Accumulator::bias, accumulator) == 3);
    REQUIRE(invoke(&Accumulator::bias, &accumulator) == 3);
    REQUIRE(invoke(&Accumulator::bias, ref(accumulator)) == 3);

    REQUIRE(invoke(&Accumulator::add, accumulator, 4) == 7);
    REQUIRE(invoke(&Accumulator::add, &accumulator, 5) == 12);
    REQUIRE(invoke(&Accumulator::add, ref(accumulator), 6) == 18);
    REQUIRE(invoke(&Accumulator::add_const, cref(accumulator), 2) == 20);
}

SCENARIO("std::reference_wrapper keeps referenced storage", "[functional]") {
    int value = 4;

    auto wrapped = ref(value);
    wrapped.get() = 9;
    REQUIRE(value == 9);

    const int constant = 12;
    auto const_wrapped = cref(constant);
    REQUIRE(const_wrapped.get() == 12);
}
