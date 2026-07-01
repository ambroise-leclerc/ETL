/// @file test/libstd/random.cpp
/// @data 01/07/2026 08:00:53
/// @author Ambroise Leclerc
/// @brief Tests for <random>
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

#define ETLSTD etlstd

#include <libstd/include/random>

using namespace ETLSTD;

SCENARIO("xorshift32_engine") {
    GIVEN("two engines seeded identically") {
        xorshift32_engine gen1(42u);
        xorshift32_engine gen2(42u);

        THEN("they produce the same sequence") {
            for (int i = 0; i < 100; ++i) {
                REQUIRE(gen1() == gen2());
            }
        }
    }

    GIVEN("two engines seeded differently") {
        xorshift32_engine gen1(1u);
        xorshift32_engine gen2(2u);

        THEN("their sequences differ") {
            bool anyDifferent = false;
            for (int i = 0; i < 10; ++i) {
                if (gen1() != gen2())
                    anyDifferent = true;
            }
            REQUIRE(anyDifferent);
        }
    }

    GIVEN("a re-seeded engine") {
        xorshift32_engine gen(7u);
        auto first = gen();
        gen.seed(7u);

        THEN("it replays from the start") { REQUIRE(gen() == first); }
    }
}

SCENARIO("uniform_int_distribution stays within bounds") {
    GIVEN("a distribution over [10, 20]") {
        xorshift32_engine gen(1234u);
        uniform_int_distribution<int> dist(10, 20);

        THEN("every draw lies within [a, b]") {
            for (int i = 0; i < 10000; ++i) {
                auto v = dist(gen);
                REQUIRE(v >= 10);
                REQUIRE(v <= 20);
            }
        }
    }

    GIVEN("a single-value distribution [5, 5]") {
        xorshift32_engine gen(99u);
        uniform_int_distribution<uint8_t> dist(5, 5);

        THEN("every draw equals 5") {
            for (int i = 0; i < 100; ++i) {
                REQUIRE(dist(gen) == 5);
            }
        }
    }

    GIVEN("a distribution over the full range of uint8_t") {
        xorshift32_engine gen(555u);
        uniform_int_distribution<uint8_t> dist(0, 255);

        THEN("draws stay within range and both halves get hit") {
            bool low = false, high = false;
            for (int i = 0; i < 1000; ++i) {
                auto v = dist(gen);
                if (v < 128)
                    low = true;
                else
                    high = true;
            }
            REQUIRE(low);
            REQUIRE(high);
        }
    }

    GIVEN("two distributions drawing from identically seeded generators") {
        xorshift32_engine gen1(2026u);
        xorshift32_engine gen2(2026u);
        uniform_int_distribution<int> dist1(0, 1000);
        uniform_int_distribution<int> dist2(0, 1000);

        THEN("the drawn sequences are identical (determinism)") {
            for (int i = 0; i < 50; ++i) {
                REQUIRE(dist1(gen1) == dist2(gen2));
            }
        }
    }
}
