/// @file test/libstd/vector.cpp
/// @data 28/09/2017 17:49:53
/// @author Ambroise Leclerc
/// @brief BDD tests for <vector>
//
// Copyright (c) 2017, Ambroise Leclerc
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
#include "MockDevice.h"
#include <libstd/include/vector>

using namespace ETLSTD;

namespace {
struct CountingObject {
    static int liveObjects;
    static int copyCount;
    static int moveCount;

    int value;

    CountingObject(int initial = 0) : value(initial) { ++liveObjects; }

    CountingObject(const CountingObject &other) : value(other.value) {
        ++liveObjects;
        ++copyCount;
    }

    CountingObject(CountingObject &&other) : value(other.value) {
        other.value = -1;
        ++liveObjects;
        ++moveCount;
    }

    CountingObject &operator=(const CountingObject &other) {
        value = other.value;
        ++copyCount;
        return *this;
    }

    CountingObject &operator=(CountingObject &&other) {
        value = other.value;
        other.value = -1;
        ++moveCount;
        return *this;
    }

    ~CountingObject() { --liveObjects; }
};

int CountingObject::liveObjects = 0;
int CountingObject::copyCount = 0;
int CountingObject::moveCount = 0;

void resetCounts() {
    CountingObject::liveObjects = 0;
    CountingObject::copyCount = 0;
    CountingObject::moveCount = 0;
}
} // namespace

SCENARIO("vector starts empty", "[libstd][vector]") {
    vector<char> values;

    REQUIRE(values.empty());
    REQUIRE(values.size() == 0);
    REQUIRE(values.capacity() == 0);
    REQUIRE(values.begin() == values.end());
}

SCENARIO("vector supports construction and assignment from ranges", "[libstd][vector]") {
    const int initial[] = {1, 2, 3, 4};
    const int replacement[] = {8, 6, 4};

    vector<int> values(initial, initial + 4);
    REQUIRE(values.size() == 4);
    REQUIRE(values.capacity() >= values.size());
    REQUIRE(values.front() == 1);
    REQUIRE(values.back() == 4);

    values.assign(5, 9);
    REQUIRE(values.size() == 5);
    REQUIRE(values.capacity() >= 5);
    for (size_t index = 0; index < values.size(); ++index) {
        REQUIRE(values[index] == 9);
    }

    values.assign(replacement, replacement + 3);
    REQUIRE(values.size() == 3);
    REQUIRE(values[0] == 8);
    REQUIRE(values[1] == 6);
    REQUIRE(values[2] == 4);

    vector<int> copy(values);
    vector<int> moved(ETLSTD::move(copy));
    REQUIRE(moved.size() == 3);
    REQUIRE(moved[0] == 8);
    REQUIRE(moved[2] == 4);
    REQUIRE(copy.empty());
}

SCENARIO("vector reserves and resizes without losing elements", "[libstd][vector]") {
    vector<char> values;

    values.reserve(4);
    REQUIRE(values.capacity() >= 4);
    REQUIRE(values.empty());

    values.push_back('a');
    values.emplace_back('b');
    values.resize(4, 'z');
    REQUIRE(values.size() == 4);
    REQUIRE(values[0] == 'a');
    REQUIRE(values[1] == 'b');
    REQUIRE(values[2] == 'z');
    REQUIRE(values[3] == 'z');

    values.resize(1);
    REQUIRE(values.size() == 1);
    REQUIRE(values.front() == 'a');

    values.pop_back();
    REQUIRE(values.empty());
}

SCENARIO("vector manages non-trivial element lifetimes", "[libstd][vector]") {
    resetCounts();

    {
        vector<CountingObject> values;
        values.emplace_back(7);

        CountingObject copySource(9);
        values.push_back(copySource);
        REQUIRE(CountingObject::liveObjects == 3);

        values.reserve(64);
        REQUIRE(values.size() == 2);
        REQUIRE(values[0].value == 7);
        REQUIRE(values[1].value == 9);

        values.resize(3, CountingObject(11));
        REQUIRE(values.size() == 3);
        REQUIRE(values[2].value == 11);
        REQUIRE(CountingObject::liveObjects == 4);

        values.assign(2, CountingObject(5));
        REQUIRE(values.size() == 2);
        REQUIRE(values[0].value == 5);
        REQUIRE(values[1].value == 5);
        REQUIRE(CountingObject::liveObjects == 3);
    }

    REQUIRE(CountingObject::liveObjects == 0);
    REQUIRE(CountingObject::copyCount > 0);
    REQUIRE(CountingObject::moveCount > 0);
}
