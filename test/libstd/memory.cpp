/// @file test/libstd/memory.cpp
/// @data 06/06/2016 22:23:53
/// @author Ambroise Leclerc
/// @brief BDD tests for <memory>
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
#include <iostream>

#define ETLSTD etlstd
#include <libstd/include/memory>
#include <libstd/include/utility>
#include <libstd/include/array>
#include <libstd/include/queue>
#include <etl/CircularBuffer.h>

class MyClass {
public:
    MyClass(uint32_t id) : id(id) {
        std::cout << "" << id << "\n";
        instances++;
    }

    ~MyClass() {
        std::cout << "~" << id << "\n";
        instances--;
    }
public:
    static uint8_t instances;
    uint32_t id;
};

uint8_t MyClass::instances = 0;
using namespace ETLSTD;

SCENARIO("std::unique_ptr") {
    GIVEN("0 class instances") {
        MyClass::instances = 0;
        WHEN("a unique_ptr is created") {
            THEN("") {
                auto obj = make_unique<MyClass>(123456);
                REQUIRE(MyClass::instances == 1);
                auto obj2 = move(obj);
                REQUIRE(MyClass::instances == 1);
                REQUIRE(obj2->id == 123456);
            }
            REQUIRE(MyClass::instances == 0);
        }
    }
    GIVEN("A fifo of unique_ptrs") {
        using Fifo = queue<unique_ptr<MyClass>, etl::CircularBuffer<unique_ptr<MyClass>, 16>>;
        Fifo fifo;
        WHEN("ptrs are pushed and consumed") {
            std::string output;
            for (uint8_t i = 0; i < 10; ++i)
                fifo.push(make_unique<MyClass>(i));
            REQUIRE(MyClass::instances == 10);

            for (uint8_t i = 0; i < 10; ++i) {
                fifo.push(make_unique<MyClass>(i + 10));
                output += std::to_string(fifo.front()->id);
                fifo.pop();
            }
            //REQUIRE(MyClass::instances == 10);
            REQUIRE(output == "0123456789");

            for (uint8_t i = 0; i < 10; ++i) {
                output += std::to_string(fifo.front()->id);
                fifo.pop();
            }
            REQUIRE(MyClass::instances == 0);
            REQUIRE(output == "012345678910111213141516171819");
        }
    }
    GIVEN("array of unique_ptrs") {
        array<unique_ptr<MyClass>, 64> arrayOfPtrs;

        for (uint8_t i=0; i<64; ++i) {
            arrayOfPtrs[i] = make_unique<MyClass>(i);
        }
        REQUIRE(MyClass::instances == 64);
        REQUIRE(arrayOfPtrs[10]->id == 10);
        REQUIRE(arrayOfPtrs[25]->id == 25);
        REQUIRE(arrayOfPtrs[42]->id == 42);

        array<unique_ptr<MyClass>, 64> otherArrayOfPtrs;
        for (uint8_t i=0; i<64; ++i) {
            otherArrayOfPtrs[63 - i] = move(arrayOfPtrs[i]);
        }
        REQUIRE(MyClass::instances == 64);
        REQUIRE(otherArrayOfPtrs[10]->id == 53);
        REQUIRE(otherArrayOfPtrs[25]->id == 38);
        REQUIRE(otherArrayOfPtrs[42]->id == 21);

        etl::CircularBuffer<unique_ptr<MyClass>, 50> buffer;
        for (auto&& elem : arrayOfPtrs) {
            buffer.push_back(move(elem));
        }
        REQUIRE(MyClass::instances == 64);

        for (auto&& elem : otherArrayOfPtrs) {
            buffer.push_back(move(elem));
        }
        REQUIRE(MyClass::instances == 50);
        REQUIRE(buffer.front()->id == 49);
        buffer.front()->id *= 2;
        REQUIRE(buffer.front()->id == 98);
        buffer.pop_front();
        REQUIRE(MyClass::instances == 49);

        auto elem100 = make_unique<MyClass>(100);
        REQUIRE(MyClass::instances == 50);
        buffer.front().swap(elem100);
        REQUIRE(MyClass::instances == 50);
        {
            auto elem = move(buffer.front());
            REQUIRE(elem->id == 100);
            REQUIRE(MyClass::instances == 50);
            buffer.pop_front();
        }
        REQUIRE(MyClass::instances == 49);
    }
}

SCENARIO("std::shared_ptr") {
    GIVEN("A shared_ptr constructed by make_shared") {
        MyClass::instances = 0;
        auto obj = ETLSTD::make_shared<MyClass>(123456);
        REQUIRE(MyClass::instances == 1);

        WHEN("ptr is copied") {
            auto obj2 = obj;
            THEN("both points to the same object") {
                obj->id++;
                REQUIRE(obj2->id == obj->id);
                REQUIRE(obj.use_count() == 2);
            }
        }

        WHEN("ptr is copied twice") {
            auto obj2 = obj;
            auto obj3 = obj;
            THEN("3 references are counted")
                REQUIRE(obj.use_count() == 3);
            
            obj = nullptr;
            THEN("ref count is decreased") 
                REQUIRE(obj2.use_count() == 2);
            
            obj3.reset();
            REQUIRE(!obj3);
            REQUIRE(obj2.unique());
        }

    }
    REQUIRE(MyClass::instances == 0);
}
