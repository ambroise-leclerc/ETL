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

#define __Mock_Mock__
#define ETLSTD etlstd
#include <libstd/include/memory>
#include <libstd/include/utility>

class MyClass {
public:
    MyClass(uint32_t id) : id(id) { instances++; }

    ~MyClass() { instances--; }

public:
    static uint8_t instances;
    uint32_t id;
};

uint8_t MyClass::instances = 0;
// using namespace etlTest::std;

SCENARIO("std::unique_ptr", "[libstd][memory]") {
    GIVEN("0 class instances") {
        MyClass::instances = 0;
        WHEN("a unique_ptr is created") {
            THEN("") {
                auto obj = ETLSTD::make_unique<MyClass>(123456);
                REQUIRE(MyClass::instances == 1);
                auto obj2 = ETLSTD::move(obj);
                REQUIRE(MyClass::instances == 1);
                REQUIRE(obj2->id == 123456);
            }
            REQUIRE(MyClass::instances == 0);
        }
    }
}

SCENARIO("std::shared_ptr", "[libstd][memory]") {
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
            THEN("references are counted and released correctly as owners give up their reference") {
                REQUIRE(obj.use_count() == 3);

                obj = nullptr;
                REQUIRE(obj2.use_count() == 2);

                obj3.reset();
                REQUIRE(!obj3);
                REQUIRE(obj2.unique());
            }
        }

        WHEN("ptr is copy-assigned") {
            auto assignee = ETLSTD::make_shared<MyClass>(654321);
            REQUIRE(MyClass::instances == 2);

            assignee = obj;

            THEN("the previous pointee is released and ownership is shared") {
                REQUIRE(MyClass::instances == 1);
                REQUIRE(assignee.get() == obj.get());
                REQUIRE(assignee->id == obj->id);
                REQUIRE(obj.use_count() == 2);
                REQUIRE(assignee.use_count() == 2);
            }

            obj.reset();

            THEN("the assigned pointer keeps the managed object alive") {
                REQUIRE(!obj);
                REQUIRE(assignee);
                REQUIRE(assignee->id == 123456);
                REQUIRE(assignee.use_count() == 1);
                REQUIRE(MyClass::instances == 1);
            }
        }

        WHEN("ptr is assigned to itself") {
            obj = obj;

            THEN("ownership stays unchanged") {
                REQUIRE(obj);
                REQUIRE(obj.unique());
                REQUIRE(obj.use_count() == 1);
                REQUIRE(MyClass::instances == 1);
            }
        }

        WHEN("ptr is reset to nullptr") {
            auto obj2 = obj;
            REQUIRE(obj.use_count() == 2);

            obj.reset(nullptr);

            THEN("only that owner releases its reference") {
                REQUIRE(!obj);
                REQUIRE(obj.use_count() == 0);
                REQUIRE(obj2);
                REQUIRE(obj2.use_count() == 1);
                REQUIRE(MyClass::instances == 1);
            }

            obj2.reset();

            THEN("the last release destroys the object") {
                REQUIRE(!obj2);
                REQUIRE(obj2.use_count() == 0);
                REQUIRE(MyClass::instances == 0);
            }
        }

        WHEN("ptrs are swapped") {
            auto obj2 = ETLSTD::make_shared<MyClass>(654321);
            auto obj3 = obj;
            REQUIRE(obj.use_count() == 2);
            REQUIRE(obj2.use_count() == 1);

            obj.swap(obj2);

            THEN("ownership and counts follow the handles") {
                REQUIRE(obj->id == 654321);
                REQUIRE(obj.use_count() == 1);
                REQUIRE(obj2->id == 123456);
                REQUIRE(obj2.use_count() == 2);
                REQUIRE(obj3.get() == obj2.get());
                REQUIRE(MyClass::instances == 2);
            }
        }

        WHEN("copies are created and destroyed across nested scopes") {
            {
                auto obj2 = obj;
                REQUIRE(obj.use_count() == 2);

                {
                    auto obj3 = obj2;
                    REQUIRE(obj.use_count() == 3);
                    REQUIRE(obj3.use_count() == 3);
                }

                REQUIRE(obj.use_count() == 2);
                obj2.reset();
                REQUIRE(obj.use_count() == 1);
            }

            THEN("the managed object stays alive until the last owner releases it") {
                REQUIRE(obj);
                REQUIRE(obj.unique());
                REQUIRE(obj.use_count() == 1);
                REQUIRE(MyClass::instances == 1);
            }
        }
    }

    GIVEN("A shared_ptr constructed from a null raw pointer") {
        MyClass::instances = 0;
        ETLSTD::shared_ptr<MyClass> obj(static_cast<MyClass *>(nullptr));

        THEN("it remains empty and does not create a control block") {
            REQUIRE(!obj);
            REQUIRE(obj.use_count() == 0);
            REQUIRE(MyClass::instances == 0);
        }
    }
    REQUIRE(MyClass::instances == 0);
}

namespace {
struct Container {
    MyClass first;
    MyClass second;
    Container() : first(1), second(2) {}
};
} // namespace

SCENARIO("std::shared_ptr aliasing constructor") {
    GIVEN("A shared_ptr owning a Container with two MyClass members") {
        MyClass::instances = 0;
        auto owner = ETLSTD::make_shared<Container>();
        REQUIRE(MyClass::instances == 2);

        WHEN("an aliasing shared_ptr is built pointing at a member of owner") {
            ETLSTD::shared_ptr<MyClass> alias(owner, &owner->second);

            THEN("it points at the sub-object but shares ownership with owner") {
                REQUIRE(alias.get() == &owner->second);
                REQUIRE(alias->id == 2);
                REQUIRE(owner.use_count() == 2);
                REQUIRE(alias.use_count() == 2);
            }

            THEN("releasing owner keeps the aliased sub-object alive") {
                owner.reset();
                REQUIRE(MyClass::instances == 2);
                REQUIRE(alias->id == 2);
                REQUIRE(alias.use_count() == 1);

                alias.reset();
                REQUIRE(MyClass::instances == 0);
            }
        }
    }
    REQUIRE(MyClass::instances == 0);
}
