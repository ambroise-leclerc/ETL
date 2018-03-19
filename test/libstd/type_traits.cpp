/// @file test/libstd/type_traits.cpp
/// @data 19/08/2016 17:40:53
/// @author Ambroise Leclerc
/// @brief BDD tests for <type_traits>
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

#define ETLSTD etlstd

#include <libstd/include/type_traits>

using namespace ETLSTD;

SCENARIO("std::signed") {
    auto s1 = is_signed<uint8_t>::value;    REQUIRE(s1 == false);
    auto s2 = is_signed<char>::value;       REQUIRE(s2 == true);
    auto s3 = is_signed<int32_t>::value;    REQUIRE(s3 == true);
    auto s4 = is_signed<uint64_t>::value;   REQUIRE(s4 == false);
    REQUIRE(is_signed<uint8_t>::value == false);
	REQUIRE(is_unsigned<uint8_t>::value == true);

    REQUIRE(is_signed_v<int16_t> == true);
    REQUIRE(is_signed_v<uint32_t> == false);
	REQUIRE(is_unsigned_v<uint8_t> == true);
	REQUIRE(is_unsigned_v<int64_t> == false);
}

SCENARIO("std::common_type, std::same_type") {
    class Base {};
    class Incarnation1 : Base {};
    class Incarnation2 : Base {};

    //using CommonType = common_type<Incarnation1, Incarnation2>::type;
#if defined(__GNU_G__)
    auto test = is_same<Base, common_type<Incarnation1, Incarnation2>::type >::value == true;
#endif
}


SCENARIO("std::make_signed, std::make_unsigned") {
    auto comp = is_same_v<make_signed_t<uint16_t>, int16_t>;
    REQUIRE(comp == true);

    auto comp2 = is_const_v<make_signed_t<const int32_t>>;
    REQUIRE(comp2 == true);

    REQUIRE(is_volatile_v<make_unsigned_t<volatile const int16_t>> == true);
}




SCENARIO("Primary types _v") {
    REQUIRE(is_void_v<void>);
    REQUIRE(!is_void_v<double>);

    union UnionType {
        int a;
        char b;
    };
    UnionType value;
    REQUIRE(is_union_v<decltype(value)>);
    REQUIRE(!is_union_v<float>);

    enum EnumType { One, Two, Three };
    REQUIRE(is_enum_v<EnumType>);
    REQUIRE(!is_enum_v<void>);

    class ClassType {
        ClassType() = default;
        int a;
    };
    REQUIRE(is_class_v<ClassType>);
    REQUIRE(!is_class_v<UnionType>);
    REQUIRE(!is_class_v<EnumType>);

    REQUIRE(is_integral_v<uint64_t>);
    REQUIRE(is_integral_v<char>);
    REQUIRE(!is_integral_v<ClassType>);
    REQUIRE(!is_integral_v<UnionType>);

    REQUIRE(is_floating_point_v<float>);
    REQUIRE(is_floating_point_v<double>);
    REQUIRE(!is_floating_point_v<char>);
    REQUIRE(!is_floating_point_v<ClassType>);

    REQUIRE(is_pointer_v<float*>);
    REQUIRE(is_pointer_v<EnumType*>);
    REQUIRE(!is_pointer_v<float>);



   // REQUIRE(is_member_pointer_v<ClassType::*>);


    struct A {
        void function() {};
        uint8_t member;
    };
    void(A::*functionPointer)() = &A::function;
    uint8_t A::* memberPointer = &A::member;

    REQUIRE(!is_member_pointer_v<A*>);
    REQUIRE(is_member_pointer_v<uint8_t A::*>);
    REQUIRE(is_member_pointer_v<void(A::*)()>);
    REQUIRE(is_member_pointer_v<decltype(memberPointer)>);

    REQUIRE(is_member_function_pointer_v<void(A::*)()>);
    REQUIRE(!is_member_function_pointer_v<A*>);
    REQUIRE(is_member_function_pointer_v<decltype(functionPointer)>);

    REQUIRE(!is_member_object_pointer_v<A*>);
    REQUIRE(is_member_object_pointer_v<int A::*>);
    REQUIRE(is_member_object_pointer<decltype(memberPointer)>::value);

}

