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
#include "../libstd_harness/harness.h"

#include <type_traits>

using namespace std;

TEST_CASE("std::signed, std_unsigned", "[libstd][type_traits]") {
    auto s1 = is_signed<uint8_t>::value;    CHECK(s1 == false);
    auto s2 = is_signed<char>::value;       CHECK(s2 == true);
    auto s3 = is_signed<int32_t>::value;    CHECK(s3 == true);
    auto s4 = is_signed<uint64_t>::value;   CHECK(s4 == false);
    CHECK(is_signed<uint8_t>::value == false);
    CHECK(is_unsigned<uint8_t>::value == true);

    CHECK(is_signed_v<int16_t> == true);
    CHECK(is_signed_v<uint32_t> == false);
    CHECK(is_signed_v<float> == true);
    CHECK(is_signed_v<double> == true);

    CHECK(is_unsigned<uint8_t>::value == true);
    CHECK(is_unsigned<char>::value == false);
    CHECK(is_unsigned<unsigned char>::value == true);
    CHECK(is_unsigned_v<int16_t> == false);
    CHECK(is_unsigned_v<uint32_t> == true);
    CHECK(is_unsigned_v<float> == false);
    CHECK(is_unsigned_v<double> == false);
    CHECK(is_unsigned_v<uint8_t> == true);
    CHECK(is_unsigned_v<int64_t> == false);
}

TEST_CASE("std::common_type, std::same_type", "[libstd][type_traits]") {
    class Base {};
    class Incarnation1 : Base {};
    class Incarnation2 : Base {};

    //using CommonType = common_type<Incarnation1, Incarnation2>::type;
#if defined(__GNU_G__)
    auto test = is_same<Base, common_type<Incarnation1, Incarnation2>::type >::value == true;
#endif
}


TEST_CASE("std::make_signed, std::make_unsigned", "[libstd][type_traits]") {
    auto comp = is_same_v<make_signed_t<uint16_t>, int16_t>;
    CHECK(comp == true);

    auto comp2 = is_const_v<make_signed_t<const int32_t>>;
    CHECK(comp2 == true);

    CHECK(is_volatile_v<make_unsigned_t<volatile const int16_t>> == true);
}




TEST_CASE("Primary types _v", "[libstd][type_traits]") {
    CHECK(is_void_v<void>);
    CHECK(!is_void_v<double>);

    union UnionType {
        int a;
        char b;
    };
    UnionType value;
    CHECK(is_union_v<decltype(value)>);
    CHECK(!is_union_v<float>);

    enum EnumType { One, Two, Three };
    CHECK(is_enum_v<EnumType>);
    CHECK(!is_enum_v<void>);

    class ClassType {
        ClassType() = default;
        int a;
    };
    CHECK(is_class_v<ClassType>);
    CHECK(!is_class_v<UnionType>);
    CHECK(!is_class_v<EnumType>);

    CHECK(is_integral_v<uint64_t>);
    CHECK(is_integral_v<char>);
    CHECK(!is_integral_v<ClassType>);
    CHECK(!is_integral_v<UnionType>);

    CHECK(is_floating_point_v<float>);
    CHECK(is_floating_point_v<double>);
    CHECK(!is_floating_point_v<char>);
    CHECK(!is_floating_point_v<ClassType>);

    CHECK(is_pointer_v<float*>);
    CHECK(is_pointer_v<EnumType*>);
    CHECK(!is_pointer_v<float>);



   // CHECK(is_member_pointer_v<ClassType::*>);


    struct A {
        void function() {};
        uint8_t member;
    };
    void(A::*functionPointer)() = &A::function;
    uint8_t A::* memberPointer = &A::member;

    CHECK(!is_member_pointer_v<A*>);
    CHECK(is_member_pointer_v<uint8_t A::*>);
    CHECK(is_member_pointer_v<void(A::*)()>);
    CHECK(is_member_pointer_v<decltype(memberPointer)>);

    CHECK(is_member_function_pointer_v<void(A::*)()>);
    CHECK(!is_member_function_pointer_v<A*>);
    CHECK(is_member_function_pointer_v<decltype(functionPointer)>);

    CHECK(!is_member_object_pointer_v<A*>);
    CHECK(is_member_object_pointer_v<int A::*>);
    CHECK(is_member_object_pointer<decltype(memberPointer)>::value);

}
