/// @file test/libstd/bitset.cpp
/// @data 29/08/2017 08:00:53
/// @author Ambroise Leclerc
/// @brief Tests for <bitset>
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

#define ETLSTD etlstd

#include <libstd/include/bitset>

using namespace ETLSTD;

SCENARIO("bitset") {
    
    bitset<17> bits0;
    REQUIRE(bits0.size() == 17);
    REQUIRE(sizeof(bits0) == 3);    // 17 bits should not take more than 3 bytes
    REQUIRE(bits0.to_ulong() == 0b0);

    bits0.set(1);
    REQUIRE(bits0.to_ulong() == 0b10);
    bits0.set(3);
    REQUIRE(bits0.to_ulong() == 0b1010);
    bits0.set(8);
    REQUIRE(bits0.to_ulong() == 0b100001010);
    bits0.set(16);
    REQUIRE(bits0.to_ulong() == 0b10000000100001010);
    bits0.set(8, false);
    REQUIRE(bits0.to_ulong() == 0b10000000000001010);
    bits0.reset(3);
    REQUIRE(bits0.to_ulong() == 0b10000000000000010);
    bits0.set(0).reset(1);
    REQUIRE(bits0.to_ulong() == 0b10000000000000001);
    bits0.set(2).set(13);
    REQUIRE(bits0.test(2));
    REQUIRE(bits0.test(13));



    bitset<23> bits1(0b00001111000011111111111);
    REQUIRE(bits1.size() == 23);
    REQUIRE(sizeof(bits1) == 3);    // 23 bits should not take more than 3 bytes
    REQUIRE(bits1.to_ulong() == 0b00001111000011111111111);
    bits1.set(22);
    REQUIRE(bits1.to_ulong() == 0b10001111000011111111111);
    bits1.reset().set(0).set(22);
    REQUIRE(bits1.to_ulong() == 0b10000000000000000000001);
    bits1.set().reset(21).set(1, false); 
    REQUIRE(bits1.to_ulong() == 0b10111111111111111111101);
    bits1.flip().flip(22).flip(0);
    REQUIRE(bits1.to_ulong() == 0b11000000000000000000011);
    bits1.flip(3).flip(19);
    REQUIRE(bits1[3]);
    REQUIRE(bits1[19]);
    REQUIRE(!bits1[12]);
    bits1[4] = true;
    bits1[18] = true;
    bits1[1] = false;
    bits1[21] = false;
    REQUIRE(bits1.to_ulong() == 0b10011000000000000011001);
    REQUIRE(~bits1[2]);
    REQUIRE(~bits1[20]);
    bits1[2].flip();
    bits1[20].flip();
    REQUIRE(bits1.to_ulong() == 0b10111000000000000011101);
    REQUIRE(!bits1.all());
    bits1.set();
    REQUIRE(bits1.all());
    
    WHEN("bitset is one bit sized") {
        bitset<1> bits;
        bits.set();
        REQUIRE(bits.all());
        REQUIRE(!bits.none());
        REQUIRE(bits.any());

        bits[0].flip();
        REQUIRE(!bits.all());

        bits.reset();
        REQUIRE(!bits.all());
        REQUIRE(bits.none());
        REQUIRE(!bits.any());

        bits[0].flip();
        REQUIRE(bits.any());
    }

    WHEN("bitset is 8 bits sized") {
        bitset<8> bits;
        bits.set();
        REQUIRE(bits.all());
        REQUIRE(!bits.none());
        REQUIRE(bits.any());

        bits[0].flip();
        REQUIRE(!bits.all());

        bits.reset();
        REQUIRE(!bits.all());
        REQUIRE(bits.none());
        REQUIRE(!bits.any());

        bits[0].flip();
        REQUIRE(bits.any());
    }

    WHEN("bitset is 27 bits sized") {
        bitset<27> bits;
        bits.set();
        REQUIRE(bits.all());
        REQUIRE(!bits.none());
        REQUIRE(bits.any());

        bits[0].flip();
        REQUIRE(!bits.all());

        bits.reset();
        REQUIRE(!bits.all());
        REQUIRE(bits.none());
        REQUIRE(!bits.any());

        bits[0].flip();
        REQUIRE(bits.any());
    }

}