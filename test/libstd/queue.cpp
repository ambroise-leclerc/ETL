/// @file test/libstd/queue.cpp
/// @data 20/09/2016 17:40:53
/// @author Ambroise Leclerc and Cécile Gomes
/// @brief BDD tests for <queue>
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

#define __Mock_Mock__
#define ETLSTD etlstd
#include <libstd/include/queue>
#include <libstd/include/cstddef>
#include <libstd/include/memory>

using namespace ETLSTD;

uint32_t mockStatus;

class SequenceContainerMock {
public:
    using value_type = uint8_t;
    using size_type = ETLSTD::size_t;
    using reference = value_type&;
    using const_reference = const value_type&;

    SequenceContainerMock() : elemF(8), elemB(2), contSize(0) { }
    bool empty() const { mockStatus += Empty; return false; }
    size_type size() const { mockStatus += Size; return contSize; }
    reference front() { mockStatus += Front; return elemF; }
    reference back() { mockStatus += Back; return elemB; }
    void push_back(const_reference elem) { mockStatus += PushBack; contSize++; elemB = elem; }
    void pop_front() { mockStatus += PopFront; contSize--; }
    template<typename... Args>
    void emplace_back(Args&& ... args) { mockStatus += Emplace; new(&elemB) uint8_t(std::forward<Args>(args)...); }
    enum method { Size = 1, Front = 10, Back = 100, PushBack = 1000, PopFront = 10000,
                  Empty = 100000, Emplace = 1000000, Swap = 10000000 };
protected:
    value_type elemF, elemB;
    size_type contSize;
};

SCENARIO("Queue") {
    using Fifo = ETLSTD::queue<uint8_t, SequenceContainerMock>;
    GIVEN("An empty Fifo") {
        Fifo fifo;
        mockStatus = 0;
        REQUIRE(fifo.size() == 0);

        while (fifo.size() < fifo.front())
            fifo.push(2);
        REQUIRE(mockStatus == 8100);

        while (fifo.size() > fifo.back())
            fifo.pop();
        REQUIRE(mockStatus == 68807);
        
        fifo.empty();
        REQUIRE(mockStatus == 168807);

        fifo.emplace(150);
        REQUIRE(fifo.back() == 150);
        REQUIRE(mockStatus == 1168907);

        Fifo fifo2;
        fifo2.swap(fifo);
        REQUIRE(fifo2.back() == 150);
        REQUIRE(fifo.back() == 2);
        REQUIRE(mockStatus == 1169107);
        
    }
}
