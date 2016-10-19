#include <catch.hpp>

#define __Mock_Mock__


#define ETLSTD etlstd
#include <etl/CircularBuffer.h>
#include <libstd/include/queue> 
#include <libstd/include/utility>
#include <iostream>

using namespace ETLSTD;

SCENARIO("CircularBuffer for fifo of bytes") {
    using fifo = queue<uint8_t, etl::CircularBuffer<uint8_t, 32>>;

    fifo etlQueue;
    REQUIRE(etlQueue.size() == 0);
    REQUIRE(etlQueue.empty());
    etlQueue.push(30);
    REQUIRE(etlQueue.size() == 1);
    REQUIRE(etlQueue.front() == 30);
    REQUIRE(etlQueue.back() == 30);
    REQUIRE(etlQueue.size() == 1);
    etlQueue.pop();
    REQUIRE(etlQueue.size() == 0);
    REQUIRE(etlQueue.empty());

    etlQueue.push(30);
    etlQueue.push(20);
    etlQueue.push(10);
    REQUIRE(etlQueue.size() == 3);
    REQUIRE(!etlQueue.empty());
    REQUIRE(etlQueue.front() == 30);
    REQUIRE(etlQueue.back() == 10);
    etlQueue.pop();
    REQUIRE(etlQueue.front() == 20);
    REQUIRE(etlQueue.back() == 10);
    REQUIRE(etlQueue.size() == 2);
    
    fifo etlQueue2;
    etlQueue2.push(18);
    REQUIRE(etlQueue2.front() == 18);
    etlQueue.swap(etlQueue2);
    REQUIRE(etlQueue.front() == 18);
    REQUIRE(etlQueue.size() == 1);
    REQUIRE(etlQueue2.front() == 20);
    REQUIRE(etlQueue2.back() == 10);
    REQUIRE(etlQueue.front() == 18);
    REQUIRE(etlQueue.back() == 18);
    REQUIRE(etlQueue.size() == 1);

    etlQueue.emplace(10);
    REQUIRE(etlQueue.size() == 2);
    REQUIRE(etlQueue.back() == 10);
}

class Element {
    uint8_t id;
public:
    Element(uint8_t id) : id(id) { ++nbCreated; ++nbActive; std::cout << +id << "(" << +nbCreated << ", " << +nbActive << ") - ";  }
    ~Element() { --nbActive; }
    auto getId() { return id; }

    static uint8_t nbCreated, nbActive;
};

uint8_t Element::nbCreated = 0;
uint8_t Element::nbActive = 0;

SCENARIO("CircularBuffer holding unique_ptrs") {
    using Buffer = etl::CircularBuffer<unique_ptr<Element>, 8>;
    Buffer buffer;
    for (uint8_t i = 0; i < 9; ++i)
        buffer.push_back(make_unique<Element>(i));
    REQUIRE(Element::nbCreated == 9);
    REQUIRE(Element::nbActive == 8);
    REQUIRE(buffer.front()->getId() == 1);
    REQUIRE(buffer.back()->getId() == 8);
    
    
    auto sum = 1 + 2 + 3 + 4 + 5 + 6 + 7 + 8;
    for (uint8_t i = 0; i < 10; i++) {
        auto a = move(buffer.front());
        buffer.pop_front();
        if (a)
            sum -= a->getId();
    }
    REQUIRE(sum == 0);
    REQUIRE(Element::nbActive == 0);
    REQUIRE(buffer.empty());


    array<unique_ptr<Element>, 15> source;
    for (uint8_t i = 0; i < 15; i++)
        source[i] = make_unique<Element>(100 + i);

    REQUIRE(Element::nbCreated == 24);
    REQUIRE(Element::nbActive == 15);
    
    queue<uint8_t, Buffer> fifo;
    for (auto&& elem : source)
        fifo.push(move(elem));
}

