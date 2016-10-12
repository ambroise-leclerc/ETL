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



SCENARIO("CircularBuffer holding unique_ptrs") {
    class Element {
        uint8_t id;
    public:
        Element(uint8_t id) : id(id) {}
        ~Element() { std::cout << "delete " << +id << "\n"; }

        void dump(std::string text = "") const { std::cout << text << " : " << +id << "\n";  }

    };


    using Buffer = etl::CircularBuffer<unique_ptr<Element>, 8>;

    Buffer buffer;
    for (uint8_t i = 0; i < 9; ++i)
        buffer.push_back(make_unique<Element>(i));

    buffer.front()->dump("front");
    buffer.back()->dump("back");
    for (uint8_t i = 0; i < 10; i++) {
        auto a = move(buffer.pop_front());
        if (a)
            a->dump("moved");
    }
}

