#include <catch.hpp>

#define __Mock_Mock__
#include <etl/ioports.h>

#include <../ETL/libstd/include/queue> 
#include <../ETL/include/etl/container.h>
#include <iostream>
 // namespace etl
SCENARIO("CircularBuffer") {
    using namespace etl;
    using namespace std;

    using fifo = CircularBuffer<uint8_t, 32>;

 

    std::queue<uint8_t, fifo > etlQueue;
    REQUIRE(etlQueue.size() == 0);
    REQUIRE(etlQueue.empty());
    etlQueue.push(30);
    REQUIRE(etlQueue.size() == 1);
    REQUIRE(etlQueue.front() == 30);
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
    
    std::queue<uint8_t, fifo > etlQueue2;
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