#include <catch.hpp>

#define __Mock_Mock__
#include <etl/ioports.h>

#include <../ETL/libstd/include/queue> 
#include <iostream>

namespace etl {
template<typename T, uint8_t N>
class CircularBuffer {
public:
    CircularBuffer() : readIndex(0), writeIndex(0) {
    }

    void push_back(T elem) {
        buffer[writeIndex] = elem;
        writeIndex = (writeIndex + 1) % N;
       
    }

    T pop_front() {
        T value = buffer[readIndex];
        readIndex = (readIndex + 1) % N;
            return value;
    }

    T front() {
        T value = buffer[readIndex];
        return value;
    }

    T back() {
        auto index = 0;
        if (writeIndex == 0) {
            index = N-1;
        }
        else {
            index = writeIndex-1;
        }
        T value = buffer[index];
        return value;
    }

    uint8_t size() {
        return writeIndex >= readIndex ? writeIndex - readIndex : writeIndex + N - readIndex;
    }

    template<typename... Args>
    void emplace_back(Args&& ... args) {
        std::allocator<T> all;
        all.construct(&buffer[writeIndex], std::forward<Args>(args)...);
        writeIndex = (writeIndex + 1) % N;
    }


private:
    std::array<T, N> buffer;
    uint8_t readIndex, writeIndex;
};


} // namespace etl
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