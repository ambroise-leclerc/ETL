#include <catch.hpp>

#define __Mock_Mock__
#include <etl/ioports.h>
#include <queue> 
#include <iostream>

namespace etl {
template<typename T, uint8_t N>
class CircularBuffer {
public:
    CircularBuffer() : readIndex(0), writeIndex(0),round(false) {
    }

    void push_back(T elem) {
        buffer[writeIndex] = elem;
        writeIndex = (writeIndex + 1) % N;
        if (writeIndex == 0 && !round) {
            round = true;
        }
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
        auto index;
        if (round) {
            index = (writeIndex+1)%N
        }
        else {
            index = 0;
        }
        T value = buffer[index];
        return value;
    }

    uint8_t size() {
        return writeIndex >= readIndex ? writeIndex - readIndex : writeIndex + N - readIndex;
    }

private:
    std::array<T, N> buffer;
    uint8_t readIndex, writeIndex;
    bool round ;
};

template<typename T, typename Container = etl::CircularBuffer<T,32> >
class Queue {
public:
    auto empty() {
        return container.size() == 0;
    }

    auto size() {
        return container.size();
    }

    auto front() {
        return container.front();
    }

    void pop() {
        container.pop_front();
    }

    auto back() {
        return container.back();
    }

    void push(T element) {
        container.push_back(element);
    }

    template<typename... Args>
    void emplace(Args&& ... args) {
        //TODO
        T object = new T(args);
        container.push(object);
    }

    void swap(Queue& right) {
       //TODO echanger les container.
    }

private:
    Container container;
};

} // namespace etl

SCENARIO("CircularBuffer") {
    using namespace etl;
    using namespace std;

    CircularBuffer<uint8_t, 5> fifo;

    REQUIRE(fifo.size() == 0);

    fifo.push_back(30);
    REQUIRE(fifo.size() == 1);
    REQUIRE(fifo.pop_front() == 30);
    REQUIRE(fifo.size() == 0);

    etl::Queue<uint8_t> etlQueue;
    REQUIRE(etlQueue.size() == 0);
    REQUIRE(etlQueue.empty());
    etlQueue.push(30);
    REQUIRE(etlQueue.size() == 1);
    REQUIRE(etlQueue.front() == 30);
    REQUIRE(etlQueue.size() == 1);
    etlQueue.pop();
    REQUIRE(etlQueue.size() == 0);
    REQUIRE(etlQueue.empty());
    //TODO
    /*etlQueue.emplace(10);
    REQUIRE(etlQueue.size() == 1);
    REQUIRE(etlQueue.front() == 10);*/
    std::queue<int> myq;
}