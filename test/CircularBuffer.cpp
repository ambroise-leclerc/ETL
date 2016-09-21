#include <catch.hpp>

#define __Mock_Mock__
#include <etl/ioports.h>

#include <queue> 
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
        container.emplace_back(std::forward<Args>(args)...);
    }

    void swap(Queue<T,Container>& right) {
        Container* pointeurLeft = &container;
        Container* pointeurRight = &right.container;
        Container temp = *pointeurLeft;
        *pointeurLeft = *pointeurRight;
        *pointeurRight = temp;
    }

protected:
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

    etl::Queue<uint8_t> etlQueue2;
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

    //TODO
    etlQueue.emplace(10);
    REQUIRE(etlQueue.size() == 2);
    REQUIRE(etlQueue.back() == 10);
    std::queue<int> myq;
    myq.emplace(10);
    REQUIRE(myq.size() == 1);
    REQUIRE(myq.front() == 10);
}