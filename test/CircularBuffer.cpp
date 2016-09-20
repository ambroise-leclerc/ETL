#include <catch.hpp>

#define __Mock_Mock__
#include <etl/ioports.h>

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

    uint8_t size() {
        return writeIndex >= readIndex ? writeIndex - readIndex : writeIndex + N - readIndex;
    }

private:
    std::array<T, N> buffer;
    uint8_t readIndex, writeIndex;
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

    
    
}