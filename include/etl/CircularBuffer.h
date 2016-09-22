#pragma once

#include <libstd/include/array>
#include <libstd/include/memory>


namespace etl {

template<typename T, uint8_t N>
class CircularBuffer {
public:
    using value_type = T;
    using size_type = uint8_t;
    using reference = value_type&;
    using const_reference = const value_type&;

    CircularBuffer() : readIndex(0), writeIndex(0) {
    }

    void push_back(T elem) {
        buffer[writeIndex] = elem;
        writeIndex = (writeIndex + 1) % N;

    }

    reference pop_front() {
        auto& value = buffer[readIndex];
        readIndex = (readIndex + 1) % N;
        return value;
    }

    const_reference front() const {
        return buffer[readIndex];
    }

    reference front() {
        return buffer[readIndex];
    }

    reference back() {
        return buffer[writeIndex == 0 ? N - 1 : writeIndex - 1];
    }

    const_reference back() const {
        return buffer[writeIndex == 0 ? N - 1 : writeIndex - 1];
    }

    uint8_t size() const {
        return writeIndex >= readIndex ? writeIndex - readIndex : writeIndex + N - readIndex;
    }

    bool empty() const { return writeIndex == readIndex; }

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

}