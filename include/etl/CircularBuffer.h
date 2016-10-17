#pragma once

#include <libstd/include/array>
#include <libstd/include/memory>


namespace etl {

template<typename T, uint8_t N, typename Allocator = ETLSTD::allocator<T>>
class CircularBuffer {
public:
    using value_type = T;
    using size_type = uint8_t;
    using reference = value_type&;
    using const_reference = const value_type&;

    CircularBuffer() : index(0), nbElems(0) {}

    void push_back(const value_type& elem) {
        buffer[(index + nbElems) % N] = elem;
        incNbElems();
    }

    void push_back(value_type&& elem) {
        buffer[(index + nbElems) % N] = ETLSTD::move(elem);
        incNbElems();
    }
 

    reference pop_front() {
        auto&& value = buffer[index];
        index = (index + 1) % N;
        if (nbElems > 0) --nbElems;
        return value;
    }
    
     const_reference read(uint8_t i) const {
            auto indexToRead = (readIndex + i) % N;
            auto& value = buffer[indexToRead];
            return value;
     }

     reference read(uint8_t i) {
             auto indexToRead = (readIndex + i) % N;
             auto& value = buffer[indexToRead];
             return value;
     }

    const_reference front() const { return buffer[index]; }
    reference front() { return buffer[index];}
    reference back() { return buffer[(index + nbElems - 1) % N]; }
    const_reference back() const { return buffer[(index + nbElems - 1) % N]; }
    uint8_t size() const { return nbElems; }
    bool empty() const { return nbElems == 0; }

    template<typename... Args>
    void emplace_back(Args&& ... args) {
        Allocator all;
        all.construct(&buffer[(index + nbElems) % N], ETLSTD::forward<Args>(args)...);
        incNbElems();
    }


private:
    ETLSTD::array<T, N> buffer;
    uint8_t index, nbElems;

    void incNbElems() {
        if (nbElems < N) {
            ++nbElems;
        }
        else {
            index = (index + 1) % N;
        }
    }
};


template<typename Cout ,typename Print ,typename T, uint8_t N, typename Allocator = std::allocator<T>>
class CircularBufferPointer {
    public:
    using value_type = std::unique_ptr<T>;
    using size_type = uint8_t;
    using reference = value_type&;
    using const_reference = const value_type&;

    CircularBufferPointer() : readIndex(0), writeIndex(0) {
    }

    void push_back(std::unique_ptr<T>&& elem) {
        cout.print(hD44780,"write index: ");
        hD44780.display((char)(48+writeIndex));
        buffer[writeIndex] = std::move(elem);
        writeIndex = (writeIndex + 1) % N;

    }
    

    std::unique_ptr<T>&& pop_front() {
        cout.print(hD44780,"readIndex index: ");
        hD44780.display((char)(48+readIndex));
      	auto&& ptr = std::move(buffer[readIndex]);
      	readIndex = (readIndex + 1) % N;
      	return  std::move(ptr);
    }
    
  /*  const_reference read(uint8_t i) const {
        auto indexToRead = (readIndex + i) % N;
        return buffer[indexToRead];
    }

    reference read(uint8_t i) {
        auto indexToRead = (readIndex + i) % N;
        return buffer[indexToRead];
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
    }*/

    uint8_t size() const {
        return writeIndex >= readIndex ? writeIndex - readIndex : writeIndex + N - readIndex;
    }

    bool empty() const { return writeIndex == readIndex; }

    /*template<typename... Args>
    void emplace_back(Args&& ... args) {
        Allocator all;
        all.construct(&buffer[writeIndex], std::forward<Args>(args)...);
        writeIndex = (writeIndex + 1) % N;
    }*/


    private:
    std::array<std::unique_ptr<T>, N> buffer;
    uint8_t readIndex, writeIndex;
     Print hD44780;
     Cout cout;
};

}