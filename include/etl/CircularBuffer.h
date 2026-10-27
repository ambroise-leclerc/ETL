#pragma once

#include <array>
#include <memory>
#include <type_traits>
#include <limits>
#include <utility>


namespace etl {

template<typename T, size_t N, typename Allocator = std::allocator<T>>
class CircularBuffer {
public:
    using value_type = T;
    using size_type = typename std::conditional_t<N < 256, uint8_t,
                                                  std::conditional_t<N < 65536, uint16_t, uint32_t>>;
    using reference = value_type&;
    using const_reference = const value_type&;

    // constexpr: a static CircularBuffer is constant-initialized, with no startup code
    constexpr CircularBuffer() : buffer{}, index(0), nbElems(0) {}

    void push_back(const value_type& elem) {
        buffer[(index + nbElems) % N] = elem;
        incNbElems();
    }

    void push_back(value_type&& elem) {
        buffer[(index + nbElems) % N] = std::move(elem);
        incNbElems();
    }

    reference pop_front() {
        auto&& value = buffer[index];
        index = (index + 1) % N;
        if (nbElems > 0) --nbElems;
        return value;
    }

    const_reference front() const { return buffer[index]; }
    reference front() { return buffer[index];}
    reference back() { return buffer[(index + nbElems - 1) % N]; }
    const_reference back() const { return buffer[(index + nbElems - 1) % N]; }
    size_type size() const { return nbElems; }
    bool empty() const { return nbElems == 0; }

    template<typename... Args>
    void emplace_back(Args&& ... args) {
        Allocator all;
        std::allocator_traits<Allocator>::construct(all, &buffer[(index + nbElems) % N], std::forward<Args>(args)...);
        incNbElems();
    }


private:
    std::array<T, N> buffer;
    size_type index, nbElems;

    void incNbElems() {
        if (nbElems < N) {
            ++nbElems;
        }
        else {
            index = (index + 1) % N;
        }
    }
};
}