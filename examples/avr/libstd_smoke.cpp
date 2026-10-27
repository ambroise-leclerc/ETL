
#include <etl.h>
#include <etl/CircularBuffer.h>
#include <array>
#include <chrono>
#include <queue>
#include <string_view>

int main() {
    constexpr std::string_view text("avr-libstd");
    std::array<unsigned char, 3> data{{1, 2, 3}};
    std::queue<unsigned char, etl::CircularBuffer<unsigned char, 4>> fifo;
    fifo.push(data[0]);
    fifo.push(static_cast<unsigned char>(std::chrono::milliseconds(2).count()));

    return text.starts_with("avr") && fifo.front() == 1 ? 0 : 1;
}
