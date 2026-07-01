#define ETLSTD etlstd

#include <etl.h>
#include <etl/CircularBuffer.h>
#include <libstd/include/array>
#include <libstd/include/chrono>
#include <libstd/include/queue>
#include <libstd/include/string_view>

int main() {
    constexpr ETLSTD::string_view text("avr-libstd");
    ETLSTD::array<unsigned char, 3> data{{1, 2, 3}};
    ETLSTD::queue<unsigned char, etl::CircularBuffer<unsigned char, 4>> fifo;
    fifo.push(data[0]);
    fifo.push(static_cast<unsigned char>(ETLSTD::chrono::milliseconds(2).count()));

    return text.starts_with("avr") && fifo.front() == 1 ? 0 : 1;
}
