#include <etl.h>
#include <etl/CircularBuffer.h>

int main() {
    etl::CircularBuffer<unsigned char, 4> buffer;
    buffer.push_back(0x12);
    buffer.push_back(0x34);

    return buffer.front() == 0x12 ? 0 : 1;
}
