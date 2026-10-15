// Compiled for every supported MCU by the CI: the device header must build, and a handler can be
// attached to a timer vector.
#include <etl.h>

volatile uint8_t ticks;

int main() {
    etl::Interrupt<etl::Timer0::Overflow>::attach<[] { ticks = ticks + 1; }>();
    for (;;) {}
}
