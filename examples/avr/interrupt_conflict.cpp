// Must NOT build: two different handlers attached to the same vector.
// The CI checks that the build fails on the duplicate vector.
#include <etl.h>

volatile uint8_t ticks;

int main() {
    etl::Interrupt<etl::Timer0::Overflow>::attach<[] { ticks = ticks + 1; }>();
    etl::Interrupt<etl::Timer0::Overflow>::attach<[] { ticks = ticks + 2; }>();
    for (;;) {}
}
