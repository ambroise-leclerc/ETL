// Same handlers as isr_equivalence_libc.cpp, attached with etl::Interrupt: the CI checks that the
// generated interrupt code is identical.
#include <etl.h>

volatile uint8_t ticks;
volatile uint8_t matches;

int main() {
    etl::Interrupt<etl::Timer0::Overflow>::attach<[] { ticks = ticks + GPIOR0; }>();
    etl::Interrupt<etl::Timer0::CompareMatchA>::attach<[] { matches = matches + 1; }, etl::IsrKind::NonBlocking>();
    for (;;) {}
}
