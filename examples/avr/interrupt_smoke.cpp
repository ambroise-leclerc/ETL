// Interrupt handlers attached with etl::Interrupt: a timer overflow (blocking), a compare match
// (non-blocking), a naked handler, and a pin change interrupt on MCUs that have one.
#include <etl.h>

volatile uint8_t ticks;
volatile uint8_t matches;
volatile uint8_t changes;

// static (or inline): inlined into the interrupt entry, no call from the interrupt
static void onOverflow() { ticks = ticks + 1; }

// Naked handler: no prologue or epilogue, it saves what it uses and returns with reti
[[gnu::naked]] void onCompareMatchB() {
    asm volatile("sbi %0, 0\n\treti" ::"I"(_SFR_IO_ADDR(GPIOR0)));
}

int main() {
    etl::Interrupt<etl::Timer0::Overflow>::attach<onOverflow>();
    etl::Interrupt<etl::Timer0::CompareMatchA>::attach<[] { matches = matches + 1; }, etl::IsrKind::NonBlocking>();
    etl::Interrupt<etl::Timer0::CompareMatchB>::attach<onCompareMatchB, etl::IsrKind::Naked>();
    etl::Timer0::setInterruptMask(etl::Timer0::OVERFLOW | etl::Timer0::COMPARE_MATCH_A | etl::Timer0::COMPARE_MATCH_B);

#ifdef PCICR
    etl::Interrupt<etl::PinChangeIRQ0::Trigger>::attach<[] { changes = changes + 1; }>();
    etl::PinChangeIRQ0::enableSource(PCINT0);
#endif

    etl::Timer0::setPrescaler(etl::Timer0::CLK_DIV_64);
    sei();
    for (;;) {}
}
