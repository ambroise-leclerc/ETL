// Reference for isr_equivalence_etl.cpp, written with avr-libc ISR().
#include <etl.h>
#include <avr/interrupt.h>

volatile uint8_t ticks;
volatile uint8_t matches;

#ifdef TIM0_OVF_vect  // ATtiny names
ISR(TIM0_OVF_vect) { ticks = ticks + GPIOR0; }
ISR(TIM0_COMPA_vect, ISR_NOBLOCK) { matches = matches + 1; }
#else
ISR(TIMER0_OVF_vect) { ticks = ticks + GPIOR0; }
ISR(TIMER0_COMPA_vect, ISR_NOBLOCK) { matches = matches + 1; }
#endif

int main() {
    for (;;) {}
}
