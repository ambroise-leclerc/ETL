// A user-defined USART interrupt handler next to ETL: ETL must leave the USART vectors alone
// unless a Uart is started.
#include <etl.h>
#include <avr/interrupt.h>

#if defined(USART_RX_vect) || defined(USART0_RX_vect)
#ifdef USART_RX_vect
ISR(USART_RX_vect)
#else
ISR(USART0_RX_vect)
#endif
{
    GPIOR0 = UDR0;
}

int main() {
    UCSR0B = (1 << RXEN0) | (1 << RXCIE0);
    sei();
    for (;;) {}
}
#else
// No hardware USART on this MCU
int main() { return 0; }
#endif
