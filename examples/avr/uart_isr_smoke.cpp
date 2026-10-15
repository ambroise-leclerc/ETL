// Checks how the USART vectors are shared between ETL and user code:
// - without ETL_UART0_ISR, including ETL leaves the vectors free for user handlers;
// - with ETL_UART0_ISR (set by CMake), ETL installs its own handlers for Uart0.
#include <etl.h>
#include <avr/interrupt.h>

#if defined(USART_RX_vect) || defined(USART0_RX_vect)

#ifdef ETL_UART0_ISR
using Serial = etl::Uart0<9600, etl::FrameFormat::_8N1, uint8_t>;

int main() {
    Serial::start([](uint8_t datum) { Serial::writeAsync(datum); });
    sei();
    Serial::writeAsync('E');
    for (;;) {}
}
#else
#ifdef USART_RX_vect
#define USER_USART_RX_vect USART_RX_vect
#else
#define USER_USART_RX_vect USART0_RX_vect
#endif

volatile uint8_t received;

ISR(USER_USART_RX_vect) { received = UDR0; }

int main() {
    UCSR0B = (1 << RXEN0) | (1 << RXCIE0);
    sei();
    for (;;) {}
}
#endif

#else
// No hardware USART on this MCU
int main() { return 0; }
#endif
