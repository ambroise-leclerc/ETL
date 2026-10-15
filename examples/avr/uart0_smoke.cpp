// Starting Uart0 installs the ETL USART interrupt handlers.
#include <etl.h>
#include <avr/interrupt.h>

#if defined(USART_RX_vect) || defined(USART0_RX_vect)
using Serial = etl::Uart0<9600, etl::FrameFormat::_8N1, uint8_t>;

int main() {
    Serial::start([](uint8_t datum) { Serial::writeAsync(datum); });
    sei();
    Serial::writeAsync('E');
    for (;;) {}
}
#else
// No hardware USART on this MCU
int main() { return 0; }
#endif
