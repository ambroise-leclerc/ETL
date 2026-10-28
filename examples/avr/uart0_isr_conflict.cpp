// Must NOT build: starting Uart0 while defining a handler for the same USART vector.
// The CI checks that the build fails on the duplicate vector.
#include <etl.h>
#include <avr/interrupt.h>

using Serial = etl::Uart0<9600, etl::FrameFormat::_8N1, uint8_t>;

ISR(USART_RX_vect) { GPIOR0 = UDR0; }

int main() {
    Serial::start();
    for (;;) {}
}
