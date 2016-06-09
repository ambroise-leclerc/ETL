#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#define __Mock_Mock__
#include <etl/ioports.h>
#include <etl/architecture/uart_Mock.h>
#include <algorithm>



using namespace etl;
/*
SCENARIO("GPIO directions", "[DDR]") {
    GIVEN("an MCU with default output gpios") {
        Device::initialize();
        Port0::setOutput(0b1111111111111111);
        REQUIRE(MockDevice::getInstance().readRegister(Device::DDR0) == 0b1111111111111111);
        WHEN("ports are set to input") {
            Pin0::setInput();
            Pin2::setInput();
            THEN("the direction register changes") {
                //REQUIRE(MockDevice::getInstance().readRegister(Device::DDR0) == 0b1111111111111010);
            }
        }
    }
}
*/
/*
template<typename Timer>
class TimerDutyCycle {
public:
    static void Init(uint8_t dutyCyclePercent) {
        Interrupts::Disable();
        Timer::
        Interrupts::Enable();
    }
}

SCENARIO("Timers") {
    
}

*/

namespace etl {
struct FakePin {
    static void setOutput() {

    }

    static void set() {
        std::cout << '1';
    }

    static void clear() {
        std::cout << '0';
    }
};

template<> struct is_uart_txd_capable<FakePin> : std::true_type {};
}

SCENARIO("Uart") {
    using uart = etl::Uart<Pin0, etl::FakePin>;
    uart::start();
    uart::write(2);
    uart::write(5);
    uart::write(7);

    using uart2 = etl::Uart<Pin0, FakePin,14500, FrameFormat::_5E2,uint8_t>;
    uart2::start();
    uart2::write(2);
    uart2::write(5);
    uart2::write(7);
}
