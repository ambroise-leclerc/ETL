#include <catch.hpp>

#define __Mock_Mock__
#include <etl/ioports.h>
#include <algorithm>



using namespace etl;

SCENARIO("GPIO basic tests", "[GPIO]") {
    GIVEN("an MCU with default output gpios") {
        Device::initialize();
        Port0::setOutput(0b1111111111111111);
        Port0::assign(0b0000000000000000);
        Device::pragma(Pragma("BitLink").reg(Pin0::Port::getOutputRegister()).bit(Pin0::bit()).reg(Pin0::Port::getInputRegister()).bit(Pin0::bit()));
        Device::pragma(Pragma("BitLink").reg(Pin1::Port::getOutputRegister()).bit(Pin1::bit()).reg(Pin1::Port::getInputRegister()).bit(Pin1::bit()));
        Device::pragma(Pragma("BitLink").reg(Pin2::Port::getOutputRegister()).bit(Pin2::bit()).reg(Pin2::Port::getInputRegister()).bit(Pin2::bit()));
        WHEN("Pins output value change") {
            Pin0::set();
            Pin2::set();
            REQUIRE(Port0::testBits(0b101));
            Pin1::set(Pin0::test());
            REQUIRE(Port0::testBits(0b111));
            Pin0::clear();
            Pin2::set(Pin0::test());
            REQUIRE(Port0::testBits(0b10));
            THEN("") {
            }
        }
    }
}
