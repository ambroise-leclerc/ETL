#include <catch.hpp>

#define __Mock_Mock__
#include <etl/ioports.h>
#include <algorithm>



using namespace etl;
/*
SCENARIO("GPIO basic tests", "[GPIO]") {
    GIVEN("an MCU with default output gpios") {
        Device::initialize();
        Port0::setOutput(0b1111111111111111);
        Port0::assign(0b0000000000000000);
        REQUIRE()
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