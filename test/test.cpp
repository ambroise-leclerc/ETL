#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#define __Mock_Mock__
#include <etl/ioports.h>

using namespace etl;

SCENARIO("GPIO directions", "[DDR]") {
    GIVEN("an MCU with default output gpios") {

        Device::initialize();
        Port0::setOutput(0b1111111111111111);
        REQUIRE(MockDevice::getInstance().readRegister(Device::DDR0) == 0b1111111111111111);
        WHEN("ports are set to input") {
            Pin0::setInput();
            Pin2::setInput();
            THEN("the direction register changes") {
                REQUIRE(MockDevice::getInstance().readRegister(Device::DDR0) == 0b1111111111111010);
            }
        }
    }
}

