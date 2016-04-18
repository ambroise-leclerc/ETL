#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#define __Mock_Mock__
#include <etl/ioports.h>

using namespace etl;

SCENARIO("GPIO directions", "[DDR]") {
    GIVEN("an MCU with default output gpios") {

        Device::Initialize();
        Port0::SetOutput(0b1111111111111111);
        REQUIRE(MockDevice::Instance().ReadRegister(Device::DDR0) == 0b1111111111111111);
        WHEN("ports are set to input") {
            Pin0::SetInput();
            Pin2::SetInput();
            THEN("the direction register changes") {
                REQUIRE(MockDevice::Instance().ReadRegister(Device::DDR0) == 0b1111111111111010);
            }
        }
    }
}

