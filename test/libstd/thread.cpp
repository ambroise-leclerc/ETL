#include <catch.hpp>

#define __Mock_Mock__
#define ETLSTD etlstd

#include <etl/ioports.h>
#include <libstd/include/thread>

using namespace ETLSTD;
using namespace ETLSTD::chrono_literals;

SCENARIO("std::this_thread::sleep_for constant-duration path", "[libstd][thread]") {
    etl::Device::resetDelayTicksLog();
    using clock_cycles = ETLSTD::chrono::duration<unsigned long, ETLSTD::ratio<1, etl::Device::McuFrequency>>;

    ETLSTD::this_thread::sleep_for(3us);

    REQUIRE(etl::Device::lastDelayTicks() == chrono::duration_cast<clock_cycles>(3us).count());
    REQUIRE(etl::Device::totalDelayTicks() == etl::Device::lastDelayTicks());
}

SCENARIO("std::this_thread::sleep_for millisecond runtime path", "[libstd][thread]") {
    etl::Device::resetDelayTicksLog();
    using clock_cycles = ETLSTD::chrono::duration<unsigned long, ETLSTD::ratio<1, etl::Device::McuFrequency>>;

    auto duration = 2ms;
    ETLSTD::this_thread::sleep_for(duration);

    const auto ticksPerMs = chrono::duration_cast<clock_cycles>(1ms).count() - (64 / etl::Device::architectureWidth);
    REQUIRE(etl::Device::lastDelayTicks() == ticksPerMs);
    REQUIRE(etl::Device::totalDelayTicks() == static_cast<uint64_t>(ticksPerMs) * 2);
}
