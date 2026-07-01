/// @file test/libstd/thread.cpp
/// @date 01/07/2026
/// @author Ambroise Leclerc
/// @brief Tests for <thread>
//
// Copyright (c) 2026, Ambroise Leclerc
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in
//     the documentation and/or other materials provided with the
//     distribution.
//   * Neither the name of the copyright holders nor the names of
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
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
