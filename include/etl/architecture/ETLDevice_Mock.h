/// @file ETLDevice_Mock.h
/// @date 01/04/2016 01:04:16
/// @author Ambroise Leclerc and Cecile Thiebaut
/// @brief Atmel Espressif ESP microcontrollers architecture specifications and low level functions.
//
// Copyright (c) 2017, Ambroise Leclerc and Cecile Thiebaut
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
#pragma once

#include <thread>


namespace etl {

static uint16_t& GP0_OUT         = MockDevice::getInstance().registers[0];
static uint16_t& GP1_OUT         = MockDevice::getInstance().registers[1];
static uint16_t& GPSimuA_OUT     = MockDevice::getInstance().registers[2];
static uint16_t& GPSimuB_OUT     = MockDevice::getInstance().registers[3];
static uint16_t& GP0_IN          = MockDevice::getInstance().registers[4];
static uint16_t& GP1_IN          = MockDevice::getInstance().registers[5];
static uint16_t& GPSimuA_IN      = MockDevice::getInstance().registers[6];
static uint16_t& GPSimuB_IN      = MockDevice::getInstance().registers[7];
static uint16_t& GP0_DIR         = MockDevice::getInstance().registers[8];
static uint16_t& GP1_DIR         = MockDevice::getInstance().registers[9];
static uint16_t& GPSimuA_DIR     = MockDevice::getInstance().registers[10];
static uint16_t& GPSimuB_DIR     = MockDevice::getInstance().registers[11];
static uint16_t& GP0_OUT_SET     = MockDevice::getInstance().registers[12];
static uint16_t& GP1_OUT_SET     = MockDevice::getInstance().registers[13];
static uint16_t& GPSimuA_OUT_SET = MockDevice::getInstance().registers[14];
static uint16_t& GPSimuB_OUT_SET = MockDevice::getInstance().registers[15];
static uint16_t& GP0_OUT_CLR     = MockDevice::getInstance().registers[16];
static uint16_t& GP1_OUT_CLR     = MockDevice::getInstance().registers[17];
static uint16_t& GPSimuA_OUT_CLR = MockDevice::getInstance().registers[18];
static uint16_t& GPSimuB_OUT_CLR = MockDevice::getInstance().registers[19];
static uint16_t& GP0_DIR_SET     = MockDevice::getInstance().registers[20];
static uint16_t& GP1_DIR_SET     = MockDevice::getInstance().registers[21];
static uint16_t& GPSimuA_DIR_SET = MockDevice::getInstance().registers[22];
static uint16_t& GPSimuB_DIR_SET = MockDevice::getInstance().registers[23];
static uint16_t& GP0_DIR_CLR     = MockDevice::getInstance().registers[24];
static uint16_t& GP1_DIR_CLR     = MockDevice::getInstance().registers[25];
static uint16_t& GPSimuA_DIR_CLR = MockDevice::getInstance().registers[26];
static uint16_t& GPSimuB_DIR_CLR = MockDevice::getInstance().registers[27];

struct Pragma {
    Pragma(const std::string command) : paramsList(command) {}
    Pragma& reg(const uint16_t& r)          { paramsList += " " + std::to_string(&r - &GP0_OUT); return *this; }
    Pragma& bit(const uint8_t bitNumber)    { paramsList += " " + std::to_string(1<<bitNumber); return *this; }
    Pragma& bitmask(const uint16_t bitmask) { paramsList += " " + std::to_string(bitmask); return *this; }

    std::string paramsList;
};
class Device {
public:
    static int64_t pragma(const Pragma& param) { return MockDevice::getInstance().pragma(param.paramsList); }
    static int64_t pragma(std::string pragma)  { return MockDevice::getInstance().pragma(pragma); }
    static void initialize()                   { MockDevice::getInstance().configure(NB_PORTS); }
    static void delay_us(uint32_t us)          { std::this_thread::sleep_for(std::chrono::microseconds(us)); }
    static void delay_ms(uint32_t ms)          { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
    static void yield()                        { MockDevice::getInstance().yield(); }
    static void addOnChangeCallback(const std::function<void()> handler, uint16_t& triggerRegister, uint16_t mask) {
        auto index = &triggerRegister - MockDevice::getInstance().registers.data();
        MockDevice::getInstance().addOnChangeCallback(handler, index, mask);
    }

    static void removeOnChangeCallback(uint16_t& triggerRegister, uint16_t mask) {
        auto index = &triggerRegister - MockDevice::getInstance().registers.data();
        MockDevice::getInstance().clearAddOnChangeCallback(index, mask);
    }

    static const size_t sramSize = 10000;
    using RegisterType = uint16_t;
    static const size_t defaultBufferSize = 32;
    using OffType = uint64_t;

    static const uint8_t NB_PORTS = 4;

    static const uint8_t OUT_REG_CYCLES = 3;
    static const uint8_t OUTCLR_REG_CYCLES = 1;
    static const uint8_t OUTSET_REG_CYCLES = 1;
};

} // namespace etl
