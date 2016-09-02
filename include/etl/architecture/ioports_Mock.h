/// @file ioports_Mock.h
/// @date 01/04/2016 01:04:16
/// @author Ambroise Leclerc and Cécile Gomes
/// @brief Mock microcontroller peripherals handling classes
//
// Copyright (c) 2016, Ambroise Leclerc and Cécile Gomes
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

#include <MockDevice.h>
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
    using OffType = uint64_t;

    static const uint8_t NB_PORTS = 4;

    static const uint8_t OUT_REG_CYCLES = 3;
    static const uint8_t OUTCLR_REG_CYCLES = 1;
    static const uint8_t OUTSET_REG_CYCLES = 1;
};

struct PinChangeIRQ0;
struct PinChangeIRQ1;
struct PinChangeIRQSimuA;
struct PinChangeIRQSimuB;

class Port0 {
public:
    using PinChangeIRQ = PinChangeIRQ0;

    /// Assigns a value to Port0
    /// @param[in] value value to affect to port0
    static void assign(uint16_t value)     {
        if (Device::OUT_REG_CYCLES < (Device::OUTSET_REG_CYCLES + Device::OUTCLR_REG_CYCLES)) {
            GP0_OUT = value;
        }
        else {
            GP0_OUT_CLR = 0b1111111111111111;
            GP0_OUT_SET = value;
        }
        Device::yield();
    }

    /// Sets masked bits in PORT0
    /// @param[in] mask bits to set
    static void setBits(uint16_t mask)     { GP0_OUT |= mask; Device::yield(); }

    /// Clears masked bits in PORT0
    /// @param[in] mask bits to clear
    static void clearBits(uint16_t mask)   { GP0_OUT &= ~mask; Device::yield(); } 

    /// Changes values of masked bits in PORT0
    /// @param[in] mask bits to change
    /// @param[in] value new bits values
    static void changeBits(uint16_t mask, uint16_t value) { auto tmp = GP0_OUT & ~mask; GP0_OUT = tmp | value; Device::yield(); }

    /// Toggles masked bits in PORT0
    /// @param[in] mask bits to toggle
    static void toggleBits(uint16_t mask)  { GP0_OUT ^= mask; Device::yield(); } 

    /// Pulses masked bits in PORT0 with high state first.
    /// @param[in] mask bits to pulse
    static void pulseHigh(uint16_t mask)   { setBits(mask); clearBits(mask); Device::yield(); }

    /// Pulses masked bits in PORT0 with low state first.
    /// @param[in] mask bits to pulse
    static void pulseLow(uint16_t mask)    { clearBits(mask); setBits(mask); Device::yield(); }

    /// Set corresponding masked bits of PORT0 to output direction.
    /// @param[in] mask bits
    static void setOutput(uint16_t mask)   { GP0_DIR |= mask; Device::yield(); }

    /// Set corresponding masked bits of PORT0 to input direction.
    /// @param[in] mask bits
    static void setInput(uint16_t mask)    { GP0_DIR &= ~mask; Device::yield(); }

    /// Tests masked bits of PORT0
    /// @param[in] mask bits
    /// @param[in] true if the corresponding bits are all set, false otherwise.
    static bool testBits(uint16_t mask)    { return (GP0_IN & mask) == mask;}

    /// Returns the value of the bit at the position pos.
    /// @param[in] position of the bit to return
    /// @return true if the requested bit is set, false otherwise.
    static bool test(uint8_t pos)          { return (GP0_IN & (1<<pos)) != 0; }

    /// Returns the native output register associated to Port0
    static uint16_t& getOutputRegister()    { return GP0_OUT; }

    /// Returns the native input register associated to Port0
    static uint16_t& getInputRegister()     { return GP0_IN; }

    /// Returns the native direction register associated to Port0
    static uint16_t& getDirectionRegister() { return GP0_DIR; }

    static void onChange(const std::function<void()>& callback, uint16_t mask) {
        Device::addOnChangeCallback(callback, getInputRegister(), mask);
    }

    static void clearOnChange(uint16_t mask) {
        Device::removeOnChangeCallback(getInputRegister(), mask);
    }
};

class Pin0 {
public:
    /// Sets Pin0 to HIGH.
    static void set()       { Port0::setBits(1<<0); }

    /// Sets Pin0 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port0::setBits(1<<0) : Port0::clearBits(1<<0); }

    /// Sets Pin0 to LOW.
    static void clear()     { Port0::clearBits(1<<0); }

    /// Toggles Pin0 value.
    static void toggle()    { Port0::toggleBits(1<<0); }

    /// Configures Pin0 as an output pin.
    static void setOutput() { Port0::setOutput(1<<0); }

    /// Configures Pin0 as an input pin.
    static void setInput()  { Port0::setInput(1<<0); }

    /// Pulses Pin0 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin0 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin0 value.
    /// @return true if Pin0 is high, false otherwise.
    static bool test()      { return Port0::test(0); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<0)
    static constexpr uint16_t bitmask() { return (1<<0); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 0
    static constexpr uint8_t bit()      { return 0; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port0;
};

class Pin1 {
public:
    /// Sets Pin1 to HIGH.
    static void set()       { Port0::setBits(1<<1); }

    /// Sets Pin1 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port0::setBits(1<<1) : Port0::clearBits(1<<1); }

    /// Sets Pin1 to LOW.
    static void clear()     { Port0::clearBits(1<<1); }

    /// Toggles Pin1 value.
    static void toggle()    { Port0::toggleBits(1<<1); }

    /// Configures Pin1 as an output pin.
    static void setOutput() { Port0::setOutput(1<<1); }

    /// Configures Pin1 as an input pin.
    static void setInput()  { Port0::setInput(1<<1); }

    /// Pulses Pin1 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin1 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin1 value.
    /// @return true if Pin1 is high, false otherwise.
    static bool test()      { return Port0::test(1); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<1)
    static constexpr uint16_t bitmask() { return (1<<1); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 1
    static constexpr uint8_t bit()      { return 1; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port0;
};

class Pin2 {
public:
    /// Sets Pin2 to HIGH.
    static void set()       { Port0::setBits(1<<2); }

    /// Sets Pin2 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port0::setBits(1<<2) : Port0::clearBits(1<<2); }

    /// Sets Pin2 to LOW.
    static void clear()     { Port0::clearBits(1<<2); }

    /// Toggles Pin2 value.
    static void toggle()    { Port0::toggleBits(1<<2); }

    /// Configures Pin2 as an output pin.
    static void setOutput() { Port0::setOutput(1<<2); }

    /// Configures Pin2 as an input pin.
    static void setInput()  { Port0::setInput(1<<2); }

    /// Pulses Pin2 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin2 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin2 value.
    /// @return true if Pin2 is high, false otherwise.
    static bool test()      { return Port0::test(2); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<2)
    static constexpr uint16_t bitmask() { return (1<<2); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 2
    static constexpr uint8_t bit()      { return 2; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port0;
};

class Pin3 {
public:
    /// Sets Pin3 to HIGH.
    static void set()       { Port0::setBits(1<<3); }

    /// Sets Pin3 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port0::setBits(1<<3) : Port0::clearBits(1<<3); }

    /// Sets Pin3 to LOW.
    static void clear()     { Port0::clearBits(1<<3); }

    /// Toggles Pin3 value.
    static void toggle()    { Port0::toggleBits(1<<3); }

    /// Configures Pin3 as an output pin.
    static void setOutput() { Port0::setOutput(1<<3); }

    /// Configures Pin3 as an input pin.
    static void setInput()  { Port0::setInput(1<<3); }

    /// Pulses Pin3 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin3 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin3 value.
    /// @return true if Pin3 is high, false otherwise.
    static bool test()      { return Port0::test(3); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<3)
    static constexpr uint16_t bitmask() { return (1<<3); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 3
    static constexpr uint8_t bit()      { return 3; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port0;
};

class Pin4 {
public:
    /// Sets Pin4 to HIGH.
    static void set()       { Port0::setBits(1<<4); }

    /// Sets Pin4 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port0::setBits(1<<4) : Port0::clearBits(1<<4); }

    /// Sets Pin4 to LOW.
    static void clear()     { Port0::clearBits(1<<4); }

    /// Toggles Pin4 value.
    static void toggle()    { Port0::toggleBits(1<<4); }

    /// Configures Pin4 as an output pin.
    static void setOutput() { Port0::setOutput(1<<4); }

    /// Configures Pin4 as an input pin.
    static void setInput()  { Port0::setInput(1<<4); }

    /// Pulses Pin4 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin4 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin4 value.
    /// @return true if Pin4 is high, false otherwise.
    static bool test()      { return Port0::test(4); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<4)
    static constexpr uint16_t bitmask() { return (1<<4); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 4
    static constexpr uint8_t bit()      { return 4; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port0;
};

class Pin5 {
public:
    /// Sets Pin5 to HIGH.
    static void set()       { Port0::setBits(1<<5); }

    /// Sets Pin5 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port0::setBits(1<<5) : Port0::clearBits(1<<5); }

    /// Sets Pin5 to LOW.
    static void clear()     { Port0::clearBits(1<<5); }

    /// Toggles Pin5 value.
    static void toggle()    { Port0::toggleBits(1<<5); }

    /// Configures Pin5 as an output pin.
    static void setOutput() { Port0::setOutput(1<<5); }

    /// Configures Pin5 as an input pin.
    static void setInput()  { Port0::setInput(1<<5); }

    /// Pulses Pin5 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin5 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin5 value.
    /// @return true if Pin5 is high, false otherwise.
    static bool test()      { return Port0::test(5); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<5)
    static constexpr uint16_t bitmask() { return (1<<5); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 5
    static constexpr uint8_t bit()      { return 5; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port0;
};

class Pin6 {
public:
    /// Sets Pin6 to HIGH.
    static void set()       { Port0::setBits(1<<6); }

    /// Sets Pin6 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port0::setBits(1<<6) : Port0::clearBits(1<<6); }

    /// Sets Pin6 to LOW.
    static void clear()     { Port0::clearBits(1<<6); }

    /// Toggles Pin6 value.
    static void toggle()    { Port0::toggleBits(1<<6); }

    /// Configures Pin6 as an output pin.
    static void setOutput() { Port0::setOutput(1<<6); }

    /// Configures Pin6 as an input pin.
    static void setInput()  { Port0::setInput(1<<6); }

    /// Pulses Pin6 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin6 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin6 value.
    /// @return true if Pin6 is high, false otherwise.
    static bool test()      { return Port0::test(6); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<6)
    static constexpr uint16_t bitmask() { return (1<<6); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 6
    static constexpr uint8_t bit()      { return 6; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port0;
};

class Pin7 {
public:
    /// Sets Pin7 to HIGH.
    static void set()       { Port0::setBits(1<<7); }

    /// Sets Pin7 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port0::setBits(1<<7) : Port0::clearBits(1<<7); }

    /// Sets Pin7 to LOW.
    static void clear()     { Port0::clearBits(1<<7); }

    /// Toggles Pin7 value.
    static void toggle()    { Port0::toggleBits(1<<7); }

    /// Configures Pin7 as an output pin.
    static void setOutput() { Port0::setOutput(1<<7); }

    /// Configures Pin7 as an input pin.
    static void setInput()  { Port0::setInput(1<<7); }

    /// Pulses Pin7 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin7 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin7 value.
    /// @return true if Pin7 is high, false otherwise.
    static bool test()      { return Port0::test(7); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<7)
    static constexpr uint16_t bitmask() { return (1<<7); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 7
    static constexpr uint8_t bit()      { return 7; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port0;
};

class Pin8 {
public:
    /// Sets Pin8 to HIGH.
    static void set()       { Port0::setBits(1<<8); }

    /// Sets Pin8 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port0::setBits(1<<8) : Port0::clearBits(1<<8); }

    /// Sets Pin8 to LOW.
    static void clear()     { Port0::clearBits(1<<8); }

    /// Toggles Pin8 value.
    static void toggle()    { Port0::toggleBits(1<<8); }

    /// Configures Pin8 as an output pin.
    static void setOutput() { Port0::setOutput(1<<8); }

    /// Configures Pin8 as an input pin.
    static void setInput()  { Port0::setInput(1<<8); }

    /// Pulses Pin8 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin8 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin8 value.
    /// @return true if Pin8 is high, false otherwise.
    static bool test()      { return Port0::test(8); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<8)
    static constexpr uint16_t bitmask() { return (1<<8); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 8
    static constexpr uint8_t bit()      { return 8; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port0;
};

class Pin9 {
public:
    /// Sets Pin9 to HIGH.
    static void set()       { Port0::setBits(1<<9); }

    /// Sets Pin9 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port0::setBits(1<<9) : Port0::clearBits(1<<9); }

    /// Sets Pin9 to LOW.
    static void clear()     { Port0::clearBits(1<<9); }

    /// Toggles Pin9 value.
    static void toggle()    { Port0::toggleBits(1<<9); }

    /// Configures Pin9 as an output pin.
    static void setOutput() { Port0::setOutput(1<<9); }

    /// Configures Pin9 as an input pin.
    static void setInput()  { Port0::setInput(1<<9); }

    /// Pulses Pin9 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin9 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin9 value.
    /// @return true if Pin9 is high, false otherwise.
    static bool test()      { return Port0::test(9); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<9)
    static constexpr uint16_t bitmask() { return (1<<9); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 9
    static constexpr uint8_t bit()      { return 9; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port0;
};

class Pin10 {
public:
    /// Sets Pin10 to HIGH.
    static void set()       { Port0::setBits(1<<10); }

    /// Sets Pin10 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port0::setBits(1<<10) : Port0::clearBits(1<<10); }

    /// Sets Pin10 to LOW.
    static void clear()     { Port0::clearBits(1<<10); }

    /// Toggles Pin10 value.
    static void toggle()    { Port0::toggleBits(1<<10); }

    /// Configures Pin10 as an output pin.
    static void setOutput() { Port0::setOutput(1<<10); }

    /// Configures Pin10 as an input pin.
    static void setInput()  { Port0::setInput(1<<10); }

    /// Pulses Pin10 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin10 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin10 value.
    /// @return true if Pin10 is high, false otherwise.
    static bool test()      { return Port0::test(10); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<10)
    static constexpr uint16_t bitmask() { return (1<<10); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 10
    static constexpr uint8_t bit()      { return 10; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port0;
};

class Pin11 {
public:
    /// Sets Pin11 to HIGH.
    static void set()       { Port0::setBits(1<<11); }

    /// Sets Pin11 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port0::setBits(1<<11) : Port0::clearBits(1<<11); }

    /// Sets Pin11 to LOW.
    static void clear()     { Port0::clearBits(1<<11); }

    /// Toggles Pin11 value.
    static void toggle()    { Port0::toggleBits(1<<11); }

    /// Configures Pin11 as an output pin.
    static void setOutput() { Port0::setOutput(1<<11); }

    /// Configures Pin11 as an input pin.
    static void setInput()  { Port0::setInput(1<<11); }

    /// Pulses Pin11 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin11 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin11 value.
    /// @return true if Pin11 is high, false otherwise.
    static bool test()      { return Port0::test(11); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<11)
    static constexpr uint16_t bitmask() { return (1<<11); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 11
    static constexpr uint8_t bit()      { return 11; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port0;
};

class Pin12 {
public:
    /// Sets Pin12 to HIGH.
    static void set()       { Port0::setBits(1<<12); }

    /// Sets Pin12 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port0::setBits(1<<12) : Port0::clearBits(1<<12); }

    /// Sets Pin12 to LOW.
    static void clear()     { Port0::clearBits(1<<12); }

    /// Toggles Pin12 value.
    static void toggle()    { Port0::toggleBits(1<<12); }

    /// Configures Pin12 as an output pin.
    static void setOutput() { Port0::setOutput(1<<12); }

    /// Configures Pin12 as an input pin.
    static void setInput()  { Port0::setInput(1<<12); }

    /// Pulses Pin12 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin12 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin12 value.
    /// @return true if Pin12 is high, false otherwise.
    static bool test()      { return Port0::test(12); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<12)
    static constexpr uint16_t bitmask() { return (1<<12); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 12
    static constexpr uint8_t bit()      { return 12; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port0;
};

class Pin13 {
public:
    /// Sets Pin13 to HIGH.
    static void set()       { Port0::setBits(1<<13); }

    /// Sets Pin13 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port0::setBits(1<<13) : Port0::clearBits(1<<13); }

    /// Sets Pin13 to LOW.
    static void clear()     { Port0::clearBits(1<<13); }

    /// Toggles Pin13 value.
    static void toggle()    { Port0::toggleBits(1<<13); }

    /// Configures Pin13 as an output pin.
    static void setOutput() { Port0::setOutput(1<<13); }

    /// Configures Pin13 as an input pin.
    static void setInput()  { Port0::setInput(1<<13); }

    /// Pulses Pin13 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin13 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin13 value.
    /// @return true if Pin13 is high, false otherwise.
    static bool test()      { return Port0::test(13); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<13)
    static constexpr uint16_t bitmask() { return (1<<13); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 13
    static constexpr uint8_t bit()      { return 13; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port0;
};

class Pin14 {
public:
    /// Sets Pin14 to HIGH.
    static void set()       { Port0::setBits(1<<14); }

    /// Sets Pin14 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port0::setBits(1<<14) : Port0::clearBits(1<<14); }

    /// Sets Pin14 to LOW.
    static void clear()     { Port0::clearBits(1<<14); }

    /// Toggles Pin14 value.
    static void toggle()    { Port0::toggleBits(1<<14); }

    /// Configures Pin14 as an output pin.
    static void setOutput() { Port0::setOutput(1<<14); }

    /// Configures Pin14 as an input pin.
    static void setInput()  { Port0::setInput(1<<14); }

    /// Pulses Pin14 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin14 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin14 value.
    /// @return true if Pin14 is high, false otherwise.
    static bool test()      { return Port0::test(14); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<14)
    static constexpr uint16_t bitmask() { return (1<<14); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 14
    static constexpr uint8_t bit()      { return 14; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port0;
};

class Pin15 {
public:
    /// Sets Pin15 to HIGH.
    static void set()       { Port0::setBits(1<<15); }

    /// Sets Pin15 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port0::setBits(1<<15) : Port0::clearBits(1<<15); }

    /// Sets Pin15 to LOW.
    static void clear()     { Port0::clearBits(1<<15); }

    /// Toggles Pin15 value.
    static void toggle()    { Port0::toggleBits(1<<15); }

    /// Configures Pin15 as an output pin.
    static void setOutput() { Port0::setOutput(1<<15); }

    /// Configures Pin15 as an input pin.
    static void setInput()  { Port0::setInput(1<<15); }

    /// Pulses Pin15 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin15 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin15 value.
    /// @return true if Pin15 is high, false otherwise.
    static bool test()      { return Port0::test(15); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<15)
    static constexpr uint16_t bitmask() { return (1<<15); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 15
    static constexpr uint8_t bit()      { return 15; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port0;
};


class Port1 {
public:
    using PinChangeIRQ = PinChangeIRQ1;

    /// Assigns a value to Port1
    /// @param[in] value value to affect to port1
    static void assign(uint16_t value)     {
        if (Device::OUT_REG_CYCLES < (Device::OUTSET_REG_CYCLES + Device::OUTCLR_REG_CYCLES)) {
            GP1_OUT = value;
        }
        else {
            GP1_OUT_CLR = 0b1111111111111111;
            GP1_OUT_SET = value;
        }
        Device::yield();
    }

    /// Sets masked bits in PORT1
    /// @param[in] mask bits to set
    static void setBits(uint16_t mask)     { GP1_OUT |= mask; Device::yield(); }

    /// Clears masked bits in PORT1
    /// @param[in] mask bits to clear
    static void clearBits(uint16_t mask)   { GP1_OUT &= ~mask; Device::yield(); } 

    /// Changes values of masked bits in PORT1
    /// @param[in] mask bits to change
    /// @param[in] value new bits values
    static void changeBits(uint16_t mask, uint16_t value) { auto tmp = GP1_OUT & ~mask; GP1_OUT = tmp | value; Device::yield(); }

    /// Toggles masked bits in PORT1
    /// @param[in] mask bits to toggle
    static void toggleBits(uint16_t mask)  { GP1_OUT ^= mask; Device::yield(); } 

    /// Pulses masked bits in PORT1 with high state first.
    /// @param[in] mask bits to pulse
    static void pulseHigh(uint16_t mask)   { setBits(mask); clearBits(mask); Device::yield(); }

    /// Pulses masked bits in PORT1 with low state first.
    /// @param[in] mask bits to pulse
    static void pulseLow(uint16_t mask)    { clearBits(mask); setBits(mask); Device::yield(); }

    /// Set corresponding masked bits of PORT1 to output direction.
    /// @param[in] mask bits
    static void setOutput(uint16_t mask)   { GP1_DIR |= mask; Device::yield(); }

    /// Set corresponding masked bits of PORT1 to input direction.
    /// @param[in] mask bits
    static void setInput(uint16_t mask)    { GP1_DIR &= ~mask; Device::yield(); }

    /// Tests masked bits of PORT1
    /// @param[in] mask bits
    /// @param[in] true if the corresponding bits are all set, false otherwise.
    static bool testBits(uint16_t mask)    { return (GP1_IN & mask) == mask;}

    /// Returns the value of the bit at the position pos.
    /// @param[in] position of the bit to return
    /// @return true if the requested bit is set, false otherwise.
    static bool test(uint8_t pos)          { return (GP1_IN & (1<<pos)) != 0; }

    /// Returns the native output register associated to Port1
    static uint16_t& getOutputRegister()    { return GP1_OUT; }

    /// Returns the native input register associated to Port1
    static uint16_t& getInputRegister()     { return GP1_IN; }

    /// Returns the native direction register associated to Port1
    static uint16_t& getDirectionRegister() { return GP1_DIR; }

    static void onChange(const std::function<void()>& callback, uint16_t mask) {
        Device::addOnChangeCallback(callback, getInputRegister(), mask);
    }

    static void clearOnChange(uint16_t mask) {
        Device::removeOnChangeCallback(getInputRegister(), mask);
    }
};

class Pin16 {
public:
    /// Sets Pin16 to HIGH.
    static void set()       { Port1::setBits(1<<0); }

    /// Sets Pin16 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port1::setBits(1<<0) : Port1::clearBits(1<<0); }

    /// Sets Pin16 to LOW.
    static void clear()     { Port1::clearBits(1<<0); }

    /// Toggles Pin16 value.
    static void toggle()    { Port1::toggleBits(1<<0); }

    /// Configures Pin16 as an output pin.
    static void setOutput() { Port1::setOutput(1<<0); }

    /// Configures Pin16 as an input pin.
    static void setInput()  { Port1::setInput(1<<0); }

    /// Pulses Pin16 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin16 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin16 value.
    /// @return true if Pin16 is high, false otherwise.
    static bool test()      { return Port1::test(0); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<0)
    static constexpr uint16_t bitmask() { return (1<<0); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 0
    static constexpr uint8_t bit()      { return 0; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port1;
};

class Pin17 {
public:
    /// Sets Pin17 to HIGH.
    static void set()       { Port1::setBits(1<<1); }

    /// Sets Pin17 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port1::setBits(1<<1) : Port1::clearBits(1<<1); }

    /// Sets Pin17 to LOW.
    static void clear()     { Port1::clearBits(1<<1); }

    /// Toggles Pin17 value.
    static void toggle()    { Port1::toggleBits(1<<1); }

    /// Configures Pin17 as an output pin.
    static void setOutput() { Port1::setOutput(1<<1); }

    /// Configures Pin17 as an input pin.
    static void setInput()  { Port1::setInput(1<<1); }

    /// Pulses Pin17 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin17 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin17 value.
    /// @return true if Pin17 is high, false otherwise.
    static bool test()      { return Port1::test(1); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<1)
    static constexpr uint16_t bitmask() { return (1<<1); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 1
    static constexpr uint8_t bit()      { return 1; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port1;
};

class Pin18 {
public:
    /// Sets Pin18 to HIGH.
    static void set()       { Port1::setBits(1<<2); }

    /// Sets Pin18 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port1::setBits(1<<2) : Port1::clearBits(1<<2); }

    /// Sets Pin18 to LOW.
    static void clear()     { Port1::clearBits(1<<2); }

    /// Toggles Pin18 value.
    static void toggle()    { Port1::toggleBits(1<<2); }

    /// Configures Pin18 as an output pin.
    static void setOutput() { Port1::setOutput(1<<2); }

    /// Configures Pin18 as an input pin.
    static void setInput()  { Port1::setInput(1<<2); }

    /// Pulses Pin18 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin18 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin18 value.
    /// @return true if Pin18 is high, false otherwise.
    static bool test()      { return Port1::test(2); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<2)
    static constexpr uint16_t bitmask() { return (1<<2); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 2
    static constexpr uint8_t bit()      { return 2; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port1;
};

class Pin19 {
public:
    /// Sets Pin19 to HIGH.
    static void set()       { Port1::setBits(1<<3); }

    /// Sets Pin19 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port1::setBits(1<<3) : Port1::clearBits(1<<3); }

    /// Sets Pin19 to LOW.
    static void clear()     { Port1::clearBits(1<<3); }

    /// Toggles Pin19 value.
    static void toggle()    { Port1::toggleBits(1<<3); }

    /// Configures Pin19 as an output pin.
    static void setOutput() { Port1::setOutput(1<<3); }

    /// Configures Pin19 as an input pin.
    static void setInput()  { Port1::setInput(1<<3); }

    /// Pulses Pin19 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin19 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin19 value.
    /// @return true if Pin19 is high, false otherwise.
    static bool test()      { return Port1::test(3); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<3)
    static constexpr uint16_t bitmask() { return (1<<3); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 3
    static constexpr uint8_t bit()      { return 3; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port1;
};

class Pin20 {
public:
    /// Sets Pin20 to HIGH.
    static void set()       { Port1::setBits(1<<4); }

    /// Sets Pin20 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? Port1::setBits(1<<4) : Port1::clearBits(1<<4); }

    /// Sets Pin20 to LOW.
    static void clear()     { Port1::clearBits(1<<4); }

    /// Toggles Pin20 value.
    static void toggle()    { Port1::toggleBits(1<<4); }

    /// Configures Pin20 as an output pin.
    static void setOutput() { Port1::setOutput(1<<4); }

    /// Configures Pin20 as an input pin.
    static void setInput()  { Port1::setInput(1<<4); }

    /// Pulses Pin20 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses Pin20 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads Pin20 value.
    /// @return true if Pin20 is high, false otherwise.
    static bool test()      { return Port1::test(4); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<4)
    static constexpr uint16_t bitmask() { return (1<<4); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 4
    static constexpr uint8_t bit()      { return 4; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = Port1;
};


class PortSimuA {
public:
    using PinChangeIRQ = PinChangeIRQSimuA;

    /// Assigns a value to PortSimuA
    /// @param[in] value value to affect to portSimuA
    static void assign(uint16_t value)     {
        if (Device::OUT_REG_CYCLES < (Device::OUTSET_REG_CYCLES + Device::OUTCLR_REG_CYCLES)) {
            GPSimuA_OUT = value;
        }
        else {
            GPSimuA_OUT_CLR = 0b1111111111111111;
            GPSimuA_OUT_SET = value;
        }
        Device::yield();
    }

    /// Sets masked bits in PORTSimuA
    /// @param[in] mask bits to set
    static void setBits(uint16_t mask)     { GPSimuA_OUT |= mask; Device::yield(); }

    /// Clears masked bits in PORTSimuA
    /// @param[in] mask bits to clear
    static void clearBits(uint16_t mask)   { GPSimuA_OUT &= ~mask; Device::yield(); } 

    /// Changes values of masked bits in PORTSimuA
    /// @param[in] mask bits to change
    /// @param[in] value new bits values
    static void changeBits(uint16_t mask, uint16_t value) { auto tmp = GPSimuA_OUT & ~mask; GPSimuA_OUT = tmp | value; Device::yield(); }

    /// Toggles masked bits in PORTSimuA
    /// @param[in] mask bits to toggle
    static void toggleBits(uint16_t mask)  { GPSimuA_OUT ^= mask; Device::yield(); } 

    /// Pulses masked bits in PORTSimuA with high state first.
    /// @param[in] mask bits to pulse
    static void pulseHigh(uint16_t mask)   { setBits(mask); clearBits(mask); Device::yield(); }

    /// Pulses masked bits in PORTSimuA with low state first.
    /// @param[in] mask bits to pulse
    static void pulseLow(uint16_t mask)    { clearBits(mask); setBits(mask); Device::yield(); }

    /// Set corresponding masked bits of PORTSimuA to output direction.
    /// @param[in] mask bits
    static void setOutput(uint16_t mask)   { GPSimuA_DIR |= mask; Device::yield(); }

    /// Set corresponding masked bits of PORTSimuA to input direction.
    /// @param[in] mask bits
    static void setInput(uint16_t mask)    { GPSimuA_DIR &= ~mask; Device::yield(); }

    /// Tests masked bits of PORTSimuA
    /// @param[in] mask bits
    /// @param[in] true if the corresponding bits are all set, false otherwise.
    static bool testBits(uint16_t mask)    { return (GPSimuA_IN & mask) == mask;}

    /// Returns the value of the bit at the position pos.
    /// @param[in] position of the bit to return
    /// @return true if the requested bit is set, false otherwise.
    static bool test(uint8_t pos)          { return (GPSimuA_IN & (1<<pos)) != 0; }

    /// Returns the native output register associated to PortSimuA
    static uint16_t& getOutputRegister()    { return GPSimuA_OUT; }

    /// Returns the native input register associated to PortSimuA
    static uint16_t& getInputRegister()     { return GPSimuA_IN; }

    /// Returns the native direction register associated to PortSimuA
    static uint16_t& getDirectionRegister() { return GPSimuA_DIR; }

    static void onChange(const std::function<void()>& callback, uint16_t mask) {
        Device::addOnChangeCallback(callback, getInputRegister(), mask);
    }

    static void clearOnChange(uint16_t mask) {
        Device::removeOnChangeCallback(getInputRegister(), mask);
    }
};

class PinS0 {
public:
    /// Sets PinS0 to HIGH.
    static void set()       { PortSimuA::setBits(1<<0); }

    /// Sets PinS0 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? PortSimuA::setBits(1<<0) : PortSimuA::clearBits(1<<0); }

    /// Sets PinS0 to LOW.
    static void clear()     { PortSimuA::clearBits(1<<0); }

    /// Toggles PinS0 value.
    static void toggle()    { PortSimuA::toggleBits(1<<0); }

    /// Configures PinS0 as an output pin.
    static void setOutput() { PortSimuA::setOutput(1<<0); }

    /// Configures PinS0 as an input pin.
    static void setInput()  { PortSimuA::setInput(1<<0); }

    /// Pulses PinS0 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses PinS0 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads PinS0 value.
    /// @return true if PinS0 is high, false otherwise.
    static bool test()      { return PortSimuA::test(0); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<0)
    static constexpr uint16_t bitmask() { return (1<<0); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 0
    static constexpr uint8_t bit()      { return 0; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortSimuA;
};

class PinS1 {
public:
    /// Sets PinS1 to HIGH.
    static void set()       { PortSimuA::setBits(1<<1); }

    /// Sets PinS1 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? PortSimuA::setBits(1<<1) : PortSimuA::clearBits(1<<1); }

    /// Sets PinS1 to LOW.
    static void clear()     { PortSimuA::clearBits(1<<1); }

    /// Toggles PinS1 value.
    static void toggle()    { PortSimuA::toggleBits(1<<1); }

    /// Configures PinS1 as an output pin.
    static void setOutput() { PortSimuA::setOutput(1<<1); }

    /// Configures PinS1 as an input pin.
    static void setInput()  { PortSimuA::setInput(1<<1); }

    /// Pulses PinS1 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses PinS1 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads PinS1 value.
    /// @return true if PinS1 is high, false otherwise.
    static bool test()      { return PortSimuA::test(1); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<1)
    static constexpr uint16_t bitmask() { return (1<<1); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 1
    static constexpr uint8_t bit()      { return 1; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortSimuA;
};

class PinS2 {
public:
    /// Sets PinS2 to HIGH.
    static void set()       { PortSimuA::setBits(1<<2); }

    /// Sets PinS2 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? PortSimuA::setBits(1<<2) : PortSimuA::clearBits(1<<2); }

    /// Sets PinS2 to LOW.
    static void clear()     { PortSimuA::clearBits(1<<2); }

    /// Toggles PinS2 value.
    static void toggle()    { PortSimuA::toggleBits(1<<2); }

    /// Configures PinS2 as an output pin.
    static void setOutput() { PortSimuA::setOutput(1<<2); }

    /// Configures PinS2 as an input pin.
    static void setInput()  { PortSimuA::setInput(1<<2); }

    /// Pulses PinS2 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses PinS2 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads PinS2 value.
    /// @return true if PinS2 is high, false otherwise.
    static bool test()      { return PortSimuA::test(2); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<2)
    static constexpr uint16_t bitmask() { return (1<<2); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 2
    static constexpr uint8_t bit()      { return 2; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortSimuA;
};

class PinS3 {
public:
    /// Sets PinS3 to HIGH.
    static void set()       { PortSimuA::setBits(1<<3); }

    /// Sets PinS3 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? PortSimuA::setBits(1<<3) : PortSimuA::clearBits(1<<3); }

    /// Sets PinS3 to LOW.
    static void clear()     { PortSimuA::clearBits(1<<3); }

    /// Toggles PinS3 value.
    static void toggle()    { PortSimuA::toggleBits(1<<3); }

    /// Configures PinS3 as an output pin.
    static void setOutput() { PortSimuA::setOutput(1<<3); }

    /// Configures PinS3 as an input pin.
    static void setInput()  { PortSimuA::setInput(1<<3); }

    /// Pulses PinS3 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses PinS3 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads PinS3 value.
    /// @return true if PinS3 is high, false otherwise.
    static bool test()      { return PortSimuA::test(3); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<3)
    static constexpr uint16_t bitmask() { return (1<<3); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 3
    static constexpr uint8_t bit()      { return 3; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortSimuA;
};

class PinS4 {
public:
    /// Sets PinS4 to HIGH.
    static void set()       { PortSimuA::setBits(1<<4); }

    /// Sets PinS4 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? PortSimuA::setBits(1<<4) : PortSimuA::clearBits(1<<4); }

    /// Sets PinS4 to LOW.
    static void clear()     { PortSimuA::clearBits(1<<4); }

    /// Toggles PinS4 value.
    static void toggle()    { PortSimuA::toggleBits(1<<4); }

    /// Configures PinS4 as an output pin.
    static void setOutput() { PortSimuA::setOutput(1<<4); }

    /// Configures PinS4 as an input pin.
    static void setInput()  { PortSimuA::setInput(1<<4); }

    /// Pulses PinS4 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses PinS4 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads PinS4 value.
    /// @return true if PinS4 is high, false otherwise.
    static bool test()      { return PortSimuA::test(4); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<4)
    static constexpr uint16_t bitmask() { return (1<<4); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 4
    static constexpr uint8_t bit()      { return 4; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortSimuA;
};

class PinS5 {
public:
    /// Sets PinS5 to HIGH.
    static void set()       { PortSimuA::setBits(1<<5); }

    /// Sets PinS5 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? PortSimuA::setBits(1<<5) : PortSimuA::clearBits(1<<5); }

    /// Sets PinS5 to LOW.
    static void clear()     { PortSimuA::clearBits(1<<5); }

    /// Toggles PinS5 value.
    static void toggle()    { PortSimuA::toggleBits(1<<5); }

    /// Configures PinS5 as an output pin.
    static void setOutput() { PortSimuA::setOutput(1<<5); }

    /// Configures PinS5 as an input pin.
    static void setInput()  { PortSimuA::setInput(1<<5); }

    /// Pulses PinS5 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses PinS5 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads PinS5 value.
    /// @return true if PinS5 is high, false otherwise.
    static bool test()      { return PortSimuA::test(5); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<5)
    static constexpr uint16_t bitmask() { return (1<<5); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 5
    static constexpr uint8_t bit()      { return 5; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortSimuA;
};

class PinS6 {
public:
    /// Sets PinS6 to HIGH.
    static void set()       { PortSimuA::setBits(1<<6); }

    /// Sets PinS6 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? PortSimuA::setBits(1<<6) : PortSimuA::clearBits(1<<6); }

    /// Sets PinS6 to LOW.
    static void clear()     { PortSimuA::clearBits(1<<6); }

    /// Toggles PinS6 value.
    static void toggle()    { PortSimuA::toggleBits(1<<6); }

    /// Configures PinS6 as an output pin.
    static void setOutput() { PortSimuA::setOutput(1<<6); }

    /// Configures PinS6 as an input pin.
    static void setInput()  { PortSimuA::setInput(1<<6); }

    /// Pulses PinS6 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses PinS6 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads PinS6 value.
    /// @return true if PinS6 is high, false otherwise.
    static bool test()      { return PortSimuA::test(6); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<6)
    static constexpr uint16_t bitmask() { return (1<<6); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 6
    static constexpr uint8_t bit()      { return 6; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortSimuA;
};

class PinS7 {
public:
    /// Sets PinS7 to HIGH.
    static void set()       { PortSimuA::setBits(1<<7); }

    /// Sets PinS7 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? PortSimuA::setBits(1<<7) : PortSimuA::clearBits(1<<7); }

    /// Sets PinS7 to LOW.
    static void clear()     { PortSimuA::clearBits(1<<7); }

    /// Toggles PinS7 value.
    static void toggle()    { PortSimuA::toggleBits(1<<7); }

    /// Configures PinS7 as an output pin.
    static void setOutput() { PortSimuA::setOutput(1<<7); }

    /// Configures PinS7 as an input pin.
    static void setInput()  { PortSimuA::setInput(1<<7); }

    /// Pulses PinS7 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses PinS7 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads PinS7 value.
    /// @return true if PinS7 is high, false otherwise.
    static bool test()      { return PortSimuA::test(7); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<7)
    static constexpr uint16_t bitmask() { return (1<<7); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 7
    static constexpr uint8_t bit()      { return 7; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortSimuA;
};


class PortSimuB {
public:
    using PinChangeIRQ = PinChangeIRQSimuB;

    /// Assigns a value to PortSimuB
    /// @param[in] value value to affect to portSimuB
    static void assign(uint16_t value)     {
        if (Device::OUT_REG_CYCLES < (Device::OUTSET_REG_CYCLES + Device::OUTCLR_REG_CYCLES)) {
            GPSimuB_OUT = value;
        }
        else {
            GPSimuB_OUT_CLR = 0b1111111111111111;
            GPSimuB_OUT_SET = value;
        }
        Device::yield();
    }

    /// Sets masked bits in PORTSimuB
    /// @param[in] mask bits to set
    static void setBits(uint16_t mask)     { GPSimuB_OUT |= mask; Device::yield(); }

    /// Clears masked bits in PORTSimuB
    /// @param[in] mask bits to clear
    static void clearBits(uint16_t mask)   { GPSimuB_OUT &= ~mask; Device::yield(); } 

    /// Changes values of masked bits in PORTSimuB
    /// @param[in] mask bits to change
    /// @param[in] value new bits values
    static void changeBits(uint16_t mask, uint16_t value) { auto tmp = GPSimuB_OUT & ~mask; GPSimuB_OUT = tmp | value; Device::yield(); }

    /// Toggles masked bits in PORTSimuB
    /// @param[in] mask bits to toggle
    static void toggleBits(uint16_t mask)  { GPSimuB_OUT ^= mask; Device::yield(); } 

    /// Pulses masked bits in PORTSimuB with high state first.
    /// @param[in] mask bits to pulse
    static void pulseHigh(uint16_t mask)   { setBits(mask); clearBits(mask); Device::yield(); }

    /// Pulses masked bits in PORTSimuB with low state first.
    /// @param[in] mask bits to pulse
    static void pulseLow(uint16_t mask)    { clearBits(mask); setBits(mask); Device::yield(); }

    /// Set corresponding masked bits of PORTSimuB to output direction.
    /// @param[in] mask bits
    static void setOutput(uint16_t mask)   { GPSimuB_DIR |= mask; Device::yield(); }

    /// Set corresponding masked bits of PORTSimuB to input direction.
    /// @param[in] mask bits
    static void setInput(uint16_t mask)    { GPSimuB_DIR &= ~mask; Device::yield(); }

    /// Tests masked bits of PORTSimuB
    /// @param[in] mask bits
    /// @param[in] true if the corresponding bits are all set, false otherwise.
    static bool testBits(uint16_t mask)    { return (GPSimuB_IN & mask) == mask;}

    /// Returns the value of the bit at the position pos.
    /// @param[in] position of the bit to return
    /// @return true if the requested bit is set, false otherwise.
    static bool test(uint8_t pos)          { return (GPSimuB_IN & (1<<pos)) != 0; }

    /// Returns the native output register associated to PortSimuB
    static uint16_t& getOutputRegister()    { return GPSimuB_OUT; }

    /// Returns the native input register associated to PortSimuB
    static uint16_t& getInputRegister()     { return GPSimuB_IN; }

    /// Returns the native direction register associated to PortSimuB
    static uint16_t& getDirectionRegister() { return GPSimuB_DIR; }

    static void onChange(const std::function<void()>& callback, uint16_t mask) {
        Device::addOnChangeCallback(callback, getInputRegister(), mask);
    }

    static void clearOnChange(uint16_t mask) {
        Device::removeOnChangeCallback(getInputRegister(), mask);
    }
};

class PinS8 {
public:
    /// Sets PinS8 to HIGH.
    static void set()       { PortSimuB::setBits(1<<0); }

    /// Sets PinS8 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? PortSimuB::setBits(1<<0) : PortSimuB::clearBits(1<<0); }

    /// Sets PinS8 to LOW.
    static void clear()     { PortSimuB::clearBits(1<<0); }

    /// Toggles PinS8 value.
    static void toggle()    { PortSimuB::toggleBits(1<<0); }

    /// Configures PinS8 as an output pin.
    static void setOutput() { PortSimuB::setOutput(1<<0); }

    /// Configures PinS8 as an input pin.
    static void setInput()  { PortSimuB::setInput(1<<0); }

    /// Pulses PinS8 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses PinS8 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads PinS8 value.
    /// @return true if PinS8 is high, false otherwise.
    static bool test()      { return PortSimuB::test(0); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<0)
    static constexpr uint16_t bitmask() { return (1<<0); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 0
    static constexpr uint8_t bit()      { return 0; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortSimuB;
};

class PinS9 {
public:
    /// Sets PinS9 to HIGH.
    static void set()       { PortSimuB::setBits(1<<1); }

    /// Sets PinS9 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? PortSimuB::setBits(1<<1) : PortSimuB::clearBits(1<<1); }

    /// Sets PinS9 to LOW.
    static void clear()     { PortSimuB::clearBits(1<<1); }

    /// Toggles PinS9 value.
    static void toggle()    { PortSimuB::toggleBits(1<<1); }

    /// Configures PinS9 as an output pin.
    static void setOutput() { PortSimuB::setOutput(1<<1); }

    /// Configures PinS9 as an input pin.
    static void setInput()  { PortSimuB::setInput(1<<1); }

    /// Pulses PinS9 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses PinS9 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads PinS9 value.
    /// @return true if PinS9 is high, false otherwise.
    static bool test()      { return PortSimuB::test(1); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<1)
    static constexpr uint16_t bitmask() { return (1<<1); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 1
    static constexpr uint8_t bit()      { return 1; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortSimuB;
};

class PinS10 {
public:
    /// Sets PinS10 to HIGH.
    static void set()       { PortSimuB::setBits(1<<2); }

    /// Sets PinS10 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? PortSimuB::setBits(1<<2) : PortSimuB::clearBits(1<<2); }

    /// Sets PinS10 to LOW.
    static void clear()     { PortSimuB::clearBits(1<<2); }

    /// Toggles PinS10 value.
    static void toggle()    { PortSimuB::toggleBits(1<<2); }

    /// Configures PinS10 as an output pin.
    static void setOutput() { PortSimuB::setOutput(1<<2); }

    /// Configures PinS10 as an input pin.
    static void setInput()  { PortSimuB::setInput(1<<2); }

    /// Pulses PinS10 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses PinS10 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads PinS10 value.
    /// @return true if PinS10 is high, false otherwise.
    static bool test()      { return PortSimuB::test(2); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<2)
    static constexpr uint16_t bitmask() { return (1<<2); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 2
    static constexpr uint8_t bit()      { return 2; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortSimuB;
};

class PinS11 {
public:
    /// Sets PinS11 to HIGH.
    static void set()       { PortSimuB::setBits(1<<3); }

    /// Sets PinS11 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? PortSimuB::setBits(1<<3) : PortSimuB::clearBits(1<<3); }

    /// Sets PinS11 to LOW.
    static void clear()     { PortSimuB::clearBits(1<<3); }

    /// Toggles PinS11 value.
    static void toggle()    { PortSimuB::toggleBits(1<<3); }

    /// Configures PinS11 as an output pin.
    static void setOutput() { PortSimuB::setOutput(1<<3); }

    /// Configures PinS11 as an input pin.
    static void setInput()  { PortSimuB::setInput(1<<3); }

    /// Pulses PinS11 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses PinS11 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads PinS11 value.
    /// @return true if PinS11 is high, false otherwise.
    static bool test()      { return PortSimuB::test(3); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<3)
    static constexpr uint16_t bitmask() { return (1<<3); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 3
    static constexpr uint8_t bit()      { return 3; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortSimuB;
};

class PinS12 {
public:
    /// Sets PinS12 to HIGH.
    static void set()       { PortSimuB::setBits(1<<4); }

    /// Sets PinS12 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? PortSimuB::setBits(1<<4) : PortSimuB::clearBits(1<<4); }

    /// Sets PinS12 to LOW.
    static void clear()     { PortSimuB::clearBits(1<<4); }

    /// Toggles PinS12 value.
    static void toggle()    { PortSimuB::toggleBits(1<<4); }

    /// Configures PinS12 as an output pin.
    static void setOutput() { PortSimuB::setOutput(1<<4); }

    /// Configures PinS12 as an input pin.
    static void setInput()  { PortSimuB::setInput(1<<4); }

    /// Pulses PinS12 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses PinS12 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads PinS12 value.
    /// @return true if PinS12 is high, false otherwise.
    static bool test()      { return PortSimuB::test(4); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<4)
    static constexpr uint16_t bitmask() { return (1<<4); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 4
    static constexpr uint8_t bit()      { return 4; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortSimuB;
};

class PinS13 {
public:
    /// Sets PinS13 to HIGH.
    static void set()       { PortSimuB::setBits(1<<5); }

    /// Sets PinS13 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? PortSimuB::setBits(1<<5) : PortSimuB::clearBits(1<<5); }

    /// Sets PinS13 to LOW.
    static void clear()     { PortSimuB::clearBits(1<<5); }

    /// Toggles PinS13 value.
    static void toggle()    { PortSimuB::toggleBits(1<<5); }

    /// Configures PinS13 as an output pin.
    static void setOutput() { PortSimuB::setOutput(1<<5); }

    /// Configures PinS13 as an input pin.
    static void setInput()  { PortSimuB::setInput(1<<5); }

    /// Pulses PinS13 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses PinS13 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads PinS13 value.
    /// @return true if PinS13 is high, false otherwise.
    static bool test()      { return PortSimuB::test(5); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<5)
    static constexpr uint16_t bitmask() { return (1<<5); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 5
    static constexpr uint8_t bit()      { return 5; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortSimuB;
};

class PinS14 {
public:
    /// Sets PinS14 to HIGH.
    static void set()       { PortSimuB::setBits(1<<6); }

    /// Sets PinS14 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? PortSimuB::setBits(1<<6) : PortSimuB::clearBits(1<<6); }

    /// Sets PinS14 to LOW.
    static void clear()     { PortSimuB::clearBits(1<<6); }

    /// Toggles PinS14 value.
    static void toggle()    { PortSimuB::toggleBits(1<<6); }

    /// Configures PinS14 as an output pin.
    static void setOutput() { PortSimuB::setOutput(1<<6); }

    /// Configures PinS14 as an input pin.
    static void setInput()  { PortSimuB::setInput(1<<6); }

    /// Pulses PinS14 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses PinS14 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads PinS14 value.
    /// @return true if PinS14 is high, false otherwise.
    static bool test()      { return PortSimuB::test(6); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<6)
    static constexpr uint16_t bitmask() { return (1<<6); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 6
    static constexpr uint8_t bit()      { return 6; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortSimuB;
};

class PinS15 {
public:
    /// Sets PinS15 to HIGH.
    static void set()       { PortSimuB::setBits(1<<7); }

    /// Sets PinS15 to asked status.
    /// @param setHigh set pin to HIGH if true, LOW otherwise
    static void set(bool setHigh) { setHigh ? PortSimuB::setBits(1<<7) : PortSimuB::clearBits(1<<7); }

    /// Sets PinS15 to LOW.
    static void clear()     { PortSimuB::clearBits(1<<7); }

    /// Toggles PinS15 value.
    static void toggle()    { PortSimuB::toggleBits(1<<7); }

    /// Configures PinS15 as an output pin.
    static void setOutput() { PortSimuB::setOutput(1<<7); }

    /// Configures PinS15 as an input pin.
    static void setInput()  { PortSimuB::setInput(1<<7); }

    /// Pulses PinS15 with high state first.
    static void pulseHigh() { set(); clear(); }

    /// Pulses PinS15 with low state first.
    static void pulseLow()  { clear(); set(); }

    /// Reads PinS15 value.
    /// @return true if PinS15 is high, false otherwise.
    static bool test()      { return PortSimuB::test(7); }

    /// Returns the bitmask corresponding to this pin in the associated Port.
    /// @return (1<<7)
    static constexpr uint16_t bitmask() { return (1<<7); }

    /// Returns the bit number corresponding to this pin in the associated Port.
    /// @return 7
    static constexpr uint8_t bit()      { return 7; }

    static void onChange(const std::function<void()> &callback) { Port::onChange(callback, bitmask()); }

    static void clearOnChange() { Port::clearOnChange(bitmask()); }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortSimuB;
};


} // namespace etl
