/// @file ioports_Mock.h
/// @date 4/14/16 4:15 PM
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

namespace etl {
class Device {
public:
    static void Initialize() { MockDevice::Instance().Configure(2); }
    template<typename T> static void Pragma(T pragma) { MockDevice::Instance().Pragma(pragma); }
};

struct PinChangeIRQ0;
struct PinChangeIRQ1;

class Port0 {
public:
  using PinChangeIRQ = PinChangeIRQ0;

  /// Assigns a value to PORT0.
  /// @param[in] value value affected to PORT0
  static void Assign(uint32_t value)    { MockDevice::Instance().WritePort(0, value); }

  /// Sets masked bits in PORT0.
  /// @param[in] mask bits to set
  static void SetBits(uint32_t mask)    { MockDevice::Instance().WritePort(0, MockDevice::Instance().ReadPort(0) | mask); }

  /// Clears masked bits in PORT0.
  /// @param[in] mask bits to clear
  static void ClearBits(uint32_t mask)  { MockDevice::Instance().WritePort(0, MockDevice::Instance().ReadPort(0) & ~mask); } 

  /// Changes values of masked bits in PORT0.
  /// @param[in] mask bits to change
  /// @param[in] value new bits values
  static void ChangeBits(uint32_t mask, uint32_t value) { MockDevice::Instance().WritePort(0, (MockDevice::Instance().ReadPort(0) & ~mask) | value); } 

  /// Toggles masked bits in PORT0.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint32_t mask) { MockDevice::Instance().WritePort(0, MockDevice::Instance().ReadPort(0) ^ mask); } 

  /// Pulses masked bits in PORT0 with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint32_t mask)  { SetBits(mask); ClearBits(mask); }

  /// Pulses masked bits in PORT0 with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint32_t mask)   { ClearBits(mask); SetBits(mask); }

  /// Set corresponding masked bits of PORT0 to output direction.
  /// @param[in] mask bits
  static void SetOutput(uint32_t mask)  { MockDevice::Instance().WriteDirection(0, MockDevice::Instance().ReadDirection(0) | mask); }

  /// Set corresponding masked bits of PORT0 to input direction.
  /// @param[in] mask bits
  static void SetInput(uint32_t mask)   { MockDevice::Instance().WriteDirection(0, MockDevice::Instance().ReadDirection(0) & ~mask); }

  /// Tests masked bits of PORT0
  /// @param[in] mask bits
  /// @param[in] true if the corresponding bits are all set, false otherwise.
  static bool TestBits(uint32_t mask)   { return (MockDevice::Instance().ReadPort(0) & mask) == mask; }

  /// Returns the value of the bit at the position pos.
  /// @param[in] position of the bit to return
  /// @return true if the requested bit is set, false otherwise.
  static bool Test(uint8_t pos)        { return (MockDevice::Instance().ReadPort(0) & (1<<pos)) != 0; }

};

class Pin0 : public Pin<Port0> {
public:
  /// Sets Pin0 to HIGH.
  static void Set()       { Port0::SetBits(1<<0); }

  /// Sets Pin0 to LOW.
  static void Clear()     { Port0::ClearBits(1<<0); }

  /// Toggles Pin0 value.
  static void Toggle()    { Port0::ToggleBits(1<<0); }

  /// Configures Pin0 as an output pin.
  static void SetOutput() { Port0::SetOutput(1<<0); }

  /// Configures Pin0 as an input pin.
  static void SetInput()  { Port0::SetInput(1<<0); }

  /// Pulses Pin0 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin0 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin0  value.
  /// @return Port pin value.
  static bool Test()      { return Port0::Test(0); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<0)
  static constexpr uint32_t bitmask()               { return (1<<0); }

  /// Returns the bit corresponding to this pin.
  /// @return 0
  static constexpr uint8_t bit()                   { return 0; }
};

class Pin1 : public Pin<Port0> {
public:
  /// Sets Pin1 to HIGH.
  static void Set()       { Port0::SetBits(1<<1); }

  /// Sets Pin1 to LOW.
  static void Clear()     { Port0::ClearBits(1<<1); }

  /// Toggles Pin1 value.
  static void Toggle()    { Port0::ToggleBits(1<<1); }

  /// Configures Pin1 as an output pin.
  static void SetOutput() { Port0::SetOutput(1<<1); }

  /// Configures Pin1 as an input pin.
  static void SetInput()  { Port0::SetInput(1<<1); }

  /// Pulses Pin1 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin1 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin1  value.
  /// @return Port pin value.
  static bool Test()      { return Port0::Test(1); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<1)
  static constexpr uint32_t bitmask()               { return (1<<1); }

  /// Returns the bit corresponding to this pin.
  /// @return 1
  static constexpr uint8_t bit()                   { return 1; }
};

class Pin2 : public Pin<Port0> {
public:
  /// Sets Pin2 to HIGH.
  static void Set()       { Port0::SetBits(1<<2); }

  /// Sets Pin2 to LOW.
  static void Clear()     { Port0::ClearBits(1<<2); }

  /// Toggles Pin2 value.
  static void Toggle()    { Port0::ToggleBits(1<<2); }

  /// Configures Pin2 as an output pin.
  static void SetOutput() { Port0::SetOutput(1<<2); }

  /// Configures Pin2 as an input pin.
  static void SetInput()  { Port0::SetInput(1<<2); }

  /// Pulses Pin2 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin2 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin2  value.
  /// @return Port pin value.
  static bool Test()      { return Port0::Test(2); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<2)
  static constexpr uint32_t bitmask()               { return (1<<2); }

  /// Returns the bit corresponding to this pin.
  /// @return 2
  static constexpr uint8_t bit()                   { return 2; }
};

class Pin3 : public Pin<Port0> {
public:
  /// Sets Pin3 to HIGH.
  static void Set()       { Port0::SetBits(1<<3); }

  /// Sets Pin3 to LOW.
  static void Clear()     { Port0::ClearBits(1<<3); }

  /// Toggles Pin3 value.
  static void Toggle()    { Port0::ToggleBits(1<<3); }

  /// Configures Pin3 as an output pin.
  static void SetOutput() { Port0::SetOutput(1<<3); }

  /// Configures Pin3 as an input pin.
  static void SetInput()  { Port0::SetInput(1<<3); }

  /// Pulses Pin3 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin3 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin3  value.
  /// @return Port pin value.
  static bool Test()      { return Port0::Test(3); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<3)
  static constexpr uint32_t bitmask()               { return (1<<3); }

  /// Returns the bit corresponding to this pin.
  /// @return 3
  static constexpr uint8_t bit()                   { return 3; }
};

class Pin4 : public Pin<Port0> {
public:
  /// Sets Pin4 to HIGH.
  static void Set()       { Port0::SetBits(1<<4); }

  /// Sets Pin4 to LOW.
  static void Clear()     { Port0::ClearBits(1<<4); }

  /// Toggles Pin4 value.
  static void Toggle()    { Port0::ToggleBits(1<<4); }

  /// Configures Pin4 as an output pin.
  static void SetOutput() { Port0::SetOutput(1<<4); }

  /// Configures Pin4 as an input pin.
  static void SetInput()  { Port0::SetInput(1<<4); }

  /// Pulses Pin4 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin4 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin4  value.
  /// @return Port pin value.
  static bool Test()      { return Port0::Test(4); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<4)
  static constexpr uint32_t bitmask()               { return (1<<4); }

  /// Returns the bit corresponding to this pin.
  /// @return 4
  static constexpr uint8_t bit()                   { return 4; }
};

class Pin5 : public Pin<Port0> {
public:
  /// Sets Pin5 to HIGH.
  static void Set()       { Port0::SetBits(1<<5); }

  /// Sets Pin5 to LOW.
  static void Clear()     { Port0::ClearBits(1<<5); }

  /// Toggles Pin5 value.
  static void Toggle()    { Port0::ToggleBits(1<<5); }

  /// Configures Pin5 as an output pin.
  static void SetOutput() { Port0::SetOutput(1<<5); }

  /// Configures Pin5 as an input pin.
  static void SetInput()  { Port0::SetInput(1<<5); }

  /// Pulses Pin5 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin5 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin5  value.
  /// @return Port pin value.
  static bool Test()      { return Port0::Test(5); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<5)
  static constexpr uint32_t bitmask()               { return (1<<5); }

  /// Returns the bit corresponding to this pin.
  /// @return 5
  static constexpr uint8_t bit()                   { return 5; }
};

class Pin6 : public Pin<Port0> {
public:
  /// Sets Pin6 to HIGH.
  static void Set()       { Port0::SetBits(1<<6); }

  /// Sets Pin6 to LOW.
  static void Clear()     { Port0::ClearBits(1<<6); }

  /// Toggles Pin6 value.
  static void Toggle()    { Port0::ToggleBits(1<<6); }

  /// Configures Pin6 as an output pin.
  static void SetOutput() { Port0::SetOutput(1<<6); }

  /// Configures Pin6 as an input pin.
  static void SetInput()  { Port0::SetInput(1<<6); }

  /// Pulses Pin6 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin6 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin6  value.
  /// @return Port pin value.
  static bool Test()      { return Port0::Test(6); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<6)
  static constexpr uint32_t bitmask()               { return (1<<6); }

  /// Returns the bit corresponding to this pin.
  /// @return 6
  static constexpr uint8_t bit()                   { return 6; }
};

class Pin7 : public Pin<Port0> {
public:
  /// Sets Pin7 to HIGH.
  static void Set()       { Port0::SetBits(1<<7); }

  /// Sets Pin7 to LOW.
  static void Clear()     { Port0::ClearBits(1<<7); }

  /// Toggles Pin7 value.
  static void Toggle()    { Port0::ToggleBits(1<<7); }

  /// Configures Pin7 as an output pin.
  static void SetOutput() { Port0::SetOutput(1<<7); }

  /// Configures Pin7 as an input pin.
  static void SetInput()  { Port0::SetInput(1<<7); }

  /// Pulses Pin7 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin7 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin7  value.
  /// @return Port pin value.
  static bool Test()      { return Port0::Test(7); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<7)
  static constexpr uint32_t bitmask()               { return (1<<7); }

  /// Returns the bit corresponding to this pin.
  /// @return 7
  static constexpr uint8_t bit()                   { return 7; }
};

class Pin8 : public Pin<Port0> {
public:
  /// Sets Pin8 to HIGH.
  static void Set()       { Port0::SetBits(1<<8); }

  /// Sets Pin8 to LOW.
  static void Clear()     { Port0::ClearBits(1<<8); }

  /// Toggles Pin8 value.
  static void Toggle()    { Port0::ToggleBits(1<<8); }

  /// Configures Pin8 as an output pin.
  static void SetOutput() { Port0::SetOutput(1<<8); }

  /// Configures Pin8 as an input pin.
  static void SetInput()  { Port0::SetInput(1<<8); }

  /// Pulses Pin8 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin8 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin8  value.
  /// @return Port pin value.
  static bool Test()      { return Port0::Test(8); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<8)
  static constexpr uint32_t bitmask()               { return (1<<8); }

  /// Returns the bit corresponding to this pin.
  /// @return 8
  static constexpr uint8_t bit()                   { return 8; }
};

class Pin9 : public Pin<Port0> {
public:
  /// Sets Pin9 to HIGH.
  static void Set()       { Port0::SetBits(1<<9); }

  /// Sets Pin9 to LOW.
  static void Clear()     { Port0::ClearBits(1<<9); }

  /// Toggles Pin9 value.
  static void Toggle()    { Port0::ToggleBits(1<<9); }

  /// Configures Pin9 as an output pin.
  static void SetOutput() { Port0::SetOutput(1<<9); }

  /// Configures Pin9 as an input pin.
  static void SetInput()  { Port0::SetInput(1<<9); }

  /// Pulses Pin9 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin9 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin9  value.
  /// @return Port pin value.
  static bool Test()      { return Port0::Test(9); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<9)
  static constexpr uint32_t bitmask()               { return (1<<9); }

  /// Returns the bit corresponding to this pin.
  /// @return 9
  static constexpr uint8_t bit()                   { return 9; }
};

class Pin10 : public Pin<Port0> {
public:
  /// Sets Pin10 to HIGH.
  static void Set()       { Port0::SetBits(1<<10); }

  /// Sets Pin10 to LOW.
  static void Clear()     { Port0::ClearBits(1<<10); }

  /// Toggles Pin10 value.
  static void Toggle()    { Port0::ToggleBits(1<<10); }

  /// Configures Pin10 as an output pin.
  static void SetOutput() { Port0::SetOutput(1<<10); }

  /// Configures Pin10 as an input pin.
  static void SetInput()  { Port0::SetInput(1<<10); }

  /// Pulses Pin10 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin10 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin10  value.
  /// @return Port pin value.
  static bool Test()      { return Port0::Test(10); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<10)
  static constexpr uint32_t bitmask()               { return (1<<10); }

  /// Returns the bit corresponding to this pin.
  /// @return 10
  static constexpr uint8_t bit()                   { return 10; }
};

class Pin11 : public Pin<Port0> {
public:
  /// Sets Pin11 to HIGH.
  static void Set()       { Port0::SetBits(1<<11); }

  /// Sets Pin11 to LOW.
  static void Clear()     { Port0::ClearBits(1<<11); }

  /// Toggles Pin11 value.
  static void Toggle()    { Port0::ToggleBits(1<<11); }

  /// Configures Pin11 as an output pin.
  static void SetOutput() { Port0::SetOutput(1<<11); }

  /// Configures Pin11 as an input pin.
  static void SetInput()  { Port0::SetInput(1<<11); }

  /// Pulses Pin11 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin11 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin11  value.
  /// @return Port pin value.
  static bool Test()      { return Port0::Test(11); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<11)
  static constexpr uint32_t bitmask()               { return (1<<11); }

  /// Returns the bit corresponding to this pin.
  /// @return 11
  static constexpr uint8_t bit()                   { return 11; }
};

class Pin12 : public Pin<Port0> {
public:
  /// Sets Pin12 to HIGH.
  static void Set()       { Port0::SetBits(1<<12); }

  /// Sets Pin12 to LOW.
  static void Clear()     { Port0::ClearBits(1<<12); }

  /// Toggles Pin12 value.
  static void Toggle()    { Port0::ToggleBits(1<<12); }

  /// Configures Pin12 as an output pin.
  static void SetOutput() { Port0::SetOutput(1<<12); }

  /// Configures Pin12 as an input pin.
  static void SetInput()  { Port0::SetInput(1<<12); }

  /// Pulses Pin12 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin12 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin12  value.
  /// @return Port pin value.
  static bool Test()      { return Port0::Test(12); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<12)
  static constexpr uint32_t bitmask()               { return (1<<12); }

  /// Returns the bit corresponding to this pin.
  /// @return 12
  static constexpr uint8_t bit()                   { return 12; }
};

class Pin13 : public Pin<Port0> {
public:
  /// Sets Pin13 to HIGH.
  static void Set()       { Port0::SetBits(1<<13); }

  /// Sets Pin13 to LOW.
  static void Clear()     { Port0::ClearBits(1<<13); }

  /// Toggles Pin13 value.
  static void Toggle()    { Port0::ToggleBits(1<<13); }

  /// Configures Pin13 as an output pin.
  static void SetOutput() { Port0::SetOutput(1<<13); }

  /// Configures Pin13 as an input pin.
  static void SetInput()  { Port0::SetInput(1<<13); }

  /// Pulses Pin13 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin13 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin13  value.
  /// @return Port pin value.
  static bool Test()      { return Port0::Test(13); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<13)
  static constexpr uint32_t bitmask()               { return (1<<13); }

  /// Returns the bit corresponding to this pin.
  /// @return 13
  static constexpr uint8_t bit()                   { return 13; }
};

class Pin14 : public Pin<Port0> {
public:
  /// Sets Pin14 to HIGH.
  static void Set()       { Port0::SetBits(1<<14); }

  /// Sets Pin14 to LOW.
  static void Clear()     { Port0::ClearBits(1<<14); }

  /// Toggles Pin14 value.
  static void Toggle()    { Port0::ToggleBits(1<<14); }

  /// Configures Pin14 as an output pin.
  static void SetOutput() { Port0::SetOutput(1<<14); }

  /// Configures Pin14 as an input pin.
  static void SetInput()  { Port0::SetInput(1<<14); }

  /// Pulses Pin14 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin14 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin14  value.
  /// @return Port pin value.
  static bool Test()      { return Port0::Test(14); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<14)
  static constexpr uint32_t bitmask()               { return (1<<14); }

  /// Returns the bit corresponding to this pin.
  /// @return 14
  static constexpr uint8_t bit()                   { return 14; }
};

class Pin15 : public Pin<Port0> {
public:
  /// Sets Pin15 to HIGH.
  static void Set()       { Port0::SetBits(1<<15); }

  /// Sets Pin15 to LOW.
  static void Clear()     { Port0::ClearBits(1<<15); }

  /// Toggles Pin15 value.
  static void Toggle()    { Port0::ToggleBits(1<<15); }

  /// Configures Pin15 as an output pin.
  static void SetOutput() { Port0::SetOutput(1<<15); }

  /// Configures Pin15 as an input pin.
  static void SetInput()  { Port0::SetInput(1<<15); }

  /// Pulses Pin15 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin15 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin15  value.
  /// @return Port pin value.
  static bool Test()      { return Port0::Test(15); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<15)
  static constexpr uint32_t bitmask()               { return (1<<15); }

  /// Returns the bit corresponding to this pin.
  /// @return 15
  static constexpr uint8_t bit()                   { return 15; }
};


class Port1 {
public:
  using PinChangeIRQ = PinChangeIRQ1;

  /// Assigns a value to PORT1.
  /// @param[in] value value affected to PORT1
  static void Assign(uint32_t value)    { MockDevice::Instance().WritePort(1, value); }

  /// Sets masked bits in PORT1.
  /// @param[in] mask bits to set
  static void SetBits(uint32_t mask)    { MockDevice::Instance().WritePort(1, MockDevice::Instance().ReadPort(1) | mask); }

  /// Clears masked bits in PORT1.
  /// @param[in] mask bits to clear
  static void ClearBits(uint32_t mask)  { MockDevice::Instance().WritePort(1, MockDevice::Instance().ReadPort(1) & ~mask); } 

  /// Changes values of masked bits in PORT1.
  /// @param[in] mask bits to change
  /// @param[in] value new bits values
  static void ChangeBits(uint32_t mask, uint32_t value) { MockDevice::Instance().WritePort(1, (MockDevice::Instance().ReadPort(1) & ~mask) | value); } 

  /// Toggles masked bits in PORT1.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint32_t mask) { MockDevice::Instance().WritePort(1, MockDevice::Instance().ReadPort(1) ^ mask); } 

  /// Pulses masked bits in PORT1 with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint32_t mask)  { SetBits(mask); ClearBits(mask); }

  /// Pulses masked bits in PORT1 with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint32_t mask)   { ClearBits(mask); SetBits(mask); }

  /// Set corresponding masked bits of PORT1 to output direction.
  /// @param[in] mask bits
  static void SetOutput(uint32_t mask)  { MockDevice::Instance().WriteDirection(1, MockDevice::Instance().ReadDirection(1) | mask); }

  /// Set corresponding masked bits of PORT1 to input direction.
  /// @param[in] mask bits
  static void SetInput(uint32_t mask)   { MockDevice::Instance().WriteDirection(1, MockDevice::Instance().ReadDirection(1) & ~mask); }

  /// Tests masked bits of PORT1
  /// @param[in] mask bits
  /// @param[in] true if the corresponding bits are all set, false otherwise.
  static bool TestBits(uint32_t mask)   { return (MockDevice::Instance().ReadPort(1) & mask) == mask; }

  /// Returns the value of the bit at the position pos.
  /// @param[in] position of the bit to return
  /// @return true if the requested bit is set, false otherwise.
  static bool Test(uint8_t pos)        { return (MockDevice::Instance().ReadPort(1) & (1<<pos)) != 0; }

};

class Pin16 : public Pin<Port1> {
public:
  /// Sets Pin16 to HIGH.
  static void Set()       { Port1::SetBits(1<<0); }

  /// Sets Pin16 to LOW.
  static void Clear()     { Port1::ClearBits(1<<0); }

  /// Toggles Pin16 value.
  static void Toggle()    { Port1::ToggleBits(1<<0); }

  /// Configures Pin16 as an output pin.
  static void SetOutput() { Port1::SetOutput(1<<0); }

  /// Configures Pin16 as an input pin.
  static void SetInput()  { Port1::SetInput(1<<0); }

  /// Pulses Pin16 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin16 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin16  value.
  /// @return Port pin value.
  static bool Test()      { return Port1::Test(0); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<0)
  static constexpr uint32_t bitmask()               { return (1<<0); }

  /// Returns the bit corresponding to this pin.
  /// @return 0
  static constexpr uint8_t bit()                   { return 0; }
};

class Pin17 : public Pin<Port1> {
public:
  /// Sets Pin17 to HIGH.
  static void Set()       { Port1::SetBits(1<<1); }

  /// Sets Pin17 to LOW.
  static void Clear()     { Port1::ClearBits(1<<1); }

  /// Toggles Pin17 value.
  static void Toggle()    { Port1::ToggleBits(1<<1); }

  /// Configures Pin17 as an output pin.
  static void SetOutput() { Port1::SetOutput(1<<1); }

  /// Configures Pin17 as an input pin.
  static void SetInput()  { Port1::SetInput(1<<1); }

  /// Pulses Pin17 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin17 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin17  value.
  /// @return Port pin value.
  static bool Test()      { return Port1::Test(1); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<1)
  static constexpr uint32_t bitmask()               { return (1<<1); }

  /// Returns the bit corresponding to this pin.
  /// @return 1
  static constexpr uint8_t bit()                   { return 1; }
};

class Pin18 : public Pin<Port1> {
public:
  /// Sets Pin18 to HIGH.
  static void Set()       { Port1::SetBits(1<<2); }

  /// Sets Pin18 to LOW.
  static void Clear()     { Port1::ClearBits(1<<2); }

  /// Toggles Pin18 value.
  static void Toggle()    { Port1::ToggleBits(1<<2); }

  /// Configures Pin18 as an output pin.
  static void SetOutput() { Port1::SetOutput(1<<2); }

  /// Configures Pin18 as an input pin.
  static void SetInput()  { Port1::SetInput(1<<2); }

  /// Pulses Pin18 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin18 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin18  value.
  /// @return Port pin value.
  static bool Test()      { return Port1::Test(2); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<2)
  static constexpr uint32_t bitmask()               { return (1<<2); }

  /// Returns the bit corresponding to this pin.
  /// @return 2
  static constexpr uint8_t bit()                   { return 2; }
};

class Pin19 : public Pin<Port1> {
public:
  /// Sets Pin19 to HIGH.
  static void Set()       { Port1::SetBits(1<<3); }

  /// Sets Pin19 to LOW.
  static void Clear()     { Port1::ClearBits(1<<3); }

  /// Toggles Pin19 value.
  static void Toggle()    { Port1::ToggleBits(1<<3); }

  /// Configures Pin19 as an output pin.
  static void SetOutput() { Port1::SetOutput(1<<3); }

  /// Configures Pin19 as an input pin.
  static void SetInput()  { Port1::SetInput(1<<3); }

  /// Pulses Pin19 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin19 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin19  value.
  /// @return Port pin value.
  static bool Test()      { return Port1::Test(3); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<3)
  static constexpr uint32_t bitmask()               { return (1<<3); }

  /// Returns the bit corresponding to this pin.
  /// @return 3
  static constexpr uint8_t bit()                   { return 3; }
};

class Pin20 : public Pin<Port1> {
public:
  /// Sets Pin20 to HIGH.
  static void Set()       { Port1::SetBits(1<<4); }

  /// Sets Pin20 to LOW.
  static void Clear()     { Port1::ClearBits(1<<4); }

  /// Toggles Pin20 value.
  static void Toggle()    { Port1::ToggleBits(1<<4); }

  /// Configures Pin20 as an output pin.
  static void SetOutput() { Port1::SetOutput(1<<4); }

  /// Configures Pin20 as an input pin.
  static void SetInput()  { Port1::SetInput(1<<4); }

  /// Pulses Pin20 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin20 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin20  value.
  /// @return Port pin value.
  static bool Test()      { return Port1::Test(4); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<4)
  static constexpr uint32_t bitmask()               { return (1<<4); }

  /// Returns the bit corresponding to this pin.
  /// @return 4
  static constexpr uint8_t bit()                   { return 4; }
};

class Pin21 : public Pin<Port1> {
public:
  /// Sets Pin21 to HIGH.
  static void Set()       { Port1::SetBits(1<<5); }

  /// Sets Pin21 to LOW.
  static void Clear()     { Port1::ClearBits(1<<5); }

  /// Toggles Pin21 value.
  static void Toggle()    { Port1::ToggleBits(1<<5); }

  /// Configures Pin21 as an output pin.
  static void SetOutput() { Port1::SetOutput(1<<5); }

  /// Configures Pin21 as an input pin.
  static void SetInput()  { Port1::SetInput(1<<5); }

  /// Pulses Pin21 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin21 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin21  value.
  /// @return Port pin value.
  static bool Test()      { return Port1::Test(5); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<5)
  static constexpr uint32_t bitmask()               { return (1<<5); }

  /// Returns the bit corresponding to this pin.
  /// @return 5
  static constexpr uint8_t bit()                   { return 5; }
};

class Pin22 : public Pin<Port1> {
public:
  /// Sets Pin22 to HIGH.
  static void Set()       { Port1::SetBits(1<<6); }

  /// Sets Pin22 to LOW.
  static void Clear()     { Port1::ClearBits(1<<6); }

  /// Toggles Pin22 value.
  static void Toggle()    { Port1::ToggleBits(1<<6); }

  /// Configures Pin22 as an output pin.
  static void SetOutput() { Port1::SetOutput(1<<6); }

  /// Configures Pin22 as an input pin.
  static void SetInput()  { Port1::SetInput(1<<6); }

  /// Pulses Pin22 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin22 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin22  value.
  /// @return Port pin value.
  static bool Test()      { return Port1::Test(6); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<6)
  static constexpr uint32_t bitmask()               { return (1<<6); }

  /// Returns the bit corresponding to this pin.
  /// @return 6
  static constexpr uint8_t bit()                   { return 6; }
};

class Pin23 : public Pin<Port1> {
public:
  /// Sets Pin23 to HIGH.
  static void Set()       { Port1::SetBits(1<<7); }

  /// Sets Pin23 to LOW.
  static void Clear()     { Port1::ClearBits(1<<7); }

  /// Toggles Pin23 value.
  static void Toggle()    { Port1::ToggleBits(1<<7); }

  /// Configures Pin23 as an output pin.
  static void SetOutput() { Port1::SetOutput(1<<7); }

  /// Configures Pin23 as an input pin.
  static void SetInput()  { Port1::SetInput(1<<7); }

  /// Pulses Pin23 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin23 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin23  value.
  /// @return Port pin value.
  static bool Test()      { return Port1::Test(7); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<7)
  static constexpr uint32_t bitmask()               { return (1<<7); }

  /// Returns the bit corresponding to this pin.
  /// @return 7
  static constexpr uint8_t bit()                   { return 7; }
};

class Pin24 : public Pin<Port1> {
public:
  /// Sets Pin24 to HIGH.
  static void Set()       { Port1::SetBits(1<<8); }

  /// Sets Pin24 to LOW.
  static void Clear()     { Port1::ClearBits(1<<8); }

  /// Toggles Pin24 value.
  static void Toggle()    { Port1::ToggleBits(1<<8); }

  /// Configures Pin24 as an output pin.
  static void SetOutput() { Port1::SetOutput(1<<8); }

  /// Configures Pin24 as an input pin.
  static void SetInput()  { Port1::SetInput(1<<8); }

  /// Pulses Pin24 with high state first.
  static void PulseHigh() { Set(); Clear(); }

  /// Pulses Pin24 with low state first.
  static void PulseLow()  { Clear(); Set(); }

  /// Reads Pin24  value.
  /// @return Port pin value.
  static bool Test()      { return Port1::Test(8); }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<8)
  static constexpr uint32_t bitmask()               { return (1<<8); }

  /// Returns the bit corresponding to this pin.
  /// @return 8
  static constexpr uint8_t bit()                   { return 8; }
};


} // namespace etl
