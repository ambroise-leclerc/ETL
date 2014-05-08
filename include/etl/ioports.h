/// @file ioports.h
/// @date 19/01/2014 22:28:16
/// @author Ambroise Leclerc
/// @brief AVR 8-bit microcontrollers ports handling classes
//
// Copyright (c) 2014, Ambroise Leclerc
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

#ifndef ETL_IOPORTS_H_
#define ETL_IOPORTS_H_


namespace etl {
#ifdef PORTB
struct PortB {
  /// Assigns a value to PORTB.
  /// @param[in] value value affected to PORTB
  static void Assign(uint8_t value)   { PORTB = value; }

  /// Sets masked bits in PORTB.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTB |= mask;}

  /// Clears masked bits in PORTB.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTB &= ~mask;} 

  /// Toggles masked bits in PORTB.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTB ^= mask;} 

  /// Pulses masked bits in PORTB with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTB |= mask; PORTB &= ~mask; }

  /// Pulses masked bits in PORTB with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTB &= ~mask; PORTB |= mask; }

  /// Set corresponding masked bits of PORTB to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRB |= mask; }

  /// Set corresponding masked bits of PORTB to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRB &= ~mask; }

  /// Returns PINB register.
  static uint8_t GetPIN()             { return PINB; }

  /// Tests masked bits of PORTB
  /// @param[in] mask bits
  /// @param[in] true if the corresponding bits are all set, false otherwise.
  static bool TestBits(uint8_t mask)  { return (PINB & mask) == mask; }

  /// Returns the value of the bit at the position pos.
  /// @param[in] position of the bit to return
  /// @return true if the requested bit is set, false otherwise.
  static bool Test(uint8_t bit) { return PINB & (1<<bit); }
};

struct PinB0 {
  /// Sets PinB0 to HIGH.
  static void Set()       { PORTB |= (1<<0); }

  /// Sets PinB0 to LOW.
  static void Clear()     { PORTB &= ~(1<<0); }

  /// Toggles PinB0 value.
  static void Toggle()    { PINB |= (1<<0); }

  /// Configures PinB0  as an output pin.
  static void SetOutput() { DDRB |= (1<<0); }

  /// Configures PinB0  as an input pin.
  static void SetInput()  { DDRB &= ~(1<<0); }

  /// Pulses PinB0 with high state first.
  static void PulseHigh() { PORTB |= (1<<0); PORTB &= ~(1<<0); }

  /// Pulses PinB0 with low state first.
  static void PulseLow()  { PORTB &= ~(1<<0); PORTB |= (1<<0); }

  /// Reads PinB0  value.
  /// @return Port pin value.
  static bool Test()      { return PINB & (1<<0); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTB
  static constexpr decltype(PORTB) GetNativePort() { return PORTB; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<0)
  static constexpr uint8_t bitmask()               { return (1<<0); }

  /// Returns the bit corresponding to this pin.
  /// @return 0
  static constexpr uint8_t bit()                   { return 0; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortB;
};

struct PinB1 {
  /// Sets PinB1 to HIGH.
  static void Set()       { PORTB |= (1<<1); }

  /// Sets PinB1 to LOW.
  static void Clear()     { PORTB &= ~(1<<1); }

  /// Toggles PinB1 value.
  static void Toggle()    { PINB |= (1<<1); }

  /// Configures PinB1  as an output pin.
  static void SetOutput() { DDRB |= (1<<1); }

  /// Configures PinB1  as an input pin.
  static void SetInput()  { DDRB &= ~(1<<1); }

  /// Pulses PinB1 with high state first.
  static void PulseHigh() { PORTB |= (1<<1); PORTB &= ~(1<<1); }

  /// Pulses PinB1 with low state first.
  static void PulseLow()  { PORTB &= ~(1<<1); PORTB |= (1<<1); }

  /// Reads PinB1  value.
  /// @return Port pin value.
  static bool Test()      { return PINB & (1<<1); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTB
  static constexpr decltype(PORTB) GetNativePort() { return PORTB; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<1)
  static constexpr uint8_t bitmask()               { return (1<<1); }

  /// Returns the bit corresponding to this pin.
  /// @return 1
  static constexpr uint8_t bit()                   { return 1; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortB;
};

struct PinB2 {
  /// Sets PinB2 to HIGH.
  static void Set()       { PORTB |= (1<<2); }

  /// Sets PinB2 to LOW.
  static void Clear()     { PORTB &= ~(1<<2); }

  /// Toggles PinB2 value.
  static void Toggle()    { PINB |= (1<<2); }

  /// Configures PinB2  as an output pin.
  static void SetOutput() { DDRB |= (1<<2); }

  /// Configures PinB2  as an input pin.
  static void SetInput()  { DDRB &= ~(1<<2); }

  /// Pulses PinB2 with high state first.
  static void PulseHigh() { PORTB |= (1<<2); PORTB &= ~(1<<2); }

  /// Pulses PinB2 with low state first.
  static void PulseLow()  { PORTB &= ~(1<<2); PORTB |= (1<<2); }

  /// Reads PinB2  value.
  /// @return Port pin value.
  static bool Test()      { return PINB & (1<<2); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTB
  static constexpr decltype(PORTB) GetNativePort() { return PORTB; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<2)
  static constexpr uint8_t bitmask()               { return (1<<2); }

  /// Returns the bit corresponding to this pin.
  /// @return 2
  static constexpr uint8_t bit()                   { return 2; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortB;
};

struct PinB3 {
  /// Sets PinB3 to HIGH.
  static void Set()       { PORTB |= (1<<3); }

  /// Sets PinB3 to LOW.
  static void Clear()     { PORTB &= ~(1<<3); }

  /// Toggles PinB3 value.
  static void Toggle()    { PINB |= (1<<3); }

  /// Configures PinB3  as an output pin.
  static void SetOutput() { DDRB |= (1<<3); }

  /// Configures PinB3  as an input pin.
  static void SetInput()  { DDRB &= ~(1<<3); }

  /// Pulses PinB3 with high state first.
  static void PulseHigh() { PORTB |= (1<<3); PORTB &= ~(1<<3); }

  /// Pulses PinB3 with low state first.
  static void PulseLow()  { PORTB &= ~(1<<3); PORTB |= (1<<3); }

  /// Reads PinB3  value.
  /// @return Port pin value.
  static bool Test()      { return PINB & (1<<3); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTB
  static constexpr decltype(PORTB) GetNativePort() { return PORTB; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<3)
  static constexpr uint8_t bitmask()               { return (1<<3); }

  /// Returns the bit corresponding to this pin.
  /// @return 3
  static constexpr uint8_t bit()                   { return 3; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortB;
};

struct PinB4 {
  /// Sets PinB4 to HIGH.
  static void Set()       { PORTB |= (1<<4); }

  /// Sets PinB4 to LOW.
  static void Clear()     { PORTB &= ~(1<<4); }

  /// Toggles PinB4 value.
  static void Toggle()    { PINB |= (1<<4); }

  /// Configures PinB4  as an output pin.
  static void SetOutput() { DDRB |= (1<<4); }

  /// Configures PinB4  as an input pin.
  static void SetInput()  { DDRB &= ~(1<<4); }

  /// Pulses PinB4 with high state first.
  static void PulseHigh() { PORTB |= (1<<4); PORTB &= ~(1<<4); }

  /// Pulses PinB4 with low state first.
  static void PulseLow()  { PORTB &= ~(1<<4); PORTB |= (1<<4); }

  /// Reads PinB4  value.
  /// @return Port pin value.
  static bool Test()      { return PINB & (1<<4); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTB
  static constexpr decltype(PORTB) GetNativePort() { return PORTB; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<4)
  static constexpr uint8_t bitmask()               { return (1<<4); }

  /// Returns the bit corresponding to this pin.
  /// @return 4
  static constexpr uint8_t bit()                   { return 4; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortB;
};

struct PinB5 {
  /// Sets PinB5 to HIGH.
  static void Set()       { PORTB |= (1<<5); }

  /// Sets PinB5 to LOW.
  static void Clear()     { PORTB &= ~(1<<5); }

  /// Toggles PinB5 value.
  static void Toggle()    { PINB |= (1<<5); }

  /// Configures PinB5  as an output pin.
  static void SetOutput() { DDRB |= (1<<5); }

  /// Configures PinB5  as an input pin.
  static void SetInput()  { DDRB &= ~(1<<5); }

  /// Pulses PinB5 with high state first.
  static void PulseHigh() { PORTB |= (1<<5); PORTB &= ~(1<<5); }

  /// Pulses PinB5 with low state first.
  static void PulseLow()  { PORTB &= ~(1<<5); PORTB |= (1<<5); }

  /// Reads PinB5  value.
  /// @return Port pin value.
  static bool Test()      { return PINB & (1<<5); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTB
  static constexpr decltype(PORTB) GetNativePort() { return PORTB; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<5)
  static constexpr uint8_t bitmask()               { return (1<<5); }

  /// Returns the bit corresponding to this pin.
  /// @return 5
  static constexpr uint8_t bit()                   { return 5; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortB;
};

struct PinB6 {
  /// Sets PinB6 to HIGH.
  static void Set()       { PORTB |= (1<<6); }

  /// Sets PinB6 to LOW.
  static void Clear()     { PORTB &= ~(1<<6); }

  /// Toggles PinB6 value.
  static void Toggle()    { PINB |= (1<<6); }

  /// Configures PinB6  as an output pin.
  static void SetOutput() { DDRB |= (1<<6); }

  /// Configures PinB6  as an input pin.
  static void SetInput()  { DDRB &= ~(1<<6); }

  /// Pulses PinB6 with high state first.
  static void PulseHigh() { PORTB |= (1<<6); PORTB &= ~(1<<6); }

  /// Pulses PinB6 with low state first.
  static void PulseLow()  { PORTB &= ~(1<<6); PORTB |= (1<<6); }

  /// Reads PinB6  value.
  /// @return Port pin value.
  static bool Test()      { return PINB & (1<<6); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTB
  static constexpr decltype(PORTB) GetNativePort() { return PORTB; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<6)
  static constexpr uint8_t bitmask()               { return (1<<6); }

  /// Returns the bit corresponding to this pin.
  /// @return 6
  static constexpr uint8_t bit()                   { return 6; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortB;
};

struct PinB7 {
  /// Sets PinB7 to HIGH.
  static void Set()       { PORTB |= (1<<7); }

  /// Sets PinB7 to LOW.
  static void Clear()     { PORTB &= ~(1<<7); }

  /// Toggles PinB7 value.
  static void Toggle()    { PINB |= (1<<7); }

  /// Configures PinB7  as an output pin.
  static void SetOutput() { DDRB |= (1<<7); }

  /// Configures PinB7  as an input pin.
  static void SetInput()  { DDRB &= ~(1<<7); }

  /// Pulses PinB7 with high state first.
  static void PulseHigh() { PORTB |= (1<<7); PORTB &= ~(1<<7); }

  /// Pulses PinB7 with low state first.
  static void PulseLow()  { PORTB &= ~(1<<7); PORTB |= (1<<7); }

  /// Reads PinB7  value.
  /// @return Port pin value.
  static bool Test()      { return PINB & (1<<7); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTB
  static constexpr decltype(PORTB) GetNativePort() { return PORTB; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<7)
  static constexpr uint8_t bitmask()               { return (1<<7); }

  /// Returns the bit corresponding to this pin.
  /// @return 7
  static constexpr uint8_t bit()                   { return 7; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortB;
};

#endif //PORTB

#ifdef PORTC
struct PortC {
  /// Assigns a value to PORTC.
  /// @param[in] value value affected to PORTC
  static void Assign(uint8_t value)   { PORTC = value; }

  /// Sets masked bits in PORTC.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTC |= mask;}

  /// Clears masked bits in PORTC.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTC &= ~mask;} 

  /// Toggles masked bits in PORTC.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTC ^= mask;} 

  /// Pulses masked bits in PORTC with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTC |= mask; PORTC &= ~mask; }

  /// Pulses masked bits in PORTC with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTC &= ~mask; PORTC |= mask; }

  /// Set corresponding masked bits of PORTC to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRC |= mask; }

  /// Set corresponding masked bits of PORTC to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRC &= ~mask; }

  /// Returns PINC register.
  static uint8_t GetPIN()             { return PINC; }

  /// Tests masked bits of PORTC
  /// @param[in] mask bits
  /// @param[in] true if the corresponding bits are all set, false otherwise.
  static bool TestBits(uint8_t mask)  { return (PINC & mask) == mask; }

  /// Returns the value of the bit at the position pos.
  /// @param[in] position of the bit to return
  /// @return true if the requested bit is set, false otherwise.
  static bool Test(uint8_t bit) { return PINC & (1<<bit); }
};

struct PinC0 {
  /// Sets PinC0 to HIGH.
  static void Set()       { PORTC |= (1<<0); }

  /// Sets PinC0 to LOW.
  static void Clear()     { PORTC &= ~(1<<0); }

  /// Toggles PinC0 value.
  static void Toggle()    { PINC |= (1<<0); }

  /// Configures PinC0  as an output pin.
  static void SetOutput() { DDRC |= (1<<0); }

  /// Configures PinC0  as an input pin.
  static void SetInput()  { DDRC &= ~(1<<0); }

  /// Pulses PinC0 with high state first.
  static void PulseHigh() { PORTC |= (1<<0); PORTC &= ~(1<<0); }

  /// Pulses PinC0 with low state first.
  static void PulseLow()  { PORTC &= ~(1<<0); PORTC |= (1<<0); }

  /// Reads PinC0  value.
  /// @return Port pin value.
  static bool Test()      { return PINC & (1<<0); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTC
  static constexpr decltype(PORTC) GetNativePort() { return PORTC; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<0)
  static constexpr uint8_t bitmask()               { return (1<<0); }

  /// Returns the bit corresponding to this pin.
  /// @return 0
  static constexpr uint8_t bit()                   { return 0; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortC;
};

struct PinC1 {
  /// Sets PinC1 to HIGH.
  static void Set()       { PORTC |= (1<<1); }

  /// Sets PinC1 to LOW.
  static void Clear()     { PORTC &= ~(1<<1); }

  /// Toggles PinC1 value.
  static void Toggle()    { PINC |= (1<<1); }

  /// Configures PinC1  as an output pin.
  static void SetOutput() { DDRC |= (1<<1); }

  /// Configures PinC1  as an input pin.
  static void SetInput()  { DDRC &= ~(1<<1); }

  /// Pulses PinC1 with high state first.
  static void PulseHigh() { PORTC |= (1<<1); PORTC &= ~(1<<1); }

  /// Pulses PinC1 with low state first.
  static void PulseLow()  { PORTC &= ~(1<<1); PORTC |= (1<<1); }

  /// Reads PinC1  value.
  /// @return Port pin value.
  static bool Test()      { return PINC & (1<<1); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTC
  static constexpr decltype(PORTC) GetNativePort() { return PORTC; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<1)
  static constexpr uint8_t bitmask()               { return (1<<1); }

  /// Returns the bit corresponding to this pin.
  /// @return 1
  static constexpr uint8_t bit()                   { return 1; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortC;
};

struct PinC2 {
  /// Sets PinC2 to HIGH.
  static void Set()       { PORTC |= (1<<2); }

  /// Sets PinC2 to LOW.
  static void Clear()     { PORTC &= ~(1<<2); }

  /// Toggles PinC2 value.
  static void Toggle()    { PINC |= (1<<2); }

  /// Configures PinC2  as an output pin.
  static void SetOutput() { DDRC |= (1<<2); }

  /// Configures PinC2  as an input pin.
  static void SetInput()  { DDRC &= ~(1<<2); }

  /// Pulses PinC2 with high state first.
  static void PulseHigh() { PORTC |= (1<<2); PORTC &= ~(1<<2); }

  /// Pulses PinC2 with low state first.
  static void PulseLow()  { PORTC &= ~(1<<2); PORTC |= (1<<2); }

  /// Reads PinC2  value.
  /// @return Port pin value.
  static bool Test()      { return PINC & (1<<2); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTC
  static constexpr decltype(PORTC) GetNativePort() { return PORTC; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<2)
  static constexpr uint8_t bitmask()               { return (1<<2); }

  /// Returns the bit corresponding to this pin.
  /// @return 2
  static constexpr uint8_t bit()                   { return 2; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortC;
};

struct PinC3 {
  /// Sets PinC3 to HIGH.
  static void Set()       { PORTC |= (1<<3); }

  /// Sets PinC3 to LOW.
  static void Clear()     { PORTC &= ~(1<<3); }

  /// Toggles PinC3 value.
  static void Toggle()    { PINC |= (1<<3); }

  /// Configures PinC3  as an output pin.
  static void SetOutput() { DDRC |= (1<<3); }

  /// Configures PinC3  as an input pin.
  static void SetInput()  { DDRC &= ~(1<<3); }

  /// Pulses PinC3 with high state first.
  static void PulseHigh() { PORTC |= (1<<3); PORTC &= ~(1<<3); }

  /// Pulses PinC3 with low state first.
  static void PulseLow()  { PORTC &= ~(1<<3); PORTC |= (1<<3); }

  /// Reads PinC3  value.
  /// @return Port pin value.
  static bool Test()      { return PINC & (1<<3); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTC
  static constexpr decltype(PORTC) GetNativePort() { return PORTC; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<3)
  static constexpr uint8_t bitmask()               { return (1<<3); }

  /// Returns the bit corresponding to this pin.
  /// @return 3
  static constexpr uint8_t bit()                   { return 3; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortC;
};

struct PinC4 {
  /// Sets PinC4 to HIGH.
  static void Set()       { PORTC |= (1<<4); }

  /// Sets PinC4 to LOW.
  static void Clear()     { PORTC &= ~(1<<4); }

  /// Toggles PinC4 value.
  static void Toggle()    { PINC |= (1<<4); }

  /// Configures PinC4  as an output pin.
  static void SetOutput() { DDRC |= (1<<4); }

  /// Configures PinC4  as an input pin.
  static void SetInput()  { DDRC &= ~(1<<4); }

  /// Pulses PinC4 with high state first.
  static void PulseHigh() { PORTC |= (1<<4); PORTC &= ~(1<<4); }

  /// Pulses PinC4 with low state first.
  static void PulseLow()  { PORTC &= ~(1<<4); PORTC |= (1<<4); }

  /// Reads PinC4  value.
  /// @return Port pin value.
  static bool Test()      { return PINC & (1<<4); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTC
  static constexpr decltype(PORTC) GetNativePort() { return PORTC; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<4)
  static constexpr uint8_t bitmask()               { return (1<<4); }

  /// Returns the bit corresponding to this pin.
  /// @return 4
  static constexpr uint8_t bit()                   { return 4; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortC;
};

struct PinC5 {
  /// Sets PinC5 to HIGH.
  static void Set()       { PORTC |= (1<<5); }

  /// Sets PinC5 to LOW.
  static void Clear()     { PORTC &= ~(1<<5); }

  /// Toggles PinC5 value.
  static void Toggle()    { PINC |= (1<<5); }

  /// Configures PinC5  as an output pin.
  static void SetOutput() { DDRC |= (1<<5); }

  /// Configures PinC5  as an input pin.
  static void SetInput()  { DDRC &= ~(1<<5); }

  /// Pulses PinC5 with high state first.
  static void PulseHigh() { PORTC |= (1<<5); PORTC &= ~(1<<5); }

  /// Pulses PinC5 with low state first.
  static void PulseLow()  { PORTC &= ~(1<<5); PORTC |= (1<<5); }

  /// Reads PinC5  value.
  /// @return Port pin value.
  static bool Test()      { return PINC & (1<<5); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTC
  static constexpr decltype(PORTC) GetNativePort() { return PORTC; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<5)
  static constexpr uint8_t bitmask()               { return (1<<5); }

  /// Returns the bit corresponding to this pin.
  /// @return 5
  static constexpr uint8_t bit()                   { return 5; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortC;
};

struct PinC6 {
  /// Sets PinC6 to HIGH.
  static void Set()       { PORTC |= (1<<6); }

  /// Sets PinC6 to LOW.
  static void Clear()     { PORTC &= ~(1<<6); }

  /// Toggles PinC6 value.
  static void Toggle()    { PINC |= (1<<6); }

  /// Configures PinC6  as an output pin.
  static void SetOutput() { DDRC |= (1<<6); }

  /// Configures PinC6  as an input pin.
  static void SetInput()  { DDRC &= ~(1<<6); }

  /// Pulses PinC6 with high state first.
  static void PulseHigh() { PORTC |= (1<<6); PORTC &= ~(1<<6); }

  /// Pulses PinC6 with low state first.
  static void PulseLow()  { PORTC &= ~(1<<6); PORTC |= (1<<6); }

  /// Reads PinC6  value.
  /// @return Port pin value.
  static bool Test()      { return PINC & (1<<6); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTC
  static constexpr decltype(PORTC) GetNativePort() { return PORTC; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<6)
  static constexpr uint8_t bitmask()               { return (1<<6); }

  /// Returns the bit corresponding to this pin.
  /// @return 6
  static constexpr uint8_t bit()                   { return 6; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortC;
};

struct PinC7 {
  /// Sets PinC7 to HIGH.
  static void Set()       { PORTC |= (1<<7); }

  /// Sets PinC7 to LOW.
  static void Clear()     { PORTC &= ~(1<<7); }

  /// Toggles PinC7 value.
  static void Toggle()    { PINC |= (1<<7); }

  /// Configures PinC7  as an output pin.
  static void SetOutput() { DDRC |= (1<<7); }

  /// Configures PinC7  as an input pin.
  static void SetInput()  { DDRC &= ~(1<<7); }

  /// Pulses PinC7 with high state first.
  static void PulseHigh() { PORTC |= (1<<7); PORTC &= ~(1<<7); }

  /// Pulses PinC7 with low state first.
  static void PulseLow()  { PORTC &= ~(1<<7); PORTC |= (1<<7); }

  /// Reads PinC7  value.
  /// @return Port pin value.
  static bool Test()      { return PINC & (1<<7); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTC
  static constexpr decltype(PORTC) GetNativePort() { return PORTC; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<7)
  static constexpr uint8_t bitmask()               { return (1<<7); }

  /// Returns the bit corresponding to this pin.
  /// @return 7
  static constexpr uint8_t bit()                   { return 7; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortC;
};

#endif //PORTC

#ifdef PORTD
struct PortD {
  /// Assigns a value to PORTD.
  /// @param[in] value value affected to PORTD
  static void Assign(uint8_t value)   { PORTD = value; }

  /// Sets masked bits in PORTD.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTD |= mask;}

  /// Clears masked bits in PORTD.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTD &= ~mask;} 

  /// Toggles masked bits in PORTD.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTD ^= mask;} 

  /// Pulses masked bits in PORTD with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTD |= mask; PORTD &= ~mask; }

  /// Pulses masked bits in PORTD with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTD &= ~mask; PORTD |= mask; }

  /// Set corresponding masked bits of PORTD to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRD |= mask; }

  /// Set corresponding masked bits of PORTD to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRD &= ~mask; }

  /// Returns PIND register.
  static uint8_t GetPIN()             { return PIND; }

  /// Tests masked bits of PORTD
  /// @param[in] mask bits
  /// @param[in] true if the corresponding bits are all set, false otherwise.
  static bool TestBits(uint8_t mask)  { return (PIND & mask) == mask; }

  /// Returns the value of the bit at the position pos.
  /// @param[in] position of the bit to return
  /// @return true if the requested bit is set, false otherwise.
  static bool Test(uint8_t bit) { return PIND & (1<<bit); }
};

struct PinD0 {
  /// Sets PinD0 to HIGH.
  static void Set()       { PORTD |= (1<<0); }

  /// Sets PinD0 to LOW.
  static void Clear()     { PORTD &= ~(1<<0); }

  /// Toggles PinD0 value.
  static void Toggle()    { PIND |= (1<<0); }

  /// Configures PinD0  as an output pin.
  static void SetOutput() { DDRD |= (1<<0); }

  /// Configures PinD0  as an input pin.
  static void SetInput()  { DDRD &= ~(1<<0); }

  /// Pulses PinD0 with high state first.
  static void PulseHigh() { PORTD |= (1<<0); PORTD &= ~(1<<0); }

  /// Pulses PinD0 with low state first.
  static void PulseLow()  { PORTD &= ~(1<<0); PORTD |= (1<<0); }

  /// Reads PinD0  value.
  /// @return Port pin value.
  static bool Test()      { return PIND & (1<<0); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTD
  static constexpr decltype(PORTD) GetNativePort() { return PORTD; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<0)
  static constexpr uint8_t bitmask()               { return (1<<0); }

  /// Returns the bit corresponding to this pin.
  /// @return 0
  static constexpr uint8_t bit()                   { return 0; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortD;
};

struct PinD1 {
  /// Sets PinD1 to HIGH.
  static void Set()       { PORTD |= (1<<1); }

  /// Sets PinD1 to LOW.
  static void Clear()     { PORTD &= ~(1<<1); }

  /// Toggles PinD1 value.
  static void Toggle()    { PIND |= (1<<1); }

  /// Configures PinD1  as an output pin.
  static void SetOutput() { DDRD |= (1<<1); }

  /// Configures PinD1  as an input pin.
  static void SetInput()  { DDRD &= ~(1<<1); }

  /// Pulses PinD1 with high state first.
  static void PulseHigh() { PORTD |= (1<<1); PORTD &= ~(1<<1); }

  /// Pulses PinD1 with low state first.
  static void PulseLow()  { PORTD &= ~(1<<1); PORTD |= (1<<1); }

  /// Reads PinD1  value.
  /// @return Port pin value.
  static bool Test()      { return PIND & (1<<1); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTD
  static constexpr decltype(PORTD) GetNativePort() { return PORTD; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<1)
  static constexpr uint8_t bitmask()               { return (1<<1); }

  /// Returns the bit corresponding to this pin.
  /// @return 1
  static constexpr uint8_t bit()                   { return 1; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortD;
};

struct PinD2 {
  /// Sets PinD2 to HIGH.
  static void Set()       { PORTD |= (1<<2); }

  /// Sets PinD2 to LOW.
  static void Clear()     { PORTD &= ~(1<<2); }

  /// Toggles PinD2 value.
  static void Toggle()    { PIND |= (1<<2); }

  /// Configures PinD2  as an output pin.
  static void SetOutput() { DDRD |= (1<<2); }

  /// Configures PinD2  as an input pin.
  static void SetInput()  { DDRD &= ~(1<<2); }

  /// Pulses PinD2 with high state first.
  static void PulseHigh() { PORTD |= (1<<2); PORTD &= ~(1<<2); }

  /// Pulses PinD2 with low state first.
  static void PulseLow()  { PORTD &= ~(1<<2); PORTD |= (1<<2); }

  /// Reads PinD2  value.
  /// @return Port pin value.
  static bool Test()      { return PIND & (1<<2); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTD
  static constexpr decltype(PORTD) GetNativePort() { return PORTD; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<2)
  static constexpr uint8_t bitmask()               { return (1<<2); }

  /// Returns the bit corresponding to this pin.
  /// @return 2
  static constexpr uint8_t bit()                   { return 2; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortD;
};

struct PinD3 {
  /// Sets PinD3 to HIGH.
  static void Set()       { PORTD |= (1<<3); }

  /// Sets PinD3 to LOW.
  static void Clear()     { PORTD &= ~(1<<3); }

  /// Toggles PinD3 value.
  static void Toggle()    { PIND |= (1<<3); }

  /// Configures PinD3  as an output pin.
  static void SetOutput() { DDRD |= (1<<3); }

  /// Configures PinD3  as an input pin.
  static void SetInput()  { DDRD &= ~(1<<3); }

  /// Pulses PinD3 with high state first.
  static void PulseHigh() { PORTD |= (1<<3); PORTD &= ~(1<<3); }

  /// Pulses PinD3 with low state first.
  static void PulseLow()  { PORTD &= ~(1<<3); PORTD |= (1<<3); }

  /// Reads PinD3  value.
  /// @return Port pin value.
  static bool Test()      { return PIND & (1<<3); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTD
  static constexpr decltype(PORTD) GetNativePort() { return PORTD; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<3)
  static constexpr uint8_t bitmask()               { return (1<<3); }

  /// Returns the bit corresponding to this pin.
  /// @return 3
  static constexpr uint8_t bit()                   { return 3; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortD;
};

struct PinD4 {
  /// Sets PinD4 to HIGH.
  static void Set()       { PORTD |= (1<<4); }

  /// Sets PinD4 to LOW.
  static void Clear()     { PORTD &= ~(1<<4); }

  /// Toggles PinD4 value.
  static void Toggle()    { PIND |= (1<<4); }

  /// Configures PinD4  as an output pin.
  static void SetOutput() { DDRD |= (1<<4); }

  /// Configures PinD4  as an input pin.
  static void SetInput()  { DDRD &= ~(1<<4); }

  /// Pulses PinD4 with high state first.
  static void PulseHigh() { PORTD |= (1<<4); PORTD &= ~(1<<4); }

  /// Pulses PinD4 with low state first.
  static void PulseLow()  { PORTD &= ~(1<<4); PORTD |= (1<<4); }

  /// Reads PinD4  value.
  /// @return Port pin value.
  static bool Test()      { return PIND & (1<<4); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTD
  static constexpr decltype(PORTD) GetNativePort() { return PORTD; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<4)
  static constexpr uint8_t bitmask()               { return (1<<4); }

  /// Returns the bit corresponding to this pin.
  /// @return 4
  static constexpr uint8_t bit()                   { return 4; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortD;
};

struct PinD5 {
  /// Sets PinD5 to HIGH.
  static void Set()       { PORTD |= (1<<5); }

  /// Sets PinD5 to LOW.
  static void Clear()     { PORTD &= ~(1<<5); }

  /// Toggles PinD5 value.
  static void Toggle()    { PIND |= (1<<5); }

  /// Configures PinD5  as an output pin.
  static void SetOutput() { DDRD |= (1<<5); }

  /// Configures PinD5  as an input pin.
  static void SetInput()  { DDRD &= ~(1<<5); }

  /// Pulses PinD5 with high state first.
  static void PulseHigh() { PORTD |= (1<<5); PORTD &= ~(1<<5); }

  /// Pulses PinD5 with low state first.
  static void PulseLow()  { PORTD &= ~(1<<5); PORTD |= (1<<5); }

  /// Reads PinD5  value.
  /// @return Port pin value.
  static bool Test()      { return PIND & (1<<5); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTD
  static constexpr decltype(PORTD) GetNativePort() { return PORTD; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<5)
  static constexpr uint8_t bitmask()               { return (1<<5); }

  /// Returns the bit corresponding to this pin.
  /// @return 5
  static constexpr uint8_t bit()                   { return 5; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortD;
};

struct PinD6 {
  /// Sets PinD6 to HIGH.
  static void Set()       { PORTD |= (1<<6); }

  /// Sets PinD6 to LOW.
  static void Clear()     { PORTD &= ~(1<<6); }

  /// Toggles PinD6 value.
  static void Toggle()    { PIND |= (1<<6); }

  /// Configures PinD6  as an output pin.
  static void SetOutput() { DDRD |= (1<<6); }

  /// Configures PinD6  as an input pin.
  static void SetInput()  { DDRD &= ~(1<<6); }

  /// Pulses PinD6 with high state first.
  static void PulseHigh() { PORTD |= (1<<6); PORTD &= ~(1<<6); }

  /// Pulses PinD6 with low state first.
  static void PulseLow()  { PORTD &= ~(1<<6); PORTD |= (1<<6); }

  /// Reads PinD6  value.
  /// @return Port pin value.
  static bool Test()      { return PIND & (1<<6); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTD
  static constexpr decltype(PORTD) GetNativePort() { return PORTD; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<6)
  static constexpr uint8_t bitmask()               { return (1<<6); }

  /// Returns the bit corresponding to this pin.
  /// @return 6
  static constexpr uint8_t bit()                   { return 6; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortD;
};

struct PinD7 {
  /// Sets PinD7 to HIGH.
  static void Set()       { PORTD |= (1<<7); }

  /// Sets PinD7 to LOW.
  static void Clear()     { PORTD &= ~(1<<7); }

  /// Toggles PinD7 value.
  static void Toggle()    { PIND |= (1<<7); }

  /// Configures PinD7  as an output pin.
  static void SetOutput() { DDRD |= (1<<7); }

  /// Configures PinD7  as an input pin.
  static void SetInput()  { DDRD &= ~(1<<7); }

  /// Pulses PinD7 with high state first.
  static void PulseHigh() { PORTD |= (1<<7); PORTD &= ~(1<<7); }

  /// Pulses PinD7 with low state first.
  static void PulseLow()  { PORTD &= ~(1<<7); PORTD |= (1<<7); }

  /// Reads PinD7  value.
  /// @return Port pin value.
  static bool Test()      { return PIND & (1<<7); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTD
  static constexpr decltype(PORTD) GetNativePort() { return PORTD; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<7)
  static constexpr uint8_t bitmask()               { return (1<<7); }

  /// Returns the bit corresponding to this pin.
  /// @return 7
  static constexpr uint8_t bit()                   { return 7; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortD;
};

#endif //PORTD

#ifdef PORTE
struct PortE {
  /// Assigns a value to PORTE.
  /// @param[in] value value affected to PORTE
  static void Assign(uint8_t value)   { PORTE = value; }

  /// Sets masked bits in PORTE.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTE |= mask;}

  /// Clears masked bits in PORTE.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTE &= ~mask;} 

  /// Toggles masked bits in PORTE.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTE ^= mask;} 

  /// Pulses masked bits in PORTE with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTE |= mask; PORTE &= ~mask; }

  /// Pulses masked bits in PORTE with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTE &= ~mask; PORTE |= mask; }

  /// Set corresponding masked bits of PORTE to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRE |= mask; }

  /// Set corresponding masked bits of PORTE to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRE &= ~mask; }

  /// Returns PINE register.
  static uint8_t GetPIN()             { return PINE; }

  /// Tests masked bits of PORTE
  /// @param[in] mask bits
  /// @param[in] true if the corresponding bits are all set, false otherwise.
  static bool TestBits(uint8_t mask)  { return (PINE & mask) == mask; }

  /// Returns the value of the bit at the position pos.
  /// @param[in] position of the bit to return
  /// @return true if the requested bit is set, false otherwise.
  static bool Test(uint8_t bit) { return PINE & (1<<bit); }
};

struct PinE0 {
  /// Sets PinE0 to HIGH.
  static void Set()       { PORTE |= (1<<0); }

  /// Sets PinE0 to LOW.
  static void Clear()     { PORTE &= ~(1<<0); }

  /// Toggles PinE0 value.
  static void Toggle()    { PINE |= (1<<0); }

  /// Configures PinE0  as an output pin.
  static void SetOutput() { DDRE |= (1<<0); }

  /// Configures PinE0  as an input pin.
  static void SetInput()  { DDRE &= ~(1<<0); }

  /// Pulses PinE0 with high state first.
  static void PulseHigh() { PORTE |= (1<<0); PORTE &= ~(1<<0); }

  /// Pulses PinE0 with low state first.
  static void PulseLow()  { PORTE &= ~(1<<0); PORTE |= (1<<0); }

  /// Reads PinE0  value.
  /// @return Port pin value.
  static bool Test()      { return PINE & (1<<0); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTE
  static constexpr decltype(PORTE) GetNativePort() { return PORTE; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<0)
  static constexpr uint8_t bitmask()               { return (1<<0); }

  /// Returns the bit corresponding to this pin.
  /// @return 0
  static constexpr uint8_t bit()                   { return 0; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortE;
};

struct PinE1 {
  /// Sets PinE1 to HIGH.
  static void Set()       { PORTE |= (1<<1); }

  /// Sets PinE1 to LOW.
  static void Clear()     { PORTE &= ~(1<<1); }

  /// Toggles PinE1 value.
  static void Toggle()    { PINE |= (1<<1); }

  /// Configures PinE1  as an output pin.
  static void SetOutput() { DDRE |= (1<<1); }

  /// Configures PinE1  as an input pin.
  static void SetInput()  { DDRE &= ~(1<<1); }

  /// Pulses PinE1 with high state first.
  static void PulseHigh() { PORTE |= (1<<1); PORTE &= ~(1<<1); }

  /// Pulses PinE1 with low state first.
  static void PulseLow()  { PORTE &= ~(1<<1); PORTE |= (1<<1); }

  /// Reads PinE1  value.
  /// @return Port pin value.
  static bool Test()      { return PINE & (1<<1); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTE
  static constexpr decltype(PORTE) GetNativePort() { return PORTE; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<1)
  static constexpr uint8_t bitmask()               { return (1<<1); }

  /// Returns the bit corresponding to this pin.
  /// @return 1
  static constexpr uint8_t bit()                   { return 1; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortE;
};

struct PinE2 {
  /// Sets PinE2 to HIGH.
  static void Set()       { PORTE |= (1<<2); }

  /// Sets PinE2 to LOW.
  static void Clear()     { PORTE &= ~(1<<2); }

  /// Toggles PinE2 value.
  static void Toggle()    { PINE |= (1<<2); }

  /// Configures PinE2  as an output pin.
  static void SetOutput() { DDRE |= (1<<2); }

  /// Configures PinE2  as an input pin.
  static void SetInput()  { DDRE &= ~(1<<2); }

  /// Pulses PinE2 with high state first.
  static void PulseHigh() { PORTE |= (1<<2); PORTE &= ~(1<<2); }

  /// Pulses PinE2 with low state first.
  static void PulseLow()  { PORTE &= ~(1<<2); PORTE |= (1<<2); }

  /// Reads PinE2  value.
  /// @return Port pin value.
  static bool Test()      { return PINE & (1<<2); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTE
  static constexpr decltype(PORTE) GetNativePort() { return PORTE; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<2)
  static constexpr uint8_t bitmask()               { return (1<<2); }

  /// Returns the bit corresponding to this pin.
  /// @return 2
  static constexpr uint8_t bit()                   { return 2; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortE;
};

struct PinE3 {
  /// Sets PinE3 to HIGH.
  static void Set()       { PORTE |= (1<<3); }

  /// Sets PinE3 to LOW.
  static void Clear()     { PORTE &= ~(1<<3); }

  /// Toggles PinE3 value.
  static void Toggle()    { PINE |= (1<<3); }

  /// Configures PinE3  as an output pin.
  static void SetOutput() { DDRE |= (1<<3); }

  /// Configures PinE3  as an input pin.
  static void SetInput()  { DDRE &= ~(1<<3); }

  /// Pulses PinE3 with high state first.
  static void PulseHigh() { PORTE |= (1<<3); PORTE &= ~(1<<3); }

  /// Pulses PinE3 with low state first.
  static void PulseLow()  { PORTE &= ~(1<<3); PORTE |= (1<<3); }

  /// Reads PinE3  value.
  /// @return Port pin value.
  static bool Test()      { return PINE & (1<<3); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTE
  static constexpr decltype(PORTE) GetNativePort() { return PORTE; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<3)
  static constexpr uint8_t bitmask()               { return (1<<3); }

  /// Returns the bit corresponding to this pin.
  /// @return 3
  static constexpr uint8_t bit()                   { return 3; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortE;
};

struct PinE4 {
  /// Sets PinE4 to HIGH.
  static void Set()       { PORTE |= (1<<4); }

  /// Sets PinE4 to LOW.
  static void Clear()     { PORTE &= ~(1<<4); }

  /// Toggles PinE4 value.
  static void Toggle()    { PINE |= (1<<4); }

  /// Configures PinE4  as an output pin.
  static void SetOutput() { DDRE |= (1<<4); }

  /// Configures PinE4  as an input pin.
  static void SetInput()  { DDRE &= ~(1<<4); }

  /// Pulses PinE4 with high state first.
  static void PulseHigh() { PORTE |= (1<<4); PORTE &= ~(1<<4); }

  /// Pulses PinE4 with low state first.
  static void PulseLow()  { PORTE &= ~(1<<4); PORTE |= (1<<4); }

  /// Reads PinE4  value.
  /// @return Port pin value.
  static bool Test()      { return PINE & (1<<4); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTE
  static constexpr decltype(PORTE) GetNativePort() { return PORTE; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<4)
  static constexpr uint8_t bitmask()               { return (1<<4); }

  /// Returns the bit corresponding to this pin.
  /// @return 4
  static constexpr uint8_t bit()                   { return 4; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortE;
};

struct PinE5 {
  /// Sets PinE5 to HIGH.
  static void Set()       { PORTE |= (1<<5); }

  /// Sets PinE5 to LOW.
  static void Clear()     { PORTE &= ~(1<<5); }

  /// Toggles PinE5 value.
  static void Toggle()    { PINE |= (1<<5); }

  /// Configures PinE5  as an output pin.
  static void SetOutput() { DDRE |= (1<<5); }

  /// Configures PinE5  as an input pin.
  static void SetInput()  { DDRE &= ~(1<<5); }

  /// Pulses PinE5 with high state first.
  static void PulseHigh() { PORTE |= (1<<5); PORTE &= ~(1<<5); }

  /// Pulses PinE5 with low state first.
  static void PulseLow()  { PORTE &= ~(1<<5); PORTE |= (1<<5); }

  /// Reads PinE5  value.
  /// @return Port pin value.
  static bool Test()      { return PINE & (1<<5); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTE
  static constexpr decltype(PORTE) GetNativePort() { return PORTE; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<5)
  static constexpr uint8_t bitmask()               { return (1<<5); }

  /// Returns the bit corresponding to this pin.
  /// @return 5
  static constexpr uint8_t bit()                   { return 5; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortE;
};

struct PinE6 {
  /// Sets PinE6 to HIGH.
  static void Set()       { PORTE |= (1<<6); }

  /// Sets PinE6 to LOW.
  static void Clear()     { PORTE &= ~(1<<6); }

  /// Toggles PinE6 value.
  static void Toggle()    { PINE |= (1<<6); }

  /// Configures PinE6  as an output pin.
  static void SetOutput() { DDRE |= (1<<6); }

  /// Configures PinE6  as an input pin.
  static void SetInput()  { DDRE &= ~(1<<6); }

  /// Pulses PinE6 with high state first.
  static void PulseHigh() { PORTE |= (1<<6); PORTE &= ~(1<<6); }

  /// Pulses PinE6 with low state first.
  static void PulseLow()  { PORTE &= ~(1<<6); PORTE |= (1<<6); }

  /// Reads PinE6  value.
  /// @return Port pin value.
  static bool Test()      { return PINE & (1<<6); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTE
  static constexpr decltype(PORTE) GetNativePort() { return PORTE; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<6)
  static constexpr uint8_t bitmask()               { return (1<<6); }

  /// Returns the bit corresponding to this pin.
  /// @return 6
  static constexpr uint8_t bit()                   { return 6; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortE;
};

struct PinE7 {
  /// Sets PinE7 to HIGH.
  static void Set()       { PORTE |= (1<<7); }

  /// Sets PinE7 to LOW.
  static void Clear()     { PORTE &= ~(1<<7); }

  /// Toggles PinE7 value.
  static void Toggle()    { PINE |= (1<<7); }

  /// Configures PinE7  as an output pin.
  static void SetOutput() { DDRE |= (1<<7); }

  /// Configures PinE7  as an input pin.
  static void SetInput()  { DDRE &= ~(1<<7); }

  /// Pulses PinE7 with high state first.
  static void PulseHigh() { PORTE |= (1<<7); PORTE &= ~(1<<7); }

  /// Pulses PinE7 with low state first.
  static void PulseLow()  { PORTE &= ~(1<<7); PORTE |= (1<<7); }

  /// Reads PinE7  value.
  /// @return Port pin value.
  static bool Test()      { return PINE & (1<<7); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTE
  static constexpr decltype(PORTE) GetNativePort() { return PORTE; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<7)
  static constexpr uint8_t bitmask()               { return (1<<7); }

  /// Returns the bit corresponding to this pin.
  /// @return 7
  static constexpr uint8_t bit()                   { return 7; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortE;
};

#endif //PORTE

#ifdef PORTF
struct PortF {
  /// Assigns a value to PORTF.
  /// @param[in] value value affected to PORTF
  static void Assign(uint8_t value)   { PORTF = value; }

  /// Sets masked bits in PORTF.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTF |= mask;}

  /// Clears masked bits in PORTF.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTF &= ~mask;} 

  /// Toggles masked bits in PORTF.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTF ^= mask;} 

  /// Pulses masked bits in PORTF with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTF |= mask; PORTF &= ~mask; }

  /// Pulses masked bits in PORTF with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTF &= ~mask; PORTF |= mask; }

  /// Set corresponding masked bits of PORTF to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRF |= mask; }

  /// Set corresponding masked bits of PORTF to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRF &= ~mask; }

  /// Returns PINF register.
  static uint8_t GetPIN()             { return PINF; }

  /// Tests masked bits of PORTF
  /// @param[in] mask bits
  /// @param[in] true if the corresponding bits are all set, false otherwise.
  static bool TestBits(uint8_t mask)  { return (PINF & mask) == mask; }

  /// Returns the value of the bit at the position pos.
  /// @param[in] position of the bit to return
  /// @return true if the requested bit is set, false otherwise.
  static bool Test(uint8_t bit) { return PINF & (1<<bit); }
};

struct PinF0 {
  /// Sets PinF0 to HIGH.
  static void Set()       { PORTF |= (1<<0); }

  /// Sets PinF0 to LOW.
  static void Clear()     { PORTF &= ~(1<<0); }

  /// Toggles PinF0 value.
  static void Toggle()    { PINF |= (1<<0); }

  /// Configures PinF0  as an output pin.
  static void SetOutput() { DDRF |= (1<<0); }

  /// Configures PinF0  as an input pin.
  static void SetInput()  { DDRF &= ~(1<<0); }

  /// Pulses PinF0 with high state first.
  static void PulseHigh() { PORTF |= (1<<0); PORTF &= ~(1<<0); }

  /// Pulses PinF0 with low state first.
  static void PulseLow()  { PORTF &= ~(1<<0); PORTF |= (1<<0); }

  /// Reads PinF0  value.
  /// @return Port pin value.
  static bool Test()      { return PINF & (1<<0); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTF
  static constexpr decltype(PORTF) GetNativePort() { return PORTF; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<0)
  static constexpr uint8_t bitmask()               { return (1<<0); }

  /// Returns the bit corresponding to this pin.
  /// @return 0
  static constexpr uint8_t bit()                   { return 0; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortF;
};

struct PinF1 {
  /// Sets PinF1 to HIGH.
  static void Set()       { PORTF |= (1<<1); }

  /// Sets PinF1 to LOW.
  static void Clear()     { PORTF &= ~(1<<1); }

  /// Toggles PinF1 value.
  static void Toggle()    { PINF |= (1<<1); }

  /// Configures PinF1  as an output pin.
  static void SetOutput() { DDRF |= (1<<1); }

  /// Configures PinF1  as an input pin.
  static void SetInput()  { DDRF &= ~(1<<1); }

  /// Pulses PinF1 with high state first.
  static void PulseHigh() { PORTF |= (1<<1); PORTF &= ~(1<<1); }

  /// Pulses PinF1 with low state first.
  static void PulseLow()  { PORTF &= ~(1<<1); PORTF |= (1<<1); }

  /// Reads PinF1  value.
  /// @return Port pin value.
  static bool Test()      { return PINF & (1<<1); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTF
  static constexpr decltype(PORTF) GetNativePort() { return PORTF; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<1)
  static constexpr uint8_t bitmask()               { return (1<<1); }

  /// Returns the bit corresponding to this pin.
  /// @return 1
  static constexpr uint8_t bit()                   { return 1; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortF;
};

struct PinF2 {
  /// Sets PinF2 to HIGH.
  static void Set()       { PORTF |= (1<<2); }

  /// Sets PinF2 to LOW.
  static void Clear()     { PORTF &= ~(1<<2); }

  /// Toggles PinF2 value.
  static void Toggle()    { PINF |= (1<<2); }

  /// Configures PinF2  as an output pin.
  static void SetOutput() { DDRF |= (1<<2); }

  /// Configures PinF2  as an input pin.
  static void SetInput()  { DDRF &= ~(1<<2); }

  /// Pulses PinF2 with high state first.
  static void PulseHigh() { PORTF |= (1<<2); PORTF &= ~(1<<2); }

  /// Pulses PinF2 with low state first.
  static void PulseLow()  { PORTF &= ~(1<<2); PORTF |= (1<<2); }

  /// Reads PinF2  value.
  /// @return Port pin value.
  static bool Test()      { return PINF & (1<<2); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTF
  static constexpr decltype(PORTF) GetNativePort() { return PORTF; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<2)
  static constexpr uint8_t bitmask()               { return (1<<2); }

  /// Returns the bit corresponding to this pin.
  /// @return 2
  static constexpr uint8_t bit()                   { return 2; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortF;
};

struct PinF3 {
  /// Sets PinF3 to HIGH.
  static void Set()       { PORTF |= (1<<3); }

  /// Sets PinF3 to LOW.
  static void Clear()     { PORTF &= ~(1<<3); }

  /// Toggles PinF3 value.
  static void Toggle()    { PINF |= (1<<3); }

  /// Configures PinF3  as an output pin.
  static void SetOutput() { DDRF |= (1<<3); }

  /// Configures PinF3  as an input pin.
  static void SetInput()  { DDRF &= ~(1<<3); }

  /// Pulses PinF3 with high state first.
  static void PulseHigh() { PORTF |= (1<<3); PORTF &= ~(1<<3); }

  /// Pulses PinF3 with low state first.
  static void PulseLow()  { PORTF &= ~(1<<3); PORTF |= (1<<3); }

  /// Reads PinF3  value.
  /// @return Port pin value.
  static bool Test()      { return PINF & (1<<3); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTF
  static constexpr decltype(PORTF) GetNativePort() { return PORTF; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<3)
  static constexpr uint8_t bitmask()               { return (1<<3); }

  /// Returns the bit corresponding to this pin.
  /// @return 3
  static constexpr uint8_t bit()                   { return 3; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortF;
};

struct PinF4 {
  /// Sets PinF4 to HIGH.
  static void Set()       { PORTF |= (1<<4); }

  /// Sets PinF4 to LOW.
  static void Clear()     { PORTF &= ~(1<<4); }

  /// Toggles PinF4 value.
  static void Toggle()    { PINF |= (1<<4); }

  /// Configures PinF4  as an output pin.
  static void SetOutput() { DDRF |= (1<<4); }

  /// Configures PinF4  as an input pin.
  static void SetInput()  { DDRF &= ~(1<<4); }

  /// Pulses PinF4 with high state first.
  static void PulseHigh() { PORTF |= (1<<4); PORTF &= ~(1<<4); }

  /// Pulses PinF4 with low state first.
  static void PulseLow()  { PORTF &= ~(1<<4); PORTF |= (1<<4); }

  /// Reads PinF4  value.
  /// @return Port pin value.
  static bool Test()      { return PINF & (1<<4); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTF
  static constexpr decltype(PORTF) GetNativePort() { return PORTF; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<4)
  static constexpr uint8_t bitmask()               { return (1<<4); }

  /// Returns the bit corresponding to this pin.
  /// @return 4
  static constexpr uint8_t bit()                   { return 4; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortF;
};

struct PinF5 {
  /// Sets PinF5 to HIGH.
  static void Set()       { PORTF |= (1<<5); }

  /// Sets PinF5 to LOW.
  static void Clear()     { PORTF &= ~(1<<5); }

  /// Toggles PinF5 value.
  static void Toggle()    { PINF |= (1<<5); }

  /// Configures PinF5  as an output pin.
  static void SetOutput() { DDRF |= (1<<5); }

  /// Configures PinF5  as an input pin.
  static void SetInput()  { DDRF &= ~(1<<5); }

  /// Pulses PinF5 with high state first.
  static void PulseHigh() { PORTF |= (1<<5); PORTF &= ~(1<<5); }

  /// Pulses PinF5 with low state first.
  static void PulseLow()  { PORTF &= ~(1<<5); PORTF |= (1<<5); }

  /// Reads PinF5  value.
  /// @return Port pin value.
  static bool Test()      { return PINF & (1<<5); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTF
  static constexpr decltype(PORTF) GetNativePort() { return PORTF; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<5)
  static constexpr uint8_t bitmask()               { return (1<<5); }

  /// Returns the bit corresponding to this pin.
  /// @return 5
  static constexpr uint8_t bit()                   { return 5; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortF;
};

struct PinF6 {
  /// Sets PinF6 to HIGH.
  static void Set()       { PORTF |= (1<<6); }

  /// Sets PinF6 to LOW.
  static void Clear()     { PORTF &= ~(1<<6); }

  /// Toggles PinF6 value.
  static void Toggle()    { PINF |= (1<<6); }

  /// Configures PinF6  as an output pin.
  static void SetOutput() { DDRF |= (1<<6); }

  /// Configures PinF6  as an input pin.
  static void SetInput()  { DDRF &= ~(1<<6); }

  /// Pulses PinF6 with high state first.
  static void PulseHigh() { PORTF |= (1<<6); PORTF &= ~(1<<6); }

  /// Pulses PinF6 with low state first.
  static void PulseLow()  { PORTF &= ~(1<<6); PORTF |= (1<<6); }

  /// Reads PinF6  value.
  /// @return Port pin value.
  static bool Test()      { return PINF & (1<<6); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTF
  static constexpr decltype(PORTF) GetNativePort() { return PORTF; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<6)
  static constexpr uint8_t bitmask()               { return (1<<6); }

  /// Returns the bit corresponding to this pin.
  /// @return 6
  static constexpr uint8_t bit()                   { return 6; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortF;
};

struct PinF7 {
  /// Sets PinF7 to HIGH.
  static void Set()       { PORTF |= (1<<7); }

  /// Sets PinF7 to LOW.
  static void Clear()     { PORTF &= ~(1<<7); }

  /// Toggles PinF7 value.
  static void Toggle()    { PINF |= (1<<7); }

  /// Configures PinF7  as an output pin.
  static void SetOutput() { DDRF |= (1<<7); }

  /// Configures PinF7  as an input pin.
  static void SetInput()  { DDRF &= ~(1<<7); }

  /// Pulses PinF7 with high state first.
  static void PulseHigh() { PORTF |= (1<<7); PORTF &= ~(1<<7); }

  /// Pulses PinF7 with low state first.
  static void PulseLow()  { PORTF &= ~(1<<7); PORTF |= (1<<7); }

  /// Reads PinF7  value.
  /// @return Port pin value.
  static bool Test()      { return PINF & (1<<7); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTF
  static constexpr decltype(PORTF) GetNativePort() { return PORTF; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<7)
  static constexpr uint8_t bitmask()               { return (1<<7); }

  /// Returns the bit corresponding to this pin.
  /// @return 7
  static constexpr uint8_t bit()                   { return 7; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortF;
};

#endif //PORTF

#ifdef PORTG
struct PortG {
  /// Assigns a value to PORTG.
  /// @param[in] value value affected to PORTG
  static void Assign(uint8_t value)   { PORTG = value; }

  /// Sets masked bits in PORTG.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTG |= mask;}

  /// Clears masked bits in PORTG.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTG &= ~mask;} 

  /// Toggles masked bits in PORTG.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTG ^= mask;} 

  /// Pulses masked bits in PORTG with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTG |= mask; PORTG &= ~mask; }

  /// Pulses masked bits in PORTG with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTG &= ~mask; PORTG |= mask; }

  /// Set corresponding masked bits of PORTG to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRG |= mask; }

  /// Set corresponding masked bits of PORTG to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRG &= ~mask; }

  /// Returns PING register.
  static uint8_t GetPIN()             { return PING; }

  /// Tests masked bits of PORTG
  /// @param[in] mask bits
  /// @param[in] true if the corresponding bits are all set, false otherwise.
  static bool TestBits(uint8_t mask)  { return (PING & mask) == mask; }

  /// Returns the value of the bit at the position pos.
  /// @param[in] position of the bit to return
  /// @return true if the requested bit is set, false otherwise.
  static bool Test(uint8_t bit) { return PING & (1<<bit); }
};

struct PinG0 {
  /// Sets PinG0 to HIGH.
  static void Set()       { PORTG |= (1<<0); }

  /// Sets PinG0 to LOW.
  static void Clear()     { PORTG &= ~(1<<0); }

  /// Toggles PinG0 value.
  static void Toggle()    { PING |= (1<<0); }

  /// Configures PinG0  as an output pin.
  static void SetOutput() { DDRG |= (1<<0); }

  /// Configures PinG0  as an input pin.
  static void SetInput()  { DDRG &= ~(1<<0); }

  /// Pulses PinG0 with high state first.
  static void PulseHigh() { PORTG |= (1<<0); PORTG &= ~(1<<0); }

  /// Pulses PinG0 with low state first.
  static void PulseLow()  { PORTG &= ~(1<<0); PORTG |= (1<<0); }

  /// Reads PinG0  value.
  /// @return Port pin value.
  static bool Test()      { return PING & (1<<0); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTG
  static constexpr decltype(PORTG) GetNativePort() { return PORTG; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<0)
  static constexpr uint8_t bitmask()               { return (1<<0); }

  /// Returns the bit corresponding to this pin.
  /// @return 0
  static constexpr uint8_t bit()                   { return 0; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortG;
};

struct PinG1 {
  /// Sets PinG1 to HIGH.
  static void Set()       { PORTG |= (1<<1); }

  /// Sets PinG1 to LOW.
  static void Clear()     { PORTG &= ~(1<<1); }

  /// Toggles PinG1 value.
  static void Toggle()    { PING |= (1<<1); }

  /// Configures PinG1  as an output pin.
  static void SetOutput() { DDRG |= (1<<1); }

  /// Configures PinG1  as an input pin.
  static void SetInput()  { DDRG &= ~(1<<1); }

  /// Pulses PinG1 with high state first.
  static void PulseHigh() { PORTG |= (1<<1); PORTG &= ~(1<<1); }

  /// Pulses PinG1 with low state first.
  static void PulseLow()  { PORTG &= ~(1<<1); PORTG |= (1<<1); }

  /// Reads PinG1  value.
  /// @return Port pin value.
  static bool Test()      { return PING & (1<<1); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTG
  static constexpr decltype(PORTG) GetNativePort() { return PORTG; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<1)
  static constexpr uint8_t bitmask()               { return (1<<1); }

  /// Returns the bit corresponding to this pin.
  /// @return 1
  static constexpr uint8_t bit()                   { return 1; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortG;
};

struct PinG2 {
  /// Sets PinG2 to HIGH.
  static void Set()       { PORTG |= (1<<2); }

  /// Sets PinG2 to LOW.
  static void Clear()     { PORTG &= ~(1<<2); }

  /// Toggles PinG2 value.
  static void Toggle()    { PING |= (1<<2); }

  /// Configures PinG2  as an output pin.
  static void SetOutput() { DDRG |= (1<<2); }

  /// Configures PinG2  as an input pin.
  static void SetInput()  { DDRG &= ~(1<<2); }

  /// Pulses PinG2 with high state first.
  static void PulseHigh() { PORTG |= (1<<2); PORTG &= ~(1<<2); }

  /// Pulses PinG2 with low state first.
  static void PulseLow()  { PORTG &= ~(1<<2); PORTG |= (1<<2); }

  /// Reads PinG2  value.
  /// @return Port pin value.
  static bool Test()      { return PING & (1<<2); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTG
  static constexpr decltype(PORTG) GetNativePort() { return PORTG; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<2)
  static constexpr uint8_t bitmask()               { return (1<<2); }

  /// Returns the bit corresponding to this pin.
  /// @return 2
  static constexpr uint8_t bit()                   { return 2; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortG;
};

struct PinG3 {
  /// Sets PinG3 to HIGH.
  static void Set()       { PORTG |= (1<<3); }

  /// Sets PinG3 to LOW.
  static void Clear()     { PORTG &= ~(1<<3); }

  /// Toggles PinG3 value.
  static void Toggle()    { PING |= (1<<3); }

  /// Configures PinG3  as an output pin.
  static void SetOutput() { DDRG |= (1<<3); }

  /// Configures PinG3  as an input pin.
  static void SetInput()  { DDRG &= ~(1<<3); }

  /// Pulses PinG3 with high state first.
  static void PulseHigh() { PORTG |= (1<<3); PORTG &= ~(1<<3); }

  /// Pulses PinG3 with low state first.
  static void PulseLow()  { PORTG &= ~(1<<3); PORTG |= (1<<3); }

  /// Reads PinG3  value.
  /// @return Port pin value.
  static bool Test()      { return PING & (1<<3); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTG
  static constexpr decltype(PORTG) GetNativePort() { return PORTG; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<3)
  static constexpr uint8_t bitmask()               { return (1<<3); }

  /// Returns the bit corresponding to this pin.
  /// @return 3
  static constexpr uint8_t bit()                   { return 3; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortG;
};

struct PinG4 {
  /// Sets PinG4 to HIGH.
  static void Set()       { PORTG |= (1<<4); }

  /// Sets PinG4 to LOW.
  static void Clear()     { PORTG &= ~(1<<4); }

  /// Toggles PinG4 value.
  static void Toggle()    { PING |= (1<<4); }

  /// Configures PinG4  as an output pin.
  static void SetOutput() { DDRG |= (1<<4); }

  /// Configures PinG4  as an input pin.
  static void SetInput()  { DDRG &= ~(1<<4); }

  /// Pulses PinG4 with high state first.
  static void PulseHigh() { PORTG |= (1<<4); PORTG &= ~(1<<4); }

  /// Pulses PinG4 with low state first.
  static void PulseLow()  { PORTG &= ~(1<<4); PORTG |= (1<<4); }

  /// Reads PinG4  value.
  /// @return Port pin value.
  static bool Test()      { return PING & (1<<4); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTG
  static constexpr decltype(PORTG) GetNativePort() { return PORTG; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<4)
  static constexpr uint8_t bitmask()               { return (1<<4); }

  /// Returns the bit corresponding to this pin.
  /// @return 4
  static constexpr uint8_t bit()                   { return 4; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortG;
};

struct PinG5 {
  /// Sets PinG5 to HIGH.
  static void Set()       { PORTG |= (1<<5); }

  /// Sets PinG5 to LOW.
  static void Clear()     { PORTG &= ~(1<<5); }

  /// Toggles PinG5 value.
  static void Toggle()    { PING |= (1<<5); }

  /// Configures PinG5  as an output pin.
  static void SetOutput() { DDRG |= (1<<5); }

  /// Configures PinG5  as an input pin.
  static void SetInput()  { DDRG &= ~(1<<5); }

  /// Pulses PinG5 with high state first.
  static void PulseHigh() { PORTG |= (1<<5); PORTG &= ~(1<<5); }

  /// Pulses PinG5 with low state first.
  static void PulseLow()  { PORTG &= ~(1<<5); PORTG |= (1<<5); }

  /// Reads PinG5  value.
  /// @return Port pin value.
  static bool Test()      { return PING & (1<<5); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTG
  static constexpr decltype(PORTG) GetNativePort() { return PORTG; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<5)
  static constexpr uint8_t bitmask()               { return (1<<5); }

  /// Returns the bit corresponding to this pin.
  /// @return 5
  static constexpr uint8_t bit()                   { return 5; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortG;
};

struct PinG6 {
  /// Sets PinG6 to HIGH.
  static void Set()       { PORTG |= (1<<6); }

  /// Sets PinG6 to LOW.
  static void Clear()     { PORTG &= ~(1<<6); }

  /// Toggles PinG6 value.
  static void Toggle()    { PING |= (1<<6); }

  /// Configures PinG6  as an output pin.
  static void SetOutput() { DDRG |= (1<<6); }

  /// Configures PinG6  as an input pin.
  static void SetInput()  { DDRG &= ~(1<<6); }

  /// Pulses PinG6 with high state first.
  static void PulseHigh() { PORTG |= (1<<6); PORTG &= ~(1<<6); }

  /// Pulses PinG6 with low state first.
  static void PulseLow()  { PORTG &= ~(1<<6); PORTG |= (1<<6); }

  /// Reads PinG6  value.
  /// @return Port pin value.
  static bool Test()      { return PING & (1<<6); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTG
  static constexpr decltype(PORTG) GetNativePort() { return PORTG; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<6)
  static constexpr uint8_t bitmask()               { return (1<<6); }

  /// Returns the bit corresponding to this pin.
  /// @return 6
  static constexpr uint8_t bit()                   { return 6; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortG;
};

struct PinG7 {
  /// Sets PinG7 to HIGH.
  static void Set()       { PORTG |= (1<<7); }

  /// Sets PinG7 to LOW.
  static void Clear()     { PORTG &= ~(1<<7); }

  /// Toggles PinG7 value.
  static void Toggle()    { PING |= (1<<7); }

  /// Configures PinG7  as an output pin.
  static void SetOutput() { DDRG |= (1<<7); }

  /// Configures PinG7  as an input pin.
  static void SetInput()  { DDRG &= ~(1<<7); }

  /// Pulses PinG7 with high state first.
  static void PulseHigh() { PORTG |= (1<<7); PORTG &= ~(1<<7); }

  /// Pulses PinG7 with low state first.
  static void PulseLow()  { PORTG &= ~(1<<7); PORTG |= (1<<7); }

  /// Reads PinG7  value.
  /// @return Port pin value.
  static bool Test()      { return PING & (1<<7); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTG
  static constexpr decltype(PORTG) GetNativePort() { return PORTG; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<7)
  static constexpr uint8_t bitmask()               { return (1<<7); }

  /// Returns the bit corresponding to this pin.
  /// @return 7
  static constexpr uint8_t bit()                   { return 7; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortG;
};

#endif //PORTG

#ifdef PORTH
struct PortH {
  /// Assigns a value to PORTH.
  /// @param[in] value value affected to PORTH
  static void Assign(uint8_t value)   { PORTH = value; }

  /// Sets masked bits in PORTH.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTH |= mask;}

  /// Clears masked bits in PORTH.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTH &= ~mask;} 

  /// Toggles masked bits in PORTH.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTH ^= mask;} 

  /// Pulses masked bits in PORTH with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTH |= mask; PORTH &= ~mask; }

  /// Pulses masked bits in PORTH with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTH &= ~mask; PORTH |= mask; }

  /// Set corresponding masked bits of PORTH to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRH |= mask; }

  /// Set corresponding masked bits of PORTH to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRH &= ~mask; }

  /// Returns PINH register.
  static uint8_t GetPIN()             { return PINH; }

  /// Tests masked bits of PORTH
  /// @param[in] mask bits
  /// @param[in] true if the corresponding bits are all set, false otherwise.
  static bool TestBits(uint8_t mask)  { return (PINH & mask) == mask; }

  /// Returns the value of the bit at the position pos.
  /// @param[in] position of the bit to return
  /// @return true if the requested bit is set, false otherwise.
  static bool Test(uint8_t bit) { return PINH & (1<<bit); }
};

struct PinH0 {
  /// Sets PinH0 to HIGH.
  static void Set()       { PORTH |= (1<<0); }

  /// Sets PinH0 to LOW.
  static void Clear()     { PORTH &= ~(1<<0); }

  /// Toggles PinH0 value.
  static void Toggle()    { PINH |= (1<<0); }

  /// Configures PinH0  as an output pin.
  static void SetOutput() { DDRH |= (1<<0); }

  /// Configures PinH0  as an input pin.
  static void SetInput()  { DDRH &= ~(1<<0); }

  /// Pulses PinH0 with high state first.
  static void PulseHigh() { PORTH |= (1<<0); PORTH &= ~(1<<0); }

  /// Pulses PinH0 with low state first.
  static void PulseLow()  { PORTH &= ~(1<<0); PORTH |= (1<<0); }

  /// Reads PinH0  value.
  /// @return Port pin value.
  static bool Test()      { return PINH & (1<<0); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTH
  static constexpr decltype(PORTH) GetNativePort() { return PORTH; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<0)
  static constexpr uint8_t bitmask()               { return (1<<0); }

  /// Returns the bit corresponding to this pin.
  /// @return 0
  static constexpr uint8_t bit()                   { return 0; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortH;
};

struct PinH1 {
  /// Sets PinH1 to HIGH.
  static void Set()       { PORTH |= (1<<1); }

  /// Sets PinH1 to LOW.
  static void Clear()     { PORTH &= ~(1<<1); }

  /// Toggles PinH1 value.
  static void Toggle()    { PINH |= (1<<1); }

  /// Configures PinH1  as an output pin.
  static void SetOutput() { DDRH |= (1<<1); }

  /// Configures PinH1  as an input pin.
  static void SetInput()  { DDRH &= ~(1<<1); }

  /// Pulses PinH1 with high state first.
  static void PulseHigh() { PORTH |= (1<<1); PORTH &= ~(1<<1); }

  /// Pulses PinH1 with low state first.
  static void PulseLow()  { PORTH &= ~(1<<1); PORTH |= (1<<1); }

  /// Reads PinH1  value.
  /// @return Port pin value.
  static bool Test()      { return PINH & (1<<1); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTH
  static constexpr decltype(PORTH) GetNativePort() { return PORTH; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<1)
  static constexpr uint8_t bitmask()               { return (1<<1); }

  /// Returns the bit corresponding to this pin.
  /// @return 1
  static constexpr uint8_t bit()                   { return 1; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortH;
};

struct PinH2 {
  /// Sets PinH2 to HIGH.
  static void Set()       { PORTH |= (1<<2); }

  /// Sets PinH2 to LOW.
  static void Clear()     { PORTH &= ~(1<<2); }

  /// Toggles PinH2 value.
  static void Toggle()    { PINH |= (1<<2); }

  /// Configures PinH2  as an output pin.
  static void SetOutput() { DDRH |= (1<<2); }

  /// Configures PinH2  as an input pin.
  static void SetInput()  { DDRH &= ~(1<<2); }

  /// Pulses PinH2 with high state first.
  static void PulseHigh() { PORTH |= (1<<2); PORTH &= ~(1<<2); }

  /// Pulses PinH2 with low state first.
  static void PulseLow()  { PORTH &= ~(1<<2); PORTH |= (1<<2); }

  /// Reads PinH2  value.
  /// @return Port pin value.
  static bool Test()      { return PINH & (1<<2); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTH
  static constexpr decltype(PORTH) GetNativePort() { return PORTH; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<2)
  static constexpr uint8_t bitmask()               { return (1<<2); }

  /// Returns the bit corresponding to this pin.
  /// @return 2
  static constexpr uint8_t bit()                   { return 2; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortH;
};

struct PinH3 {
  /// Sets PinH3 to HIGH.
  static void Set()       { PORTH |= (1<<3); }

  /// Sets PinH3 to LOW.
  static void Clear()     { PORTH &= ~(1<<3); }

  /// Toggles PinH3 value.
  static void Toggle()    { PINH |= (1<<3); }

  /// Configures PinH3  as an output pin.
  static void SetOutput() { DDRH |= (1<<3); }

  /// Configures PinH3  as an input pin.
  static void SetInput()  { DDRH &= ~(1<<3); }

  /// Pulses PinH3 with high state first.
  static void PulseHigh() { PORTH |= (1<<3); PORTH &= ~(1<<3); }

  /// Pulses PinH3 with low state first.
  static void PulseLow()  { PORTH &= ~(1<<3); PORTH |= (1<<3); }

  /// Reads PinH3  value.
  /// @return Port pin value.
  static bool Test()      { return PINH & (1<<3); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTH
  static constexpr decltype(PORTH) GetNativePort() { return PORTH; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<3)
  static constexpr uint8_t bitmask()               { return (1<<3); }

  /// Returns the bit corresponding to this pin.
  /// @return 3
  static constexpr uint8_t bit()                   { return 3; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortH;
};

struct PinH4 {
  /// Sets PinH4 to HIGH.
  static void Set()       { PORTH |= (1<<4); }

  /// Sets PinH4 to LOW.
  static void Clear()     { PORTH &= ~(1<<4); }

  /// Toggles PinH4 value.
  static void Toggle()    { PINH |= (1<<4); }

  /// Configures PinH4  as an output pin.
  static void SetOutput() { DDRH |= (1<<4); }

  /// Configures PinH4  as an input pin.
  static void SetInput()  { DDRH &= ~(1<<4); }

  /// Pulses PinH4 with high state first.
  static void PulseHigh() { PORTH |= (1<<4); PORTH &= ~(1<<4); }

  /// Pulses PinH4 with low state first.
  static void PulseLow()  { PORTH &= ~(1<<4); PORTH |= (1<<4); }

  /// Reads PinH4  value.
  /// @return Port pin value.
  static bool Test()      { return PINH & (1<<4); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTH
  static constexpr decltype(PORTH) GetNativePort() { return PORTH; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<4)
  static constexpr uint8_t bitmask()               { return (1<<4); }

  /// Returns the bit corresponding to this pin.
  /// @return 4
  static constexpr uint8_t bit()                   { return 4; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortH;
};

struct PinH5 {
  /// Sets PinH5 to HIGH.
  static void Set()       { PORTH |= (1<<5); }

  /// Sets PinH5 to LOW.
  static void Clear()     { PORTH &= ~(1<<5); }

  /// Toggles PinH5 value.
  static void Toggle()    { PINH |= (1<<5); }

  /// Configures PinH5  as an output pin.
  static void SetOutput() { DDRH |= (1<<5); }

  /// Configures PinH5  as an input pin.
  static void SetInput()  { DDRH &= ~(1<<5); }

  /// Pulses PinH5 with high state first.
  static void PulseHigh() { PORTH |= (1<<5); PORTH &= ~(1<<5); }

  /// Pulses PinH5 with low state first.
  static void PulseLow()  { PORTH &= ~(1<<5); PORTH |= (1<<5); }

  /// Reads PinH5  value.
  /// @return Port pin value.
  static bool Test()      { return PINH & (1<<5); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTH
  static constexpr decltype(PORTH) GetNativePort() { return PORTH; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<5)
  static constexpr uint8_t bitmask()               { return (1<<5); }

  /// Returns the bit corresponding to this pin.
  /// @return 5
  static constexpr uint8_t bit()                   { return 5; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortH;
};

struct PinH6 {
  /// Sets PinH6 to HIGH.
  static void Set()       { PORTH |= (1<<6); }

  /// Sets PinH6 to LOW.
  static void Clear()     { PORTH &= ~(1<<6); }

  /// Toggles PinH6 value.
  static void Toggle()    { PINH |= (1<<6); }

  /// Configures PinH6  as an output pin.
  static void SetOutput() { DDRH |= (1<<6); }

  /// Configures PinH6  as an input pin.
  static void SetInput()  { DDRH &= ~(1<<6); }

  /// Pulses PinH6 with high state first.
  static void PulseHigh() { PORTH |= (1<<6); PORTH &= ~(1<<6); }

  /// Pulses PinH6 with low state first.
  static void PulseLow()  { PORTH &= ~(1<<6); PORTH |= (1<<6); }

  /// Reads PinH6  value.
  /// @return Port pin value.
  static bool Test()      { return PINH & (1<<6); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTH
  static constexpr decltype(PORTH) GetNativePort() { return PORTH; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<6)
  static constexpr uint8_t bitmask()               { return (1<<6); }

  /// Returns the bit corresponding to this pin.
  /// @return 6
  static constexpr uint8_t bit()                   { return 6; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortH;
};

struct PinH7 {
  /// Sets PinH7 to HIGH.
  static void Set()       { PORTH |= (1<<7); }

  /// Sets PinH7 to LOW.
  static void Clear()     { PORTH &= ~(1<<7); }

  /// Toggles PinH7 value.
  static void Toggle()    { PINH |= (1<<7); }

  /// Configures PinH7  as an output pin.
  static void SetOutput() { DDRH |= (1<<7); }

  /// Configures PinH7  as an input pin.
  static void SetInput()  { DDRH &= ~(1<<7); }

  /// Pulses PinH7 with high state first.
  static void PulseHigh() { PORTH |= (1<<7); PORTH &= ~(1<<7); }

  /// Pulses PinH7 with low state first.
  static void PulseLow()  { PORTH &= ~(1<<7); PORTH |= (1<<7); }

  /// Reads PinH7  value.
  /// @return Port pin value.
  static bool Test()      { return PINH & (1<<7); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTH
  static constexpr decltype(PORTH) GetNativePort() { return PORTH; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<7)
  static constexpr uint8_t bitmask()               { return (1<<7); }

  /// Returns the bit corresponding to this pin.
  /// @return 7
  static constexpr uint8_t bit()                   { return 7; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortH;
};

#endif //PORTH

#ifdef PORTI
struct PortI {
  /// Assigns a value to PORTI.
  /// @param[in] value value affected to PORTI
  static void Assign(uint8_t value)   { PORTI = value; }

  /// Sets masked bits in PORTI.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTI |= mask;}

  /// Clears masked bits in PORTI.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTI &= ~mask;} 

  /// Toggles masked bits in PORTI.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTI ^= mask;} 

  /// Pulses masked bits in PORTI with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTI |= mask; PORTI &= ~mask; }

  /// Pulses masked bits in PORTI with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTI &= ~mask; PORTI |= mask; }

  /// Set corresponding masked bits of PORTI to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRI |= mask; }

  /// Set corresponding masked bits of PORTI to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRI &= ~mask; }

  /// Returns PINI register.
  static uint8_t GetPIN()             { return PINI; }

  /// Tests masked bits of PORTI
  /// @param[in] mask bits
  /// @param[in] true if the corresponding bits are all set, false otherwise.
  static bool TestBits(uint8_t mask)  { return (PINI & mask) == mask; }

  /// Returns the value of the bit at the position pos.
  /// @param[in] position of the bit to return
  /// @return true if the requested bit is set, false otherwise.
  static bool Test(uint8_t bit) { return PINI & (1<<bit); }
};

struct PinI0 {
  /// Sets PinI0 to HIGH.
  static void Set()       { PORTI |= (1<<0); }

  /// Sets PinI0 to LOW.
  static void Clear()     { PORTI &= ~(1<<0); }

  /// Toggles PinI0 value.
  static void Toggle()    { PINI |= (1<<0); }

  /// Configures PinI0  as an output pin.
  static void SetOutput() { DDRI |= (1<<0); }

  /// Configures PinI0  as an input pin.
  static void SetInput()  { DDRI &= ~(1<<0); }

  /// Pulses PinI0 with high state first.
  static void PulseHigh() { PORTI |= (1<<0); PORTI &= ~(1<<0); }

  /// Pulses PinI0 with low state first.
  static void PulseLow()  { PORTI &= ~(1<<0); PORTI |= (1<<0); }

  /// Reads PinI0  value.
  /// @return Port pin value.
  static bool Test()      { return PINI & (1<<0); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTI
  static constexpr decltype(PORTI) GetNativePort() { return PORTI; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<0)
  static constexpr uint8_t bitmask()               { return (1<<0); }

  /// Returns the bit corresponding to this pin.
  /// @return 0
  static constexpr uint8_t bit()                   { return 0; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortI;
};

struct PinI1 {
  /// Sets PinI1 to HIGH.
  static void Set()       { PORTI |= (1<<1); }

  /// Sets PinI1 to LOW.
  static void Clear()     { PORTI &= ~(1<<1); }

  /// Toggles PinI1 value.
  static void Toggle()    { PINI |= (1<<1); }

  /// Configures PinI1  as an output pin.
  static void SetOutput() { DDRI |= (1<<1); }

  /// Configures PinI1  as an input pin.
  static void SetInput()  { DDRI &= ~(1<<1); }

  /// Pulses PinI1 with high state first.
  static void PulseHigh() { PORTI |= (1<<1); PORTI &= ~(1<<1); }

  /// Pulses PinI1 with low state first.
  static void PulseLow()  { PORTI &= ~(1<<1); PORTI |= (1<<1); }

  /// Reads PinI1  value.
  /// @return Port pin value.
  static bool Test()      { return PINI & (1<<1); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTI
  static constexpr decltype(PORTI) GetNativePort() { return PORTI; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<1)
  static constexpr uint8_t bitmask()               { return (1<<1); }

  /// Returns the bit corresponding to this pin.
  /// @return 1
  static constexpr uint8_t bit()                   { return 1; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortI;
};

struct PinI2 {
  /// Sets PinI2 to HIGH.
  static void Set()       { PORTI |= (1<<2); }

  /// Sets PinI2 to LOW.
  static void Clear()     { PORTI &= ~(1<<2); }

  /// Toggles PinI2 value.
  static void Toggle()    { PINI |= (1<<2); }

  /// Configures PinI2  as an output pin.
  static void SetOutput() { DDRI |= (1<<2); }

  /// Configures PinI2  as an input pin.
  static void SetInput()  { DDRI &= ~(1<<2); }

  /// Pulses PinI2 with high state first.
  static void PulseHigh() { PORTI |= (1<<2); PORTI &= ~(1<<2); }

  /// Pulses PinI2 with low state first.
  static void PulseLow()  { PORTI &= ~(1<<2); PORTI |= (1<<2); }

  /// Reads PinI2  value.
  /// @return Port pin value.
  static bool Test()      { return PINI & (1<<2); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTI
  static constexpr decltype(PORTI) GetNativePort() { return PORTI; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<2)
  static constexpr uint8_t bitmask()               { return (1<<2); }

  /// Returns the bit corresponding to this pin.
  /// @return 2
  static constexpr uint8_t bit()                   { return 2; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortI;
};

struct PinI3 {
  /// Sets PinI3 to HIGH.
  static void Set()       { PORTI |= (1<<3); }

  /// Sets PinI3 to LOW.
  static void Clear()     { PORTI &= ~(1<<3); }

  /// Toggles PinI3 value.
  static void Toggle()    { PINI |= (1<<3); }

  /// Configures PinI3  as an output pin.
  static void SetOutput() { DDRI |= (1<<3); }

  /// Configures PinI3  as an input pin.
  static void SetInput()  { DDRI &= ~(1<<3); }

  /// Pulses PinI3 with high state first.
  static void PulseHigh() { PORTI |= (1<<3); PORTI &= ~(1<<3); }

  /// Pulses PinI3 with low state first.
  static void PulseLow()  { PORTI &= ~(1<<3); PORTI |= (1<<3); }

  /// Reads PinI3  value.
  /// @return Port pin value.
  static bool Test()      { return PINI & (1<<3); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTI
  static constexpr decltype(PORTI) GetNativePort() { return PORTI; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<3)
  static constexpr uint8_t bitmask()               { return (1<<3); }

  /// Returns the bit corresponding to this pin.
  /// @return 3
  static constexpr uint8_t bit()                   { return 3; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortI;
};

struct PinI4 {
  /// Sets PinI4 to HIGH.
  static void Set()       { PORTI |= (1<<4); }

  /// Sets PinI4 to LOW.
  static void Clear()     { PORTI &= ~(1<<4); }

  /// Toggles PinI4 value.
  static void Toggle()    { PINI |= (1<<4); }

  /// Configures PinI4  as an output pin.
  static void SetOutput() { DDRI |= (1<<4); }

  /// Configures PinI4  as an input pin.
  static void SetInput()  { DDRI &= ~(1<<4); }

  /// Pulses PinI4 with high state first.
  static void PulseHigh() { PORTI |= (1<<4); PORTI &= ~(1<<4); }

  /// Pulses PinI4 with low state first.
  static void PulseLow()  { PORTI &= ~(1<<4); PORTI |= (1<<4); }

  /// Reads PinI4  value.
  /// @return Port pin value.
  static bool Test()      { return PINI & (1<<4); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTI
  static constexpr decltype(PORTI) GetNativePort() { return PORTI; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<4)
  static constexpr uint8_t bitmask()               { return (1<<4); }

  /// Returns the bit corresponding to this pin.
  /// @return 4
  static constexpr uint8_t bit()                   { return 4; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortI;
};

struct PinI5 {
  /// Sets PinI5 to HIGH.
  static void Set()       { PORTI |= (1<<5); }

  /// Sets PinI5 to LOW.
  static void Clear()     { PORTI &= ~(1<<5); }

  /// Toggles PinI5 value.
  static void Toggle()    { PINI |= (1<<5); }

  /// Configures PinI5  as an output pin.
  static void SetOutput() { DDRI |= (1<<5); }

  /// Configures PinI5  as an input pin.
  static void SetInput()  { DDRI &= ~(1<<5); }

  /// Pulses PinI5 with high state first.
  static void PulseHigh() { PORTI |= (1<<5); PORTI &= ~(1<<5); }

  /// Pulses PinI5 with low state first.
  static void PulseLow()  { PORTI &= ~(1<<5); PORTI |= (1<<5); }

  /// Reads PinI5  value.
  /// @return Port pin value.
  static bool Test()      { return PINI & (1<<5); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTI
  static constexpr decltype(PORTI) GetNativePort() { return PORTI; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<5)
  static constexpr uint8_t bitmask()               { return (1<<5); }

  /// Returns the bit corresponding to this pin.
  /// @return 5
  static constexpr uint8_t bit()                   { return 5; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortI;
};

struct PinI6 {
  /// Sets PinI6 to HIGH.
  static void Set()       { PORTI |= (1<<6); }

  /// Sets PinI6 to LOW.
  static void Clear()     { PORTI &= ~(1<<6); }

  /// Toggles PinI6 value.
  static void Toggle()    { PINI |= (1<<6); }

  /// Configures PinI6  as an output pin.
  static void SetOutput() { DDRI |= (1<<6); }

  /// Configures PinI6  as an input pin.
  static void SetInput()  { DDRI &= ~(1<<6); }

  /// Pulses PinI6 with high state first.
  static void PulseHigh() { PORTI |= (1<<6); PORTI &= ~(1<<6); }

  /// Pulses PinI6 with low state first.
  static void PulseLow()  { PORTI &= ~(1<<6); PORTI |= (1<<6); }

  /// Reads PinI6  value.
  /// @return Port pin value.
  static bool Test()      { return PINI & (1<<6); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTI
  static constexpr decltype(PORTI) GetNativePort() { return PORTI; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<6)
  static constexpr uint8_t bitmask()               { return (1<<6); }

  /// Returns the bit corresponding to this pin.
  /// @return 6
  static constexpr uint8_t bit()                   { return 6; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortI;
};

struct PinI7 {
  /// Sets PinI7 to HIGH.
  static void Set()       { PORTI |= (1<<7); }

  /// Sets PinI7 to LOW.
  static void Clear()     { PORTI &= ~(1<<7); }

  /// Toggles PinI7 value.
  static void Toggle()    { PINI |= (1<<7); }

  /// Configures PinI7  as an output pin.
  static void SetOutput() { DDRI |= (1<<7); }

  /// Configures PinI7  as an input pin.
  static void SetInput()  { DDRI &= ~(1<<7); }

  /// Pulses PinI7 with high state first.
  static void PulseHigh() { PORTI |= (1<<7); PORTI &= ~(1<<7); }

  /// Pulses PinI7 with low state first.
  static void PulseLow()  { PORTI &= ~(1<<7); PORTI |= (1<<7); }

  /// Reads PinI7  value.
  /// @return Port pin value.
  static bool Test()      { return PINI & (1<<7); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTI
  static constexpr decltype(PORTI) GetNativePort() { return PORTI; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<7)
  static constexpr uint8_t bitmask()               { return (1<<7); }

  /// Returns the bit corresponding to this pin.
  /// @return 7
  static constexpr uint8_t bit()                   { return 7; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortI;
};

#endif //PORTI

#ifdef PORTJ
struct PortJ {
  /// Assigns a value to PORTJ.
  /// @param[in] value value affected to PORTJ
  static void Assign(uint8_t value)   { PORTJ = value; }

  /// Sets masked bits in PORTJ.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTJ |= mask;}

  /// Clears masked bits in PORTJ.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTJ &= ~mask;} 

  /// Toggles masked bits in PORTJ.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTJ ^= mask;} 

  /// Pulses masked bits in PORTJ with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTJ |= mask; PORTJ &= ~mask; }

  /// Pulses masked bits in PORTJ with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTJ &= ~mask; PORTJ |= mask; }

  /// Set corresponding masked bits of PORTJ to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRJ |= mask; }

  /// Set corresponding masked bits of PORTJ to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRJ &= ~mask; }

  /// Returns PINJ register.
  static uint8_t GetPIN()             { return PINJ; }

  /// Tests masked bits of PORTJ
  /// @param[in] mask bits
  /// @param[in] true if the corresponding bits are all set, false otherwise.
  static bool TestBits(uint8_t mask)  { return (PINJ & mask) == mask; }

  /// Returns the value of the bit at the position pos.
  /// @param[in] position of the bit to return
  /// @return true if the requested bit is set, false otherwise.
  static bool Test(uint8_t bit) { return PINJ & (1<<bit); }
};

struct PinJ0 {
  /// Sets PinJ0 to HIGH.
  static void Set()       { PORTJ |= (1<<0); }

  /// Sets PinJ0 to LOW.
  static void Clear()     { PORTJ &= ~(1<<0); }

  /// Toggles PinJ0 value.
  static void Toggle()    { PINJ |= (1<<0); }

  /// Configures PinJ0  as an output pin.
  static void SetOutput() { DDRJ |= (1<<0); }

  /// Configures PinJ0  as an input pin.
  static void SetInput()  { DDRJ &= ~(1<<0); }

  /// Pulses PinJ0 with high state first.
  static void PulseHigh() { PORTJ |= (1<<0); PORTJ &= ~(1<<0); }

  /// Pulses PinJ0 with low state first.
  static void PulseLow()  { PORTJ &= ~(1<<0); PORTJ |= (1<<0); }

  /// Reads PinJ0  value.
  /// @return Port pin value.
  static bool Test()      { return PINJ & (1<<0); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTJ
  static constexpr decltype(PORTJ) GetNativePort() { return PORTJ; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<0)
  static constexpr uint8_t bitmask()               { return (1<<0); }

  /// Returns the bit corresponding to this pin.
  /// @return 0
  static constexpr uint8_t bit()                   { return 0; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortJ;
};

struct PinJ1 {
  /// Sets PinJ1 to HIGH.
  static void Set()       { PORTJ |= (1<<1); }

  /// Sets PinJ1 to LOW.
  static void Clear()     { PORTJ &= ~(1<<1); }

  /// Toggles PinJ1 value.
  static void Toggle()    { PINJ |= (1<<1); }

  /// Configures PinJ1  as an output pin.
  static void SetOutput() { DDRJ |= (1<<1); }

  /// Configures PinJ1  as an input pin.
  static void SetInput()  { DDRJ &= ~(1<<1); }

  /// Pulses PinJ1 with high state first.
  static void PulseHigh() { PORTJ |= (1<<1); PORTJ &= ~(1<<1); }

  /// Pulses PinJ1 with low state first.
  static void PulseLow()  { PORTJ &= ~(1<<1); PORTJ |= (1<<1); }

  /// Reads PinJ1  value.
  /// @return Port pin value.
  static bool Test()      { return PINJ & (1<<1); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTJ
  static constexpr decltype(PORTJ) GetNativePort() { return PORTJ; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<1)
  static constexpr uint8_t bitmask()               { return (1<<1); }

  /// Returns the bit corresponding to this pin.
  /// @return 1
  static constexpr uint8_t bit()                   { return 1; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortJ;
};

struct PinJ2 {
  /// Sets PinJ2 to HIGH.
  static void Set()       { PORTJ |= (1<<2); }

  /// Sets PinJ2 to LOW.
  static void Clear()     { PORTJ &= ~(1<<2); }

  /// Toggles PinJ2 value.
  static void Toggle()    { PINJ |= (1<<2); }

  /// Configures PinJ2  as an output pin.
  static void SetOutput() { DDRJ |= (1<<2); }

  /// Configures PinJ2  as an input pin.
  static void SetInput()  { DDRJ &= ~(1<<2); }

  /// Pulses PinJ2 with high state first.
  static void PulseHigh() { PORTJ |= (1<<2); PORTJ &= ~(1<<2); }

  /// Pulses PinJ2 with low state first.
  static void PulseLow()  { PORTJ &= ~(1<<2); PORTJ |= (1<<2); }

  /// Reads PinJ2  value.
  /// @return Port pin value.
  static bool Test()      { return PINJ & (1<<2); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTJ
  static constexpr decltype(PORTJ) GetNativePort() { return PORTJ; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<2)
  static constexpr uint8_t bitmask()               { return (1<<2); }

  /// Returns the bit corresponding to this pin.
  /// @return 2
  static constexpr uint8_t bit()                   { return 2; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortJ;
};

struct PinJ3 {
  /// Sets PinJ3 to HIGH.
  static void Set()       { PORTJ |= (1<<3); }

  /// Sets PinJ3 to LOW.
  static void Clear()     { PORTJ &= ~(1<<3); }

  /// Toggles PinJ3 value.
  static void Toggle()    { PINJ |= (1<<3); }

  /// Configures PinJ3  as an output pin.
  static void SetOutput() { DDRJ |= (1<<3); }

  /// Configures PinJ3  as an input pin.
  static void SetInput()  { DDRJ &= ~(1<<3); }

  /// Pulses PinJ3 with high state first.
  static void PulseHigh() { PORTJ |= (1<<3); PORTJ &= ~(1<<3); }

  /// Pulses PinJ3 with low state first.
  static void PulseLow()  { PORTJ &= ~(1<<3); PORTJ |= (1<<3); }

  /// Reads PinJ3  value.
  /// @return Port pin value.
  static bool Test()      { return PINJ & (1<<3); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTJ
  static constexpr decltype(PORTJ) GetNativePort() { return PORTJ; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<3)
  static constexpr uint8_t bitmask()               { return (1<<3); }

  /// Returns the bit corresponding to this pin.
  /// @return 3
  static constexpr uint8_t bit()                   { return 3; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortJ;
};

struct PinJ4 {
  /// Sets PinJ4 to HIGH.
  static void Set()       { PORTJ |= (1<<4); }

  /// Sets PinJ4 to LOW.
  static void Clear()     { PORTJ &= ~(1<<4); }

  /// Toggles PinJ4 value.
  static void Toggle()    { PINJ |= (1<<4); }

  /// Configures PinJ4  as an output pin.
  static void SetOutput() { DDRJ |= (1<<4); }

  /// Configures PinJ4  as an input pin.
  static void SetInput()  { DDRJ &= ~(1<<4); }

  /// Pulses PinJ4 with high state first.
  static void PulseHigh() { PORTJ |= (1<<4); PORTJ &= ~(1<<4); }

  /// Pulses PinJ4 with low state first.
  static void PulseLow()  { PORTJ &= ~(1<<4); PORTJ |= (1<<4); }

  /// Reads PinJ4  value.
  /// @return Port pin value.
  static bool Test()      { return PINJ & (1<<4); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTJ
  static constexpr decltype(PORTJ) GetNativePort() { return PORTJ; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<4)
  static constexpr uint8_t bitmask()               { return (1<<4); }

  /// Returns the bit corresponding to this pin.
  /// @return 4
  static constexpr uint8_t bit()                   { return 4; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortJ;
};

struct PinJ5 {
  /// Sets PinJ5 to HIGH.
  static void Set()       { PORTJ |= (1<<5); }

  /// Sets PinJ5 to LOW.
  static void Clear()     { PORTJ &= ~(1<<5); }

  /// Toggles PinJ5 value.
  static void Toggle()    { PINJ |= (1<<5); }

  /// Configures PinJ5  as an output pin.
  static void SetOutput() { DDRJ |= (1<<5); }

  /// Configures PinJ5  as an input pin.
  static void SetInput()  { DDRJ &= ~(1<<5); }

  /// Pulses PinJ5 with high state first.
  static void PulseHigh() { PORTJ |= (1<<5); PORTJ &= ~(1<<5); }

  /// Pulses PinJ5 with low state first.
  static void PulseLow()  { PORTJ &= ~(1<<5); PORTJ |= (1<<5); }

  /// Reads PinJ5  value.
  /// @return Port pin value.
  static bool Test()      { return PINJ & (1<<5); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTJ
  static constexpr decltype(PORTJ) GetNativePort() { return PORTJ; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<5)
  static constexpr uint8_t bitmask()               { return (1<<5); }

  /// Returns the bit corresponding to this pin.
  /// @return 5
  static constexpr uint8_t bit()                   { return 5; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortJ;
};

struct PinJ6 {
  /// Sets PinJ6 to HIGH.
  static void Set()       { PORTJ |= (1<<6); }

  /// Sets PinJ6 to LOW.
  static void Clear()     { PORTJ &= ~(1<<6); }

  /// Toggles PinJ6 value.
  static void Toggle()    { PINJ |= (1<<6); }

  /// Configures PinJ6  as an output pin.
  static void SetOutput() { DDRJ |= (1<<6); }

  /// Configures PinJ6  as an input pin.
  static void SetInput()  { DDRJ &= ~(1<<6); }

  /// Pulses PinJ6 with high state first.
  static void PulseHigh() { PORTJ |= (1<<6); PORTJ &= ~(1<<6); }

  /// Pulses PinJ6 with low state first.
  static void PulseLow()  { PORTJ &= ~(1<<6); PORTJ |= (1<<6); }

  /// Reads PinJ6  value.
  /// @return Port pin value.
  static bool Test()      { return PINJ & (1<<6); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTJ
  static constexpr decltype(PORTJ) GetNativePort() { return PORTJ; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<6)
  static constexpr uint8_t bitmask()               { return (1<<6); }

  /// Returns the bit corresponding to this pin.
  /// @return 6
  static constexpr uint8_t bit()                   { return 6; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortJ;
};

struct PinJ7 {
  /// Sets PinJ7 to HIGH.
  static void Set()       { PORTJ |= (1<<7); }

  /// Sets PinJ7 to LOW.
  static void Clear()     { PORTJ &= ~(1<<7); }

  /// Toggles PinJ7 value.
  static void Toggle()    { PINJ |= (1<<7); }

  /// Configures PinJ7  as an output pin.
  static void SetOutput() { DDRJ |= (1<<7); }

  /// Configures PinJ7  as an input pin.
  static void SetInput()  { DDRJ &= ~(1<<7); }

  /// Pulses PinJ7 with high state first.
  static void PulseHigh() { PORTJ |= (1<<7); PORTJ &= ~(1<<7); }

  /// Pulses PinJ7 with low state first.
  static void PulseLow()  { PORTJ &= ~(1<<7); PORTJ |= (1<<7); }

  /// Reads PinJ7  value.
  /// @return Port pin value.
  static bool Test()      { return PINJ & (1<<7); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTJ
  static constexpr decltype(PORTJ) GetNativePort() { return PORTJ; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<7)
  static constexpr uint8_t bitmask()               { return (1<<7); }

  /// Returns the bit corresponding to this pin.
  /// @return 7
  static constexpr uint8_t bit()                   { return 7; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortJ;
};

#endif //PORTJ

#ifdef PORTK
struct PortK {
  /// Assigns a value to PORTK.
  /// @param[in] value value affected to PORTK
  static void Assign(uint8_t value)   { PORTK = value; }

  /// Sets masked bits in PORTK.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTK |= mask;}

  /// Clears masked bits in PORTK.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTK &= ~mask;} 

  /// Toggles masked bits in PORTK.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTK ^= mask;} 

  /// Pulses masked bits in PORTK with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTK |= mask; PORTK &= ~mask; }

  /// Pulses masked bits in PORTK with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTK &= ~mask; PORTK |= mask; }

  /// Set corresponding masked bits of PORTK to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRK |= mask; }

  /// Set corresponding masked bits of PORTK to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRK &= ~mask; }

  /// Returns PINK register.
  static uint8_t GetPIN()             { return PINK; }

  /// Tests masked bits of PORTK
  /// @param[in] mask bits
  /// @param[in] true if the corresponding bits are all set, false otherwise.
  static bool TestBits(uint8_t mask)  { return (PINK & mask) == mask; }

  /// Returns the value of the bit at the position pos.
  /// @param[in] position of the bit to return
  /// @return true if the requested bit is set, false otherwise.
  static bool Test(uint8_t bit) { return PINK & (1<<bit); }
};

struct PinK0 {
  /// Sets PinK0 to HIGH.
  static void Set()       { PORTK |= (1<<0); }

  /// Sets PinK0 to LOW.
  static void Clear()     { PORTK &= ~(1<<0); }

  /// Toggles PinK0 value.
  static void Toggle()    { PINK |= (1<<0); }

  /// Configures PinK0  as an output pin.
  static void SetOutput() { DDRK |= (1<<0); }

  /// Configures PinK0  as an input pin.
  static void SetInput()  { DDRK &= ~(1<<0); }

  /// Pulses PinK0 with high state first.
  static void PulseHigh() { PORTK |= (1<<0); PORTK &= ~(1<<0); }

  /// Pulses PinK0 with low state first.
  static void PulseLow()  { PORTK &= ~(1<<0); PORTK |= (1<<0); }

  /// Reads PinK0  value.
  /// @return Port pin value.
  static bool Test()      { return PINK & (1<<0); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTK
  static constexpr decltype(PORTK) GetNativePort() { return PORTK; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<0)
  static constexpr uint8_t bitmask()               { return (1<<0); }

  /// Returns the bit corresponding to this pin.
  /// @return 0
  static constexpr uint8_t bit()                   { return 0; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortK;
};

struct PinK1 {
  /// Sets PinK1 to HIGH.
  static void Set()       { PORTK |= (1<<1); }

  /// Sets PinK1 to LOW.
  static void Clear()     { PORTK &= ~(1<<1); }

  /// Toggles PinK1 value.
  static void Toggle()    { PINK |= (1<<1); }

  /// Configures PinK1  as an output pin.
  static void SetOutput() { DDRK |= (1<<1); }

  /// Configures PinK1  as an input pin.
  static void SetInput()  { DDRK &= ~(1<<1); }

  /// Pulses PinK1 with high state first.
  static void PulseHigh() { PORTK |= (1<<1); PORTK &= ~(1<<1); }

  /// Pulses PinK1 with low state first.
  static void PulseLow()  { PORTK &= ~(1<<1); PORTK |= (1<<1); }

  /// Reads PinK1  value.
  /// @return Port pin value.
  static bool Test()      { return PINK & (1<<1); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTK
  static constexpr decltype(PORTK) GetNativePort() { return PORTK; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<1)
  static constexpr uint8_t bitmask()               { return (1<<1); }

  /// Returns the bit corresponding to this pin.
  /// @return 1
  static constexpr uint8_t bit()                   { return 1; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortK;
};

struct PinK2 {
  /// Sets PinK2 to HIGH.
  static void Set()       { PORTK |= (1<<2); }

  /// Sets PinK2 to LOW.
  static void Clear()     { PORTK &= ~(1<<2); }

  /// Toggles PinK2 value.
  static void Toggle()    { PINK |= (1<<2); }

  /// Configures PinK2  as an output pin.
  static void SetOutput() { DDRK |= (1<<2); }

  /// Configures PinK2  as an input pin.
  static void SetInput()  { DDRK &= ~(1<<2); }

  /// Pulses PinK2 with high state first.
  static void PulseHigh() { PORTK |= (1<<2); PORTK &= ~(1<<2); }

  /// Pulses PinK2 with low state first.
  static void PulseLow()  { PORTK &= ~(1<<2); PORTK |= (1<<2); }

  /// Reads PinK2  value.
  /// @return Port pin value.
  static bool Test()      { return PINK & (1<<2); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTK
  static constexpr decltype(PORTK) GetNativePort() { return PORTK; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<2)
  static constexpr uint8_t bitmask()               { return (1<<2); }

  /// Returns the bit corresponding to this pin.
  /// @return 2
  static constexpr uint8_t bit()                   { return 2; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortK;
};

struct PinK3 {
  /// Sets PinK3 to HIGH.
  static void Set()       { PORTK |= (1<<3); }

  /// Sets PinK3 to LOW.
  static void Clear()     { PORTK &= ~(1<<3); }

  /// Toggles PinK3 value.
  static void Toggle()    { PINK |= (1<<3); }

  /// Configures PinK3  as an output pin.
  static void SetOutput() { DDRK |= (1<<3); }

  /// Configures PinK3  as an input pin.
  static void SetInput()  { DDRK &= ~(1<<3); }

  /// Pulses PinK3 with high state first.
  static void PulseHigh() { PORTK |= (1<<3); PORTK &= ~(1<<3); }

  /// Pulses PinK3 with low state first.
  static void PulseLow()  { PORTK &= ~(1<<3); PORTK |= (1<<3); }

  /// Reads PinK3  value.
  /// @return Port pin value.
  static bool Test()      { return PINK & (1<<3); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTK
  static constexpr decltype(PORTK) GetNativePort() { return PORTK; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<3)
  static constexpr uint8_t bitmask()               { return (1<<3); }

  /// Returns the bit corresponding to this pin.
  /// @return 3
  static constexpr uint8_t bit()                   { return 3; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortK;
};

struct PinK4 {
  /// Sets PinK4 to HIGH.
  static void Set()       { PORTK |= (1<<4); }

  /// Sets PinK4 to LOW.
  static void Clear()     { PORTK &= ~(1<<4); }

  /// Toggles PinK4 value.
  static void Toggle()    { PINK |= (1<<4); }

  /// Configures PinK4  as an output pin.
  static void SetOutput() { DDRK |= (1<<4); }

  /// Configures PinK4  as an input pin.
  static void SetInput()  { DDRK &= ~(1<<4); }

  /// Pulses PinK4 with high state first.
  static void PulseHigh() { PORTK |= (1<<4); PORTK &= ~(1<<4); }

  /// Pulses PinK4 with low state first.
  static void PulseLow()  { PORTK &= ~(1<<4); PORTK |= (1<<4); }

  /// Reads PinK4  value.
  /// @return Port pin value.
  static bool Test()      { return PINK & (1<<4); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTK
  static constexpr decltype(PORTK) GetNativePort() { return PORTK; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<4)
  static constexpr uint8_t bitmask()               { return (1<<4); }

  /// Returns the bit corresponding to this pin.
  /// @return 4
  static constexpr uint8_t bit()                   { return 4; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortK;
};

struct PinK5 {
  /// Sets PinK5 to HIGH.
  static void Set()       { PORTK |= (1<<5); }

  /// Sets PinK5 to LOW.
  static void Clear()     { PORTK &= ~(1<<5); }

  /// Toggles PinK5 value.
  static void Toggle()    { PINK |= (1<<5); }

  /// Configures PinK5  as an output pin.
  static void SetOutput() { DDRK |= (1<<5); }

  /// Configures PinK5  as an input pin.
  static void SetInput()  { DDRK &= ~(1<<5); }

  /// Pulses PinK5 with high state first.
  static void PulseHigh() { PORTK |= (1<<5); PORTK &= ~(1<<5); }

  /// Pulses PinK5 with low state first.
  static void PulseLow()  { PORTK &= ~(1<<5); PORTK |= (1<<5); }

  /// Reads PinK5  value.
  /// @return Port pin value.
  static bool Test()      { return PINK & (1<<5); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTK
  static constexpr decltype(PORTK) GetNativePort() { return PORTK; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<5)
  static constexpr uint8_t bitmask()               { return (1<<5); }

  /// Returns the bit corresponding to this pin.
  /// @return 5
  static constexpr uint8_t bit()                   { return 5; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortK;
};

struct PinK6 {
  /// Sets PinK6 to HIGH.
  static void Set()       { PORTK |= (1<<6); }

  /// Sets PinK6 to LOW.
  static void Clear()     { PORTK &= ~(1<<6); }

  /// Toggles PinK6 value.
  static void Toggle()    { PINK |= (1<<6); }

  /// Configures PinK6  as an output pin.
  static void SetOutput() { DDRK |= (1<<6); }

  /// Configures PinK6  as an input pin.
  static void SetInput()  { DDRK &= ~(1<<6); }

  /// Pulses PinK6 with high state first.
  static void PulseHigh() { PORTK |= (1<<6); PORTK &= ~(1<<6); }

  /// Pulses PinK6 with low state first.
  static void PulseLow()  { PORTK &= ~(1<<6); PORTK |= (1<<6); }

  /// Reads PinK6  value.
  /// @return Port pin value.
  static bool Test()      { return PINK & (1<<6); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTK
  static constexpr decltype(PORTK) GetNativePort() { return PORTK; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<6)
  static constexpr uint8_t bitmask()               { return (1<<6); }

  /// Returns the bit corresponding to this pin.
  /// @return 6
  static constexpr uint8_t bit()                   { return 6; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortK;
};

struct PinK7 {
  /// Sets PinK7 to HIGH.
  static void Set()       { PORTK |= (1<<7); }

  /// Sets PinK7 to LOW.
  static void Clear()     { PORTK &= ~(1<<7); }

  /// Toggles PinK7 value.
  static void Toggle()    { PINK |= (1<<7); }

  /// Configures PinK7  as an output pin.
  static void SetOutput() { DDRK |= (1<<7); }

  /// Configures PinK7  as an input pin.
  static void SetInput()  { DDRK &= ~(1<<7); }

  /// Pulses PinK7 with high state first.
  static void PulseHigh() { PORTK |= (1<<7); PORTK &= ~(1<<7); }

  /// Pulses PinK7 with low state first.
  static void PulseLow()  { PORTK &= ~(1<<7); PORTK |= (1<<7); }

  /// Reads PinK7  value.
  /// @return Port pin value.
  static bool Test()      { return PINK & (1<<7); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTK
  static constexpr decltype(PORTK) GetNativePort() { return PORTK; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<7)
  static constexpr uint8_t bitmask()               { return (1<<7); }

  /// Returns the bit corresponding to this pin.
  /// @return 7
  static constexpr uint8_t bit()                   { return 7; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortK;
};

#endif //PORTK

#ifdef PORTL
struct PortL {
  /// Assigns a value to PORTL.
  /// @param[in] value value affected to PORTL
  static void Assign(uint8_t value)   { PORTL = value; }

  /// Sets masked bits in PORTL.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTL |= mask;}

  /// Clears masked bits in PORTL.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTL &= ~mask;} 

  /// Toggles masked bits in PORTL.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTL ^= mask;} 

  /// Pulses masked bits in PORTL with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTL |= mask; PORTL &= ~mask; }

  /// Pulses masked bits in PORTL with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTL &= ~mask; PORTL |= mask; }

  /// Set corresponding masked bits of PORTL to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRL |= mask; }

  /// Set corresponding masked bits of PORTL to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRL &= ~mask; }

  /// Returns PINL register.
  static uint8_t GetPIN()             { return PINL; }

  /// Tests masked bits of PORTL
  /// @param[in] mask bits
  /// @param[in] true if the corresponding bits are all set, false otherwise.
  static bool TestBits(uint8_t mask)  { return (PINL & mask) == mask; }

  /// Returns the value of the bit at the position pos.
  /// @param[in] position of the bit to return
  /// @return true if the requested bit is set, false otherwise.
  static bool Test(uint8_t bit) { return PINL & (1<<bit); }
};

struct PinL0 {
  /// Sets PinL0 to HIGH.
  static void Set()       { PORTL |= (1<<0); }

  /// Sets PinL0 to LOW.
  static void Clear()     { PORTL &= ~(1<<0); }

  /// Toggles PinL0 value.
  static void Toggle()    { PINL |= (1<<0); }

  /// Configures PinL0  as an output pin.
  static void SetOutput() { DDRL |= (1<<0); }

  /// Configures PinL0  as an input pin.
  static void SetInput()  { DDRL &= ~(1<<0); }

  /// Pulses PinL0 with high state first.
  static void PulseHigh() { PORTL |= (1<<0); PORTL &= ~(1<<0); }

  /// Pulses PinL0 with low state first.
  static void PulseLow()  { PORTL &= ~(1<<0); PORTL |= (1<<0); }

  /// Reads PinL0  value.
  /// @return Port pin value.
  static bool Test()      { return PINL & (1<<0); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTL
  static constexpr decltype(PORTL) GetNativePort() { return PORTL; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<0)
  static constexpr uint8_t bitmask()               { return (1<<0); }

  /// Returns the bit corresponding to this pin.
  /// @return 0
  static constexpr uint8_t bit()                   { return 0; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortL;
};

struct PinL1 {
  /// Sets PinL1 to HIGH.
  static void Set()       { PORTL |= (1<<1); }

  /// Sets PinL1 to LOW.
  static void Clear()     { PORTL &= ~(1<<1); }

  /// Toggles PinL1 value.
  static void Toggle()    { PINL |= (1<<1); }

  /// Configures PinL1  as an output pin.
  static void SetOutput() { DDRL |= (1<<1); }

  /// Configures PinL1  as an input pin.
  static void SetInput()  { DDRL &= ~(1<<1); }

  /// Pulses PinL1 with high state first.
  static void PulseHigh() { PORTL |= (1<<1); PORTL &= ~(1<<1); }

  /// Pulses PinL1 with low state first.
  static void PulseLow()  { PORTL &= ~(1<<1); PORTL |= (1<<1); }

  /// Reads PinL1  value.
  /// @return Port pin value.
  static bool Test()      { return PINL & (1<<1); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTL
  static constexpr decltype(PORTL) GetNativePort() { return PORTL; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<1)
  static constexpr uint8_t bitmask()               { return (1<<1); }

  /// Returns the bit corresponding to this pin.
  /// @return 1
  static constexpr uint8_t bit()                   { return 1; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortL;
};

struct PinL2 {
  /// Sets PinL2 to HIGH.
  static void Set()       { PORTL |= (1<<2); }

  /// Sets PinL2 to LOW.
  static void Clear()     { PORTL &= ~(1<<2); }

  /// Toggles PinL2 value.
  static void Toggle()    { PINL |= (1<<2); }

  /// Configures PinL2  as an output pin.
  static void SetOutput() { DDRL |= (1<<2); }

  /// Configures PinL2  as an input pin.
  static void SetInput()  { DDRL &= ~(1<<2); }

  /// Pulses PinL2 with high state first.
  static void PulseHigh() { PORTL |= (1<<2); PORTL &= ~(1<<2); }

  /// Pulses PinL2 with low state first.
  static void PulseLow()  { PORTL &= ~(1<<2); PORTL |= (1<<2); }

  /// Reads PinL2  value.
  /// @return Port pin value.
  static bool Test()      { return PINL & (1<<2); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTL
  static constexpr decltype(PORTL) GetNativePort() { return PORTL; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<2)
  static constexpr uint8_t bitmask()               { return (1<<2); }

  /// Returns the bit corresponding to this pin.
  /// @return 2
  static constexpr uint8_t bit()                   { return 2; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortL;
};

struct PinL3 {
  /// Sets PinL3 to HIGH.
  static void Set()       { PORTL |= (1<<3); }

  /// Sets PinL3 to LOW.
  static void Clear()     { PORTL &= ~(1<<3); }

  /// Toggles PinL3 value.
  static void Toggle()    { PINL |= (1<<3); }

  /// Configures PinL3  as an output pin.
  static void SetOutput() { DDRL |= (1<<3); }

  /// Configures PinL3  as an input pin.
  static void SetInput()  { DDRL &= ~(1<<3); }

  /// Pulses PinL3 with high state first.
  static void PulseHigh() { PORTL |= (1<<3); PORTL &= ~(1<<3); }

  /// Pulses PinL3 with low state first.
  static void PulseLow()  { PORTL &= ~(1<<3); PORTL |= (1<<3); }

  /// Reads PinL3  value.
  /// @return Port pin value.
  static bool Test()      { return PINL & (1<<3); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTL
  static constexpr decltype(PORTL) GetNativePort() { return PORTL; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<3)
  static constexpr uint8_t bitmask()               { return (1<<3); }

  /// Returns the bit corresponding to this pin.
  /// @return 3
  static constexpr uint8_t bit()                   { return 3; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortL;
};

struct PinL4 {
  /// Sets PinL4 to HIGH.
  static void Set()       { PORTL |= (1<<4); }

  /// Sets PinL4 to LOW.
  static void Clear()     { PORTL &= ~(1<<4); }

  /// Toggles PinL4 value.
  static void Toggle()    { PINL |= (1<<4); }

  /// Configures PinL4  as an output pin.
  static void SetOutput() { DDRL |= (1<<4); }

  /// Configures PinL4  as an input pin.
  static void SetInput()  { DDRL &= ~(1<<4); }

  /// Pulses PinL4 with high state first.
  static void PulseHigh() { PORTL |= (1<<4); PORTL &= ~(1<<4); }

  /// Pulses PinL4 with low state first.
  static void PulseLow()  { PORTL &= ~(1<<4); PORTL |= (1<<4); }

  /// Reads PinL4  value.
  /// @return Port pin value.
  static bool Test()      { return PINL & (1<<4); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTL
  static constexpr decltype(PORTL) GetNativePort() { return PORTL; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<4)
  static constexpr uint8_t bitmask()               { return (1<<4); }

  /// Returns the bit corresponding to this pin.
  /// @return 4
  static constexpr uint8_t bit()                   { return 4; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortL;
};

struct PinL5 {
  /// Sets PinL5 to HIGH.
  static void Set()       { PORTL |= (1<<5); }

  /// Sets PinL5 to LOW.
  static void Clear()     { PORTL &= ~(1<<5); }

  /// Toggles PinL5 value.
  static void Toggle()    { PINL |= (1<<5); }

  /// Configures PinL5  as an output pin.
  static void SetOutput() { DDRL |= (1<<5); }

  /// Configures PinL5  as an input pin.
  static void SetInput()  { DDRL &= ~(1<<5); }

  /// Pulses PinL5 with high state first.
  static void PulseHigh() { PORTL |= (1<<5); PORTL &= ~(1<<5); }

  /// Pulses PinL5 with low state first.
  static void PulseLow()  { PORTL &= ~(1<<5); PORTL |= (1<<5); }

  /// Reads PinL5  value.
  /// @return Port pin value.
  static bool Test()      { return PINL & (1<<5); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTL
  static constexpr decltype(PORTL) GetNativePort() { return PORTL; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<5)
  static constexpr uint8_t bitmask()               { return (1<<5); }

  /// Returns the bit corresponding to this pin.
  /// @return 5
  static constexpr uint8_t bit()                   { return 5; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortL;
};

struct PinL6 {
  /// Sets PinL6 to HIGH.
  static void Set()       { PORTL |= (1<<6); }

  /// Sets PinL6 to LOW.
  static void Clear()     { PORTL &= ~(1<<6); }

  /// Toggles PinL6 value.
  static void Toggle()    { PINL |= (1<<6); }

  /// Configures PinL6  as an output pin.
  static void SetOutput() { DDRL |= (1<<6); }

  /// Configures PinL6  as an input pin.
  static void SetInput()  { DDRL &= ~(1<<6); }

  /// Pulses PinL6 with high state first.
  static void PulseHigh() { PORTL |= (1<<6); PORTL &= ~(1<<6); }

  /// Pulses PinL6 with low state first.
  static void PulseLow()  { PORTL &= ~(1<<6); PORTL |= (1<<6); }

  /// Reads PinL6  value.
  /// @return Port pin value.
  static bool Test()      { return PINL & (1<<6); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTL
  static constexpr decltype(PORTL) GetNativePort() { return PORTL; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<6)
  static constexpr uint8_t bitmask()               { return (1<<6); }

  /// Returns the bit corresponding to this pin.
  /// @return 6
  static constexpr uint8_t bit()                   { return 6; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortL;
};

struct PinL7 {
  /// Sets PinL7 to HIGH.
  static void Set()       { PORTL |= (1<<7); }

  /// Sets PinL7 to LOW.
  static void Clear()     { PORTL &= ~(1<<7); }

  /// Toggles PinL7 value.
  static void Toggle()    { PINL |= (1<<7); }

  /// Configures PinL7  as an output pin.
  static void SetOutput() { DDRL |= (1<<7); }

  /// Configures PinL7  as an input pin.
  static void SetInput()  { DDRL &= ~(1<<7); }

  /// Pulses PinL7 with high state first.
  static void PulseHigh() { PORTL |= (1<<7); PORTL &= ~(1<<7); }

  /// Pulses PinL7 with low state first.
  static void PulseLow()  { PORTL &= ~(1<<7); PORTL |= (1<<7); }

  /// Reads PinL7  value.
  /// @return Port pin value.
  static bool Test()      { return PINL & (1<<7); }

  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" 
  /// @return PORTL
  static constexpr decltype(PORTL) GetNativePort() { return PORTL; }

  /// Returns the bitmask corresponding to this pin.
  /// @return (1<<7)
  static constexpr uint8_t bitmask()               { return (1<<7); }

  /// Returns the bit corresponding to this pin.
  /// @return 7
  static constexpr uint8_t bit()                   { return 7; }

  /// Port is defined as the Port object to which this pin belongs.
  using Port = PortL;
};

#endif //PORTL

struct SpiSpcr {

  /// Assigns a value to SPCR
  /// @param[in] value value affected to SPCR
  static void Assign(uint8_t value)  { SPCR = value; }

  /// Sets masked bits in SPCR
  /// @param[in] mask bits to set
  static void Set(uint8_t mask)      { SPCR |= mask; }

  /// Clears masked bits in SPCR
  /// @param[in] mask bits to clear
  static void Clear(uint8_t mask)    { SPCR &= ~mask; }
  static uint8_t Get()               { return SPCR; }
  static bool TestBits(uint8_t mask) { return SPCR & mask; }
  void operator=(uint8_t value)      { SPCR = value; }
};

struct SpiSpdr {

  /// Assigns a value to SPDR
  /// @param[in] value value affected to SPDR
  static void Assign(uint8_t value)  { SPDR = value; }

  /// Sets masked bits in SPDR
  /// @param[in] mask bits to set
  static void Set(uint8_t mask)      { SPDR |= mask; }

  /// Clears masked bits in SPDR
  /// @param[in] mask bits to clear
  static void Clear(uint8_t mask)    { SPDR &= ~mask; }
  static uint8_t Get()               { return SPDR; }
  static bool TestBits(uint8_t mask) { return SPDR & mask; }
  void operator=(uint8_t value)      { SPDR = value; }
};

struct SpiSpsr {

  /// Assigns a value to SPSR
  /// @param[in] value value affected to SPSR
  static void Assign(uint8_t value)  { SPSR = value; }

  /// Sets masked bits in SPSR
  /// @param[in] mask bits to set
  static void Set(uint8_t mask)      { SPSR |= mask; }

  /// Clears masked bits in SPSR
  /// @param[in] mask bits to clear
  static void Clear(uint8_t mask)    { SPSR &= ~mask; }
  static uint8_t Get()               { return SPSR; }
  static bool TestBits(uint8_t mask) { return SPSR & mask; }
  void operator=(uint8_t value)      { SPSR = value; }
};

struct UsartUbrr0 {

  /// Assigns a value to UBRR0
  /// @param[in] value value affected to UBRR0
  static void Assign(uint16_t value)  { UBRR0 = value; }

  /// Sets masked bits in UBRR0
  /// @param[in] mask bits to set
  static void Set(uint16_t mask)      { UBRR0 |= mask; }

  /// Clears masked bits in UBRR0
  /// @param[in] mask bits to clear
  static void Clear(uint16_t mask)    { UBRR0 &= ~mask; }
  static uint8_t Get()               { return UBRR0; }
  static bool TestBits(uint16_t mask) { return UBRR0 & mask; }
  void operator=(uint8_t value)      { UBRR0 = value; }
};

struct UsartUcsr0a {

  /// Assigns a value to UCSR0A
  /// @param[in] value value affected to UCSR0A
  static void Assign(uint8_t value)  { UCSR0A = value; }

  /// Sets masked bits in UCSR0A
  /// @param[in] mask bits to set
  static void Set(uint8_t mask)      { UCSR0A |= mask; }

  /// Clears masked bits in UCSR0A
  /// @param[in] mask bits to clear
  static void Clear(uint8_t mask)    { UCSR0A &= ~mask; }
  static uint8_t Get()               { return UCSR0A; }
  static bool TestBits(uint8_t mask) { return UCSR0A & mask; }
  void operator=(uint8_t value)      { UCSR0A = value; }
};

struct UsartUcsr0b {

  /// Assigns a value to UCSR0B
  /// @param[in] value value affected to UCSR0B
  static void Assign(uint8_t value)  { UCSR0B = value; }

  /// Sets masked bits in UCSR0B
  /// @param[in] mask bits to set
  static void Set(uint8_t mask)      { UCSR0B |= mask; }

  /// Clears masked bits in UCSR0B
  /// @param[in] mask bits to clear
  static void Clear(uint8_t mask)    { UCSR0B &= ~mask; }
  static uint8_t Get()               { return UCSR0B; }
  static bool TestBits(uint8_t mask) { return UCSR0B & mask; }
  void operator=(uint8_t value)      { UCSR0B = value; }
};

struct UsartUcsr0c {

  /// Assigns a value to UCSR0C
  /// @param[in] value value affected to UCSR0C
  static void Assign(uint8_t value)  { UCSR0C = value; }

  /// Sets masked bits in UCSR0C
  /// @param[in] mask bits to set
  static void Set(uint8_t mask)      { UCSR0C |= mask; }

  /// Clears masked bits in UCSR0C
  /// @param[in] mask bits to clear
  static void Clear(uint8_t mask)    { UCSR0C &= ~mask; }
  static uint8_t Get()               { return UCSR0C; }
  static bool TestBits(uint8_t mask) { return UCSR0C & mask; }
  void operator=(uint8_t value)      { UCSR0C = value; }
};

struct UsartUdr0 {

  /// Assigns a value to UDR0
  /// @param[in] value value affected to UDR0
  static void Assign(uint8_t value)  { UDR0 = value; }

  /// Sets masked bits in UDR0
  /// @param[in] mask bits to set
  static void Set(uint8_t mask)      { UDR0 |= mask; }

  /// Clears masked bits in UDR0
  /// @param[in] mask bits to clear
  static void Clear(uint8_t mask)    { UDR0 &= ~mask; }
  static uint8_t Get()               { return UDR0; }
  static bool TestBits(uint8_t mask) { return UDR0 & mask; }
  void operator=(uint8_t value)      { UDR0 = value; }
};

#define IOPORTS_TO_STRING(name) #name
#define IOPORTS_IRQ_HANDLER(vector, type) asm(IOPORTS_TO_STRING(vector)) __attribute__ ((type, __INTR_ATTRS))
#ifdef TIMSK0
struct Timer0 {
  using value_type = uint8_t;

  enum Prescaler : uint8_t {
    STOP_TIMER = 0, NO_PRESCALER = (1<<CS00), CLK_DIV_8 = (1<<CS01),
    CLK_DIV_64 = (1<<CS01)|(1<<CS00), CLK_DIV_256 = (1<<CS02),
    CLK_DIV_1024 = (1<<CS02)|(1<<CS00), BITS = (1<<CS02)|(1<<CS01)|(1<<CS00) };
  enum ClockSource : uint8_t {
    INC_ON_FALLING = (1<<CS02)|(1<<CS01), INC_ON_RISING = (1<<CS02)|(1<<CS01)|(1<<CS00) };

  enum Interrupt : uint8_t { 
    OVERFLOW = (1<<TOIE0), COMPARE_MATCH_A = (1<<OCIE0A), COMPARE_MATCH_B = (1<<OCIE0B) };
  enum Constants : uint8_t { ALL_BITS = 0xFF };
  static value_type GetValue()                 { return TCNT0; }
  static void SetValue(value_type value)       { TCNT0 = value; }
  static void AddValue(value_type value)       { TCNT0 += value; }
  static void SubValue(value_type value)       { TCNT0 -= value; }
  static void SetCtrlRegA(uint8_t mask)        { TCCR0A |= mask; }
  static void ClearCtrlRegA(uint8_t mask)      { TCCR0A &= ~mask; }
  static void SetCtrlRegB(uint8_t mask)        { TCCR0B |= mask; }
  static void ClearCtrlRegB(uint8_t mask)      { TCCR0B &= ~mask; }
  static void SetInterruptMask(uint8_t mask)   { TIMSK0 |= mask; }
  static void ClearInterruptMask(uint8_t mask) { TIMSK0 &= ~mask; }
  static void SetPrescaler(Prescaler val)      { TCCR0B &= ~Prescaler::BITS; TCCR0B |= val; }
  static void SetExternalClockSource(ClockSource val)  { TCCR0B &= ~Prescaler::BITS; TCCR0B |= val; }
  static void SetOutputCompareValueA(value_type value) { OCR0A = value; }
  static value_type GetOutputCompareValueA()           { return OCR0A; }
  static void SetOutputCompareValueB(value_type value) { OCR0B = value; }
  static value_type GetOutputCompareValueB()           { return OCR0B; }

  struct ISR {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER0_OVF_vect, signal);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER0_COMPA_vect, signal);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER0_COMPB_vect, signal);
  };

  struct ISRNoBlock {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER0_OVF_vect, interrupt);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER0_COMPA_vect, interrupt);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER0_COMPB_vect, interrupt);
  };

  struct ISRNaked {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER0_OVF_vect, naked);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER0_COMPA_vect, naked);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER0_COMPB_vect, naked);
  };
};  
#endif //TIMSK0

#ifdef TIMSK1
struct Timer1 {
  using value_type = uint16_t;

  enum Prescaler : uint8_t {
    STOP_TIMER = 0, NO_PRESCALER = (1<<CS10), CLK_DIV_8 = (1<<CS11),
    CLK_DIV_64 = (1<<CS11)|(1<<CS10), CLK_DIV_256 = (1<<CS12),
    CLK_DIV_1024 = (1<<CS12)|(1<<CS10), BITS = (1<<CS12)|(1<<CS11)|(1<<CS10) };
  enum ClockSource : uint8_t {
    INC_ON_FALLING = (1<<CS12)|(1<<CS11), INC_ON_RISING = (1<<CS12)|(1<<CS11)|(1<<CS10) };

  enum Interrupt : uint8_t { 
    OVERFLOW = (1<<TOIE1), COMPARE_MATCH_A = (1<<OCIE1A), COMPARE_MATCH_B = (1<<OCIE1B) };
  enum Constants : uint8_t { ALL_BITS = 0xFF };
  static value_type GetValue()                 { return TCNT1; }
  static void SetValue(value_type value)       { TCNT1 = value; }
  static void AddValue(value_type value)       { TCNT1 += value; }
  static void SubValue(value_type value)       { TCNT1 -= value; }
  static void SetCtrlRegA(uint8_t mask)        { TCCR1A |= mask; }
  static void ClearCtrlRegA(uint8_t mask)      { TCCR1A &= ~mask; }
  static void SetCtrlRegB(uint8_t mask)        { TCCR1B |= mask; }
  static void ClearCtrlRegB(uint8_t mask)      { TCCR1B &= ~mask; }
  static void SetInterruptMask(uint8_t mask)   { TIMSK1 |= mask; }
  static void ClearInterruptMask(uint8_t mask) { TIMSK1 &= ~mask; }
  static void SetPrescaler(Prescaler val)      { TCCR1B &= ~Prescaler::BITS; TCCR1B |= val; }
  static void SetExternalClockSource(ClockSource val)  { TCCR1B &= ~Prescaler::BITS; TCCR1B |= val; }
  static void SetOutputCompareValueA(value_type value) { OCR1A = value; }
  static value_type GetOutputCompareValueA()           { return OCR1A; }
  static void SetOutputCompareValueB(value_type value) { OCR1B = value; }
  static value_type GetOutputCompareValueB()           { return OCR1B; }

  struct ISR {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER1_OVF_vect, signal);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER1_COMPA_vect, signal);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER1_COMPB_vect, signal);
  };

  struct ISRNoBlock {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER1_OVF_vect, interrupt);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER1_COMPA_vect, interrupt);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER1_COMPB_vect, interrupt);
  };

  struct ISRNaked {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER1_OVF_vect, naked);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER1_COMPA_vect, naked);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER1_COMPB_vect, naked);
  };
};  
#endif //TIMSK1

#ifdef TIMSK2
struct Timer2 {
  using value_type = uint8_t;

  enum Prescaler : uint8_t {
    STOP_TIMER = 0, NO_PRESCALER = (1<<CS20), CLK_DIV_8 = (1<<CS21),
    CLK_DIV_32 = (1<<CS21)|(1<<CS20), CLK_DIV_64 = (1<<CS22),
    CLK_DIV_128 = (1<<CS22)|(1<<CS20), CLK_DIV_256 = (1<<CS22)|(1<<CS21),
    CLK_DIV_1024 = (1<<CS22)|(1<<CS21)|(1<<CS20), BITS = (1<<CS22)|(1<<CS21)|(1<<CS20) };

  enum Interrupt : uint8_t { 
    OVERFLOW = (1<<TOIE2), COMPARE_MATCH_A = (1<<OCIE2A), COMPARE_MATCH_B = (1<<OCIE2B) };
  enum Constants : uint8_t { ALL_BITS = 0xFF };
  static value_type GetValue()                 { return TCNT2; }
  static void SetValue(value_type value)       { TCNT2 = value; }
  static void AddValue(value_type value)       { TCNT2 += value; }
  static void SubValue(value_type value)       { TCNT2 -= value; }
  static void SetCtrlRegA(uint8_t mask)        { TCCR2A |= mask; }
  static void ClearCtrlRegA(uint8_t mask)      { TCCR2A &= ~mask; }
  static void SetCtrlRegB(uint8_t mask)        { TCCR2B |= mask; }
  static void ClearCtrlRegB(uint8_t mask)      { TCCR2B &= ~mask; }
  static void SetInterruptMask(uint8_t mask)   { TIMSK2 |= mask; }
  static void ClearInterruptMask(uint8_t mask) { TIMSK2 &= ~mask; }
  static void SetPrescaler(Prescaler val)      { TCCR2B &= ~Prescaler::BITS; TCCR2B |= val; }
  static void SetOutputCompareValueA(value_type value) { OCR2A = value; }
  static value_type GetOutputCompareValueA()           { return OCR2A; }
  static void SetOutputCompareValueB(value_type value) { OCR2B = value; }
  static value_type GetOutputCompareValueB()           { return OCR2B; }

  struct ISR {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER2_OVF_vect, signal);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER2_COMPA_vect, signal);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER2_COMPB_vect, signal);
  };

  struct ISRNoBlock {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER2_OVF_vect, interrupt);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER2_COMPA_vect, interrupt);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER2_COMPB_vect, interrupt);
  };

  struct ISRNaked {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER2_OVF_vect, naked);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER2_COMPA_vect, naked);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER2_COMPB_vect, naked);
  };
};  
#endif //TIMSK2

#ifdef TIMSK3
struct Timer3 {
  using value_type = uint16_t;

  enum Prescaler : uint8_t {
    STOP_TIMER = 0, NO_PRESCALER = (1<<CS30), CLK_DIV_8 = (1<<CS31),
    CLK_DIV_64 = (1<<CS31)|(1<<CS30), CLK_DIV_256 = (1<<CS32),
    CLK_DIV_1024 = (1<<CS32)|(1<<CS30), BITS = (1<<CS32)|(1<<CS31)|(1<<CS30) };
  enum ClockSource : uint8_t {
    INC_ON_FALLING = (1<<CS32)|(1<<CS31), INC_ON_RISING = (1<<CS32)|(1<<CS31)|(1<<CS30) };

  enum Interrupt : uint8_t { 
    OVERFLOW = (1<<TOIE3), COMPARE_MATCH_A = (1<<OCIE3A), COMPARE_MATCH_B = (1<<OCIE3B) };
  enum Constants : uint8_t { ALL_BITS = 0xFF };
  static value_type GetValue()                 { return TCNT3; }
  static void SetValue(value_type value)       { TCNT3 = value; }
  static void AddValue(value_type value)       { TCNT3 += value; }
  static void SubValue(value_type value)       { TCNT3 -= value; }
  static void SetCtrlRegA(uint8_t mask)        { TCCR3A |= mask; }
  static void ClearCtrlRegA(uint8_t mask)      { TCCR3A &= ~mask; }
  static void SetCtrlRegB(uint8_t mask)        { TCCR3B |= mask; }
  static void ClearCtrlRegB(uint8_t mask)      { TCCR3B &= ~mask; }
  static void SetInterruptMask(uint8_t mask)   { TIMSK3 |= mask; }
  static void ClearInterruptMask(uint8_t mask) { TIMSK3 &= ~mask; }
  static void SetPrescaler(Prescaler val)      { TCCR3B &= ~Prescaler::BITS; TCCR3B |= val; }
  static void SetExternalClockSource(ClockSource val)  { TCCR3B &= ~Prescaler::BITS; TCCR3B |= val; }
  static void SetOutputCompareValueA(value_type value) { OCR3A = value; }
  static value_type GetOutputCompareValueA()           { return OCR3A; }
  static void SetOutputCompareValueB(value_type value) { OCR3B = value; }
  static value_type GetOutputCompareValueB()           { return OCR3B; }

  struct ISR {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER3_OVF_vect, signal);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER3_COMPA_vect, signal);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER3_COMPB_vect, signal);
  };

  struct ISRNoBlock {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER3_OVF_vect, interrupt);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER3_COMPA_vect, interrupt);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER3_COMPB_vect, interrupt);
  };

  struct ISRNaked {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER3_OVF_vect, naked);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER3_COMPA_vect, naked);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER3_COMPB_vect, naked);
  };
};  
#endif //TIMSK3

#undef IOPORTS_TO_STRING
#undef IOPORTS_IRQ_HANDLER
} // namespace etl
#endif //ETL_IOPORTS_H_
