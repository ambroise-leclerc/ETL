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
class PortB {
 public:

  /// Assigns a value to PORTB.
  /// @param[in] value value affected to PORTB
  static void Assign(uint8_t value)   { PORTB = value; }

  /// Sets masked bits in PORTB.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTB |= mask;}

  /// Clears masked bits in PORTB.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTB &= (~mask);} 

  /// Toggles masked bits in PORTB.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTB ^= (~mask);} 

  /// Pulses masked bits in PORTB with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTB |= mask; PORTB &= (~mask); }

  /// Pulses masked bits in PORTB with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTB &= (~mask); PORTB |= mask; }

  /// Set corresponding masked bits of PORTB to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRB |= mask; }

  /// Set corresponding masked bits of PORTB to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRB &= (~mask); }

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

class PinB0 {
 public:
  static void Set()       { PORTB |= (1<<0); }
  static void Clear()     { PORTB &= (~(1<<0)); }
  static void Toggle()    { PORTB ^= (1<<0); }
  static void SetOutput() { DDRB |= (1<<0); }
  static void SetInput()  { DDRB &= (~(1<<0)); }
  static void PulseHigh() { PORTB |= (1<<0); PORTB &= (~(1<<0)); }
  static void PulseLow()  { PORTB &= (~(1<<0)); PORTB |= (1<<0); }
  static bool Test()      { return PINB & (1<<0); }

  static constexpr decltype(PORTB) GetNativePort() { return PORTB; }
  static constexpr uint8_t bitmask()               { return (1<<0); }
  static constexpr uint8_t bit()                   { return 0; }

  typedef PortB Port;
};

class PinB1 {
 public:
  static void Set()       { PORTB |= (1<<1); }
  static void Clear()     { PORTB &= (~(1<<1)); }
  static void Toggle()    { PORTB ^= (1<<1); }
  static void SetOutput() { DDRB |= (1<<1); }
  static void SetInput()  { DDRB &= (~(1<<1)); }
  static void PulseHigh() { PORTB |= (1<<1); PORTB &= (~(1<<1)); }
  static void PulseLow()  { PORTB &= (~(1<<1)); PORTB |= (1<<1); }
  static bool Test()      { return PINB & (1<<1); }

  static constexpr decltype(PORTB) GetNativePort() { return PORTB; }
  static constexpr uint8_t bitmask()               { return (1<<1); }
  static constexpr uint8_t bit()                   { return 1; }

  typedef PortB Port;
};

class PinB2 {
 public:
  static void Set()       { PORTB |= (1<<2); }
  static void Clear()     { PORTB &= (~(1<<2)); }
  static void Toggle()    { PORTB ^= (1<<2); }
  static void SetOutput() { DDRB |= (1<<2); }
  static void SetInput()  { DDRB &= (~(1<<2)); }
  static void PulseHigh() { PORTB |= (1<<2); PORTB &= (~(1<<2)); }
  static void PulseLow()  { PORTB &= (~(1<<2)); PORTB |= (1<<2); }
  static bool Test()      { return PINB & (1<<2); }

  static constexpr decltype(PORTB) GetNativePort() { return PORTB; }
  static constexpr uint8_t bitmask()               { return (1<<2); }
  static constexpr uint8_t bit()                   { return 2; }

  typedef PortB Port;
};

class PinB3 {
 public:
  static void Set()       { PORTB |= (1<<3); }
  static void Clear()     { PORTB &= (~(1<<3)); }
  static void Toggle()    { PORTB ^= (1<<3); }
  static void SetOutput() { DDRB |= (1<<3); }
  static void SetInput()  { DDRB &= (~(1<<3)); }
  static void PulseHigh() { PORTB |= (1<<3); PORTB &= (~(1<<3)); }
  static void PulseLow()  { PORTB &= (~(1<<3)); PORTB |= (1<<3); }
  static bool Test()      { return PINB & (1<<3); }

  static constexpr decltype(PORTB) GetNativePort() { return PORTB; }
  static constexpr uint8_t bitmask()               { return (1<<3); }
  static constexpr uint8_t bit()                   { return 3; }

  typedef PortB Port;
};

class PinB4 {
 public:
  static void Set()       { PORTB |= (1<<4); }
  static void Clear()     { PORTB &= (~(1<<4)); }
  static void Toggle()    { PORTB ^= (1<<4); }
  static void SetOutput() { DDRB |= (1<<4); }
  static void SetInput()  { DDRB &= (~(1<<4)); }
  static void PulseHigh() { PORTB |= (1<<4); PORTB &= (~(1<<4)); }
  static void PulseLow()  { PORTB &= (~(1<<4)); PORTB |= (1<<4); }
  static bool Test()      { return PINB & (1<<4); }

  static constexpr decltype(PORTB) GetNativePort() { return PORTB; }
  static constexpr uint8_t bitmask()               { return (1<<4); }
  static constexpr uint8_t bit()                   { return 4; }

  typedef PortB Port;
};

class PinB5 {
 public:
  static void Set()       { PORTB |= (1<<5); }
  static void Clear()     { PORTB &= (~(1<<5)); }
  static void Toggle()    { PORTB ^= (1<<5); }
  static void SetOutput() { DDRB |= (1<<5); }
  static void SetInput()  { DDRB &= (~(1<<5)); }
  static void PulseHigh() { PORTB |= (1<<5); PORTB &= (~(1<<5)); }
  static void PulseLow()  { PORTB &= (~(1<<5)); PORTB |= (1<<5); }
  static bool Test()      { return PINB & (1<<5); }

  static constexpr decltype(PORTB) GetNativePort() { return PORTB; }
  static constexpr uint8_t bitmask()               { return (1<<5); }
  static constexpr uint8_t bit()                   { return 5; }

  typedef PortB Port;
};

class PinB6 {
 public:
  static void Set()       { PORTB |= (1<<6); }
  static void Clear()     { PORTB &= (~(1<<6)); }
  static void Toggle()    { PORTB ^= (1<<6); }
  static void SetOutput() { DDRB |= (1<<6); }
  static void SetInput()  { DDRB &= (~(1<<6)); }
  static void PulseHigh() { PORTB |= (1<<6); PORTB &= (~(1<<6)); }
  static void PulseLow()  { PORTB &= (~(1<<6)); PORTB |= (1<<6); }
  static bool Test()      { return PINB & (1<<6); }

  static constexpr decltype(PORTB) GetNativePort() { return PORTB; }
  static constexpr uint8_t bitmask()               { return (1<<6); }
  static constexpr uint8_t bit()                   { return 6; }

  typedef PortB Port;
};

class PinB7 {
 public:
  static void Set()       { PORTB |= (1<<7); }
  static void Clear()     { PORTB &= (~(1<<7)); }
  static void Toggle()    { PORTB ^= (1<<7); }
  static void SetOutput() { DDRB |= (1<<7); }
  static void SetInput()  { DDRB &= (~(1<<7)); }
  static void PulseHigh() { PORTB |= (1<<7); PORTB &= (~(1<<7)); }
  static void PulseLow()  { PORTB &= (~(1<<7)); PORTB |= (1<<7); }
  static bool Test()      { return PINB & (1<<7); }

  static constexpr decltype(PORTB) GetNativePort() { return PORTB; }
  static constexpr uint8_t bitmask()               { return (1<<7); }
  static constexpr uint8_t bit()                   { return 7; }

  typedef PortB Port;
};

#endif //PORTB

#ifdef PORTC
class PortC {
 public:

  /// Assigns a value to PORTC.
  /// @param[in] value value affected to PORTC
  static void Assign(uint8_t value)   { PORTC = value; }

  /// Sets masked bits in PORTC.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTC |= mask;}

  /// Clears masked bits in PORTC.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTC &= (~mask);} 

  /// Toggles masked bits in PORTC.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTC ^= (~mask);} 

  /// Pulses masked bits in PORTC with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTC |= mask; PORTC &= (~mask); }

  /// Pulses masked bits in PORTC with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTC &= (~mask); PORTC |= mask; }

  /// Set corresponding masked bits of PORTC to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRC |= mask; }

  /// Set corresponding masked bits of PORTC to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRC &= (~mask); }

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

class PinC0 {
 public:
  static void Set()       { PORTC |= (1<<0); }
  static void Clear()     { PORTC &= (~(1<<0)); }
  static void Toggle()    { PORTC ^= (1<<0); }
  static void SetOutput() { DDRC |= (1<<0); }
  static void SetInput()  { DDRC &= (~(1<<0)); }
  static void PulseHigh() { PORTC |= (1<<0); PORTC &= (~(1<<0)); }
  static void PulseLow()  { PORTC &= (~(1<<0)); PORTC |= (1<<0); }
  static bool Test()      { return PINC & (1<<0); }

  static constexpr decltype(PORTC) GetNativePort() { return PORTC; }
  static constexpr uint8_t bitmask()               { return (1<<0); }
  static constexpr uint8_t bit()                   { return 0; }

  typedef PortC Port;
};

class PinC1 {
 public:
  static void Set()       { PORTC |= (1<<1); }
  static void Clear()     { PORTC &= (~(1<<1)); }
  static void Toggle()    { PORTC ^= (1<<1); }
  static void SetOutput() { DDRC |= (1<<1); }
  static void SetInput()  { DDRC &= (~(1<<1)); }
  static void PulseHigh() { PORTC |= (1<<1); PORTC &= (~(1<<1)); }
  static void PulseLow()  { PORTC &= (~(1<<1)); PORTC |= (1<<1); }
  static bool Test()      { return PINC & (1<<1); }

  static constexpr decltype(PORTC) GetNativePort() { return PORTC; }
  static constexpr uint8_t bitmask()               { return (1<<1); }
  static constexpr uint8_t bit()                   { return 1; }

  typedef PortC Port;
};

class PinC2 {
 public:
  static void Set()       { PORTC |= (1<<2); }
  static void Clear()     { PORTC &= (~(1<<2)); }
  static void Toggle()    { PORTC ^= (1<<2); }
  static void SetOutput() { DDRC |= (1<<2); }
  static void SetInput()  { DDRC &= (~(1<<2)); }
  static void PulseHigh() { PORTC |= (1<<2); PORTC &= (~(1<<2)); }
  static void PulseLow()  { PORTC &= (~(1<<2)); PORTC |= (1<<2); }
  static bool Test()      { return PINC & (1<<2); }

  static constexpr decltype(PORTC) GetNativePort() { return PORTC; }
  static constexpr uint8_t bitmask()               { return (1<<2); }
  static constexpr uint8_t bit()                   { return 2; }

  typedef PortC Port;
};

class PinC3 {
 public:
  static void Set()       { PORTC |= (1<<3); }
  static void Clear()     { PORTC &= (~(1<<3)); }
  static void Toggle()    { PORTC ^= (1<<3); }
  static void SetOutput() { DDRC |= (1<<3); }
  static void SetInput()  { DDRC &= (~(1<<3)); }
  static void PulseHigh() { PORTC |= (1<<3); PORTC &= (~(1<<3)); }
  static void PulseLow()  { PORTC &= (~(1<<3)); PORTC |= (1<<3); }
  static bool Test()      { return PINC & (1<<3); }

  static constexpr decltype(PORTC) GetNativePort() { return PORTC; }
  static constexpr uint8_t bitmask()               { return (1<<3); }
  static constexpr uint8_t bit()                   { return 3; }

  typedef PortC Port;
};

class PinC4 {
 public:
  static void Set()       { PORTC |= (1<<4); }
  static void Clear()     { PORTC &= (~(1<<4)); }
  static void Toggle()    { PORTC ^= (1<<4); }
  static void SetOutput() { DDRC |= (1<<4); }
  static void SetInput()  { DDRC &= (~(1<<4)); }
  static void PulseHigh() { PORTC |= (1<<4); PORTC &= (~(1<<4)); }
  static void PulseLow()  { PORTC &= (~(1<<4)); PORTC |= (1<<4); }
  static bool Test()      { return PINC & (1<<4); }

  static constexpr decltype(PORTC) GetNativePort() { return PORTC; }
  static constexpr uint8_t bitmask()               { return (1<<4); }
  static constexpr uint8_t bit()                   { return 4; }

  typedef PortC Port;
};

class PinC5 {
 public:
  static void Set()       { PORTC |= (1<<5); }
  static void Clear()     { PORTC &= (~(1<<5)); }
  static void Toggle()    { PORTC ^= (1<<5); }
  static void SetOutput() { DDRC |= (1<<5); }
  static void SetInput()  { DDRC &= (~(1<<5)); }
  static void PulseHigh() { PORTC |= (1<<5); PORTC &= (~(1<<5)); }
  static void PulseLow()  { PORTC &= (~(1<<5)); PORTC |= (1<<5); }
  static bool Test()      { return PINC & (1<<5); }

  static constexpr decltype(PORTC) GetNativePort() { return PORTC; }
  static constexpr uint8_t bitmask()               { return (1<<5); }
  static constexpr uint8_t bit()                   { return 5; }

  typedef PortC Port;
};

class PinC6 {
 public:
  static void Set()       { PORTC |= (1<<6); }
  static void Clear()     { PORTC &= (~(1<<6)); }
  static void Toggle()    { PORTC ^= (1<<6); }
  static void SetOutput() { DDRC |= (1<<6); }
  static void SetInput()  { DDRC &= (~(1<<6)); }
  static void PulseHigh() { PORTC |= (1<<6); PORTC &= (~(1<<6)); }
  static void PulseLow()  { PORTC &= (~(1<<6)); PORTC |= (1<<6); }
  static bool Test()      { return PINC & (1<<6); }

  static constexpr decltype(PORTC) GetNativePort() { return PORTC; }
  static constexpr uint8_t bitmask()               { return (1<<6); }
  static constexpr uint8_t bit()                   { return 6; }

  typedef PortC Port;
};

class PinC7 {
 public:
  static void Set()       { PORTC |= (1<<7); }
  static void Clear()     { PORTC &= (~(1<<7)); }
  static void Toggle()    { PORTC ^= (1<<7); }
  static void SetOutput() { DDRC |= (1<<7); }
  static void SetInput()  { DDRC &= (~(1<<7)); }
  static void PulseHigh() { PORTC |= (1<<7); PORTC &= (~(1<<7)); }
  static void PulseLow()  { PORTC &= (~(1<<7)); PORTC |= (1<<7); }
  static bool Test()      { return PINC & (1<<7); }

  static constexpr decltype(PORTC) GetNativePort() { return PORTC; }
  static constexpr uint8_t bitmask()               { return (1<<7); }
  static constexpr uint8_t bit()                   { return 7; }

  typedef PortC Port;
};

#endif //PORTC

#ifdef PORTD
class PortD {
 public:

  /// Assigns a value to PORTD.
  /// @param[in] value value affected to PORTD
  static void Assign(uint8_t value)   { PORTD = value; }

  /// Sets masked bits in PORTD.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTD |= mask;}

  /// Clears masked bits in PORTD.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTD &= (~mask);} 

  /// Toggles masked bits in PORTD.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTD ^= (~mask);} 

  /// Pulses masked bits in PORTD with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTD |= mask; PORTD &= (~mask); }

  /// Pulses masked bits in PORTD with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTD &= (~mask); PORTD |= mask; }

  /// Set corresponding masked bits of PORTD to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRD |= mask; }

  /// Set corresponding masked bits of PORTD to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRD &= (~mask); }

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

class PinD0 {
 public:
  static void Set()       { PORTD |= (1<<0); }
  static void Clear()     { PORTD &= (~(1<<0)); }
  static void Toggle()    { PORTD ^= (1<<0); }
  static void SetOutput() { DDRD |= (1<<0); }
  static void SetInput()  { DDRD &= (~(1<<0)); }
  static void PulseHigh() { PORTD |= (1<<0); PORTD &= (~(1<<0)); }
  static void PulseLow()  { PORTD &= (~(1<<0)); PORTD |= (1<<0); }
  static bool Test()      { return PIND & (1<<0); }

  static constexpr decltype(PORTD) GetNativePort() { return PORTD; }
  static constexpr uint8_t bitmask()               { return (1<<0); }
  static constexpr uint8_t bit()                   { return 0; }

  typedef PortD Port;
};

class PinD1 {
 public:
  static void Set()       { PORTD |= (1<<1); }
  static void Clear()     { PORTD &= (~(1<<1)); }
  static void Toggle()    { PORTD ^= (1<<1); }
  static void SetOutput() { DDRD |= (1<<1); }
  static void SetInput()  { DDRD &= (~(1<<1)); }
  static void PulseHigh() { PORTD |= (1<<1); PORTD &= (~(1<<1)); }
  static void PulseLow()  { PORTD &= (~(1<<1)); PORTD |= (1<<1); }
  static bool Test()      { return PIND & (1<<1); }

  static constexpr decltype(PORTD) GetNativePort() { return PORTD; }
  static constexpr uint8_t bitmask()               { return (1<<1); }
  static constexpr uint8_t bit()                   { return 1; }

  typedef PortD Port;
};

class PinD2 {
 public:
  static void Set()       { PORTD |= (1<<2); }
  static void Clear()     { PORTD &= (~(1<<2)); }
  static void Toggle()    { PORTD ^= (1<<2); }
  static void SetOutput() { DDRD |= (1<<2); }
  static void SetInput()  { DDRD &= (~(1<<2)); }
  static void PulseHigh() { PORTD |= (1<<2); PORTD &= (~(1<<2)); }
  static void PulseLow()  { PORTD &= (~(1<<2)); PORTD |= (1<<2); }
  static bool Test()      { return PIND & (1<<2); }

  static constexpr decltype(PORTD) GetNativePort() { return PORTD; }
  static constexpr uint8_t bitmask()               { return (1<<2); }
  static constexpr uint8_t bit()                   { return 2; }

  typedef PortD Port;
};

class PinD3 {
 public:
  static void Set()       { PORTD |= (1<<3); }
  static void Clear()     { PORTD &= (~(1<<3)); }
  static void Toggle()    { PORTD ^= (1<<3); }
  static void SetOutput() { DDRD |= (1<<3); }
  static void SetInput()  { DDRD &= (~(1<<3)); }
  static void PulseHigh() { PORTD |= (1<<3); PORTD &= (~(1<<3)); }
  static void PulseLow()  { PORTD &= (~(1<<3)); PORTD |= (1<<3); }
  static bool Test()      { return PIND & (1<<3); }

  static constexpr decltype(PORTD) GetNativePort() { return PORTD; }
  static constexpr uint8_t bitmask()               { return (1<<3); }
  static constexpr uint8_t bit()                   { return 3; }

  typedef PortD Port;
};

class PinD4 {
 public:
  static void Set()       { PORTD |= (1<<4); }
  static void Clear()     { PORTD &= (~(1<<4)); }
  static void Toggle()    { PORTD ^= (1<<4); }
  static void SetOutput() { DDRD |= (1<<4); }
  static void SetInput()  { DDRD &= (~(1<<4)); }
  static void PulseHigh() { PORTD |= (1<<4); PORTD &= (~(1<<4)); }
  static void PulseLow()  { PORTD &= (~(1<<4)); PORTD |= (1<<4); }
  static bool Test()      { return PIND & (1<<4); }

  static constexpr decltype(PORTD) GetNativePort() { return PORTD; }
  static constexpr uint8_t bitmask()               { return (1<<4); }
  static constexpr uint8_t bit()                   { return 4; }

  typedef PortD Port;
};

class PinD5 {
 public:
  static void Set()       { PORTD |= (1<<5); }
  static void Clear()     { PORTD &= (~(1<<5)); }
  static void Toggle()    { PORTD ^= (1<<5); }
  static void SetOutput() { DDRD |= (1<<5); }
  static void SetInput()  { DDRD &= (~(1<<5)); }
  static void PulseHigh() { PORTD |= (1<<5); PORTD &= (~(1<<5)); }
  static void PulseLow()  { PORTD &= (~(1<<5)); PORTD |= (1<<5); }
  static bool Test()      { return PIND & (1<<5); }

  static constexpr decltype(PORTD) GetNativePort() { return PORTD; }
  static constexpr uint8_t bitmask()               { return (1<<5); }
  static constexpr uint8_t bit()                   { return 5; }

  typedef PortD Port;
};

class PinD6 {
 public:
  static void Set()       { PORTD |= (1<<6); }
  static void Clear()     { PORTD &= (~(1<<6)); }
  static void Toggle()    { PORTD ^= (1<<6); }
  static void SetOutput() { DDRD |= (1<<6); }
  static void SetInput()  { DDRD &= (~(1<<6)); }
  static void PulseHigh() { PORTD |= (1<<6); PORTD &= (~(1<<6)); }
  static void PulseLow()  { PORTD &= (~(1<<6)); PORTD |= (1<<6); }
  static bool Test()      { return PIND & (1<<6); }

  static constexpr decltype(PORTD) GetNativePort() { return PORTD; }
  static constexpr uint8_t bitmask()               { return (1<<6); }
  static constexpr uint8_t bit()                   { return 6; }

  typedef PortD Port;
};

class PinD7 {
 public:
  static void Set()       { PORTD |= (1<<7); }
  static void Clear()     { PORTD &= (~(1<<7)); }
  static void Toggle()    { PORTD ^= (1<<7); }
  static void SetOutput() { DDRD |= (1<<7); }
  static void SetInput()  { DDRD &= (~(1<<7)); }
  static void PulseHigh() { PORTD |= (1<<7); PORTD &= (~(1<<7)); }
  static void PulseLow()  { PORTD &= (~(1<<7)); PORTD |= (1<<7); }
  static bool Test()      { return PIND & (1<<7); }

  static constexpr decltype(PORTD) GetNativePort() { return PORTD; }
  static constexpr uint8_t bitmask()               { return (1<<7); }
  static constexpr uint8_t bit()                   { return 7; }

  typedef PortD Port;
};

#endif //PORTD

#ifdef PORTE
class PortE {
 public:

  /// Assigns a value to PORTE.
  /// @param[in] value value affected to PORTE
  static void Assign(uint8_t value)   { PORTE = value; }

  /// Sets masked bits in PORTE.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTE |= mask;}

  /// Clears masked bits in PORTE.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTE &= (~mask);} 

  /// Toggles masked bits in PORTE.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTE ^= (~mask);} 

  /// Pulses masked bits in PORTE with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTE |= mask; PORTE &= (~mask); }

  /// Pulses masked bits in PORTE with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTE &= (~mask); PORTE |= mask; }

  /// Set corresponding masked bits of PORTE to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRE |= mask; }

  /// Set corresponding masked bits of PORTE to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRE &= (~mask); }

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

class PinE0 {
 public:
  static void Set()       { PORTE |= (1<<0); }
  static void Clear()     { PORTE &= (~(1<<0)); }
  static void Toggle()    { PORTE ^= (1<<0); }
  static void SetOutput() { DDRE |= (1<<0); }
  static void SetInput()  { DDRE &= (~(1<<0)); }
  static void PulseHigh() { PORTE |= (1<<0); PORTE &= (~(1<<0)); }
  static void PulseLow()  { PORTE &= (~(1<<0)); PORTE |= (1<<0); }
  static bool Test()      { return PINE & (1<<0); }

  static constexpr decltype(PORTE) GetNativePort() { return PORTE; }
  static constexpr uint8_t bitmask()               { return (1<<0); }
  static constexpr uint8_t bit()                   { return 0; }

  typedef PortE Port;
};

class PinE1 {
 public:
  static void Set()       { PORTE |= (1<<1); }
  static void Clear()     { PORTE &= (~(1<<1)); }
  static void Toggle()    { PORTE ^= (1<<1); }
  static void SetOutput() { DDRE |= (1<<1); }
  static void SetInput()  { DDRE &= (~(1<<1)); }
  static void PulseHigh() { PORTE |= (1<<1); PORTE &= (~(1<<1)); }
  static void PulseLow()  { PORTE &= (~(1<<1)); PORTE |= (1<<1); }
  static bool Test()      { return PINE & (1<<1); }

  static constexpr decltype(PORTE) GetNativePort() { return PORTE; }
  static constexpr uint8_t bitmask()               { return (1<<1); }
  static constexpr uint8_t bit()                   { return 1; }

  typedef PortE Port;
};

class PinE2 {
 public:
  static void Set()       { PORTE |= (1<<2); }
  static void Clear()     { PORTE &= (~(1<<2)); }
  static void Toggle()    { PORTE ^= (1<<2); }
  static void SetOutput() { DDRE |= (1<<2); }
  static void SetInput()  { DDRE &= (~(1<<2)); }
  static void PulseHigh() { PORTE |= (1<<2); PORTE &= (~(1<<2)); }
  static void PulseLow()  { PORTE &= (~(1<<2)); PORTE |= (1<<2); }
  static bool Test()      { return PINE & (1<<2); }

  static constexpr decltype(PORTE) GetNativePort() { return PORTE; }
  static constexpr uint8_t bitmask()               { return (1<<2); }
  static constexpr uint8_t bit()                   { return 2; }

  typedef PortE Port;
};

class PinE3 {
 public:
  static void Set()       { PORTE |= (1<<3); }
  static void Clear()     { PORTE &= (~(1<<3)); }
  static void Toggle()    { PORTE ^= (1<<3); }
  static void SetOutput() { DDRE |= (1<<3); }
  static void SetInput()  { DDRE &= (~(1<<3)); }
  static void PulseHigh() { PORTE |= (1<<3); PORTE &= (~(1<<3)); }
  static void PulseLow()  { PORTE &= (~(1<<3)); PORTE |= (1<<3); }
  static bool Test()      { return PINE & (1<<3); }

  static constexpr decltype(PORTE) GetNativePort() { return PORTE; }
  static constexpr uint8_t bitmask()               { return (1<<3); }
  static constexpr uint8_t bit()                   { return 3; }

  typedef PortE Port;
};

class PinE4 {
 public:
  static void Set()       { PORTE |= (1<<4); }
  static void Clear()     { PORTE &= (~(1<<4)); }
  static void Toggle()    { PORTE ^= (1<<4); }
  static void SetOutput() { DDRE |= (1<<4); }
  static void SetInput()  { DDRE &= (~(1<<4)); }
  static void PulseHigh() { PORTE |= (1<<4); PORTE &= (~(1<<4)); }
  static void PulseLow()  { PORTE &= (~(1<<4)); PORTE |= (1<<4); }
  static bool Test()      { return PINE & (1<<4); }

  static constexpr decltype(PORTE) GetNativePort() { return PORTE; }
  static constexpr uint8_t bitmask()               { return (1<<4); }
  static constexpr uint8_t bit()                   { return 4; }

  typedef PortE Port;
};

class PinE5 {
 public:
  static void Set()       { PORTE |= (1<<5); }
  static void Clear()     { PORTE &= (~(1<<5)); }
  static void Toggle()    { PORTE ^= (1<<5); }
  static void SetOutput() { DDRE |= (1<<5); }
  static void SetInput()  { DDRE &= (~(1<<5)); }
  static void PulseHigh() { PORTE |= (1<<5); PORTE &= (~(1<<5)); }
  static void PulseLow()  { PORTE &= (~(1<<5)); PORTE |= (1<<5); }
  static bool Test()      { return PINE & (1<<5); }

  static constexpr decltype(PORTE) GetNativePort() { return PORTE; }
  static constexpr uint8_t bitmask()               { return (1<<5); }
  static constexpr uint8_t bit()                   { return 5; }

  typedef PortE Port;
};

class PinE6 {
 public:
  static void Set()       { PORTE |= (1<<6); }
  static void Clear()     { PORTE &= (~(1<<6)); }
  static void Toggle()    { PORTE ^= (1<<6); }
  static void SetOutput() { DDRE |= (1<<6); }
  static void SetInput()  { DDRE &= (~(1<<6)); }
  static void PulseHigh() { PORTE |= (1<<6); PORTE &= (~(1<<6)); }
  static void PulseLow()  { PORTE &= (~(1<<6)); PORTE |= (1<<6); }
  static bool Test()      { return PINE & (1<<6); }

  static constexpr decltype(PORTE) GetNativePort() { return PORTE; }
  static constexpr uint8_t bitmask()               { return (1<<6); }
  static constexpr uint8_t bit()                   { return 6; }

  typedef PortE Port;
};

class PinE7 {
 public:
  static void Set()       { PORTE |= (1<<7); }
  static void Clear()     { PORTE &= (~(1<<7)); }
  static void Toggle()    { PORTE ^= (1<<7); }
  static void SetOutput() { DDRE |= (1<<7); }
  static void SetInput()  { DDRE &= (~(1<<7)); }
  static void PulseHigh() { PORTE |= (1<<7); PORTE &= (~(1<<7)); }
  static void PulseLow()  { PORTE &= (~(1<<7)); PORTE |= (1<<7); }
  static bool Test()      { return PINE & (1<<7); }

  static constexpr decltype(PORTE) GetNativePort() { return PORTE; }
  static constexpr uint8_t bitmask()               { return (1<<7); }
  static constexpr uint8_t bit()                   { return 7; }

  typedef PortE Port;
};

#endif //PORTE

#ifdef PORTF
class PortF {
 public:

  /// Assigns a value to PORTF.
  /// @param[in] value value affected to PORTF
  static void Assign(uint8_t value)   { PORTF = value; }

  /// Sets masked bits in PORTF.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTF |= mask;}

  /// Clears masked bits in PORTF.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTF &= (~mask);} 

  /// Toggles masked bits in PORTF.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTF ^= (~mask);} 

  /// Pulses masked bits in PORTF with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTF |= mask; PORTF &= (~mask); }

  /// Pulses masked bits in PORTF with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTF &= (~mask); PORTF |= mask; }

  /// Set corresponding masked bits of PORTF to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRF |= mask; }

  /// Set corresponding masked bits of PORTF to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRF &= (~mask); }

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

class PinF0 {
 public:
  static void Set()       { PORTF |= (1<<0); }
  static void Clear()     { PORTF &= (~(1<<0)); }
  static void Toggle()    { PORTF ^= (1<<0); }
  static void SetOutput() { DDRF |= (1<<0); }
  static void SetInput()  { DDRF &= (~(1<<0)); }
  static void PulseHigh() { PORTF |= (1<<0); PORTF &= (~(1<<0)); }
  static void PulseLow()  { PORTF &= (~(1<<0)); PORTF |= (1<<0); }
  static bool Test()      { return PINF & (1<<0); }

  static constexpr decltype(PORTF) GetNativePort() { return PORTF; }
  static constexpr uint8_t bitmask()               { return (1<<0); }
  static constexpr uint8_t bit()                   { return 0; }

  typedef PortF Port;
};

class PinF1 {
 public:
  static void Set()       { PORTF |= (1<<1); }
  static void Clear()     { PORTF &= (~(1<<1)); }
  static void Toggle()    { PORTF ^= (1<<1); }
  static void SetOutput() { DDRF |= (1<<1); }
  static void SetInput()  { DDRF &= (~(1<<1)); }
  static void PulseHigh() { PORTF |= (1<<1); PORTF &= (~(1<<1)); }
  static void PulseLow()  { PORTF &= (~(1<<1)); PORTF |= (1<<1); }
  static bool Test()      { return PINF & (1<<1); }

  static constexpr decltype(PORTF) GetNativePort() { return PORTF; }
  static constexpr uint8_t bitmask()               { return (1<<1); }
  static constexpr uint8_t bit()                   { return 1; }

  typedef PortF Port;
};

class PinF2 {
 public:
  static void Set()       { PORTF |= (1<<2); }
  static void Clear()     { PORTF &= (~(1<<2)); }
  static void Toggle()    { PORTF ^= (1<<2); }
  static void SetOutput() { DDRF |= (1<<2); }
  static void SetInput()  { DDRF &= (~(1<<2)); }
  static void PulseHigh() { PORTF |= (1<<2); PORTF &= (~(1<<2)); }
  static void PulseLow()  { PORTF &= (~(1<<2)); PORTF |= (1<<2); }
  static bool Test()      { return PINF & (1<<2); }

  static constexpr decltype(PORTF) GetNativePort() { return PORTF; }
  static constexpr uint8_t bitmask()               { return (1<<2); }
  static constexpr uint8_t bit()                   { return 2; }

  typedef PortF Port;
};

class PinF3 {
 public:
  static void Set()       { PORTF |= (1<<3); }
  static void Clear()     { PORTF &= (~(1<<3)); }
  static void Toggle()    { PORTF ^= (1<<3); }
  static void SetOutput() { DDRF |= (1<<3); }
  static void SetInput()  { DDRF &= (~(1<<3)); }
  static void PulseHigh() { PORTF |= (1<<3); PORTF &= (~(1<<3)); }
  static void PulseLow()  { PORTF &= (~(1<<3)); PORTF |= (1<<3); }
  static bool Test()      { return PINF & (1<<3); }

  static constexpr decltype(PORTF) GetNativePort() { return PORTF; }
  static constexpr uint8_t bitmask()               { return (1<<3); }
  static constexpr uint8_t bit()                   { return 3; }

  typedef PortF Port;
};

class PinF4 {
 public:
  static void Set()       { PORTF |= (1<<4); }
  static void Clear()     { PORTF &= (~(1<<4)); }
  static void Toggle()    { PORTF ^= (1<<4); }
  static void SetOutput() { DDRF |= (1<<4); }
  static void SetInput()  { DDRF &= (~(1<<4)); }
  static void PulseHigh() { PORTF |= (1<<4); PORTF &= (~(1<<4)); }
  static void PulseLow()  { PORTF &= (~(1<<4)); PORTF |= (1<<4); }
  static bool Test()      { return PINF & (1<<4); }

  static constexpr decltype(PORTF) GetNativePort() { return PORTF; }
  static constexpr uint8_t bitmask()               { return (1<<4); }
  static constexpr uint8_t bit()                   { return 4; }

  typedef PortF Port;
};

class PinF5 {
 public:
  static void Set()       { PORTF |= (1<<5); }
  static void Clear()     { PORTF &= (~(1<<5)); }
  static void Toggle()    { PORTF ^= (1<<5); }
  static void SetOutput() { DDRF |= (1<<5); }
  static void SetInput()  { DDRF &= (~(1<<5)); }
  static void PulseHigh() { PORTF |= (1<<5); PORTF &= (~(1<<5)); }
  static void PulseLow()  { PORTF &= (~(1<<5)); PORTF |= (1<<5); }
  static bool Test()      { return PINF & (1<<5); }

  static constexpr decltype(PORTF) GetNativePort() { return PORTF; }
  static constexpr uint8_t bitmask()               { return (1<<5); }
  static constexpr uint8_t bit()                   { return 5; }

  typedef PortF Port;
};

class PinF6 {
 public:
  static void Set()       { PORTF |= (1<<6); }
  static void Clear()     { PORTF &= (~(1<<6)); }
  static void Toggle()    { PORTF ^= (1<<6); }
  static void SetOutput() { DDRF |= (1<<6); }
  static void SetInput()  { DDRF &= (~(1<<6)); }
  static void PulseHigh() { PORTF |= (1<<6); PORTF &= (~(1<<6)); }
  static void PulseLow()  { PORTF &= (~(1<<6)); PORTF |= (1<<6); }
  static bool Test()      { return PINF & (1<<6); }

  static constexpr decltype(PORTF) GetNativePort() { return PORTF; }
  static constexpr uint8_t bitmask()               { return (1<<6); }
  static constexpr uint8_t bit()                   { return 6; }

  typedef PortF Port;
};

class PinF7 {
 public:
  static void Set()       { PORTF |= (1<<7); }
  static void Clear()     { PORTF &= (~(1<<7)); }
  static void Toggle()    { PORTF ^= (1<<7); }
  static void SetOutput() { DDRF |= (1<<7); }
  static void SetInput()  { DDRF &= (~(1<<7)); }
  static void PulseHigh() { PORTF |= (1<<7); PORTF &= (~(1<<7)); }
  static void PulseLow()  { PORTF &= (~(1<<7)); PORTF |= (1<<7); }
  static bool Test()      { return PINF & (1<<7); }

  static constexpr decltype(PORTF) GetNativePort() { return PORTF; }
  static constexpr uint8_t bitmask()               { return (1<<7); }
  static constexpr uint8_t bit()                   { return 7; }

  typedef PortF Port;
};

#endif //PORTF

#ifdef PORTG
class PortG {
 public:

  /// Assigns a value to PORTG.
  /// @param[in] value value affected to PORTG
  static void Assign(uint8_t value)   { PORTG = value; }

  /// Sets masked bits in PORTG.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTG |= mask;}

  /// Clears masked bits in PORTG.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTG &= (~mask);} 

  /// Toggles masked bits in PORTG.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTG ^= (~mask);} 

  /// Pulses masked bits in PORTG with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTG |= mask; PORTG &= (~mask); }

  /// Pulses masked bits in PORTG with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTG &= (~mask); PORTG |= mask; }

  /// Set corresponding masked bits of PORTG to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRG |= mask; }

  /// Set corresponding masked bits of PORTG to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRG &= (~mask); }

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

class PinG0 {
 public:
  static void Set()       { PORTG |= (1<<0); }
  static void Clear()     { PORTG &= (~(1<<0)); }
  static void Toggle()    { PORTG ^= (1<<0); }
  static void SetOutput() { DDRG |= (1<<0); }
  static void SetInput()  { DDRG &= (~(1<<0)); }
  static void PulseHigh() { PORTG |= (1<<0); PORTG &= (~(1<<0)); }
  static void PulseLow()  { PORTG &= (~(1<<0)); PORTG |= (1<<0); }
  static bool Test()      { return PING & (1<<0); }

  static constexpr decltype(PORTG) GetNativePort() { return PORTG; }
  static constexpr uint8_t bitmask()               { return (1<<0); }
  static constexpr uint8_t bit()                   { return 0; }

  typedef PortG Port;
};

class PinG1 {
 public:
  static void Set()       { PORTG |= (1<<1); }
  static void Clear()     { PORTG &= (~(1<<1)); }
  static void Toggle()    { PORTG ^= (1<<1); }
  static void SetOutput() { DDRG |= (1<<1); }
  static void SetInput()  { DDRG &= (~(1<<1)); }
  static void PulseHigh() { PORTG |= (1<<1); PORTG &= (~(1<<1)); }
  static void PulseLow()  { PORTG &= (~(1<<1)); PORTG |= (1<<1); }
  static bool Test()      { return PING & (1<<1); }

  static constexpr decltype(PORTG) GetNativePort() { return PORTG; }
  static constexpr uint8_t bitmask()               { return (1<<1); }
  static constexpr uint8_t bit()                   { return 1; }

  typedef PortG Port;
};

class PinG2 {
 public:
  static void Set()       { PORTG |= (1<<2); }
  static void Clear()     { PORTG &= (~(1<<2)); }
  static void Toggle()    { PORTG ^= (1<<2); }
  static void SetOutput() { DDRG |= (1<<2); }
  static void SetInput()  { DDRG &= (~(1<<2)); }
  static void PulseHigh() { PORTG |= (1<<2); PORTG &= (~(1<<2)); }
  static void PulseLow()  { PORTG &= (~(1<<2)); PORTG |= (1<<2); }
  static bool Test()      { return PING & (1<<2); }

  static constexpr decltype(PORTG) GetNativePort() { return PORTG; }
  static constexpr uint8_t bitmask()               { return (1<<2); }
  static constexpr uint8_t bit()                   { return 2; }

  typedef PortG Port;
};

class PinG3 {
 public:
  static void Set()       { PORTG |= (1<<3); }
  static void Clear()     { PORTG &= (~(1<<3)); }
  static void Toggle()    { PORTG ^= (1<<3); }
  static void SetOutput() { DDRG |= (1<<3); }
  static void SetInput()  { DDRG &= (~(1<<3)); }
  static void PulseHigh() { PORTG |= (1<<3); PORTG &= (~(1<<3)); }
  static void PulseLow()  { PORTG &= (~(1<<3)); PORTG |= (1<<3); }
  static bool Test()      { return PING & (1<<3); }

  static constexpr decltype(PORTG) GetNativePort() { return PORTG; }
  static constexpr uint8_t bitmask()               { return (1<<3); }
  static constexpr uint8_t bit()                   { return 3; }

  typedef PortG Port;
};

class PinG4 {
 public:
  static void Set()       { PORTG |= (1<<4); }
  static void Clear()     { PORTG &= (~(1<<4)); }
  static void Toggle()    { PORTG ^= (1<<4); }
  static void SetOutput() { DDRG |= (1<<4); }
  static void SetInput()  { DDRG &= (~(1<<4)); }
  static void PulseHigh() { PORTG |= (1<<4); PORTG &= (~(1<<4)); }
  static void PulseLow()  { PORTG &= (~(1<<4)); PORTG |= (1<<4); }
  static bool Test()      { return PING & (1<<4); }

  static constexpr decltype(PORTG) GetNativePort() { return PORTG; }
  static constexpr uint8_t bitmask()               { return (1<<4); }
  static constexpr uint8_t bit()                   { return 4; }

  typedef PortG Port;
};

class PinG5 {
 public:
  static void Set()       { PORTG |= (1<<5); }
  static void Clear()     { PORTG &= (~(1<<5)); }
  static void Toggle()    { PORTG ^= (1<<5); }
  static void SetOutput() { DDRG |= (1<<5); }
  static void SetInput()  { DDRG &= (~(1<<5)); }
  static void PulseHigh() { PORTG |= (1<<5); PORTG &= (~(1<<5)); }
  static void PulseLow()  { PORTG &= (~(1<<5)); PORTG |= (1<<5); }
  static bool Test()      { return PING & (1<<5); }

  static constexpr decltype(PORTG) GetNativePort() { return PORTG; }
  static constexpr uint8_t bitmask()               { return (1<<5); }
  static constexpr uint8_t bit()                   { return 5; }

  typedef PortG Port;
};

class PinG6 {
 public:
  static void Set()       { PORTG |= (1<<6); }
  static void Clear()     { PORTG &= (~(1<<6)); }
  static void Toggle()    { PORTG ^= (1<<6); }
  static void SetOutput() { DDRG |= (1<<6); }
  static void SetInput()  { DDRG &= (~(1<<6)); }
  static void PulseHigh() { PORTG |= (1<<6); PORTG &= (~(1<<6)); }
  static void PulseLow()  { PORTG &= (~(1<<6)); PORTG |= (1<<6); }
  static bool Test()      { return PING & (1<<6); }

  static constexpr decltype(PORTG) GetNativePort() { return PORTG; }
  static constexpr uint8_t bitmask()               { return (1<<6); }
  static constexpr uint8_t bit()                   { return 6; }

  typedef PortG Port;
};

class PinG7 {
 public:
  static void Set()       { PORTG |= (1<<7); }
  static void Clear()     { PORTG &= (~(1<<7)); }
  static void Toggle()    { PORTG ^= (1<<7); }
  static void SetOutput() { DDRG |= (1<<7); }
  static void SetInput()  { DDRG &= (~(1<<7)); }
  static void PulseHigh() { PORTG |= (1<<7); PORTG &= (~(1<<7)); }
  static void PulseLow()  { PORTG &= (~(1<<7)); PORTG |= (1<<7); }
  static bool Test()      { return PING & (1<<7); }

  static constexpr decltype(PORTG) GetNativePort() { return PORTG; }
  static constexpr uint8_t bitmask()               { return (1<<7); }
  static constexpr uint8_t bit()                   { return 7; }

  typedef PortG Port;
};

#endif //PORTG

#ifdef PORTH
class PortH {
 public:

  /// Assigns a value to PORTH.
  /// @param[in] value value affected to PORTH
  static void Assign(uint8_t value)   { PORTH = value; }

  /// Sets masked bits in PORTH.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTH |= mask;}

  /// Clears masked bits in PORTH.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTH &= (~mask);} 

  /// Toggles masked bits in PORTH.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTH ^= (~mask);} 

  /// Pulses masked bits in PORTH with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTH |= mask; PORTH &= (~mask); }

  /// Pulses masked bits in PORTH with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTH &= (~mask); PORTH |= mask; }

  /// Set corresponding masked bits of PORTH to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRH |= mask; }

  /// Set corresponding masked bits of PORTH to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRH &= (~mask); }

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

class PinH0 {
 public:
  static void Set()       { PORTH |= (1<<0); }
  static void Clear()     { PORTH &= (~(1<<0)); }
  static void Toggle()    { PORTH ^= (1<<0); }
  static void SetOutput() { DDRH |= (1<<0); }
  static void SetInput()  { DDRH &= (~(1<<0)); }
  static void PulseHigh() { PORTH |= (1<<0); PORTH &= (~(1<<0)); }
  static void PulseLow()  { PORTH &= (~(1<<0)); PORTH |= (1<<0); }
  static bool Test()      { return PINH & (1<<0); }

  static constexpr decltype(PORTH) GetNativePort() { return PORTH; }
  static constexpr uint8_t bitmask()               { return (1<<0); }
  static constexpr uint8_t bit()                   { return 0; }

  typedef PortH Port;
};

class PinH1 {
 public:
  static void Set()       { PORTH |= (1<<1); }
  static void Clear()     { PORTH &= (~(1<<1)); }
  static void Toggle()    { PORTH ^= (1<<1); }
  static void SetOutput() { DDRH |= (1<<1); }
  static void SetInput()  { DDRH &= (~(1<<1)); }
  static void PulseHigh() { PORTH |= (1<<1); PORTH &= (~(1<<1)); }
  static void PulseLow()  { PORTH &= (~(1<<1)); PORTH |= (1<<1); }
  static bool Test()      { return PINH & (1<<1); }

  static constexpr decltype(PORTH) GetNativePort() { return PORTH; }
  static constexpr uint8_t bitmask()               { return (1<<1); }
  static constexpr uint8_t bit()                   { return 1; }

  typedef PortH Port;
};

class PinH2 {
 public:
  static void Set()       { PORTH |= (1<<2); }
  static void Clear()     { PORTH &= (~(1<<2)); }
  static void Toggle()    { PORTH ^= (1<<2); }
  static void SetOutput() { DDRH |= (1<<2); }
  static void SetInput()  { DDRH &= (~(1<<2)); }
  static void PulseHigh() { PORTH |= (1<<2); PORTH &= (~(1<<2)); }
  static void PulseLow()  { PORTH &= (~(1<<2)); PORTH |= (1<<2); }
  static bool Test()      { return PINH & (1<<2); }

  static constexpr decltype(PORTH) GetNativePort() { return PORTH; }
  static constexpr uint8_t bitmask()               { return (1<<2); }
  static constexpr uint8_t bit()                   { return 2; }

  typedef PortH Port;
};

class PinH3 {
 public:
  static void Set()       { PORTH |= (1<<3); }
  static void Clear()     { PORTH &= (~(1<<3)); }
  static void Toggle()    { PORTH ^= (1<<3); }
  static void SetOutput() { DDRH |= (1<<3); }
  static void SetInput()  { DDRH &= (~(1<<3)); }
  static void PulseHigh() { PORTH |= (1<<3); PORTH &= (~(1<<3)); }
  static void PulseLow()  { PORTH &= (~(1<<3)); PORTH |= (1<<3); }
  static bool Test()      { return PINH & (1<<3); }

  static constexpr decltype(PORTH) GetNativePort() { return PORTH; }
  static constexpr uint8_t bitmask()               { return (1<<3); }
  static constexpr uint8_t bit()                   { return 3; }

  typedef PortH Port;
};

class PinH4 {
 public:
  static void Set()       { PORTH |= (1<<4); }
  static void Clear()     { PORTH &= (~(1<<4)); }
  static void Toggle()    { PORTH ^= (1<<4); }
  static void SetOutput() { DDRH |= (1<<4); }
  static void SetInput()  { DDRH &= (~(1<<4)); }
  static void PulseHigh() { PORTH |= (1<<4); PORTH &= (~(1<<4)); }
  static void PulseLow()  { PORTH &= (~(1<<4)); PORTH |= (1<<4); }
  static bool Test()      { return PINH & (1<<4); }

  static constexpr decltype(PORTH) GetNativePort() { return PORTH; }
  static constexpr uint8_t bitmask()               { return (1<<4); }
  static constexpr uint8_t bit()                   { return 4; }

  typedef PortH Port;
};

class PinH5 {
 public:
  static void Set()       { PORTH |= (1<<5); }
  static void Clear()     { PORTH &= (~(1<<5)); }
  static void Toggle()    { PORTH ^= (1<<5); }
  static void SetOutput() { DDRH |= (1<<5); }
  static void SetInput()  { DDRH &= (~(1<<5)); }
  static void PulseHigh() { PORTH |= (1<<5); PORTH &= (~(1<<5)); }
  static void PulseLow()  { PORTH &= (~(1<<5)); PORTH |= (1<<5); }
  static bool Test()      { return PINH & (1<<5); }

  static constexpr decltype(PORTH) GetNativePort() { return PORTH; }
  static constexpr uint8_t bitmask()               { return (1<<5); }
  static constexpr uint8_t bit()                   { return 5; }

  typedef PortH Port;
};

class PinH6 {
 public:
  static void Set()       { PORTH |= (1<<6); }
  static void Clear()     { PORTH &= (~(1<<6)); }
  static void Toggle()    { PORTH ^= (1<<6); }
  static void SetOutput() { DDRH |= (1<<6); }
  static void SetInput()  { DDRH &= (~(1<<6)); }
  static void PulseHigh() { PORTH |= (1<<6); PORTH &= (~(1<<6)); }
  static void PulseLow()  { PORTH &= (~(1<<6)); PORTH |= (1<<6); }
  static bool Test()      { return PINH & (1<<6); }

  static constexpr decltype(PORTH) GetNativePort() { return PORTH; }
  static constexpr uint8_t bitmask()               { return (1<<6); }
  static constexpr uint8_t bit()                   { return 6; }

  typedef PortH Port;
};

class PinH7 {
 public:
  static void Set()       { PORTH |= (1<<7); }
  static void Clear()     { PORTH &= (~(1<<7)); }
  static void Toggle()    { PORTH ^= (1<<7); }
  static void SetOutput() { DDRH |= (1<<7); }
  static void SetInput()  { DDRH &= (~(1<<7)); }
  static void PulseHigh() { PORTH |= (1<<7); PORTH &= (~(1<<7)); }
  static void PulseLow()  { PORTH &= (~(1<<7)); PORTH |= (1<<7); }
  static bool Test()      { return PINH & (1<<7); }

  static constexpr decltype(PORTH) GetNativePort() { return PORTH; }
  static constexpr uint8_t bitmask()               { return (1<<7); }
  static constexpr uint8_t bit()                   { return 7; }

  typedef PortH Port;
};

#endif //PORTH

#ifdef PORTI
class PortI {
 public:

  /// Assigns a value to PORTI.
  /// @param[in] value value affected to PORTI
  static void Assign(uint8_t value)   { PORTI = value; }

  /// Sets masked bits in PORTI.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTI |= mask;}

  /// Clears masked bits in PORTI.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTI &= (~mask);} 

  /// Toggles masked bits in PORTI.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTI ^= (~mask);} 

  /// Pulses masked bits in PORTI with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTI |= mask; PORTI &= (~mask); }

  /// Pulses masked bits in PORTI with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTI &= (~mask); PORTI |= mask; }

  /// Set corresponding masked bits of PORTI to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRI |= mask; }

  /// Set corresponding masked bits of PORTI to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRI &= (~mask); }

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

class PinI0 {
 public:
  static void Set()       { PORTI |= (1<<0); }
  static void Clear()     { PORTI &= (~(1<<0)); }
  static void Toggle()    { PORTI ^= (1<<0); }
  static void SetOutput() { DDRI |= (1<<0); }
  static void SetInput()  { DDRI &= (~(1<<0)); }
  static void PulseHigh() { PORTI |= (1<<0); PORTI &= (~(1<<0)); }
  static void PulseLow()  { PORTI &= (~(1<<0)); PORTI |= (1<<0); }
  static bool Test()      { return PINI & (1<<0); }

  static constexpr decltype(PORTI) GetNativePort() { return PORTI; }
  static constexpr uint8_t bitmask()               { return (1<<0); }
  static constexpr uint8_t bit()                   { return 0; }

  typedef PortI Port;
};

class PinI1 {
 public:
  static void Set()       { PORTI |= (1<<1); }
  static void Clear()     { PORTI &= (~(1<<1)); }
  static void Toggle()    { PORTI ^= (1<<1); }
  static void SetOutput() { DDRI |= (1<<1); }
  static void SetInput()  { DDRI &= (~(1<<1)); }
  static void PulseHigh() { PORTI |= (1<<1); PORTI &= (~(1<<1)); }
  static void PulseLow()  { PORTI &= (~(1<<1)); PORTI |= (1<<1); }
  static bool Test()      { return PINI & (1<<1); }

  static constexpr decltype(PORTI) GetNativePort() { return PORTI; }
  static constexpr uint8_t bitmask()               { return (1<<1); }
  static constexpr uint8_t bit()                   { return 1; }

  typedef PortI Port;
};

class PinI2 {
 public:
  static void Set()       { PORTI |= (1<<2); }
  static void Clear()     { PORTI &= (~(1<<2)); }
  static void Toggle()    { PORTI ^= (1<<2); }
  static void SetOutput() { DDRI |= (1<<2); }
  static void SetInput()  { DDRI &= (~(1<<2)); }
  static void PulseHigh() { PORTI |= (1<<2); PORTI &= (~(1<<2)); }
  static void PulseLow()  { PORTI &= (~(1<<2)); PORTI |= (1<<2); }
  static bool Test()      { return PINI & (1<<2); }

  static constexpr decltype(PORTI) GetNativePort() { return PORTI; }
  static constexpr uint8_t bitmask()               { return (1<<2); }
  static constexpr uint8_t bit()                   { return 2; }

  typedef PortI Port;
};

class PinI3 {
 public:
  static void Set()       { PORTI |= (1<<3); }
  static void Clear()     { PORTI &= (~(1<<3)); }
  static void Toggle()    { PORTI ^= (1<<3); }
  static void SetOutput() { DDRI |= (1<<3); }
  static void SetInput()  { DDRI &= (~(1<<3)); }
  static void PulseHigh() { PORTI |= (1<<3); PORTI &= (~(1<<3)); }
  static void PulseLow()  { PORTI &= (~(1<<3)); PORTI |= (1<<3); }
  static bool Test()      { return PINI & (1<<3); }

  static constexpr decltype(PORTI) GetNativePort() { return PORTI; }
  static constexpr uint8_t bitmask()               { return (1<<3); }
  static constexpr uint8_t bit()                   { return 3; }

  typedef PortI Port;
};

class PinI4 {
 public:
  static void Set()       { PORTI |= (1<<4); }
  static void Clear()     { PORTI &= (~(1<<4)); }
  static void Toggle()    { PORTI ^= (1<<4); }
  static void SetOutput() { DDRI |= (1<<4); }
  static void SetInput()  { DDRI &= (~(1<<4)); }
  static void PulseHigh() { PORTI |= (1<<4); PORTI &= (~(1<<4)); }
  static void PulseLow()  { PORTI &= (~(1<<4)); PORTI |= (1<<4); }
  static bool Test()      { return PINI & (1<<4); }

  static constexpr decltype(PORTI) GetNativePort() { return PORTI; }
  static constexpr uint8_t bitmask()               { return (1<<4); }
  static constexpr uint8_t bit()                   { return 4; }

  typedef PortI Port;
};

class PinI5 {
 public:
  static void Set()       { PORTI |= (1<<5); }
  static void Clear()     { PORTI &= (~(1<<5)); }
  static void Toggle()    { PORTI ^= (1<<5); }
  static void SetOutput() { DDRI |= (1<<5); }
  static void SetInput()  { DDRI &= (~(1<<5)); }
  static void PulseHigh() { PORTI |= (1<<5); PORTI &= (~(1<<5)); }
  static void PulseLow()  { PORTI &= (~(1<<5)); PORTI |= (1<<5); }
  static bool Test()      { return PINI & (1<<5); }

  static constexpr decltype(PORTI) GetNativePort() { return PORTI; }
  static constexpr uint8_t bitmask()               { return (1<<5); }
  static constexpr uint8_t bit()                   { return 5; }

  typedef PortI Port;
};

class PinI6 {
 public:
  static void Set()       { PORTI |= (1<<6); }
  static void Clear()     { PORTI &= (~(1<<6)); }
  static void Toggle()    { PORTI ^= (1<<6); }
  static void SetOutput() { DDRI |= (1<<6); }
  static void SetInput()  { DDRI &= (~(1<<6)); }
  static void PulseHigh() { PORTI |= (1<<6); PORTI &= (~(1<<6)); }
  static void PulseLow()  { PORTI &= (~(1<<6)); PORTI |= (1<<6); }
  static bool Test()      { return PINI & (1<<6); }

  static constexpr decltype(PORTI) GetNativePort() { return PORTI; }
  static constexpr uint8_t bitmask()               { return (1<<6); }
  static constexpr uint8_t bit()                   { return 6; }

  typedef PortI Port;
};

class PinI7 {
 public:
  static void Set()       { PORTI |= (1<<7); }
  static void Clear()     { PORTI &= (~(1<<7)); }
  static void Toggle()    { PORTI ^= (1<<7); }
  static void SetOutput() { DDRI |= (1<<7); }
  static void SetInput()  { DDRI &= (~(1<<7)); }
  static void PulseHigh() { PORTI |= (1<<7); PORTI &= (~(1<<7)); }
  static void PulseLow()  { PORTI &= (~(1<<7)); PORTI |= (1<<7); }
  static bool Test()      { return PINI & (1<<7); }

  static constexpr decltype(PORTI) GetNativePort() { return PORTI; }
  static constexpr uint8_t bitmask()               { return (1<<7); }
  static constexpr uint8_t bit()                   { return 7; }

  typedef PortI Port;
};

#endif //PORTI

#ifdef PORTJ
class PortJ {
 public:

  /// Assigns a value to PORTJ.
  /// @param[in] value value affected to PORTJ
  static void Assign(uint8_t value)   { PORTJ = value; }

  /// Sets masked bits in PORTJ.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTJ |= mask;}

  /// Clears masked bits in PORTJ.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTJ &= (~mask);} 

  /// Toggles masked bits in PORTJ.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTJ ^= (~mask);} 

  /// Pulses masked bits in PORTJ with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTJ |= mask; PORTJ &= (~mask); }

  /// Pulses masked bits in PORTJ with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTJ &= (~mask); PORTJ |= mask; }

  /// Set corresponding masked bits of PORTJ to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRJ |= mask; }

  /// Set corresponding masked bits of PORTJ to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRJ &= (~mask); }

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

class PinJ0 {
 public:
  static void Set()       { PORTJ |= (1<<0); }
  static void Clear()     { PORTJ &= (~(1<<0)); }
  static void Toggle()    { PORTJ ^= (1<<0); }
  static void SetOutput() { DDRJ |= (1<<0); }
  static void SetInput()  { DDRJ &= (~(1<<0)); }
  static void PulseHigh() { PORTJ |= (1<<0); PORTJ &= (~(1<<0)); }
  static void PulseLow()  { PORTJ &= (~(1<<0)); PORTJ |= (1<<0); }
  static bool Test()      { return PINJ & (1<<0); }

  static constexpr decltype(PORTJ) GetNativePort() { return PORTJ; }
  static constexpr uint8_t bitmask()               { return (1<<0); }
  static constexpr uint8_t bit()                   { return 0; }

  typedef PortJ Port;
};

class PinJ1 {
 public:
  static void Set()       { PORTJ |= (1<<1); }
  static void Clear()     { PORTJ &= (~(1<<1)); }
  static void Toggle()    { PORTJ ^= (1<<1); }
  static void SetOutput() { DDRJ |= (1<<1); }
  static void SetInput()  { DDRJ &= (~(1<<1)); }
  static void PulseHigh() { PORTJ |= (1<<1); PORTJ &= (~(1<<1)); }
  static void PulseLow()  { PORTJ &= (~(1<<1)); PORTJ |= (1<<1); }
  static bool Test()      { return PINJ & (1<<1); }

  static constexpr decltype(PORTJ) GetNativePort() { return PORTJ; }
  static constexpr uint8_t bitmask()               { return (1<<1); }
  static constexpr uint8_t bit()                   { return 1; }

  typedef PortJ Port;
};

class PinJ2 {
 public:
  static void Set()       { PORTJ |= (1<<2); }
  static void Clear()     { PORTJ &= (~(1<<2)); }
  static void Toggle()    { PORTJ ^= (1<<2); }
  static void SetOutput() { DDRJ |= (1<<2); }
  static void SetInput()  { DDRJ &= (~(1<<2)); }
  static void PulseHigh() { PORTJ |= (1<<2); PORTJ &= (~(1<<2)); }
  static void PulseLow()  { PORTJ &= (~(1<<2)); PORTJ |= (1<<2); }
  static bool Test()      { return PINJ & (1<<2); }

  static constexpr decltype(PORTJ) GetNativePort() { return PORTJ; }
  static constexpr uint8_t bitmask()               { return (1<<2); }
  static constexpr uint8_t bit()                   { return 2; }

  typedef PortJ Port;
};

class PinJ3 {
 public:
  static void Set()       { PORTJ |= (1<<3); }
  static void Clear()     { PORTJ &= (~(1<<3)); }
  static void Toggle()    { PORTJ ^= (1<<3); }
  static void SetOutput() { DDRJ |= (1<<3); }
  static void SetInput()  { DDRJ &= (~(1<<3)); }
  static void PulseHigh() { PORTJ |= (1<<3); PORTJ &= (~(1<<3)); }
  static void PulseLow()  { PORTJ &= (~(1<<3)); PORTJ |= (1<<3); }
  static bool Test()      { return PINJ & (1<<3); }

  static constexpr decltype(PORTJ) GetNativePort() { return PORTJ; }
  static constexpr uint8_t bitmask()               { return (1<<3); }
  static constexpr uint8_t bit()                   { return 3; }

  typedef PortJ Port;
};

class PinJ4 {
 public:
  static void Set()       { PORTJ |= (1<<4); }
  static void Clear()     { PORTJ &= (~(1<<4)); }
  static void Toggle()    { PORTJ ^= (1<<4); }
  static void SetOutput() { DDRJ |= (1<<4); }
  static void SetInput()  { DDRJ &= (~(1<<4)); }
  static void PulseHigh() { PORTJ |= (1<<4); PORTJ &= (~(1<<4)); }
  static void PulseLow()  { PORTJ &= (~(1<<4)); PORTJ |= (1<<4); }
  static bool Test()      { return PINJ & (1<<4); }

  static constexpr decltype(PORTJ) GetNativePort() { return PORTJ; }
  static constexpr uint8_t bitmask()               { return (1<<4); }
  static constexpr uint8_t bit()                   { return 4; }

  typedef PortJ Port;
};

class PinJ5 {
 public:
  static void Set()       { PORTJ |= (1<<5); }
  static void Clear()     { PORTJ &= (~(1<<5)); }
  static void Toggle()    { PORTJ ^= (1<<5); }
  static void SetOutput() { DDRJ |= (1<<5); }
  static void SetInput()  { DDRJ &= (~(1<<5)); }
  static void PulseHigh() { PORTJ |= (1<<5); PORTJ &= (~(1<<5)); }
  static void PulseLow()  { PORTJ &= (~(1<<5)); PORTJ |= (1<<5); }
  static bool Test()      { return PINJ & (1<<5); }

  static constexpr decltype(PORTJ) GetNativePort() { return PORTJ; }
  static constexpr uint8_t bitmask()               { return (1<<5); }
  static constexpr uint8_t bit()                   { return 5; }

  typedef PortJ Port;
};

class PinJ6 {
 public:
  static void Set()       { PORTJ |= (1<<6); }
  static void Clear()     { PORTJ &= (~(1<<6)); }
  static void Toggle()    { PORTJ ^= (1<<6); }
  static void SetOutput() { DDRJ |= (1<<6); }
  static void SetInput()  { DDRJ &= (~(1<<6)); }
  static void PulseHigh() { PORTJ |= (1<<6); PORTJ &= (~(1<<6)); }
  static void PulseLow()  { PORTJ &= (~(1<<6)); PORTJ |= (1<<6); }
  static bool Test()      { return PINJ & (1<<6); }

  static constexpr decltype(PORTJ) GetNativePort() { return PORTJ; }
  static constexpr uint8_t bitmask()               { return (1<<6); }
  static constexpr uint8_t bit()                   { return 6; }

  typedef PortJ Port;
};

class PinJ7 {
 public:
  static void Set()       { PORTJ |= (1<<7); }
  static void Clear()     { PORTJ &= (~(1<<7)); }
  static void Toggle()    { PORTJ ^= (1<<7); }
  static void SetOutput() { DDRJ |= (1<<7); }
  static void SetInput()  { DDRJ &= (~(1<<7)); }
  static void PulseHigh() { PORTJ |= (1<<7); PORTJ &= (~(1<<7)); }
  static void PulseLow()  { PORTJ &= (~(1<<7)); PORTJ |= (1<<7); }
  static bool Test()      { return PINJ & (1<<7); }

  static constexpr decltype(PORTJ) GetNativePort() { return PORTJ; }
  static constexpr uint8_t bitmask()               { return (1<<7); }
  static constexpr uint8_t bit()                   { return 7; }

  typedef PortJ Port;
};

#endif //PORTJ

#ifdef PORTK
class PortK {
 public:

  /// Assigns a value to PORTK.
  /// @param[in] value value affected to PORTK
  static void Assign(uint8_t value)   { PORTK = value; }

  /// Sets masked bits in PORTK.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { PORTK |= mask;}

  /// Clears masked bits in PORTK.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { PORTK &= (~mask);} 

  /// Toggles masked bits in PORTK.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { PORTK ^= (~mask);} 

  /// Pulses masked bits in PORTK with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { PORTK |= mask; PORTK &= (~mask); }

  /// Pulses masked bits in PORTK with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { PORTK &= (~mask); PORTK |= mask; }

  /// Set corresponding masked bits of PORTK to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { DDRK |= mask; }

  /// Set corresponding masked bits of PORTK to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { DDRK &= (~mask); }

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

class PinK0 {
 public:
  static void Set()       { PORTK |= (1<<0); }
  static void Clear()     { PORTK &= (~(1<<0)); }
  static void Toggle()    { PORTK ^= (1<<0); }
  static void SetOutput() { DDRK |= (1<<0); }
  static void SetInput()  { DDRK &= (~(1<<0)); }
  static void PulseHigh() { PORTK |= (1<<0); PORTK &= (~(1<<0)); }
  static void PulseLow()  { PORTK &= (~(1<<0)); PORTK |= (1<<0); }
  static bool Test()      { return PINK & (1<<0); }

  static constexpr decltype(PORTK) GetNativePort() { return PORTK; }
  static constexpr uint8_t bitmask()               { return (1<<0); }
  static constexpr uint8_t bit()                   { return 0; }

  typedef PortK Port;
};

class PinK1 {
 public:
  static void Set()       { PORTK |= (1<<1); }
  static void Clear()     { PORTK &= (~(1<<1)); }
  static void Toggle()    { PORTK ^= (1<<1); }
  static void SetOutput() { DDRK |= (1<<1); }
  static void SetInput()  { DDRK &= (~(1<<1)); }
  static void PulseHigh() { PORTK |= (1<<1); PORTK &= (~(1<<1)); }
  static void PulseLow()  { PORTK &= (~(1<<1)); PORTK |= (1<<1); }
  static bool Test()      { return PINK & (1<<1); }

  static constexpr decltype(PORTK) GetNativePort() { return PORTK; }
  static constexpr uint8_t bitmask()               { return (1<<1); }
  static constexpr uint8_t bit()                   { return 1; }

  typedef PortK Port;
};

class PinK2 {
 public:
  static void Set()       { PORTK |= (1<<2); }
  static void Clear()     { PORTK &= (~(1<<2)); }
  static void Toggle()    { PORTK ^= (1<<2); }
  static void SetOutput() { DDRK |= (1<<2); }
  static void SetInput()  { DDRK &= (~(1<<2)); }
  static void PulseHigh() { PORTK |= (1<<2); PORTK &= (~(1<<2)); }
  static void PulseLow()  { PORTK &= (~(1<<2)); PORTK |= (1<<2); }
  static bool Test()      { return PINK & (1<<2); }

  static constexpr decltype(PORTK) GetNativePort() { return PORTK; }
  static constexpr uint8_t bitmask()               { return (1<<2); }
  static constexpr uint8_t bit()                   { return 2; }

  typedef PortK Port;
};

class PinK3 {
 public:
  static void Set()       { PORTK |= (1<<3); }
  static void Clear()     { PORTK &= (~(1<<3)); }
  static void Toggle()    { PORTK ^= (1<<3); }
  static void SetOutput() { DDRK |= (1<<3); }
  static void SetInput()  { DDRK &= (~(1<<3)); }
  static void PulseHigh() { PORTK |= (1<<3); PORTK &= (~(1<<3)); }
  static void PulseLow()  { PORTK &= (~(1<<3)); PORTK |= (1<<3); }
  static bool Test()      { return PINK & (1<<3); }

  static constexpr decltype(PORTK) GetNativePort() { return PORTK; }
  static constexpr uint8_t bitmask()               { return (1<<3); }
  static constexpr uint8_t bit()                   { return 3; }

  typedef PortK Port;
};

class PinK4 {
 public:
  static void Set()       { PORTK |= (1<<4); }
  static void Clear()     { PORTK &= (~(1<<4)); }
  static void Toggle()    { PORTK ^= (1<<4); }
  static void SetOutput() { DDRK |= (1<<4); }
  static void SetInput()  { DDRK &= (~(1<<4)); }
  static void PulseHigh() { PORTK |= (1<<4); PORTK &= (~(1<<4)); }
  static void PulseLow()  { PORTK &= (~(1<<4)); PORTK |= (1<<4); }
  static bool Test()      { return PINK & (1<<4); }

  static constexpr decltype(PORTK) GetNativePort() { return PORTK; }
  static constexpr uint8_t bitmask()               { return (1<<4); }
  static constexpr uint8_t bit()                   { return 4; }

  typedef PortK Port;
};

class PinK5 {
 public:
  static void Set()       { PORTK |= (1<<5); }
  static void Clear()     { PORTK &= (~(1<<5)); }
  static void Toggle()    { PORTK ^= (1<<5); }
  static void SetOutput() { DDRK |= (1<<5); }
  static void SetInput()  { DDRK &= (~(1<<5)); }
  static void PulseHigh() { PORTK |= (1<<5); PORTK &= (~(1<<5)); }
  static void PulseLow()  { PORTK &= (~(1<<5)); PORTK |= (1<<5); }
  static bool Test()      { return PINK & (1<<5); }

  static constexpr decltype(PORTK) GetNativePort() { return PORTK; }
  static constexpr uint8_t bitmask()               { return (1<<5); }
  static constexpr uint8_t bit()                   { return 5; }

  typedef PortK Port;
};

class PinK6 {
 public:
  static void Set()       { PORTK |= (1<<6); }
  static void Clear()     { PORTK &= (~(1<<6)); }
  static void Toggle()    { PORTK ^= (1<<6); }
  static void SetOutput() { DDRK |= (1<<6); }
  static void SetInput()  { DDRK &= (~(1<<6)); }
  static void PulseHigh() { PORTK |= (1<<6); PORTK &= (~(1<<6)); }
  static void PulseLow()  { PORTK &= (~(1<<6)); PORTK |= (1<<6); }
  static bool Test()      { return PINK & (1<<6); }

  static constexpr decltype(PORTK) GetNativePort() { return PORTK; }
  static constexpr uint8_t bitmask()               { return (1<<6); }
  static constexpr uint8_t bit()                   { return 6; }

  typedef PortK Port;
};

class PinK7 {
 public:
  static void Set()       { PORTK |= (1<<7); }
  static void Clear()     { PORTK &= (~(1<<7)); }
  static void Toggle()    { PORTK ^= (1<<7); }
  static void SetOutput() { DDRK |= (1<<7); }
  static void SetInput()  { DDRK &= (~(1<<7)); }
  static void PulseHigh() { PORTK |= (1<<7); PORTK &= (~(1<<7)); }
  static void PulseLow()  { PORTK &= (~(1<<7)); PORTK |= (1<<7); }
  static bool Test()      { return PINK & (1<<7); }

  static constexpr decltype(PORTK) GetNativePort() { return PORTK; }
  static constexpr uint8_t bitmask()               { return (1<<7); }
  static constexpr uint8_t bit()                   { return 7; }

  typedef PortK Port;
};

#endif //PORTK

struct SpiSpcr {

  /// Assigns a value to SPCR
  /// @param[in] value value affected to SPCR
  static void Assign(uint8_t value)  { SPCR = value; }

  /// Sets masked bits in SPCR
  /// @param[in] mask bits to set
  static void Set(uint8_t mask)      { SPCR |= mask; }

  /// Clears masked bits in SPCR
  /// @param[in] mask bits to clear
  static void Clear(uint8_t mask)    { SPCR &= (~mask); }
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
  static void Clear(uint8_t mask)    { SPDR &= (~mask); }
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
  static void Clear(uint8_t mask)    { SPSR &= (~mask); }
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
  static void Clear(uint16_t mask)    { UBRR0 &= (~mask); }
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
  static void Clear(uint8_t mask)    { UCSR0A &= (~mask); }
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
  static void Clear(uint8_t mask)    { UCSR0B &= (~mask); }
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
  static void Clear(uint8_t mask)    { UCSR0C &= (~mask); }
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
  static void Clear(uint8_t mask)    { UDR0 &= (~mask); }
  static uint8_t Get()               { return UDR0; }
  static bool TestBits(uint8_t mask) { return UDR0 & mask; }
  void operator=(uint8_t value)      { UDR0 = value; }
};

#ifdef TCNT0
class Timer0 {
 public:  
  typedef uint8_t value_type;
  
  static value_type GetValue()            { return TCNT0; }
  static void SetValue(value_type value)  { TCNT0 = value; }
  static void AddValue(value_type value)  { TCNT0 += value; }
  static void SubValue(value_type value)  { TCNT0 -= value; }
  static void SetCtrlRegA(uint8_t mask)   { TCCR0A |= mask; }
  static void ClearCtrlRegA(uint8_t mask) { TCCR0A &= (~mask); }
  static void SetCtrlRegB(uint8_t mask)   { TCCR0B |= mask; }
  static void ClearCtrlRegB(uint8_t mask) { TCCR0B &= (~mask); }
    
  enum Prescaler : uint8_t {
    STOP_TIMER = 0, NO_PRESCALER = (1<<CS00), CLK_DIV_8 = (1<<CS01),
    CLK_DIV_64 = (1<<CS01)|(1<<CS00), CLK_DIV_256 = (1<<CS02), CLK_DIV_1024 = (1<<CS02)|(1<<CS00),
    INC_ON_FALLING = (1<<CS02)|(1<<CS01), INC_ON_RISING = (1<<CS02)|(1<<CS01)|(1<<CS00) };
  enum Constants : uint8_t { ALL_BITS = 0xFF };
};  
#endif //TCNT0

#ifdef TCNT1
class Timer1 {
 public:  
  typedef uint16_t value_type;
  
  static value_type GetValue()            { return TCNT1; }
  static void SetValue(value_type value)  { TCNT1 = value; }
  static void AddValue(value_type value)  { TCNT1 += value; }
  static void SubValue(value_type value)  { TCNT1 -= value; }
  static void SetCtrlRegA(uint8_t mask)   { TCCR1A |= mask; }
  static void ClearCtrlRegA(uint8_t mask) { TCCR1A &= (~mask); }
  static void SetCtrlRegB(uint8_t mask)   { TCCR1B |= mask; }
  static void ClearCtrlRegB(uint8_t mask) { TCCR1B &= (~mask); }
    
  enum Prescaler : uint8_t {
    STOP_TIMER = 0, NO_PRESCALER = (1<<CS10), CLK_DIV_8 = (1<<CS11),
    CLK_DIV_64 = (1<<CS11)|(1<<CS10), CLK_DIV_256 = (1<<CS12), CLK_DIV_1024 = (1<<CS12)|(1<<CS10),
    INC_ON_FALLING = (1<<CS12)|(1<<CS11), INC_ON_RISING = (1<<CS12)|(1<<CS11)|(1<<CS10) };
  enum Constants : uint8_t { ALL_BITS = 0xFF };
};  
#endif //TCNT1

#ifdef TCNT2
class Timer2 {
 public:  
  typedef uint8_t value_type;
  
  static value_type GetValue()            { return TCNT2; }
  static void SetValue(value_type value)  { TCNT2 = value; }
  static void AddValue(value_type value)  { TCNT2 += value; }
  static void SubValue(value_type value)  { TCNT2 -= value; }
  static void SetCtrlRegA(uint8_t mask)   { TCCR2A |= mask; }
  static void ClearCtrlRegA(uint8_t mask) { TCCR2A &= (~mask); }
  static void SetCtrlRegB(uint8_t mask)   { TCCR2B |= mask; }
  static void ClearCtrlRegB(uint8_t mask) { TCCR2B &= (~mask); }
    
  enum Prescaler : uint8_t {
    STOP_TIMER = 0, NO_PRESCALER = (1<<CS20), CLK_DIV_8 = (1<<CS21),
    CLK_DIV_64 = (1<<CS21)|(1<<CS20), CLK_DIV_256 = (1<<CS22), CLK_DIV_1024 = (1<<CS22)|(1<<CS20),
    INC_ON_FALLING = (1<<CS22)|(1<<CS21), INC_ON_RISING = (1<<CS22)|(1<<CS21)|(1<<CS20) };
  enum Constants : uint8_t { ALL_BITS = 0xFF };
};  
#endif //TCNT2

#ifdef TCNT3
class Timer3 {
 public:  
  typedef uint16_t value_type;
  
  static value_type GetValue()            { return TCNT3; }
  static void SetValue(value_type value)  { TCNT3 = value; }
  static void AddValue(value_type value)  { TCNT3 += value; }
  static void SubValue(value_type value)  { TCNT3 -= value; }
  static void SetCtrlRegA(uint8_t mask)   { TCCR3A |= mask; }
  static void ClearCtrlRegA(uint8_t mask) { TCCR3A &= (~mask); }
  static void SetCtrlRegB(uint8_t mask)   { TCCR3B |= mask; }
  static void ClearCtrlRegB(uint8_t mask) { TCCR3B &= (~mask); }
    
  enum Prescaler : uint8_t {
    STOP_TIMER = 0, NO_PRESCALER = (1<<CS30), CLK_DIV_8 = (1<<CS31),
    CLK_DIV_64 = (1<<CS31)|(1<<CS30), CLK_DIV_256 = (1<<CS32), CLK_DIV_1024 = (1<<CS32)|(1<<CS30),
    INC_ON_FALLING = (1<<CS32)|(1<<CS31), INC_ON_RISING = (1<<CS32)|(1<<CS31)|(1<<CS30) };
  enum Constants : uint8_t { ALL_BITS = 0xFF };
};  
#endif //TCNT3

} // namespace etl
#endif //ETL_IOPORTS_H_
