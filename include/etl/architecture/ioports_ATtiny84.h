/// @file ioports_ATtiny84.h
/// @date 21/09/2016 21:17:16
/// @author Ambroise Leclerc and Cecile Thiebaut
/// @brief Atmel AVR 8-bit microcontrollers peripherals handling classes
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

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <libstd/include/chrono>
#include <libstd/include/queue>
#include <etl/CircularBuffer.h>
#include "ETLDevice_ATtiny84.h"

extern void __builtin_avr_delay_cycles(unsigned long);

namespace etl {
#define IOPORTS_TO_STRING(name) #name
#define IOPORTS_IRQ_HANDLER(vector, type) asm(IOPORTS_TO_STRING(vector)) __attribute__ ((type, __INTR_ATTRS))

using clock_cycles = std::chrono::duration<unsigned long, std::ratio<1, Device::McuFrequency>>;
constexpr clock_cycles operator ""clks(unsigned long long c)     { return clock_cycles(static_cast<clock_cycles::rep>(c)); }

struct PortA {

    /// Assigns a value to PORTA
    /// @param[in] value value affected to PORTA
    static void assign(uint8_t value)   { PORTA = value; }

    /// Sets masked bits in PORTA
    /// @param[in] mask bits to set
    static void setBits(uint8_t mask)   { PORTA |= mask;}

    /// Clears masked bits in PORTA
    /// @param[in] mask bits to clear
    static void clearBits(uint8_t mask) { PORTA &= ~mask;} 

    /// Changes values of masked bits in PORTA
    /// @param[in] mask bits to change
    /// @param[in] value new bits values
    static void changeBits(uint8_t mask, uint8_t value) { uint8_t tmp = PORTA & ~mask; PORTA = tmp | value; } 

    /// Toggles masked bits in PORTA
    /// @param[in] mask bits to toggle
    static void toggleBits(uint8_t mask) { PORTA ^= mask;} 

    /// Pulses masked bits in PORTA with high state first.
    /// @param[in] mask bits to pulse
    static void pulseHigh(uint8_t mask) { PORTA |= mask; PORTA &= ~mask; }

    /// Pulses masked bits in PORTA with low state first.
    /// @param[in] mask bits to pulse
    static void pulseLow(uint8_t mask)  { PORTA &= ~mask; PORTA |= mask; }

    /// Set corresponding masked bits of PORTA to output direction.
    /// @param[in] mask bits
    static void setOutput(uint8_t mask)    { DDRA |= mask; }

    /// Set corresponding masked bits of PORTA to input direction.
    /// @param[in] mask bits
    static void setInput(uint8_t mask)  { DDRA &= ~mask; }

    /// Returns PINA register.
    static uint8_t getPIN()             { return PINA; }

    /// Tests masked bits of PORTA
    /// @param[in] mask bits
    /// @param[in] true if the corresponding bits are all set, false otherwise.
    static bool testBits(uint8_t mask)  { return (PINA & mask) == mask; }

    /// Returns the value of the bit at the position pos.
    /// @param[in] position of the bit to return
    /// @return true if the requested bit is set, false otherwise.
    static bool test(uint8_t pos) { return PINA & (1<<pos); }

};

struct PinA7 {
    /// Sets PinA7 to HIGH.
    static void set()       { PORTA |= (1<<7); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinA7 to LOW.
    static void clear()     { PORTA &= ~(1<<7); }

    /// Toggles PinA7 value.
    static void toggle()    { PINA |= (1<<7); }

    /// Configures PinA7  as an output pin.
    static void setOutput() { DDRA |= (1<<7); }

    /// Configures PinA7  as an input pin.
    static void setInput()  { DDRA &= ~(1<<7); }

    /// Pulses PinA7 with high state first.
    static void pulseHigh() { PORTA |= (1<<7); PORTA &= ~(1<<7); }

    /// Pulses PinA7 with low state first.
    static void pulseLow()  { PORTA &= ~(1<<7); PORTA |= (1<<7); }

    /// Reads PinA7  value.
    /// @return Port pin value.
    static bool test()      { return PINA & (1<<7); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<7)
    static constexpr uint8_t bitmask()               { return (1<<7); }

    /// Returns the bit corresponding to this pin.
    /// @return 7
    static constexpr uint8_t bit()                   { return 7; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortA;
};

struct PinA6 {
    /// Sets PinA6 to HIGH.
    static void set()       { PORTA |= (1<<6); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinA6 to LOW.
    static void clear()     { PORTA &= ~(1<<6); }

    /// Toggles PinA6 value.
    static void toggle()    { PINA |= (1<<6); }

    /// Configures PinA6  as an output pin.
    static void setOutput() { DDRA |= (1<<6); }

    /// Configures PinA6  as an input pin.
    static void setInput()  { DDRA &= ~(1<<6); }

    /// Pulses PinA6 with high state first.
    static void pulseHigh() { PORTA |= (1<<6); PORTA &= ~(1<<6); }

    /// Pulses PinA6 with low state first.
    static void pulseLow()  { PORTA &= ~(1<<6); PORTA |= (1<<6); }

    /// Reads PinA6  value.
    /// @return Port pin value.
    static bool test()      { return PINA & (1<<6); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<6)
    static constexpr uint8_t bitmask()               { return (1<<6); }

    /// Returns the bit corresponding to this pin.
    /// @return 6
    static constexpr uint8_t bit()                   { return 6; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortA;
};

struct PinA5 {
    /// Sets PinA5 to HIGH.
    static void set()       { PORTA |= (1<<5); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinA5 to LOW.
    static void clear()     { PORTA &= ~(1<<5); }

    /// Toggles PinA5 value.
    static void toggle()    { PINA |= (1<<5); }

    /// Configures PinA5  as an output pin.
    static void setOutput() { DDRA |= (1<<5); }

    /// Configures PinA5  as an input pin.
    static void setInput()  { DDRA &= ~(1<<5); }

    /// Pulses PinA5 with high state first.
    static void pulseHigh() { PORTA |= (1<<5); PORTA &= ~(1<<5); }

    /// Pulses PinA5 with low state first.
    static void pulseLow()  { PORTA &= ~(1<<5); PORTA |= (1<<5); }

    /// Reads PinA5  value.
    /// @return Port pin value.
    static bool test()      { return PINA & (1<<5); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<5)
    static constexpr uint8_t bitmask()               { return (1<<5); }

    /// Returns the bit corresponding to this pin.
    /// @return 5
    static constexpr uint8_t bit()                   { return 5; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortA;
};

struct PinA4 {
    /// Sets PinA4 to HIGH.
    static void set()       { PORTA |= (1<<4); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinA4 to LOW.
    static void clear()     { PORTA &= ~(1<<4); }

    /// Toggles PinA4 value.
    static void toggle()    { PINA |= (1<<4); }

    /// Configures PinA4  as an output pin.
    static void setOutput() { DDRA |= (1<<4); }

    /// Configures PinA4  as an input pin.
    static void setInput()  { DDRA &= ~(1<<4); }

    /// Pulses PinA4 with high state first.
    static void pulseHigh() { PORTA |= (1<<4); PORTA &= ~(1<<4); }

    /// Pulses PinA4 with low state first.
    static void pulseLow()  { PORTA &= ~(1<<4); PORTA |= (1<<4); }

    /// Reads PinA4  value.
    /// @return Port pin value.
    static bool test()      { return PINA & (1<<4); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<4)
    static constexpr uint8_t bitmask()               { return (1<<4); }

    /// Returns the bit corresponding to this pin.
    /// @return 4
    static constexpr uint8_t bit()                   { return 4; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortA;
};

struct PinA3 {
    /// Sets PinA3 to HIGH.
    static void set()       { PORTA |= (1<<3); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinA3 to LOW.
    static void clear()     { PORTA &= ~(1<<3); }

    /// Toggles PinA3 value.
    static void toggle()    { PINA |= (1<<3); }

    /// Configures PinA3  as an output pin.
    static void setOutput() { DDRA |= (1<<3); }

    /// Configures PinA3  as an input pin.
    static void setInput()  { DDRA &= ~(1<<3); }

    /// Pulses PinA3 with high state first.
    static void pulseHigh() { PORTA |= (1<<3); PORTA &= ~(1<<3); }

    /// Pulses PinA3 with low state first.
    static void pulseLow()  { PORTA &= ~(1<<3); PORTA |= (1<<3); }

    /// Reads PinA3  value.
    /// @return Port pin value.
    static bool test()      { return PINA & (1<<3); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<3)
    static constexpr uint8_t bitmask()               { return (1<<3); }

    /// Returns the bit corresponding to this pin.
    /// @return 3
    static constexpr uint8_t bit()                   { return 3; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortA;
};

struct PinA2 {
    /// Sets PinA2 to HIGH.
    static void set()       { PORTA |= (1<<2); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinA2 to LOW.
    static void clear()     { PORTA &= ~(1<<2); }

    /// Toggles PinA2 value.
    static void toggle()    { PINA |= (1<<2); }

    /// Configures PinA2  as an output pin.
    static void setOutput() { DDRA |= (1<<2); }

    /// Configures PinA2  as an input pin.
    static void setInput()  { DDRA &= ~(1<<2); }

    /// Pulses PinA2 with high state first.
    static void pulseHigh() { PORTA |= (1<<2); PORTA &= ~(1<<2); }

    /// Pulses PinA2 with low state first.
    static void pulseLow()  { PORTA &= ~(1<<2); PORTA |= (1<<2); }

    /// Reads PinA2  value.
    /// @return Port pin value.
    static bool test()      { return PINA & (1<<2); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<2)
    static constexpr uint8_t bitmask()               { return (1<<2); }

    /// Returns the bit corresponding to this pin.
    /// @return 2
    static constexpr uint8_t bit()                   { return 2; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortA;
};

struct PinA1 {
    /// Sets PinA1 to HIGH.
    static void set()       { PORTA |= (1<<1); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinA1 to LOW.
    static void clear()     { PORTA &= ~(1<<1); }

    /// Toggles PinA1 value.
    static void toggle()    { PINA |= (1<<1); }

    /// Configures PinA1  as an output pin.
    static void setOutput() { DDRA |= (1<<1); }

    /// Configures PinA1  as an input pin.
    static void setInput()  { DDRA &= ~(1<<1); }

    /// Pulses PinA1 with high state first.
    static void pulseHigh() { PORTA |= (1<<1); PORTA &= ~(1<<1); }

    /// Pulses PinA1 with low state first.
    static void pulseLow()  { PORTA &= ~(1<<1); PORTA |= (1<<1); }

    /// Reads PinA1  value.
    /// @return Port pin value.
    static bool test()      { return PINA & (1<<1); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<1)
    static constexpr uint8_t bitmask()               { return (1<<1); }

    /// Returns the bit corresponding to this pin.
    /// @return 1
    static constexpr uint8_t bit()                   { return 1; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortA;
};

struct PinA0 {
    /// Sets PinA0 to HIGH.
    static void set()       { PORTA |= (1<<0); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinA0 to LOW.
    static void clear()     { PORTA &= ~(1<<0); }

    /// Toggles PinA0 value.
    static void toggle()    { PINA |= (1<<0); }

    /// Configures PinA0  as an output pin.
    static void setOutput() { DDRA |= (1<<0); }

    /// Configures PinA0  as an input pin.
    static void setInput()  { DDRA &= ~(1<<0); }

    /// Pulses PinA0 with high state first.
    static void pulseHigh() { PORTA |= (1<<0); PORTA &= ~(1<<0); }

    /// Pulses PinA0 with low state first.
    static void pulseLow()  { PORTA &= ~(1<<0); PORTA |= (1<<0); }

    /// Reads PinA0  value.
    /// @return Port pin value.
    static bool test()      { return PINA & (1<<0); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<0)
    static constexpr uint8_t bitmask()               { return (1<<0); }

    /// Returns the bit corresponding to this pin.
    /// @return 0
    static constexpr uint8_t bit()                   { return 0; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortA;
};


struct PortB {

    /// Assigns a value to PORTB
    /// @param[in] value value affected to PORTB
    static void assign(uint8_t value)   { PORTB = value; }

    /// Sets masked bits in PORTB
    /// @param[in] mask bits to set
    static void setBits(uint8_t mask)   { PORTB |= mask;}

    /// Clears masked bits in PORTB
    /// @param[in] mask bits to clear
    static void clearBits(uint8_t mask) { PORTB &= ~mask;} 

    /// Changes values of masked bits in PORTB
    /// @param[in] mask bits to change
    /// @param[in] value new bits values
    static void changeBits(uint8_t mask, uint8_t value) { uint8_t tmp = PORTB & ~mask; PORTB = tmp | value; } 

    /// Toggles masked bits in PORTB
    /// @param[in] mask bits to toggle
    static void toggleBits(uint8_t mask) { PORTB ^= mask;} 

    /// Pulses masked bits in PORTB with high state first.
    /// @param[in] mask bits to pulse
    static void pulseHigh(uint8_t mask) { PORTB |= mask; PORTB &= ~mask; }

    /// Pulses masked bits in PORTB with low state first.
    /// @param[in] mask bits to pulse
    static void pulseLow(uint8_t mask)  { PORTB &= ~mask; PORTB |= mask; }

    /// Set corresponding masked bits of PORTB to output direction.
    /// @param[in] mask bits
    static void setOutput(uint8_t mask)    { DDRB |= mask; }

    /// Set corresponding masked bits of PORTB to input direction.
    /// @param[in] mask bits
    static void setInput(uint8_t mask)  { DDRB &= ~mask; }

    /// Returns PINB register.
    static uint8_t getPIN()             { return PINB; }

    /// Tests masked bits of PORTB
    /// @param[in] mask bits
    /// @param[in] true if the corresponding bits are all set, false otherwise.
    static bool testBits(uint8_t mask)  { return (PINB & mask) == mask; }

    /// Returns the value of the bit at the position pos.
    /// @param[in] position of the bit to return
    /// @return true if the requested bit is set, false otherwise.
    static bool test(uint8_t pos) { return PINB & (1<<pos); }

};

struct PinB3 {
    /// Sets PinB3 to HIGH.
    static void set()       { PORTB |= (1<<3); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinB3 to LOW.
    static void clear()     { PORTB &= ~(1<<3); }

    /// Toggles PinB3 value.
    static void toggle()    { PINB |= (1<<3); }

    /// Configures PinB3  as an output pin.
    static void setOutput() { DDRB |= (1<<3); }

    /// Configures PinB3  as an input pin.
    static void setInput()  { DDRB &= ~(1<<3); }

    /// Pulses PinB3 with high state first.
    static void pulseHigh() { PORTB |= (1<<3); PORTB &= ~(1<<3); }

    /// Pulses PinB3 with low state first.
    static void pulseLow()  { PORTB &= ~(1<<3); PORTB |= (1<<3); }

    /// Reads PinB3  value.
    /// @return Port pin value.
    static bool test()      { return PINB & (1<<3); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<3)
    static constexpr uint8_t bitmask()               { return (1<<3); }

    /// Returns the bit corresponding to this pin.
    /// @return 3
    static constexpr uint8_t bit()                   { return 3; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortB;
};

struct PinB2 {
    /// Sets PinB2 to HIGH.
    static void set()       { PORTB |= (1<<2); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinB2 to LOW.
    static void clear()     { PORTB &= ~(1<<2); }

    /// Toggles PinB2 value.
    static void toggle()    { PINB |= (1<<2); }

    /// Configures PinB2  as an output pin.
    static void setOutput() { DDRB |= (1<<2); }

    /// Configures PinB2  as an input pin.
    static void setInput()  { DDRB &= ~(1<<2); }

    /// Pulses PinB2 with high state first.
    static void pulseHigh() { PORTB |= (1<<2); PORTB &= ~(1<<2); }

    /// Pulses PinB2 with low state first.
    static void pulseLow()  { PORTB &= ~(1<<2); PORTB |= (1<<2); }

    /// Reads PinB2  value.
    /// @return Port pin value.
    static bool test()      { return PINB & (1<<2); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<2)
    static constexpr uint8_t bitmask()               { return (1<<2); }

    /// Returns the bit corresponding to this pin.
    /// @return 2
    static constexpr uint8_t bit()                   { return 2; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortB;
};

struct PinB1 {
    /// Sets PinB1 to HIGH.
    static void set()       { PORTB |= (1<<1); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinB1 to LOW.
    static void clear()     { PORTB &= ~(1<<1); }

    /// Toggles PinB1 value.
    static void toggle()    { PINB |= (1<<1); }

    /// Configures PinB1  as an output pin.
    static void setOutput() { DDRB |= (1<<1); }

    /// Configures PinB1  as an input pin.
    static void setInput()  { DDRB &= ~(1<<1); }

    /// Pulses PinB1 with high state first.
    static void pulseHigh() { PORTB |= (1<<1); PORTB &= ~(1<<1); }

    /// Pulses PinB1 with low state first.
    static void pulseLow()  { PORTB &= ~(1<<1); PORTB |= (1<<1); }

    /// Reads PinB1  value.
    /// @return Port pin value.
    static bool test()      { return PINB & (1<<1); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<1)
    static constexpr uint8_t bitmask()               { return (1<<1); }

    /// Returns the bit corresponding to this pin.
    /// @return 1
    static constexpr uint8_t bit()                   { return 1; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortB;
};

struct PinB0 {
    /// Sets PinB0 to HIGH.
    static void set()       { PORTB |= (1<<0); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinB0 to LOW.
    static void clear()     { PORTB &= ~(1<<0); }

    /// Toggles PinB0 value.
    static void toggle()    { PINB |= (1<<0); }

    /// Configures PinB0  as an output pin.
    static void setOutput() { DDRB |= (1<<0); }

    /// Configures PinB0  as an input pin.
    static void setInput()  { DDRB &= ~(1<<0); }

    /// Pulses PinB0 with high state first.
    static void pulseHigh() { PORTB |= (1<<0); PORTB &= ~(1<<0); }

    /// Pulses PinB0 with low state first.
    static void pulseLow()  { PORTB &= ~(1<<0); PORTB |= (1<<0); }

    /// Reads PinB0  value.
    /// @return Port pin value.
    static bool test()      { return PINB & (1<<0); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<0)
    static constexpr uint8_t bitmask()               { return (1<<0); }

    /// Returns the bit corresponding to this pin.
    /// @return 0
    static constexpr uint8_t bit()                   { return 0; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortB;
};


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

struct Timer0 {
  using value_type = uint8_t;
  static const uint8_t timer_width = 8;

  enum Prescaler : uint8_t {
    STOP_TIMER = 0, NO_PRESCALER = (1<<CS00), CLK_DIV_8 = (1<<CS01),
    CLK_DIV_32 = (1<<CS01)|(1<<CS00), CLK_DIV_64 = (1<<CS02),
    CLK_DIV_128 = (1<<CS02)|(1<<CS00), CLK_DIV_256 = (1<<CS02)|(1<<CS01),
    CLK_DIV_1024 = (1<<CS02)|(1<<CS01)|(1<<CS00), BITS = (1<<CS02)|(1<<CS01)|(1<<CS00) };

  enum Interrupt : uint8_t { 
    OVERFLOW = (1<<TOIE0), COMPARE_MATCH_A = (1<<OCIE0A), COMPARE_MATCH_B = (1<<OCIE0B)};
  enum Constants : uint8_t { ALL_BITS = 0xFF };
  static value_type getValue()                 { return TCNT0; }
  static void setValue(value_type value)       { TCNT0 = value; }
  static void addValue(value_type value)       { TCNT0 += value; }
  static void subValue(value_type value)       { TCNT0 -= value; }
  static void setCtrlRegA(uint8_t mask)        { TCCR0A |= mask; }
  static void clearCtrlRegA(uint8_t mask)      { TCCR0A &= ~mask; }
  static void setCtrlRegB(uint8_t mask)        { TCCR0B |= mask; }
  static void clearCtrlRegB(uint8_t mask)      { TCCR0B &= ~mask; }
  static void setInterruptMask(uint8_t mask)   { TIMSK0 |= mask; }
  static void clearInterruptMask(uint8_t mask) { TIMSK0 &= ~mask; }
  static void setPrescaler(Prescaler val)      { TCCR0B &= ~Prescaler::BITS; TCCR0B |= val; }
  static void setOutputCompareValueA(value_type value) { OCR0A = value; }
  static value_type getOutputCompareValueA()           { return OCR0A; }
  static void setOutputCompareValueB(value_type value) { OCR0B = value; }
  static value_type getOutputCompareValueB()           { return OCR0B; }

  struct Isr {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER0_OVF_vect, signal);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER0_COMPA_vect, signal);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER0_COMPB_vect, signal);
  };

  struct IsrNoBlock {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER0_OVF_vect, interrupt);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER0_COMPA_vect, interrupt);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER0_COMPB_vect, interrupt);
  };

  struct IsrNaked {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER0_OVF_vect, naked);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER0_COMPA_vect, naked);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER0_COMPB_vect, naked);
  };
};  

struct Timer1 {
  using value_type = uint16_t;
  static const uint8_t timer_width = 16;

  enum Prescaler : uint8_t {
    STOP_TIMER = 0, NO_PRESCALER = (1<<CS10), CLK_DIV_8 = (1<<CS11),
    CLK_DIV_32 = (1<<CS11)|(1<<CS10), CLK_DIV_64 = (1<<CS12),
    CLK_DIV_128 = (1<<CS12)|(1<<CS10), CLK_DIV_256 = (1<<CS12)|(1<<CS11),
    CLK_DIV_1024 = (1<<CS12)|(1<<CS11)|(1<<CS10), BITS = (1<<CS12)|(1<<CS11)|(1<<CS10) };

  enum Interrupt : uint8_t { 
    OVERFLOW = (1<<TOIE1), CAPTURE = (1<<ICIE1), COMPARE_MATCH_A = (1<<OCIE1A), COMPARE_MATCH_B = (1<<OCIE1B)};
  enum Constants : uint8_t { ALL_BITS = 0xFF };
  static value_type getValue()                 { return TCNT1; }
  static void setValue(value_type value)       { TCNT1 = value; }
  static void addValue(value_type value)       { TCNT1 += value; }
  static void subValue(value_type value)       { TCNT1 -= value; }
  static void setCtrlRegA(uint8_t mask)        { TCCR1A |= mask; }
  static void clearCtrlRegA(uint8_t mask)      { TCCR1A &= ~mask; }
  static void setCtrlRegB(uint8_t mask)        { TCCR1B |= mask; }
  static void clearCtrlRegB(uint8_t mask)      { TCCR1B &= ~mask; }
  static void setInterruptMask(uint8_t mask)   { TIMSK1 |= mask; }
  static void clearInterruptMask(uint8_t mask) { TIMSK1 &= ~mask; }
  static void setPrescaler(Prescaler val)      { TCCR1B &= ~Prescaler::BITS; TCCR1B |= val; }
  static void setOutputCompareValueA(value_type value) { OCR1A = value; }
  static value_type getOutputCompareValueA()           { return OCR1A; }
  static void setOutputCompareValueB(value_type value) { OCR1B = value; }
  static value_type getOutputCompareValueB()           { return OCR1B; }

  struct Isr {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER1_OVF_vect, signal);
    static void Capture() IOPORTS_IRQ_HANDLER(TIMER1_CAPT_vect, signal);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER1_COMPA_vect, signal);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER1_COMPB_vect, signal);
  };

  struct IsrNoBlock {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER1_OVF_vect, interrupt);
    static void Capture() IOPORTS_IRQ_HANDLER(TIMER1_CAPT_vect, interrupt);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER1_COMPA_vect, interrupt);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER1_COMPB_vect, interrupt);
  };

  struct IsrNaked {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER1_OVF_vect, naked);
    static void Capture() IOPORTS_IRQ_HANDLER(TIMER1_CAPT_vect, naked);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER1_COMPA_vect, naked);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER1_COMPB_vect, naked);
  };
};  

template<typename> struct is_uart_txd_capable : std::false_type {};
template<typename> struct is_uart_rxd_capable : std::false_type {};
enum FrameFormat {
    _5N1 = 0b00000, _5N2 = 0b000001, _5E1 = 0b00010, _5E2 = 0b00011, _5O1 = 0b00100, _5O2 = 0b00101,       // Encoding 5-N-1 : nbBits - no, even or odd parity - 1 or 2 stop bits : 0b00 - 00 - 0
    _6N1 = 0b01000, _6N2 = 0b010001, _6E1 = 0b01010, _6E2 = 0b01011, _6O1 = 0b01100, _6O2 = 0b01101,
    _7N1 = 0b10000, _7N2 = 0b100001, _7E1 = 0b10010, _7E2 = 0b10011, _7O1 = 0b10100, _7O2 = 0b10101,
    _8N1 = 0b11000, _8N2 = 0b110001, _8E1 = 0b11010, _8E2 = 0b11011, _8O1 = 0b11100, _8O2 = 0b11101,
    NbBitsMask = 0b11000, ParityMask = 0b00110, StopBitMask = 0b00001
};
#ifdef PCICR
struct PinChangeControlRegister {
  static void setBits(uint8_t mask)   { PCICR |= mask; }
  static void clearBits(uint8_t mask) { PCICR &= ~mask; }
};
#endif // PCICR

#undef IOPORTS_TO_STRING
#undef IOPORTS_IRQ_HANDLER
} // namespace etl
