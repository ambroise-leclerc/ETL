/// @file ioports_ATmega644A.h
/// @date 18/06/2016 18:17:16
/// @author Ambroise Leclerc and Cécile Gomes
/// @brief Atmel AVR 8-bit microcontrollers peripherals handling classes
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

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <chrono>

extern void __builtin_avr_delay_cycles(unsigned long);

namespace etl {
#define IOPORTS_TO_STRING(name) #name
#define IOPORTS_IRQ_HANDLER(vector, type) asm(IOPORTS_TO_STRING(vector)) __attribute__ ((type, __INTR_ATTRS))

class Device {
public:
    static void delayTicks(uint32_t ticks)            { __builtin_avr_delay_cycles(ticks); }
    static const auto flashSize = 65536;
    static const auto eepromSize = 2048;
    static const auto sramSize = 4096;
    static const auto architectureWidth = 8;
    static const uint32_t McuFrequency = F_CPU;

    /// Enables interrupts by setting the global interrupt mask.
    /// This function generates a single 'sei' instruction with
    /// no overhead.
    static void enableInterrupts() { asm volatile("sei" ::: "memory"); }

    /// Disables interrupts by clearing the global interrupt mask.
    /// This function generates a single 'cli' instruction with
    /// no overhead.
    static void disableInterrupts() { asm volatile("cli" ::: "memory"); }
};

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

struct PinB7 {
    /// Sets PinB7 to HIGH.
    static void set()       { PORTB |= (1<<7); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinB7 to LOW.
    static void clear()     { PORTB &= ~(1<<7); }

    /// Toggles PinB7 value.
    static void toggle()    { PINB |= (1<<7); }

    /// Configures PinB7  as an output pin.
    static void setOutput() { DDRB |= (1<<7); }

    /// Configures PinB7  as an input pin.
    static void setInput()  { DDRB &= ~(1<<7); }

    /// Pulses PinB7 with high state first.
    static void pulseHigh() { PORTB |= (1<<7); PORTB &= ~(1<<7); }

    /// Pulses PinB7 with low state first.
    static void pulseLow()  { PORTB &= ~(1<<7); PORTB |= (1<<7); }

    /// Reads PinB7  value.
    /// @return Port pin value.
    static bool test()      { return PINB & (1<<7); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<7)
    static constexpr uint8_t bitmask()               { return (1<<7); }

    /// Returns the bit corresponding to this pin.
    /// @return 7
    static constexpr uint8_t bit()                   { return 7; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortB;
};

struct PinB6 {
    /// Sets PinB6 to HIGH.
    static void set()       { PORTB |= (1<<6); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinB6 to LOW.
    static void clear()     { PORTB &= ~(1<<6); }

    /// Toggles PinB6 value.
    static void toggle()    { PINB |= (1<<6); }

    /// Configures PinB6  as an output pin.
    static void setOutput() { DDRB |= (1<<6); }

    /// Configures PinB6  as an input pin.
    static void setInput()  { DDRB &= ~(1<<6); }

    /// Pulses PinB6 with high state first.
    static void pulseHigh() { PORTB |= (1<<6); PORTB &= ~(1<<6); }

    /// Pulses PinB6 with low state first.
    static void pulseLow()  { PORTB &= ~(1<<6); PORTB |= (1<<6); }

    /// Reads PinB6  value.
    /// @return Port pin value.
    static bool test()      { return PINB & (1<<6); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<6)
    static constexpr uint8_t bitmask()               { return (1<<6); }

    /// Returns the bit corresponding to this pin.
    /// @return 6
    static constexpr uint8_t bit()                   { return 6; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortB;
};

struct PinB5 {
    /// Sets PinB5 to HIGH.
    static void set()       { PORTB |= (1<<5); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinB5 to LOW.
    static void clear()     { PORTB &= ~(1<<5); }

    /// Toggles PinB5 value.
    static void toggle()    { PINB |= (1<<5); }

    /// Configures PinB5  as an output pin.
    static void setOutput() { DDRB |= (1<<5); }

    /// Configures PinB5  as an input pin.
    static void setInput()  { DDRB &= ~(1<<5); }

    /// Pulses PinB5 with high state first.
    static void pulseHigh() { PORTB |= (1<<5); PORTB &= ~(1<<5); }

    /// Pulses PinB5 with low state first.
    static void pulseLow()  { PORTB &= ~(1<<5); PORTB |= (1<<5); }

    /// Reads PinB5  value.
    /// @return Port pin value.
    static bool test()      { return PINB & (1<<5); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<5)
    static constexpr uint8_t bitmask()               { return (1<<5); }

    /// Returns the bit corresponding to this pin.
    /// @return 5
    static constexpr uint8_t bit()                   { return 5; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortB;
};

struct PinB4 {
    /// Sets PinB4 to HIGH.
    static void set()       { PORTB |= (1<<4); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinB4 to LOW.
    static void clear()     { PORTB &= ~(1<<4); }

    /// Toggles PinB4 value.
    static void toggle()    { PINB |= (1<<4); }

    /// Configures PinB4  as an output pin.
    static void setOutput() { DDRB |= (1<<4); }

    /// Configures PinB4  as an input pin.
    static void setInput()  { DDRB &= ~(1<<4); }

    /// Pulses PinB4 with high state first.
    static void pulseHigh() { PORTB |= (1<<4); PORTB &= ~(1<<4); }

    /// Pulses PinB4 with low state first.
    static void pulseLow()  { PORTB &= ~(1<<4); PORTB |= (1<<4); }

    /// Reads PinB4  value.
    /// @return Port pin value.
    static bool test()      { return PINB & (1<<4); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<4)
    static constexpr uint8_t bitmask()               { return (1<<4); }

    /// Returns the bit corresponding to this pin.
    /// @return 4
    static constexpr uint8_t bit()                   { return 4; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortB;
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


struct PortC {

    /// Assigns a value to PORTC
    /// @param[in] value value affected to PORTC
    static void assign(uint8_t value)   { PORTC = value; }

    /// Sets masked bits in PORTC
    /// @param[in] mask bits to set
    static void setBits(uint8_t mask)   { PORTC |= mask;}

    /// Clears masked bits in PORTC
    /// @param[in] mask bits to clear
    static void clearBits(uint8_t mask) { PORTC &= ~mask;} 

    /// Changes values of masked bits in PORTC
    /// @param[in] mask bits to change
    /// @param[in] value new bits values
    static void changeBits(uint8_t mask, uint8_t value) { uint8_t tmp = PORTC & ~mask; PORTC = tmp | value; } 

    /// Toggles masked bits in PORTC
    /// @param[in] mask bits to toggle
    static void toggleBits(uint8_t mask) { PORTC ^= mask;} 

    /// Pulses masked bits in PORTC with high state first.
    /// @param[in] mask bits to pulse
    static void pulseHigh(uint8_t mask) { PORTC |= mask; PORTC &= ~mask; }

    /// Pulses masked bits in PORTC with low state first.
    /// @param[in] mask bits to pulse
    static void pulseLow(uint8_t mask)  { PORTC &= ~mask; PORTC |= mask; }

    /// Set corresponding masked bits of PORTC to output direction.
    /// @param[in] mask bits
    static void setOutput(uint8_t mask)    { DDRC |= mask; }

    /// Set corresponding masked bits of PORTC to input direction.
    /// @param[in] mask bits
    static void setInput(uint8_t mask)  { DDRC &= ~mask; }

    /// Returns PINC register.
    static uint8_t getPIN()             { return PINC; }

    /// Tests masked bits of PORTC
    /// @param[in] mask bits
    /// @param[in] true if the corresponding bits are all set, false otherwise.
    static bool testBits(uint8_t mask)  { return (PINC & mask) == mask; }

    /// Returns the value of the bit at the position pos.
    /// @param[in] position of the bit to return
    /// @return true if the requested bit is set, false otherwise.
    static bool test(uint8_t pos) { return PINC & (1<<pos); }

};

struct PinC7 {
    /// Sets PinC7 to HIGH.
    static void set()       { PORTC |= (1<<7); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinC7 to LOW.
    static void clear()     { PORTC &= ~(1<<7); }

    /// Toggles PinC7 value.
    static void toggle()    { PINC |= (1<<7); }

    /// Configures PinC7  as an output pin.
    static void setOutput() { DDRC |= (1<<7); }

    /// Configures PinC7  as an input pin.
    static void setInput()  { DDRC &= ~(1<<7); }

    /// Pulses PinC7 with high state first.
    static void pulseHigh() { PORTC |= (1<<7); PORTC &= ~(1<<7); }

    /// Pulses PinC7 with low state first.
    static void pulseLow()  { PORTC &= ~(1<<7); PORTC |= (1<<7); }

    /// Reads PinC7  value.
    /// @return Port pin value.
    static bool test()      { return PINC & (1<<7); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<7)
    static constexpr uint8_t bitmask()               { return (1<<7); }

    /// Returns the bit corresponding to this pin.
    /// @return 7
    static constexpr uint8_t bit()                   { return 7; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortC;
};

struct PinC6 {
    /// Sets PinC6 to HIGH.
    static void set()       { PORTC |= (1<<6); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinC6 to LOW.
    static void clear()     { PORTC &= ~(1<<6); }

    /// Toggles PinC6 value.
    static void toggle()    { PINC |= (1<<6); }

    /// Configures PinC6  as an output pin.
    static void setOutput() { DDRC |= (1<<6); }

    /// Configures PinC6  as an input pin.
    static void setInput()  { DDRC &= ~(1<<6); }

    /// Pulses PinC6 with high state first.
    static void pulseHigh() { PORTC |= (1<<6); PORTC &= ~(1<<6); }

    /// Pulses PinC6 with low state first.
    static void pulseLow()  { PORTC &= ~(1<<6); PORTC |= (1<<6); }

    /// Reads PinC6  value.
    /// @return Port pin value.
    static bool test()      { return PINC & (1<<6); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<6)
    static constexpr uint8_t bitmask()               { return (1<<6); }

    /// Returns the bit corresponding to this pin.
    /// @return 6
    static constexpr uint8_t bit()                   { return 6; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortC;
};

struct PinC5 {
    /// Sets PinC5 to HIGH.
    static void set()       { PORTC |= (1<<5); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinC5 to LOW.
    static void clear()     { PORTC &= ~(1<<5); }

    /// Toggles PinC5 value.
    static void toggle()    { PINC |= (1<<5); }

    /// Configures PinC5  as an output pin.
    static void setOutput() { DDRC |= (1<<5); }

    /// Configures PinC5  as an input pin.
    static void setInput()  { DDRC &= ~(1<<5); }

    /// Pulses PinC5 with high state first.
    static void pulseHigh() { PORTC |= (1<<5); PORTC &= ~(1<<5); }

    /// Pulses PinC5 with low state first.
    static void pulseLow()  { PORTC &= ~(1<<5); PORTC |= (1<<5); }

    /// Reads PinC5  value.
    /// @return Port pin value.
    static bool test()      { return PINC & (1<<5); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<5)
    static constexpr uint8_t bitmask()               { return (1<<5); }

    /// Returns the bit corresponding to this pin.
    /// @return 5
    static constexpr uint8_t bit()                   { return 5; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortC;
};

struct PinC4 {
    /// Sets PinC4 to HIGH.
    static void set()       { PORTC |= (1<<4); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinC4 to LOW.
    static void clear()     { PORTC &= ~(1<<4); }

    /// Toggles PinC4 value.
    static void toggle()    { PINC |= (1<<4); }

    /// Configures PinC4  as an output pin.
    static void setOutput() { DDRC |= (1<<4); }

    /// Configures PinC4  as an input pin.
    static void setInput()  { DDRC &= ~(1<<4); }

    /// Pulses PinC4 with high state first.
    static void pulseHigh() { PORTC |= (1<<4); PORTC &= ~(1<<4); }

    /// Pulses PinC4 with low state first.
    static void pulseLow()  { PORTC &= ~(1<<4); PORTC |= (1<<4); }

    /// Reads PinC4  value.
    /// @return Port pin value.
    static bool test()      { return PINC & (1<<4); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<4)
    static constexpr uint8_t bitmask()               { return (1<<4); }

    /// Returns the bit corresponding to this pin.
    /// @return 4
    static constexpr uint8_t bit()                   { return 4; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortC;
};

struct PinC3 {
    /// Sets PinC3 to HIGH.
    static void set()       { PORTC |= (1<<3); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinC3 to LOW.
    static void clear()     { PORTC &= ~(1<<3); }

    /// Toggles PinC3 value.
    static void toggle()    { PINC |= (1<<3); }

    /// Configures PinC3  as an output pin.
    static void setOutput() { DDRC |= (1<<3); }

    /// Configures PinC3  as an input pin.
    static void setInput()  { DDRC &= ~(1<<3); }

    /// Pulses PinC3 with high state first.
    static void pulseHigh() { PORTC |= (1<<3); PORTC &= ~(1<<3); }

    /// Pulses PinC3 with low state first.
    static void pulseLow()  { PORTC &= ~(1<<3); PORTC |= (1<<3); }

    /// Reads PinC3  value.
    /// @return Port pin value.
    static bool test()      { return PINC & (1<<3); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<3)
    static constexpr uint8_t bitmask()               { return (1<<3); }

    /// Returns the bit corresponding to this pin.
    /// @return 3
    static constexpr uint8_t bit()                   { return 3; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortC;
};

struct PinC2 {
    /// Sets PinC2 to HIGH.
    static void set()       { PORTC |= (1<<2); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinC2 to LOW.
    static void clear()     { PORTC &= ~(1<<2); }

    /// Toggles PinC2 value.
    static void toggle()    { PINC |= (1<<2); }

    /// Configures PinC2  as an output pin.
    static void setOutput() { DDRC |= (1<<2); }

    /// Configures PinC2  as an input pin.
    static void setInput()  { DDRC &= ~(1<<2); }

    /// Pulses PinC2 with high state first.
    static void pulseHigh() { PORTC |= (1<<2); PORTC &= ~(1<<2); }

    /// Pulses PinC2 with low state first.
    static void pulseLow()  { PORTC &= ~(1<<2); PORTC |= (1<<2); }

    /// Reads PinC2  value.
    /// @return Port pin value.
    static bool test()      { return PINC & (1<<2); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<2)
    static constexpr uint8_t bitmask()               { return (1<<2); }

    /// Returns the bit corresponding to this pin.
    /// @return 2
    static constexpr uint8_t bit()                   { return 2; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortC;
};

struct PinC1 {
    /// Sets PinC1 to HIGH.
    static void set()       { PORTC |= (1<<1); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinC1 to LOW.
    static void clear()     { PORTC &= ~(1<<1); }

    /// Toggles PinC1 value.
    static void toggle()    { PINC |= (1<<1); }

    /// Configures PinC1  as an output pin.
    static void setOutput() { DDRC |= (1<<1); }

    /// Configures PinC1  as an input pin.
    static void setInput()  { DDRC &= ~(1<<1); }

    /// Pulses PinC1 with high state first.
    static void pulseHigh() { PORTC |= (1<<1); PORTC &= ~(1<<1); }

    /// Pulses PinC1 with low state first.
    static void pulseLow()  { PORTC &= ~(1<<1); PORTC |= (1<<1); }

    /// Reads PinC1  value.
    /// @return Port pin value.
    static bool test()      { return PINC & (1<<1); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<1)
    static constexpr uint8_t bitmask()               { return (1<<1); }

    /// Returns the bit corresponding to this pin.
    /// @return 1
    static constexpr uint8_t bit()                   { return 1; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortC;
};

struct PinC0 {
    /// Sets PinC0 to HIGH.
    static void set()       { PORTC |= (1<<0); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinC0 to LOW.
    static void clear()     { PORTC &= ~(1<<0); }

    /// Toggles PinC0 value.
    static void toggle()    { PINC |= (1<<0); }

    /// Configures PinC0  as an output pin.
    static void setOutput() { DDRC |= (1<<0); }

    /// Configures PinC0  as an input pin.
    static void setInput()  { DDRC &= ~(1<<0); }

    /// Pulses PinC0 with high state first.
    static void pulseHigh() { PORTC |= (1<<0); PORTC &= ~(1<<0); }

    /// Pulses PinC0 with low state first.
    static void pulseLow()  { PORTC &= ~(1<<0); PORTC |= (1<<0); }

    /// Reads PinC0  value.
    /// @return Port pin value.
    static bool test()      { return PINC & (1<<0); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<0)
    static constexpr uint8_t bitmask()               { return (1<<0); }

    /// Returns the bit corresponding to this pin.
    /// @return 0
    static constexpr uint8_t bit()                   { return 0; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortC;
};


struct PortD {

    /// Assigns a value to PORTD
    /// @param[in] value value affected to PORTD
    static void assign(uint8_t value)   { PORTD = value; }

    /// Sets masked bits in PORTD
    /// @param[in] mask bits to set
    static void setBits(uint8_t mask)   { PORTD |= mask;}

    /// Clears masked bits in PORTD
    /// @param[in] mask bits to clear
    static void clearBits(uint8_t mask) { PORTD &= ~mask;} 

    /// Changes values of masked bits in PORTD
    /// @param[in] mask bits to change
    /// @param[in] value new bits values
    static void changeBits(uint8_t mask, uint8_t value) { uint8_t tmp = PORTD & ~mask; PORTD = tmp | value; } 

    /// Toggles masked bits in PORTD
    /// @param[in] mask bits to toggle
    static void toggleBits(uint8_t mask) { PORTD ^= mask;} 

    /// Pulses masked bits in PORTD with high state first.
    /// @param[in] mask bits to pulse
    static void pulseHigh(uint8_t mask) { PORTD |= mask; PORTD &= ~mask; }

    /// Pulses masked bits in PORTD with low state first.
    /// @param[in] mask bits to pulse
    static void pulseLow(uint8_t mask)  { PORTD &= ~mask; PORTD |= mask; }

    /// Set corresponding masked bits of PORTD to output direction.
    /// @param[in] mask bits
    static void setOutput(uint8_t mask)    { DDRD |= mask; }

    /// Set corresponding masked bits of PORTD to input direction.
    /// @param[in] mask bits
    static void setInput(uint8_t mask)  { DDRD &= ~mask; }

    /// Returns PIND register.
    static uint8_t getPIN()             { return PIND; }

    /// Tests masked bits of PORTD
    /// @param[in] mask bits
    /// @param[in] true if the corresponding bits are all set, false otherwise.
    static bool testBits(uint8_t mask)  { return (PIND & mask) == mask; }

    /// Returns the value of the bit at the position pos.
    /// @param[in] position of the bit to return
    /// @return true if the requested bit is set, false otherwise.
    static bool test(uint8_t pos) { return PIND & (1<<pos); }

};

struct PinD7 {
    /// Sets PinD7 to HIGH.
    static void set()       { PORTD |= (1<<7); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinD7 to LOW.
    static void clear()     { PORTD &= ~(1<<7); }

    /// Toggles PinD7 value.
    static void toggle()    { PIND |= (1<<7); }

    /// Configures PinD7  as an output pin.
    static void setOutput() { DDRD |= (1<<7); }

    /// Configures PinD7  as an input pin.
    static void setInput()  { DDRD &= ~(1<<7); }

    /// Pulses PinD7 with high state first.
    static void pulseHigh() { PORTD |= (1<<7); PORTD &= ~(1<<7); }

    /// Pulses PinD7 with low state first.
    static void pulseLow()  { PORTD &= ~(1<<7); PORTD |= (1<<7); }

    /// Reads PinD7  value.
    /// @return Port pin value.
    static bool test()      { return PIND & (1<<7); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<7)
    static constexpr uint8_t bitmask()               { return (1<<7); }

    /// Returns the bit corresponding to this pin.
    /// @return 7
    static constexpr uint8_t bit()                   { return 7; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortD;
};

struct PinD6 {
    /// Sets PinD6 to HIGH.
    static void set()       { PORTD |= (1<<6); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinD6 to LOW.
    static void clear()     { PORTD &= ~(1<<6); }

    /// Toggles PinD6 value.
    static void toggle()    { PIND |= (1<<6); }

    /// Configures PinD6  as an output pin.
    static void setOutput() { DDRD |= (1<<6); }

    /// Configures PinD6  as an input pin.
    static void setInput()  { DDRD &= ~(1<<6); }

    /// Pulses PinD6 with high state first.
    static void pulseHigh() { PORTD |= (1<<6); PORTD &= ~(1<<6); }

    /// Pulses PinD6 with low state first.
    static void pulseLow()  { PORTD &= ~(1<<6); PORTD |= (1<<6); }

    /// Reads PinD6  value.
    /// @return Port pin value.
    static bool test()      { return PIND & (1<<6); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<6)
    static constexpr uint8_t bitmask()               { return (1<<6); }

    /// Returns the bit corresponding to this pin.
    /// @return 6
    static constexpr uint8_t bit()                   { return 6; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortD;
};

struct PinD5 {
    /// Sets PinD5 to HIGH.
    static void set()       { PORTD |= (1<<5); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinD5 to LOW.
    static void clear()     { PORTD &= ~(1<<5); }

    /// Toggles PinD5 value.
    static void toggle()    { PIND |= (1<<5); }

    /// Configures PinD5  as an output pin.
    static void setOutput() { DDRD |= (1<<5); }

    /// Configures PinD5  as an input pin.
    static void setInput()  { DDRD &= ~(1<<5); }

    /// Pulses PinD5 with high state first.
    static void pulseHigh() { PORTD |= (1<<5); PORTD &= ~(1<<5); }

    /// Pulses PinD5 with low state first.
    static void pulseLow()  { PORTD &= ~(1<<5); PORTD |= (1<<5); }

    /// Reads PinD5  value.
    /// @return Port pin value.
    static bool test()      { return PIND & (1<<5); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<5)
    static constexpr uint8_t bitmask()               { return (1<<5); }

    /// Returns the bit corresponding to this pin.
    /// @return 5
    static constexpr uint8_t bit()                   { return 5; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortD;
};

struct PinD4 {
    /// Sets PinD4 to HIGH.
    static void set()       { PORTD |= (1<<4); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinD4 to LOW.
    static void clear()     { PORTD &= ~(1<<4); }

    /// Toggles PinD4 value.
    static void toggle()    { PIND |= (1<<4); }

    /// Configures PinD4  as an output pin.
    static void setOutput() { DDRD |= (1<<4); }

    /// Configures PinD4  as an input pin.
    static void setInput()  { DDRD &= ~(1<<4); }

    /// Pulses PinD4 with high state first.
    static void pulseHigh() { PORTD |= (1<<4); PORTD &= ~(1<<4); }

    /// Pulses PinD4 with low state first.
    static void pulseLow()  { PORTD &= ~(1<<4); PORTD |= (1<<4); }

    /// Reads PinD4  value.
    /// @return Port pin value.
    static bool test()      { return PIND & (1<<4); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<4)
    static constexpr uint8_t bitmask()               { return (1<<4); }

    /// Returns the bit corresponding to this pin.
    /// @return 4
    static constexpr uint8_t bit()                   { return 4; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortD;
};

struct PinD3 {
    /// Sets PinD3 to HIGH.
    static void set()       { PORTD |= (1<<3); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinD3 to LOW.
    static void clear()     { PORTD &= ~(1<<3); }

    /// Toggles PinD3 value.
    static void toggle()    { PIND |= (1<<3); }

    /// Configures PinD3  as an output pin.
    static void setOutput() { DDRD |= (1<<3); }

    /// Configures PinD3  as an input pin.
    static void setInput()  { DDRD &= ~(1<<3); }

    /// Pulses PinD3 with high state first.
    static void pulseHigh() { PORTD |= (1<<3); PORTD &= ~(1<<3); }

    /// Pulses PinD3 with low state first.
    static void pulseLow()  { PORTD &= ~(1<<3); PORTD |= (1<<3); }

    /// Reads PinD3  value.
    /// @return Port pin value.
    static bool test()      { return PIND & (1<<3); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<3)
    static constexpr uint8_t bitmask()               { return (1<<3); }

    /// Returns the bit corresponding to this pin.
    /// @return 3
    static constexpr uint8_t bit()                   { return 3; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortD;
};

struct PinD2 {
    /// Sets PinD2 to HIGH.
    static void set()       { PORTD |= (1<<2); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinD2 to LOW.
    static void clear()     { PORTD &= ~(1<<2); }

    /// Toggles PinD2 value.
    static void toggle()    { PIND |= (1<<2); }

    /// Configures PinD2  as an output pin.
    static void setOutput() { DDRD |= (1<<2); }

    /// Configures PinD2  as an input pin.
    static void setInput()  { DDRD &= ~(1<<2); }

    /// Pulses PinD2 with high state first.
    static void pulseHigh() { PORTD |= (1<<2); PORTD &= ~(1<<2); }

    /// Pulses PinD2 with low state first.
    static void pulseLow()  { PORTD &= ~(1<<2); PORTD |= (1<<2); }

    /// Reads PinD2  value.
    /// @return Port pin value.
    static bool test()      { return PIND & (1<<2); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<2)
    static constexpr uint8_t bitmask()               { return (1<<2); }

    /// Returns the bit corresponding to this pin.
    /// @return 2
    static constexpr uint8_t bit()                   { return 2; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortD;
};

struct PinD1 {
    /// Sets PinD1 to HIGH.
    static void set()       { PORTD |= (1<<1); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinD1 to LOW.
    static void clear()     { PORTD &= ~(1<<1); }

    /// Toggles PinD1 value.
    static void toggle()    { PIND |= (1<<1); }

    /// Configures PinD1  as an output pin.
    static void setOutput() { DDRD |= (1<<1); }

    /// Configures PinD1  as an input pin.
    static void setInput()  { DDRD &= ~(1<<1); }

    /// Pulses PinD1 with high state first.
    static void pulseHigh() { PORTD |= (1<<1); PORTD &= ~(1<<1); }

    /// Pulses PinD1 with low state first.
    static void pulseLow()  { PORTD &= ~(1<<1); PORTD |= (1<<1); }

    /// Reads PinD1  value.
    /// @return Port pin value.
    static bool test()      { return PIND & (1<<1); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<1)
    static constexpr uint8_t bitmask()               { return (1<<1); }

    /// Returns the bit corresponding to this pin.
    /// @return 1
    static constexpr uint8_t bit()                   { return 1; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortD;
};

struct PinD0 {
    /// Sets PinD0 to HIGH.
    static void set()       { PORTD |= (1<<0); }

    static void set(bool v) { v ? set() : clear(); }

    /// Sets PinD0 to LOW.
    static void clear()     { PORTD &= ~(1<<0); }

    /// Toggles PinD0 value.
    static void toggle()    { PIND |= (1<<0); }

    /// Configures PinD0  as an output pin.
    static void setOutput() { DDRD |= (1<<0); }

    /// Configures PinD0  as an input pin.
    static void setInput()  { DDRD &= ~(1<<0); }

    /// Pulses PinD0 with high state first.
    static void pulseHigh() { PORTD |= (1<<0); PORTD &= ~(1<<0); }

    /// Pulses PinD0 with low state first.
    static void pulseLow()  { PORTD &= ~(1<<0); PORTD |= (1<<0); }

    /// Reads PinD0  value.
    /// @return Port pin value.
    static bool test()      { return PIND & (1<<0); }

    /// Returns the bitmask corresponding to this pin.
    /// @return (1<<0)
    static constexpr uint8_t bitmask()               { return (1<<0); }

    /// Returns the bit corresponding to this pin.
    /// @return 0
    static constexpr uint8_t bit()                   { return 0; }

    /// Port is defined as the Port object to which this pin belongs.
    using Port = PortD;
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

struct Timer2 {
  using value_type = uint8_t;
  static const uint8_t timer_width = 8;

  enum Prescaler : uint8_t {
    STOP_TIMER = 0, NO_PRESCALER = (1<<CS20), CLK_DIV_8 = (1<<CS21),
    CLK_DIV_64 = (1<<CS21)|(1<<CS20), CLK_DIV_256 = (1<<CS22),
    CLK_DIV_1024 = (1<<CS22)|(1<<CS20), BITS = (1<<CS22)|(1<<CS21)|(1<<CS20) };
  enum ClockSource : uint8_t {
    INC_ON_FALLING = (1<<CS22)|(1<<CS21), INC_ON_RISING = (1<<CS22)|(1<<CS21)|(1<<CS20) };

  enum Interrupt : uint8_t { 
    OVERFLOW = (1<<TOIE2), COMPARE_MATCH_A = (1<<OCIE2A), COMPARE_MATCH_B = (1<<OCIE2B)};
  enum Constants : uint8_t { ALL_BITS = 0xFF };
  static value_type getValue()                 { return TCNT2; }
  static void setValue(value_type value)       { TCNT2 = value; }
  static void addValue(value_type value)       { TCNT2 += value; }
  static void subValue(value_type value)       { TCNT2 -= value; }
  static void setCtrlRegA(uint8_t mask)        { TCCR2A |= mask; }
  static void clearCtrlRegA(uint8_t mask)      { TCCR2A &= ~mask; }
  static void setCtrlRegB(uint8_t mask)        { TCCR2B |= mask; }
  static void clearCtrlRegB(uint8_t mask)      { TCCR2B &= ~mask; }
  static void setInterruptMask(uint8_t mask)   { TIMSK2 |= mask; }
  static void clearInterruptMask(uint8_t mask) { TIMSK2 &= ~mask; }
  static void setPrescaler(Prescaler val)      { TCCR2B &= ~Prescaler::BITS; TCCR2B |= val; }
  static void setExternalClockSource(ClockSource val)  { TCCR2B &= ~Prescaler::BITS; TCCR2B |= val; }
  static void setOutputCompareValueA(value_type value) { OCR2A = value; }
  static value_type getOutputCompareValueA()           { return OCR2A; }
  static void setOutputCompareValueB(value_type value) { OCR2B = value; }
  static value_type getOutputCompareValueB()           { return OCR2B; }

  struct Isr {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER2_OVF_vect, signal);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER2_COMPA_vect, signal);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER2_COMPB_vect, signal);
  };

  struct IsrNoBlock {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER2_OVF_vect, interrupt);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER2_COMPA_vect, interrupt);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER2_COMPB_vect, interrupt);
  };

  struct IsrNaked {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER2_OVF_vect, naked);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER2_COMPA_vect, naked);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER2_COMPB_vect, naked);
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

template<> struct is_uart_rxd_capable<PinD0> : std::true_type {};
template<> struct is_uart_txd_capable<PinD1> : std::true_type {};

class Uart0Driver {
    static const uint8_t BufferSize = 32;
    static uint8_t circularBuffer[BufferSize];
    static uint8_t readIndex, writeIndex;
protected:
    static void start(uint32_t BAUD_RATE, FrameFormat FRAME_FORMAT) {
        readIndex = 0;
        writeIndex = 0;
        uint16_t spd = (Device::McuFrequency / (BAUD_RATE << 4)) - 1;
        UBRR0H = static_cast<uint8_t>(spd>>8);
        UBRR0L = static_cast<uint8_t>(spd);
        
        auto nbBits = (FRAME_FORMAT & FrameFormat::NbBitsMask) >> 3;
        auto nbBits1 = (nbBits & 0b10) == 0b10;
        auto nbBits0 = (nbBits & 0b1) == 0b1;
        
        auto parityMode = (FRAME_FORMAT & FrameFormat::ParityMask) >> 1;
        auto parity0 = parityMode == 0b10;
        auto parity1 = parityMode != 0b00;
         
        auto stopBit = (FRAME_FORMAT & FrameFormat::StopBitMask) >> 0;
        
        UCSR0C = (nbBits1<<UCSZ01) | (nbBits0<<UCSZ00) | (parity0<<UPM00) | (parity1<<UPM01) | (stopBit<<USBS0);
        UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);
     }

    using ReceiveCallback = void(*)(uint8_t);
    static ReceiveCallback receiveCallback;

public:
    static void writeAsync(uint8_t datum) {
        if (UCSR0A & (1<<UDRE0)) {
            UDR0 = datum;
        }
        else {
            circularBuffer[writeIndex] = datum;
            if (writeIndex == (BufferSize - 1)) {
                writeIndex = 0;
            } else {
                writeIndex++;
            }
            UCSR0B |= 1<<UDRIE0;
        }
    }

    static uint8_t getRemainingBufferSize() {
        return writeIndex > readIndex ? writeIndex - readIndex : writeIndex + BufferSize - readIndex;
    }

    struct Isr {
        static void receiveComplete() IOPORTS_IRQ_HANDLER(USART0_RX_vect, signal);
        static void fifoEmpty() IOPORTS_IRQ_HANDLER(USART0_UDRE_vect, signal);
    };
};

uint8_t Uart0Driver::readIndex = 0;
uint8_t Uart0Driver::writeIndex = 0;
uint8_t Uart0Driver::circularBuffer[32];
Uart0Driver::ReceiveCallback Uart0Driver::receiveCallback = nullptr;

void Uart0Driver::Isr::fifoEmpty() {
     if (readIndex != writeIndex) {
        UDR0 = circularBuffer[readIndex];
        if (readIndex == (BufferSize - 1)) {
            readIndex = 0;
        } else {
            readIndex++;
        }
    }
    else {
        UCSR0B &= ~(1<<UDRIE0);
    }
}

void Uart0Driver::Isr::receiveComplete() {
    if (receiveCallback) {
        receiveCallback(UDR0);
    }
}

template<uint32_t BAUD_RATE, FrameFormat FRAME_FORMAT, typename CharType>
class Uart0 : public Uart0Driver {
public:
    using ReceiveCallback = void(*)(CharType);
    enum class StatusFlags : uint8_t {
        ReceiveComplete = 1<<RXC0,
        TransmitComplete = 1<<TXC0,
        FrameError = 1<<FE0,
        DataOverrun = 1<<DOR0,
        ParityError = 1<<UPE0,
    };
    
    static void start() { Uart0Driver::start(BAUD_RATE, FRAME_FORMAT); }
    static void start(ReceiveCallback func) {
        receiveCallback = static_cast<Uart0Driver::ReceiveCallback>(func);
        start();
    }

static void write(CharType datum) { }
};

template<> struct is_uart_rxd_capable<PinD2> : std::true_type {};
template<> struct is_uart_txd_capable<PinD3> : std::true_type {};

class Uart1Driver {
    static const uint8_t BufferSize = 32;
    static uint8_t circularBuffer[BufferSize];
    static uint8_t readIndex, writeIndex;
protected:
    static void start(uint32_t BAUD_RATE, FrameFormat FRAME_FORMAT) {
        readIndex = 0;
        writeIndex = 0;
        uint16_t spd = (Device::McuFrequency / (BAUD_RATE << 4)) - 1;
        UBRR1H = static_cast<uint8_t>(spd>>8);
        UBRR1L = static_cast<uint8_t>(spd);
        
        auto nbBits = (FRAME_FORMAT & FrameFormat::NbBitsMask) >> 3;
        auto nbBits1 = (nbBits & 0b10) == 0b10;
        auto nbBits0 = (nbBits & 0b1) == 0b1;
        
        auto parityMode = (FRAME_FORMAT & FrameFormat::ParityMask) >> 1;
        auto parity0 = parityMode == 0b10;
        auto parity1 = parityMode != 0b00;
         
        auto stopBit = (FRAME_FORMAT & FrameFormat::StopBitMask) >> 0;
        
        UCSR1C = (nbBits1<<UCSZ11) | (nbBits0<<UCSZ10) | (parity0<<UPM10) | (parity1<<UPM11) | (stopBit<<USBS1);
        UCSR1B = (1<<RXEN1) | (1<<TXEN1) | (1<<RXCIE1);
     }

    using ReceiveCallback = void(*)(uint8_t);
    static ReceiveCallback receiveCallback;

public:
    static void writeAsync(uint8_t datum) {
        if (UCSR1A & (1<<UDRE1)) {
            UDR1 = datum;
        }
        else {
            circularBuffer[writeIndex] = datum;
            if (writeIndex == (BufferSize - 1)) {
                writeIndex = 0;
            } else {
                writeIndex++;
            }
            UCSR1B |= 1<<UDRIE1;
        }
    }

    static uint8_t getRemainingBufferSize() {
        return writeIndex > readIndex ? writeIndex - readIndex : writeIndex + BufferSize - readIndex;
    }

    struct Isr {
        static void receiveComplete() IOPORTS_IRQ_HANDLER(USART1_RX_vect, signal);
        static void fifoEmpty() IOPORTS_IRQ_HANDLER(USART1_UDRE_vect, signal);
    };
};

uint8_t Uart1Driver::readIndex = 0;
uint8_t Uart1Driver::writeIndex = 0;
uint8_t Uart1Driver::circularBuffer[32];
Uart1Driver::ReceiveCallback Uart1Driver::receiveCallback = nullptr;

void Uart1Driver::Isr::fifoEmpty() {
     if (readIndex != writeIndex) {
        UDR1 = circularBuffer[readIndex];
        if (readIndex == (BufferSize - 1)) {
            readIndex = 0;
        } else {
            readIndex++;
        }
    }
    else {
        UCSR1B &= ~(1<<UDRIE1);
    }
}

void Uart1Driver::Isr::receiveComplete() {
    if (receiveCallback) {
        receiveCallback(UDR1);
    }
}

template<uint32_t BAUD_RATE, FrameFormat FRAME_FORMAT, typename CharType>
class Uart1 : public Uart1Driver {
public:
    using ReceiveCallback = void(*)(CharType);
    enum class StatusFlags : uint8_t {
        ReceiveComplete = 1<<RXC1,
        TransmitComplete = 1<<TXC1,
        FrameError = 1<<FE1,
        DataOverrun = 1<<DOR1,
        ParityError = 1<<UPE1,
    };
    
    static void start() { Uart1Driver::start(BAUD_RATE, FRAME_FORMAT); }
    static void start(ReceiveCallback func) {
        receiveCallback = static_cast<Uart1Driver::ReceiveCallback>(func);
        start();
    }

static void write(CharType datum) { }
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
