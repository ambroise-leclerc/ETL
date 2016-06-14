/// @file ioports_ATmega168A.h
/// @date 11/05/2014 19:26:16
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


namespace etl {
#define IOPORTS_TO_STRING(name) #name
#define IOPORTS_IRQ_HANDLER(vector, type) asm(IOPORTS_TO_STRING(vector)) __attribute__ ((type, __INTR_ATTRS))

class Device {
public:
    static void delay_us(uint32_t us)          { _delay_us(us); }
    static void delay_ms(uint32_t ms)          { _delay_ms(ms); }
    static const size_t flash_size = 16384;
    static const size_t eeprom_size = 512;
    static const size_t sram_size = 1024;
};

struct PinChangeIRQ0;
struct PinChangeIRQ1;
struct PinChangeIRQ2;

struct PortB {
  using PinChangeIRQ = PinChangeIRQ0;

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

struct PinB7 : public Pin<PortB> {
  /// Sets PinB7 to HIGH.
  static void set()       { PORTB |= (1<<7); }

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
};

struct PinB6 : public Pin<PortB> {
  /// Sets PinB6 to HIGH.
  static void set()       { PORTB |= (1<<6); }

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
};

struct PinB5 : public Pin<PortB> {
  /// Sets PinB5 to HIGH.
  static void set()       { PORTB |= (1<<5); }

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
};

struct PinB4 : public Pin<PortB> {
  /// Sets PinB4 to HIGH.
  static void set()       { PORTB |= (1<<4); }

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
};

struct PinB3 : public Pin<PortB> {
  /// Sets PinB3 to HIGH.
  static void set()       { PORTB |= (1<<3); }

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
};

struct PinB2 : public Pin<PortB> {
  /// Sets PinB2 to HIGH.
  static void set()       { PORTB |= (1<<2); }

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
};

struct PinB1 : public Pin<PortB> {
  /// Sets PinB1 to HIGH.
  static void set()       { PORTB |= (1<<1); }

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
};

struct PinB0 : public Pin<PortB> {
  /// Sets PinB0 to HIGH.
  static void set()       { PORTB |= (1<<0); }

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
};


struct PortC {
  using PinChangeIRQ = PinChangeIRQ1;

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

struct PinC6 : public Pin<PortC> {
  /// Sets PinC6 to HIGH.
  static void set()       { PORTC |= (1<<6); }

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
};

struct PinC5 : public Pin<PortC> {
  /// Sets PinC5 to HIGH.
  static void set()       { PORTC |= (1<<5); }

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
};

struct PinC4 : public Pin<PortC> {
  /// Sets PinC4 to HIGH.
  static void set()       { PORTC |= (1<<4); }

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
};

struct PinC3 : public Pin<PortC> {
  /// Sets PinC3 to HIGH.
  static void set()       { PORTC |= (1<<3); }

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
};

struct PinC2 : public Pin<PortC> {
  /// Sets PinC2 to HIGH.
  static void set()       { PORTC |= (1<<2); }

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
};

struct PinC1 : public Pin<PortC> {
  /// Sets PinC1 to HIGH.
  static void set()       { PORTC |= (1<<1); }

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
};

struct PinC0 : public Pin<PortC> {
  /// Sets PinC0 to HIGH.
  static void set()       { PORTC |= (1<<0); }

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
};


struct PortD {
  using PinChangeIRQ = PinChangeIRQ2;

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

struct PinD7 : public Pin<PortD> {
  /// Sets PinD7 to HIGH.
  static void set()       { PORTD |= (1<<7); }

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
};

struct PinD6 : public Pin<PortD> {
  /// Sets PinD6 to HIGH.
  static void set()       { PORTD |= (1<<6); }

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
};

struct PinD5 : public Pin<PortD> {
  /// Sets PinD5 to HIGH.
  static void set()       { PORTD |= (1<<5); }

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
};

struct PinD4 : public Pin<PortD> {
  /// Sets PinD4 to HIGH.
  static void set()       { PORTD |= (1<<4); }

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
};

struct PinD3 : public Pin<PortD> {
  /// Sets PinD3 to HIGH.
  static void set()       { PORTD |= (1<<3); }

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
};

struct PinD2 : public Pin<PortD> {
  /// Sets PinD2 to HIGH.
  static void set()       { PORTD |= (1<<2); }

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
};

struct PinD1 : public Pin<PortD> {
  /// Sets PinD1 to HIGH.
  static void set()       { PORTD |= (1<<1); }

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
};

struct PinD0 : public Pin<PortD> {
  /// Sets PinD0 to HIGH.
  static void set()       { PORTD |= (1<<0); }

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

struct Timer0 {
  using value_type = uint8_t;
  static const uint8_t timer_width = 8;

  enum Prescaler : uint8_t {
    STOP_TIMER = 0, NO_PRESCALER = (1<<CS00), CLK_DIV_8 = (1<<CS01),
    CLK_DIV_32 = (1<<CS01)|(1<<CS00), CLK_DIV_64 = (1<<CS02),
    CLK_DIV_128 = (1<<CS02)|(1<<CS00), CLK_DIV_256 = (1<<CS02)|(1<<CS01),
    CLK_DIV_1024 = (1<<CS02)|(1<<CS01)|(1<<CS00), BITS = (1<<CS02)|(1<<CS01)|(1<<CS00) };

  enum Interrupt : uint8_t { 
    OVERFLOW = (1<<TOIE0), COMPAREMATCHA = (1<<OCIE0A), COMPAREMATCHB = (1<<OCIE0B)};
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

struct Timer1 {
  using value_type = uint16_t;
  static const uint8_t timer_width = 16;

  enum Prescaler : uint8_t {
    STOP_TIMER = 0, NO_PRESCALER = (1<<CS10), CLK_DIV_8 = (1<<CS11),
    CLK_DIV_32 = (1<<CS11)|(1<<CS10), CLK_DIV_64 = (1<<CS12),
    CLK_DIV_128 = (1<<CS12)|(1<<CS10), CLK_DIV_256 = (1<<CS12)|(1<<CS11),
    CLK_DIV_1024 = (1<<CS12)|(1<<CS11)|(1<<CS10), BITS = (1<<CS12)|(1<<CS11)|(1<<CS10) };

  enum Interrupt : uint8_t { 
    OVERFLOW = (1<<TOIE1), CAPTURE = (1<<ICIE1), COMPAREMATCHA = (1<<OCIE1A), COMPAREMATCHB = (1<<OCIE1B)};
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
  static void SetOutputCompareValueA(value_type value) { OCR1A = value; }
  static value_type GetOutputCompareValueA()           { return OCR1A; }
  static void SetOutputCompareValueB(value_type value) { OCR1B = value; }
  static value_type GetOutputCompareValueB()           { return OCR1B; }

  struct ISR {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER1_OVF_vect, signal);
    static void Capture() IOPORTS_IRQ_HANDLER(TIMER1_CAPT_vect, signal);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER1_COMPA_vect, signal);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER1_COMPB_vect, signal);
  };

  struct ISRNoBlock {
    static void Overflow() IOPORTS_IRQ_HANDLER(TIMER1_OVF_vect, interrupt);
    static void Capture() IOPORTS_IRQ_HANDLER(TIMER1_CAPT_vect, interrupt);
    static void CompareMatchA() IOPORTS_IRQ_HANDLER(TIMER1_COMPA_vect, interrupt);
    static void CompareMatchB() IOPORTS_IRQ_HANDLER(TIMER1_COMPB_vect, interrupt);
  };

  struct ISRNaked {
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
    OVERFLOW = (1<<TOIE2), COMPAREMATCHA = (1<<OCIE2A), COMPAREMATCHB = (1<<OCIE2B)};
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
  static void SetExternalClockSource(ClockSource val)  { TCCR2B &= ~Prescaler::BITS; TCCR2B |= val; }
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

#ifdef PCICR
struct PinChangeControlRegister {
  static void SetBits(uint8_t mask)   { PCICR |= mask; }
  static void ClearBits(uint8_t mask) { PCICR &= ~mask; }
};
#endif // PCICR

struct PinChangeMask0 {
  static void SetBits(uint8_t mask)   { PCMSK0 |= mask; }
  static void ClearBits(uint8_t mask) { PCMSK0 &= ~mask; }
  static uint8_t Get()                { return PCMSK0; }
};

struct PinChangeIRQ0 {
  static void EnableSource(uint8_t PCINT) {
    PinChangeControlRegister::SetBits(1<<PCIE0);
    PinChangeMask0::SetBits(1<<PCINT);
  }

  static void DisableSource(uint8_t PCINT) {
    PinChangeMask0::ClearBits(1<<PCINT);
    if (0 == PinChangeMask0::Get()) {
      PinChangeControlRegister::ClearBits(1<<PCIE0);
    }
  }

  struct ISR {
    static void Trigger() IOPORTS_IRQ_HANDLER(PCINT0_vect, signal);
  };

  struct ISRNoBlock {
    static void Trigger() IOPORTS_IRQ_HANDLER(PCINT0_vect, interrupt);
  };

  struct ISRNaked {
    static void Trigger() IOPORTS_IRQ_HANDLER(PCINT0_vect, naked);
  };

};

struct PinChangeMask1 {
  static void SetBits(uint8_t mask)   { PCMSK1 |= mask; }
  static void ClearBits(uint8_t mask) { PCMSK1 &= ~mask; }
  static uint8_t Get()                { return PCMSK1; }
};

struct PinChangeIRQ1 {
  static void EnableSource(uint8_t PCINT) {
    PinChangeControlRegister::SetBits(1<<PCIE1);
    PinChangeMask1::SetBits(1<<PCINT);
  }

  static void DisableSource(uint8_t PCINT) {
    PinChangeMask1::ClearBits(1<<PCINT);
    if (0 == PinChangeMask1::Get()) {
      PinChangeControlRegister::ClearBits(1<<PCIE1);
    }
  }

  struct ISR {
    static void Trigger() IOPORTS_IRQ_HANDLER(PCINT1_vect, signal);
  };

  struct ISRNoBlock {
    static void Trigger() IOPORTS_IRQ_HANDLER(PCINT1_vect, interrupt);
  };

  struct ISRNaked {
    static void Trigger() IOPORTS_IRQ_HANDLER(PCINT1_vect, naked);
  };

};

struct PinChangeMask2 {
  static void SetBits(uint8_t mask)   { PCMSK2 |= mask; }
  static void ClearBits(uint8_t mask) { PCMSK2 &= ~mask; }
  static uint8_t Get()                { return PCMSK2; }
};

struct PinChangeIRQ2 {
  static void EnableSource(uint8_t PCINT) {
    PinChangeControlRegister::SetBits(1<<PCIE2);
    PinChangeMask2::SetBits(1<<PCINT);
  }

  static void DisableSource(uint8_t PCINT) {
    PinChangeMask2::ClearBits(1<<PCINT);
    if (0 == PinChangeMask2::Get()) {
      PinChangeControlRegister::ClearBits(1<<PCIE2);
    }
  }

  struct ISR {
    static void Trigger() IOPORTS_IRQ_HANDLER(PCINT2_vect, signal);
  };

  struct ISRNoBlock {
    static void Trigger() IOPORTS_IRQ_HANDLER(PCINT2_vect, interrupt);
  };

  struct ISRNaked {
    static void Trigger() IOPORTS_IRQ_HANDLER(PCINT2_vect, naked);
  };

};

#undef IOPORTS_TO_STRING
#undef IOPORTS_IRQ_HANDLER
} // namespace etl
