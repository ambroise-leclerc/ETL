/// @file tm1638.h
/// @date 09/09/2016 17:17:16
/// @author Ambroise Leclerc and Cécile Gomes
/// @brief TM1638 led&key board driver
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

namespace etl {

template <typename Strobe, typename Clk, typename Data>
class TM1638 {
public:
    static void init(uint8_t brightness) {
        Strobe::setOutput();
        Clk::setOutput();
        Data::setOutput();
        Strobe::set();
        issueCommand(Commands::ACTIVATE, brightness & 0b1111);
        reset();
    }
	
    static void setLeds(uint8_t value) {
        for (auto i = 0; i < 8; i++) {
            setLed(i, (value & (1 << i)) != 0);
        }
    }

    static void setLed(uint8_t ledNumber, uint8_t value) {
        issueCommand(Commands::WRITE);
        issueCommand(Commands::RAM_LED, ledNumber << 1, value);
    }
    
    static void display(uint8_t position, char character) {
        issueCommand(Commands::WRITE);
        issueCommand(Commands::CHAR_LED, position << 1, font[character - 32]);
    }
    
    static void reset() {
        for (uint8_t index = 0; index < 8; index++) {
            issueCommand(Commands::WRITE);
            issueCommand(Commands::CHAR_LED, index << 1, 0);
        }
    }

    static uint8_t readKey() {
        using namespace std;
        using namespace chrono_literals;
       
	    Strobe::clear();
        write(static_cast<uint8_t>(Commands::READ));
        Data::setInput();
        this_thread::sleep_for(1us);
        uint8_t buttons = 0;
        for (auto bitIndex = 0; bitIndex < 4; ++bitIndex) {
            auto byteRead = read();
            buttons |= (byteRead << bitIndex);
        }
        Data::setOutput();
        Strobe::set();
        return buttons;
    }

private:
    enum class Commands : uint8_t {
        READ = 0x42,
        WRITE = 0x44,
        ACTIVATE = 0x80,
        CHAR_LED = 0xC0,
        RAM_LED = 0xC1		
    };

    static const uint8_t digits[];
    static const uint8_t font[];

    static void issueCommand(Commands cmd, uint8_t param = 0) {
        Strobe::clear();
        write(static_cast<uint8_t>(cmd) + param);
        Strobe::set();
    }

    static void issueCommand(Commands cmd, uint8_t param, uint8_t data) {
        Strobe::clear();
        write(static_cast<uint8_t>(cmd) + param);
        write(data);
        Strobe::set();
    }

    static void write(uint8_t data) {
        for (auto bitIndex = 0; bitIndex < 8; ++bitIndex) {
            Data::set((data & (1 << bitIndex)) != 0);
            Clk::pulseHigh();
        }
    }

    static uint8_t read() {
        uint8_t value = 0;
        for (auto bitIndex = 0; bitIndex < 8; ++bitIndex) {
            Clk::set();
            value |= Data::test() << bitIndex;
            Clk::clear();
        }
        return value;
    }    
};




template <typename Strobe, typename Clk, typename Data> const uint8_t TM1638<Strobe, Clk, Data>::digits[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f};
template <typename Strobe, typename Clk, typename Data> const uint8_t TM1638<Strobe, Clk, Data>::font[] = {
    0b00000000 /* */, 0b10000110 /*!*/, 0b00100010 /*"*/, 0b01111110 /*#*/, 0b01101101 /*$*/, 0b00000000 /*%*/, 0b00000000 /*&*/, 0b00000010 /*'*/,
    0b00110000 /*(*/, 0b00000110 /*)*/, 0b01100011 /***/, 0b00000000 /*+*/, 0b00000100 /*,*/, 0b01000000 /*-*/, 0b10000000 /*.*/, 0b01010010 /*/*/,
    0b00111111 /*0*/, 0b00000110 /*1*/, 0b01011011 /*2*/, 0b01001111 /*3*/, 0b01100110 /*4*/, 0b01101101 /*5*/, 0b01111101 /*6*/, 0b00100111 /*7*/,
    0b01111111 /*8*/, 0b01101111 /*9*/, 0b00000000 /*:*/, 0b00000000 /*;*/, 0b00000000 /*<*/, 0b01001000 /*=*/, 0b00000000 /*>*/, 0b01010011 /*?*/,
    0b01011111 /*@*/, 0b01110111 /*A*/, 0b01111111 /*B*/, 0b00111001 /*C*/, 0b00111111 /*D*/, 0b01111001 /*E*/, 0b01110001 /*F*/, 0b00111101 /*G*/,
    0b01110110 /*H*/, 0b00000110 /*I*/, 0b00011111 /*J*/, 0b01101001 /*K*/, 0b00111000 /*L*/, 0b00010101 /*M*/, 0b00110111 /*N*/, 0b00111111 /*O*/,
    0b01110011 /*P*/, 0b01100111 /*Q*/, 0b00110001 /*R*/, 0b01101101 /*S*/, 0b01111000 /*T*/, 0b00111110 /*U*/, 0b00101010 /*V*/, 0b00011101 /*W*/,
    0b01110110 /*X*/, 0b01101110 /*Y*/, 0b01011011 /*Z*/, 0b00111001 /*[*/, 0b01100100 /*\*/, 0b00001111 /*]*/, 0b00000000 /*^*/, 0b00001000 /*_*/,
    0b00100000 /*`*/, 0b01011111 /*a*/, 0b01111100 /*b*/, 0b01011000 /*c*/, 0b01011110 /*d*/, 0b01111011 /*e*/, 0b00110001 /*f*/, 0b01101111 /*g*/,
    0b01110100 /*h*/, 0b00000100 /*i*/, 0b00001110 /*j*/, 0b01110101 /*k*/, 0b00110000 /*l*/, 0b01010101 /*m*/, 0b01010100 /*n*/, 0b01011100 /*o*/,
    0b01110011 /*p*/, 0b01100111 /*q*/, 0b01010000 /*r*/, 0b01101101 /*s*/, 0b01111000 /*t*/, 0b00011100 /*u*/, 0b00101010 /*v*/, 0b00011101 /*w*/,
    0b01110110 /*x*/, 0b01101110 /*y*/, 0b01000111 /*z*/, 0b01000110 /*{*/, 0b00000110 /*|*/, 0b01110000 /*}*/, 0b00000001 /*~*/ };

}// namespace etl

