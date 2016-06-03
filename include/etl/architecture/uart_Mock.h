/// @file uart_Mock.h
/// @date 03/06/2016 09:34:16
/// @author Ambroise Leclerc and Cécile Gomes
/// @brief Microcontrollers peripherals handling classes
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

// Traits à définir dans etl/architecture/uart_archxx.h
template<typename> struct is_uart_txd_capable  : false_type {};
template<> struct is_uart_txd_capable<Pin0>    : true_type  {};
template<> struct is_uart_txd_capable<Pin14>   : true_type  {};

template<typename> struct is_uart_rxd_capable  : false_type {};
template<> struct is_uart_rxd_capable<Pin0>    : true_type    {};
template<> struct is_uart_rxd_capable<Pin14>   : true_type   {};


enum FrameFormat {
    _5N1 = 0b000000, _5N2 = 0b0000001, _5E1 = 0b000010, _5E2 = 0b000011, _5O1 = 0b000100, _5O2 = 0b000101,       // Encoding 5-N-1 : nbBits - no, even or odd parity - 1 or 2 stop bits : 0b000 - 00 - 0
    _6N1 = 0b001000, _6N2 = 0b0010001, _6E1 = 0b001010, _6E2 = 0b001011, _6O1 = 0b001100, _6O2 = 0b001101,
    _7N1 = 0b010000, _7N2 = 0b0100001, _7E1 = 0b010010, _7E2 = 0b010011, _7O1 = 0b010100, _7O2 = 0b010101,
    _8N1 = 0b011000, _8N2 = 0b0110001, _8E1 = 0b011010, _8E2 = 0b011011, _8O1 = 0b011100, _8O2 = 0b011101,
    _9N1 = 0b100000, _9N2 = 0b1000001, _9E1 = 0b100010, _9E2 = 0b100011, _9O1 = 0b100100, _9O2 = 0b100101
};

template<typename RxD, typename TxD, uint32_t BAUD_RATE = 9600, FrameFormat FRAME_FORMAT = FrameFormat::_8N1>
class Uart {
public:
    static void start() {
        static_assert(is_uart_txd_capable(TxD), "Pin has no TxD capacity");
        static_assert(is_uart_rxd_capable(RxD), "Pin has no RxD capacity");
    }

    static void stop() {
        
    }

    

};

} // namespace etl