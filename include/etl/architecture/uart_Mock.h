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
#include <cstdint>
#include <type_traits>
#include <thread>
#include <tuple>  
#include <chrono>   

namespace etl {

    // Traits à définir dans etl/architecture/uart_archxx.h


    template<typename> struct is_uart_txd_capable : std::false_type {};
    template<> struct is_uart_txd_capable<Pin0> : std::true_type {};
    template<> struct is_uart_txd_capable<Pin14> : std::true_type {};

    template<typename> struct is_uart_rxd_capable : std::false_type {};
    template<> struct is_uart_rxd_capable<Pin0> : std::true_type {};
    template<> struct is_uart_rxd_capable<Pin14> : std::true_type {};


    enum FrameFormat {
        _5N1 = 0b000000, _5N2 = 0b0000001, _5E1 = 0b000010, _5E2 = 0b000011, _5O1 = 0b000100, _5O2 = 0b000101,       // Encoding 5-N-1 : nbBits - no, even or odd parity - 1 or 2 stop bits : 0b000 - 00 - 0
        _6N1 = 0b001000, _6N2 = 0b0010001, _6E1 = 0b001010, _6E2 = 0b001011, _6O1 = 0b001100, _6O2 = 0b001101,
        _7N1 = 0b010000, _7N2 = 0b0100001, _7E1 = 0b010010, _7E2 = 0b010011, _7O1 = 0b010100, _7O2 = 0b010101,
        _8N1 = 0b011000, _8N2 = 0b0110001, _8E1 = 0b011010, _8E2 = 0b011011, _8O1 = 0b011100, _8O2 = 0b011101,
        _9N1 = 0b100000, _9N2 = 0b1000001, _9E1 = 0b100010, _9E2 = 0b100011, _9O1 = 0b100100, _9O2 = 0b100101
    };

    template<typename RxD, typename TxD, uint32_t BAUD_RATE = 9600, FrameFormat FRAME_FORMAT = FrameFormat::_8N1, typename SizeUint = uint8_t>
    class Uart {
    public:
        static_assert(is_uart_txd_capable<TxD>::value, "Pin has no TxD capacity");
        static_assert(is_uart_rxd_capable<RxD>::value, "Pin has no RxD capacity");

        static void start() {
            std::cout << '\n';
            RxD::setInput();
            TxD::setOutput();
            TxD::set();
            RxD::set();
            std::cout << '\n';
            std::cout << "bitNumber: " << bitNumber << '\n';
            std::cout << "parity: " << parity << '\n';
            std::cout << "stopBit: " << stopBit << '\n';
            std::cout << "RxD::test(): " << RxD::test() << '\n';
        }

        static void stop() {

        }

        static SizeUint read() {

            while (rxdRead()) {

            }
            auto returnValue = readBits();
            assert(readParity(std::get<1>(returnValue)));
            assert(readStopBits());
            return std::get<0>(returnValue);
        };

        static void write(SizeUint datum) {
            txdClear();
            uint8_t nbOdd = sendBit(datum);
            sendParity(nbOdd);
            sendStopBit();
        }

    private:
        enum Parity { None, Even, Odd };
        enum StopBit { One, Two };
        enum BitNumber { Five = 5, Six = 6, Seven = 7, Eight = 8, Nine = 9 };


        static const auto bitNumberMask = 0b111000;
        static const auto stopBitMask = 0b000001;
        static const auto parityMask = 0b000110;


        static constexpr  auto parity = static_cast<Parity>((FRAME_FORMAT & parityMask) >> 1);
        static constexpr  auto stopBit = static_cast<StopBit>(FRAME_FORMAT & stopBitMask);
        static constexpr  auto bitNumber = static_cast<BitNumber>((FRAME_FORMAT & bitNumberMask) >> 3) + 5;

        static auto  readStopBits() {
            bool ok = true;
            switch (stopBit) {
            case One:
                return (rxdRead());
                break;
            case Two:
                ok &= (rxdRead());
                ok &= (rxdRead());
                break;
            }
            return ok;
        }

        static auto  readBits() {
            SizeUint returnValue = 0;
            uint8_t nbOdd = 0;
            for (auto i = 0; i < bitNumber; i++) {
                if (rxdRead()) {
                    nbOdd++;
                    returnValue = (returnValue << 1) | 1;
                }
                else {
                    returnValue = (returnValue << 1);
                }
            }
            return std::make_tuple(returnValue, nbOdd);
        }

        static bool  readParity(uint8_t odd) {
            bool parityBit = rxdRead();
            switch (parity) {
            case Even:
                return (parityBit == (odd % 2));
                break;
            case Odd:
                return (parityBit == (((odd % 2) == 0) ? 1 : 0));
                break;
            default:
                return true;
            }
        }


        static auto  sendBit(SizeUint datum) {
            uint8_t nbOdd = 0;
            for (auto i = bitNumber - 1; i >= 0; i--) {
                auto value = (datum >> i) & 0x01;
                if (value == 1) {
                    nbOdd++;
                }
                txdSet(value);
            }
            return nbOdd;
        }

        static void sendParity(uint8_t odd) {
            switch (parity) {
            case None:
                txdClear();
                break;
            case Even:
                txdSet(odd % 2);
                break;
            case Odd:
                txdSet(((odd % 2) == 0) ? 1 : 0);
                break;
            }
        }

        static void sendStopBit() {
            switch (stopBit) {
            case One:
                txdSet();
                break;
            case Two:
                txdSet();
                txdSet();
                break;
            }
        }

        static void waitRead() {
            while (!dataToRead) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000u / BAUD_RATE));
            }

        }

        static void waitWrite() {
            while (dataToRead) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000u / BAUD_RATE));
            }
        }

        static void txdSet(uint8_t value) {
            waitWrite();
            if (value == 1)
            {
                TxD::set();
            }
            else {
                TxD::clear();
            }
            dataToRead = true;
        }

        static void txdSet() {
            waitWrite();
            TxD::set();
            dataToRead = true;
        }

        static void txdClear() {
            waitWrite();
            TxD::clear();
            dataToRead = true;
        }


        static bool rxdRead() {
            waitRead();
            bool value = RxD::test();
            dataToRead = false;
            return value;
        }

        static bool dataToRead;


    };

    template<typename RxD, typename TxD, uint32_t BAUD_RATE, FrameFormat FRAME_FORMAT, typename SizeUint> bool Uart<RxD, TxD, BAUD_RATE, FRAME_FORMAT, SizeUint>::dataToRead = false;
} // namespace etl