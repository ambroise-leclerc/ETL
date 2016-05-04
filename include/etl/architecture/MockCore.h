/// @file MockCore.h
/// @date 04 May 2016 01:04:16
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
#include <cstdint>
#include <array>
#include <list>
#include <iostream>
#include <vector>
#include <sstream>


namespace etl {
    class StringHash {
    public:
        static std::uint64_t calc(char const* zStr) {
            std::uint64_t ret{ basis };
            while (*zStr) {
                ret ^= *zStr;
                ret *= prime;
                zStr++;
            }
            return ret;
        }
    private:
        static constexpr std::uint64_t compileTime(char const* zStr, std::uint64_t lastValue = basis) {
            return *zStr ? compileTime(zStr + 1, (*zStr ^ lastValue) * prime) : lastValue;
        }

        static const auto basis = 14'695'981'039'346'656'037ull;
        static const auto prime = 1'099'511'628'211ull;

        friend constexpr std::uint64_t operator""_hash(char const* zStr, size_t);
    };

    constexpr std::uint64_t operator""_hash(char const* zStr, size_t) {
        return etl::StringHash::compileTime(zStr);
    }


class MockCore {
public:
    using Register = uint8_t;
    using RegisterType = uint16_t;
    static const Register NbPorts = 2;
    std::array<RegisterType, 7 * NbPorts> registers;
    RegisterType *gpOut, *gpIn, *gpDir, *gpOutSet, *gpOutClr, *gpDirSet, *gpDirClr;

    MockCore() : gpOut(&registers[0 * NbPorts]),
        gpIn(&registers[1 * NbPorts]),
        gpDir(&registers[2 * NbPorts]),
        gpOutSet(&registers[3 * NbPorts]),
        gpOutClr(&registers[4 * NbPorts]),
        gpDirSet(&registers[5 * NbPorts]),
        gpDirClr(&registers[6 * NbPorts]) {
    }

    MockCore(MockCore const&) = delete;
    MockCore(MockCore&&) = delete;
    MockCore& operator=(MockCore const&) = delete;
    MockCore& operator=(MockCore&&) = delete;

    void configure(uint8_t numberOfPorts) { };

    void yield() {
        consume1TRegisters();
        applyBitLogicOps();
        generateInterrupts();
    }

private:
    void consume1TRegisters() {
        for (auto portId = 0; portId < NbPorts; ++portId) {
            gpOut[portId] |= gpOutSet[portId];
            gpOutSet[portId] = 0;

            gpOut[portId] &= ~gpOutClr[portId];
            gpOutClr[portId] = 0;

            gpDir[portId] |= gpDirSet[portId];
            gpDirSet[portId] = 0;

            gpDir[portId] &= ~gpDirClr[portId];
            gpDirClr[portId] = 0;

        }
    }

    class BitLogic {
    public:
        BitLogic(MockCore& mock, Register inputReg, RegisterType inputMask, Register outputReg, RegisterType outputMask)
            : mock(mock), inputReg(inputReg), outputReg(outputReg), inputMask(inputMask), outputMask(outputMask) {}
    protected:
        MockCore& mock;
        Register inputReg, outputReg;
        RegisterType inputMask, outputMask;
    };

    /// Transmit status of masked input bits to masked output bits
    class BitLogicLink : public BitLogic {
    public:
        BitLogicLink(MockCore& mock, Register inputReg, RegisterType inputMask, Register outputReg, RegisterType outputMask)
            : BitLogic(mock, inputReg, inputMask, outputReg, outputMask) {
            std::cout << "BitLink " << inputReg << ", " << ", " << inputMask << ", " << outputReg << ", " << outputMask << "\n"; 
        }
        void apply() {
            std::cout << "reg[" << (int)inputReg << "] = " << mock.registers[inputReg] << " (" << inputMask << ")  ->  ";
            if ((mock.registers[inputReg] & inputMask) == inputMask) {
                
                mock.registers[outputReg] |= outputMask;
            }
            else {
                mock.registers[outputReg] &= ~outputMask;
            }
            std::cout << "reg[" << (int)outputReg << "] = " << mock.registers[outputReg] << "\n";
        }
    };

    void applyBitLogicOps() {
        for (auto& logicOp : bitLogicLinkOps) {
            logicOp.apply();
        }
    }

    void generateInterrupts() {
        
    }

    std::list<BitLogicLink> bitLogicLinkOps;

protected:
    int64_t pragma(std::string pragma) {
        using namespace std;
        istringstream ss(pragma);
        vector<string> tokens(istream_iterator<string>{ss}, istream_iterator<string>{});

        switch (etl::StringHash::calc(tokens[0].c_str())) {
        case "BitLink"_hash: {
            cout << "text pragma '" << tokens[1] << "', '" << tokens[2] << "', '" << tokens[3] << "', '" << tokens[4] << "'\n";
            Register i1 = stoi(tokens[1]), i3 = stoi(tokens[3]);
            RegisterType i2 = stoi(tokens[2]), i4 = stoi(tokens[4]);
            bitLogicLinkOps.emplace_back(*this, i1, i2, i3, i4);

            return 0;
        }

        case "TestSwitch"_hash:
            cout << "TestSwitch" << "\n";
            return 0;

        case "TestPragma"_hash:
            cout << "TestPragma" << "\n";
            return 0;
        }

        return -1;
    }
};

} // namespace etl
