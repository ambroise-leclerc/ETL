#pragma once
#include <cstdint>
#include <array>
#include <list>
#include <iostream>


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

        bitLogicLinkOps.emplace_back(*this, 0, 0b001, 2, 0b001);
        bitLogicLinkOps.emplace_back(*this, 0, 0b010, 2, 0b010);
        bitLogicLinkOps.emplace_back(*this, 0, 0b100, 2, 0b100);
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
            : BitLogic(mock, inputReg, inputMask, outputReg, outputMask) {}
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
};
