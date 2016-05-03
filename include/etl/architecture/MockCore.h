#pragma once
#include <cstdint>
#include <array>


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

    MockCore(MockCore const&)            = delete;
    MockCore(MockCore&&)                 = delete;
    MockCore& operator=(MockCore const&) = delete;
    MockCore& operator=(MockCore&&)      = delete;

    void configure(uint8_t numberOfPorts) { };

    void yield() {
        consume1TRegisters();
        applyBitLogic();
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

    void applyBitLogic() {
        
    }

    void generateInterrupts() {
        
    }

};
