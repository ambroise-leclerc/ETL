#pragma once
#include <cstdint>
#include <string>
#include <array>
#include <vector>
#include <numeric>


class MockCore {
public:
    using Register = uint8_t;
    using RegisterType = uint16_t;
    static const Register NbPorts = 2;
    std::array<RegisterType, 7 * NbPorts> registers;
    enum RegisterFamily : Register {
        GP_OUT = 0, GP_IN = NbPorts, GP_DIR = 2 * NbPorts, GP_OUT_SET = 3 * NbPorts,
        GP_OUT_CLR = 4 * NbPorts, GP_DIR_SET = 5 * NbPorts, GP_DIR_CLR = 6 * NbPorts
    };

    std::vector<Register> gpOut, gpIn, gpDir, gpOutSet, gpOutClr, gpDirSet, gpDirClr;

    MockCore() : gpOut(NbPorts), gpIn(NbPorts), gpDir(NbPorts), gpOutSet(NbPorts), gpOutClr(NbPorts), gpDirSet(NbPorts), gpDirClr(NbPorts) {
        std::iota(gpOut.begin(), gpOut.end(), static_cast<Register>(GP_OUT));
        std::iota(gpIn.begin(), gpIn.end(), static_cast<Register>(GP_IN));
        std::iota(gpDir.begin(), gpDir.end(), static_cast<Register>(GP_DIR));
        std::iota(gpOutSet.begin(), gpOutSet.end(), static_cast<Register>(GP_OUT_SET));
        std::iota(gpOutClr.begin(), gpOutClr.end(), static_cast<Register>(GP_OUT_CLR));
        std::iota(gpDirSet.begin(), gpDirSet.end(), static_cast<Register>(GP_DIR_SET));
        std::iota(gpDirClr.begin(), gpDirClr.end(), static_cast<Register>(GP_DIR_CLR));
        
    }

    MockCore(MockCore const&)            = delete;
    MockCore(MockCore&&)                 = delete;
    MockCore& operator=(MockCore const&) = delete;
    MockCore& operator=(MockCore&&)      = delete;

    void configure(uint8_t numberOfPorts) { };

    void yield() {
        for (auto reg : gpOutSet) {
            auto outReg = reg - (GP_OUT_SET - GP_OUT);
            registers[outReg] |= registers[reg];
            registers[reg] = 0;
        }

        for (auto reg : gpOutClr) {
            auto outReg = reg - (GP_OUT_CLR - GP_OUT);
            registers[outReg] &= ~registers[reg];
            registers[reg] = 0;
        }

        for (auto reg : gpDirSet) {
            auto dirReg = reg - (GP_DIR_SET - GP_DIR);
            registers[dirReg] |= registers[reg];
            registers[reg] = 0;
        }

        for (auto reg : gpDirClr) {
            auto dirReg = reg - (GP_DIR_CLR - GP_DIR);
            registers[dirReg] &= ~registers[reg];
            registers[reg] = 0;
        }
    }

};
