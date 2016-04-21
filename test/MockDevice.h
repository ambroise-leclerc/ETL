#pragma once
#include <cstdint>
#include <string>


class MockDevice {
public:
    using Register = uint8_t;

    static MockDevice& getInstance() {
        static MockDevice instance;
        return instance;
    }

    MockDevice(MockDevice const&) = delete;
    MockDevice(MockDevice&&) = delete;
    MockDevice& operator=(MockDevice const&) = delete;
    MockDevice& operator=(MockDevice &&) = delete;

    void configure(uint8_t portsNumber) { }

    void writeRegister(Register reg, uint32_t value) { internalRegister[reg] = value; }
    uint32_t readRegister(Register reg) { return internalRegister[reg]; }
    int64_t pragma(std::string pragma) { }

private:
    MockDevice() {
    }

private:
    uint32_t internalRegister[std::numeric_limits<Register>::max()];
};
