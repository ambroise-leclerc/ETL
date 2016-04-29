#pragma once
#include <cstdint>
#include <string>
#include <array>


class MockDevice {
public:
    using Register = uint8_t;
    using RegisterType = uint16_t;
    static const uint8_t NbPorts = 2;
    std::array<RegisterType, 7 * NbPorts> registers;

    static MockDevice& getInstance() {
        static MockDevice instance;
        return instance;
    }

    MockDevice(MockDevice const&) = delete;
    MockDevice(MockDevice&&) = delete;
    MockDevice& operator=(MockDevice const&) = delete;
    MockDevice& operator=(MockDevice&&) = delete;

    void configure(uint8_t numberOfPorts) { };
    void yield() {}
    int64_t pragma(std::string pragma) { }

private:
    MockDevice() {
    }
};
