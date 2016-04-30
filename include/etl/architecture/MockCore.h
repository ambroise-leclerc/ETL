#pragma once
#include <cstdint>
#include <string>
#include <array>


class MockCore {
public:
    using Register = uint8_t;
    using RegisterType = uint16_t;
    static const uint8_t NbPorts = 2;
    std::array<RegisterType, 7 * NbPorts> registers;

    MockCore()                           = default;
    MockCore(MockCore const&)            = delete;
    MockCore(MockCore&&)                 = delete;
    MockCore& operator=(MockCore const&) = delete;
    MockCore& operator=(MockCore&&)      = delete;

    void configure(uint8_t numberOfPorts) { };
    void yield() {}

};
