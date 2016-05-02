#pragma once

#include <ETL/architecture/MockCore.h>

class MockDevice : public MockCore {
public:
    static MockDevice& getInstance() {
        static MockDevice instance;
        return instance;
    }

    void yield() {
        MockCore::yield();

    }

    int64_t pragma(std::string pragma) {
    }

private:
    MockDevice() {}
};
