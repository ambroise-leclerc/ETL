#pragma once

#include <ETL/architecture/MockCore.h>

class MockDevice : public MockCore {
public:
    static MockDevice& getInstance() {
        static MockDevice instance;
        return instance;
    }

private:
    MockDevice() {}
};
