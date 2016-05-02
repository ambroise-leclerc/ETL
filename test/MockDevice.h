#pragma once

#include <ETL/architecture/MockCore.h>
#include <vector>
#include <sstream>
#include <cstdint>

namespace etl {

class StringHash {    
public:
    static constexpr std::uint64_t compileTime(char const* zStr, std::uint64_t lastValue = basis) {
        return *zStr ? compileTime(zStr + 1, (*zStr ^ lastValue) * prime) : lastValue;
    }

    std::uint64_t stringHash(char const* zStr) {
        std::uint64_t ret{ basis };
        while (*zStr) {
            ret ^= *zStr;
            ret *= prime;
            zStr++;
        }
        return ret;
    }
private:
    static const auto basis = 14'695'981'039'346'656'037ull;
    static const auto prime = 1'099'511'628'211ull;
};

constexpr std::uint64_t operator""_hash(char const* zStr, size_t) {
    return etl::StringHash::compileTime(zStr);
}


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
        using namespace std;
        istringstream ss(pragma);
        vector<string> tokens(istream_iterator<string>{ss}, istream_iterator<string>{});
    }

private:
    MockDevice() {}
};

} // namespace etl