#pragma once

#include <ETL/architecture/MockCore.h>
#include <vector>
#include <sstream>
#include <cstdint>
#include <iostream>
#include <assert.h>

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
        using namespace etl;
        istringstream ss(pragma);
        vector<string> tokens(istream_iterator<string>{ss}, istream_iterator<string>{});

        switch (etl::StringHash::calc(pragma.c_str())) {
        case "BitLink"_hash:
            std::cout << "BitLink" << "\n";

            return 0;

        case "TestSwitch"_hash:
            std::cout << "TestSwitch" << "\n";
            return 0;

        case "TestPragma"_hash:
            std::cout << "TestPragma" << "\n";
            return 0;
        }
    }

private:
    MockDevice() {}
};

} // namespace etl