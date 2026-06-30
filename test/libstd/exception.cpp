#include <catch.hpp>
#include <cstring>

#define __Mock_Mock__
#define ETLSTD etlstd

#include <libstd/include/stdexcept>

using namespace ETLSTD;

SCENARIO("std::exception diagnostics", "[libstd][exception]") {
    exception base;
    REQUIRE(base.what() != nullptr);
    REQUIRE(base.what()[0] == '\0');

    bad_exception bad;
    REQUIRE(bad.what() != nullptr);

    logic_error logic("logic");
    REQUIRE(std::strcmp(logic.what(), "logic") == 0);

    out_of_range range("range");
    REQUIRE(std::strcmp(range.what(), "range") == 0);

    runtime_error runtime("runtime");
    REQUIRE(std::strcmp(runtime.what(), "runtime") == 0);
}
