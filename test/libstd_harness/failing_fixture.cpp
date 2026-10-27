// Intentional failures: CTest runs this executable with WILL_FAIL to check that failures reach the exit status.
#include "harness.h"

TEST_CASE("intentional failures") {
    CHECK(1 + 1 == 3);
    CHECK_EQ(2 + 2, 5);
}
