#include <catch.hpp>

#define ETLSTD etlstd
#include <etl/interrupt.h>

namespace {

using TimerOverflow = etl::InterruptVector<16>;
using PinChange = etl::InterruptVector<3>;

int overflows = 0;
void onOverflow() { ++overflows; }

} // namespace

SCENARIO("Interrupt handlers attached to vectors", "[interrupt]") {
    GIVEN("vectors with no handler") {
        etl::Interrupt<TimerOverflow>::detach();
        etl::Interrupt<PinChange>::detach();
        overflows = 0;

        THEN("raising them does nothing") {
            etl::Interrupt<TimerOverflow>::raise();
            REQUIRE(overflows == 0);
        }

        WHEN("a function is attached to one vector and a lambda to another") {
            static int changes = 0;
            changes = 0;
            etl::Interrupt<TimerOverflow>::attach<onOverflow>();
            etl::Interrupt<PinChange>::attach<[] { ++changes; }, etl::IsrKind::NonBlocking>();

            THEN("each vector runs its own handler") {
                etl::Interrupt<TimerOverflow>::raise();
                etl::Interrupt<TimerOverflow>::raise();
                etl::Interrupt<PinChange>::raise();
                REQUIRE(overflows == 2);
                REQUIRE(changes == 1);
            }

            AND_WHEN("the handler is detached") {
                etl::Interrupt<TimerOverflow>::detach();
                etl::Interrupt<TimerOverflow>::raise();
                THEN("it no longer runs") { REQUIRE(overflows == 0); }
            }
        }
    }

    GIVEN("a vector type") {
        THEN("it exposes its number at compile time") {
            static_assert(TimerOverflow::number == 16);
            REQUIRE(PinChange::number == 3);
        }
    }
}
