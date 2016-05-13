#include <catch.hpp>

#define __Mock_Mock__
#include <etl/ioports.h>

#include <iostream>

static std::string testString;

struct TestInterrupt {

    static void work()
    {
        testString.append("I");
       std::cout << testString << '\n';
    }
};

SCENARIO("TEST InterruptManager") {
    using namespace etl;

    std::cout << "TEST InterruptManager" << '\n';
    std::function<void(void)> myFunction = TestInterrupt::work;

    PortSimuA::onChange(myFunction, 0b00000001);
    std::cout << "1" << '\n';
    Device::yield();
    REQUIRE(testString.length() == 0);
    GPSimuA_IN = 0b00000001;
    std::cout << "2" << '\n';
    Device::yield();
    REQUIRE(testString.length() == 1);
    GPSimuA_IN = 0b00000000;
    std::cout << "3" << '\n';
    Device::yield();
    REQUIRE(testString.length() == 2);
    std::cout << "4" << '\n';
    Device::yield();
    REQUIRE(testString.length() == 2);
}


template <typename Strobe, typename Clk, typename Data>
class Client {
public:
    bool currentBit;

    void init() {
        currentBit = false;
    }

    void strobeChangedISR() {
        if (Strobe::test()) {
            std::cout << "Strobe interrupt received : RISING EDGE\n";
        }
        else {
            std::cout << "Strobe interrupt received : FALLING EDGE\n";
        }
    }

    void clockChangedISR() {
        if (Clk::test()) {
            currentBit = Data::test();
            std::cout << "Clock interrupt received : RISING EDGE\n";
        }
        else {
            std::cout << "Clock interrupt received : FALLING EDGE\n";
        }
    }
};

SCENARIO("Test dual interruptions on strobe and clock") {
    using namespace etl;
    GIVEN("MCU with output pins 0, 1, 2 linked to input pins 3, 4, 5 with interrupt enables on change for pins 3, 4, 5") {
        using Strobe = Pin0;
        using Clk = Pin1;
        using Data = Pin2;
        using ClientStrobe = PinS0;
        using ClientClk = PinS1;
        using ClientData = PinS2;

        Client<ClientStrobe, ClientClk, ClientData> simu;

        Device::pragma(Pragma("BitLink").reg(Strobe::Port::GetOutputRegister()).bit(Strobe::bit())
                                        .reg(ClientStrobe::Port::GetInputRegister()).bit(ClientStrobe::bit()));
        Device::pragma(Pragma("BitLink").reg(Clk::Port::GetOutputRegister()).bit(Clk::bit())
                                        .reg(ClientClk::Port::GetInputRegister()).bit(ClientClk::bit()));
        Device::pragma(Pragma("BitLink").reg(Data::Port::GetOutputRegister()).bit(Data::bit())
                                        .reg(ClientData::Port::GetInputRegister()).bit(ClientData::bit()));

        ClientClk::onChange([&simu]() -> void { simu.clockChangedISR(); });
        ClientStrobe::onChange([&simu]()-> void { simu.strobeChangedISR(); });  


        WHEN("Issueing clock signals") {
            simu.init();
            REQUIRE(simu.currentBit == false);

            Data::set();
            Clk::pulseHigh();
            THEN("client is notified of a new data") {
                REQUIRE(simu.currentBit == true);
            }

            Strobe::pulseHigh();

        }
    }
}

SCENARIO("InterruptsDispatcher test") {
    using namespace etl;

    GIVEN("an interrupt dispatcher") {
        InterruptsDispatcher<uint8_t, uint8_t> dispatcher;

        std::string witness;
        enum registers { A, B, C };
        REQUIRE(witness == "");

        dispatcher.addCallback([&witness]() { witness += "A0"; }, A, 1 << 0);
        dispatcher.addCallback([&witness]() { witness += "A1"; }, A, 1 << 1);
        dispatcher.addCallback([&witness]() { witness += "A2"; }, A, 1 << 2);
        dispatcher.addCallback([&witness]() { witness += "B0"; }, B, 1 << 0);

        WHEN("callbacks are set") {
            THEN("pin changes trigger callbacks") {
                dispatcher.signalPortsPinsChange(A, 0b11111111);
                REQUIRE(witness == "A0A1A2");

                dispatcher.signalPortsPinsChange(A, 0b11111000);
                REQUIRE(witness == "A0A1A2");

                dispatcher.signalPortsPinsChange(A, 0b11111100);
                REQUIRE(witness == "A0A1A2A2");

                dispatcher.signalPortsPinsChange(B, 0b11111111);
                REQUIRE(witness == "A0A1A2A2B0");

                dispatcher.signalPortsPinsChange(C, 0b11111111);
                REQUIRE(witness == "A0A1A2A2B0");
            }
        }

        WHEN("multiple callbacks are set") {
            dispatcher.addCallback([&witness]() { witness += "A*"; }, A, 0b11111111);
            THEN("multiple calls are made on same pins") {
                dispatcher.signalPortsPinsChange(A, 0b11111111);
                REQUIRE(witness == "A0A1A2A*");
            }
        }

        WHEN("callbacks are cleared") {
            dispatcher.removeCallback(A, 1 << 1);
            dispatcher.removeCallback(B, 1 << 0);
            THEN("pin changes trigger callbacks except for removed ones") {
                dispatcher.signalPortsPinsChange(A, 0b11111111);
                REQUIRE(witness == "A0A2");

                dispatcher.signalPortsPinsChange(B, 0b11111111);
                REQUIRE(witness == "A0A2");
            }
        }
    }
}