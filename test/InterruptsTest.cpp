#include <catch.hpp>

#define __Mock_Mock__
#include <etl/ioports.h>


template <typename Strobe, typename Clk, typename Data>
class Client {
public:
    bool currentBit;

    void init() : currentBit(false) {
        
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


SCENARIO("Test leds") {
    using namespace etl;
    GIVEN("MCU with output pins 0, 1, 2 linked to input pins 3, 4, 5 with interrupt enables on change for pins 3, 4, 5") {
        using Strobe = Pin0;
        using Clk = Pin1;
        using Data = Pin2;
        using ClientStrobe = Pin3;
        using ClientClk = Pin4;
        using ClientData = Pin5;

        Client<ClientStrobe, ClientClk, ClientData> simu;

        Device::pragma(Pragma("BitLink").reg(Strobe::Port::GetOutputRegister()).bit(Strobe::bit())
                                        .reg(ClientStrobe::Port::GetInputRegister()).bit(ClientStrobe::bit()));
        Device::pragma(Pragma("BitLink").reg(Clk::Port::GetOutputRegister()).bit(Clk::bit())
                                        .reg(ClientClk::Port::GetInputRegister()).bit(ClientClk::bit()));
        Device::pragma(Pragma("BitLink").reg(Data::Port::GetOutputRegister()).bit(Data::bit())
                                        .reg(ClientData::Port::GetInputRegister()).bit(ClientData::bit()));

        ClientClk::setInterruptOnChange([simu]() { simu.clockChangedISR(); });
        ClientStrobe::setInterruptOnChange([simu]() { simu.strobeChangedISR(); });


        WHEN("Issueing clock signals") {
            simu.init();
            REQUIRE(simu.currentBit == false);

            Clk::set();
            Data::pulseHigh();
            THEN("client is notified of a new data") {
                REQUIRE(simu.currentBit == true);
            }
        }
    }
}
