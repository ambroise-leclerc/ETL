#include <catch.hpp>

#define __Mock_Mock__
#include <etl/ioports.h>


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

static std::string testString;


struct TestInterrupt {
   
    static void work()
    {
        testString.append("I");
       std::cout << testString << '\n';
    }
};


SCENARIO("TEST InterruptManager") {
    std::cout << "TEST InterruptManager" << '\n';
    using Register = uint8_t;
    using RegisterType = uint16_t;
    std::array<uint8_t, 4> fakeRegister;
    std::function<void(void)> myFunction = TestInterrupt::work;

    etl::InterruptsManager<Register, RegisterType, decltype(fakeRegister)> myManager(fakeRegister);
    fakeRegister[0] = 0;
    myManager.setInterrupt(myFunction, fakeRegister[0], (uint8_t)0b00000001);
    myManager.enable();
    std::cout << "1" << '\n';
    myManager.generateInterrupts();
    REQUIRE(testString.length() == 0);
    fakeRegister[0] = 0b00000001;
    std::cout << "2" << '\n';
    myManager.generateInterrupts();
    REQUIRE(testString.length() == 1);
    fakeRegister[0] = 0b00000000;
    std::cout << "3" << '\n';
    myManager.generateInterrupts();
    REQUIRE(testString.length() == 2);
    std::cout << "4" << '\n';
    myManager.generateInterrupts();
    REQUIRE(testString.length() == 2);
}


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

        ClientClk::setInterruptOnChange([&]() -> void { simu.clockChangedISR(); });
      //  ClientStrobe::setInterruptOnChange([simu]()-> void { simu.strobeChangedISR(); });


        WHEN("Issueing clock signals") {
            simu.init();
            REQUIRE(simu.currentBit == false);

            Data::set();
            Clk::pulseHigh();
            THEN("client is notified of a new data") {
                REQUIRE(simu.currentBit == true);
            }
        }
    }
}
