#pragma once


#include <type_traits>
extern "C"
{
#include "ets_sys.h"
#include "osapi.h"
#include "uart_register.h"
#include "user_interface.h"
    void uart_div_modify(int no, unsigned int freq);
    void system_uart_swap(void);
    void system_uart_de_swap(void);
    void ets_isr_attach(int intr, void *handler, void *arg);
    void ets_isr_unmask(unsigned intr);
    void ets_isr_mask(unsigned intr);
}
namespace etl {


enum FrameFormat {
    _5N1 = 0b000000, _5N2 = 0b0000001, _5E1 = 0b000010, _5E2 = 0b000011, _5O1 = 0b000100, _5O2 = 0b000101,       // Encoding 5-N-1 : nbBits - no, even or odd parity - 1 or 2 stop bits : 0b000 - 00 - 0
    _6N1 = 0b001000, _6N2 = 0b0010001, _6E1 = 0b001010, _6E2 = 0b001011, _6O1 = 0b001100, _6O2 = 0b001101,
    _7N1 = 0b010000, _7N2 = 0b0100001, _7E1 = 0b010010, _7E2 = 0b010011, _7O1 = 0b010100, _7O2 = 0b010101,
    _8N1 = 0b011000, _8N2 = 0b0110001, _8E1 = 0b011010, _8E2 = 0b011011, _8O1 = 0b011100, _8O2 = 0b011101,
    _9N1 = 0b100000, _9N2 = 0b1000001, _9E1 = 0b100010, _9E2 = 0b100011, _9O1 = 0b100100, _9O2 = 0b100101
};

typedef void(*CharReceiver)(uint8_t);
    
    
void intr_handler(CharReceiver func)
{
    uint8_t rcvChar;
    if (READ_PERI_REG(UART_INT_ST(0))&UART_RXFIFO_FULL_INT_ST == UART_RXFIFO_FULL_INT_ST)
    {
        ETS_UART_INTR_DISABLE();
        while (READ_PERI_REG(UART_STATUS(0)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S)) {
            func( READ_PERI_REG(UART_FIFO(0)) & 0xFF);
        }
         //clear all interrupt
        WRITE_PERI_REG(UART_INT_CLR(0), UART_RXFIFO_FULL_INT_CLR);
        ETS_UART_INTR_ENABLE();
    }
}
    
void intr_handler1(CharReceiver func)
{
    uint8_t rcvChar;
    if (READ_PERI_REG(UART_INT_ST(1))&UART_RXFIFO_FULL_INT_ST == UART_RXFIFO_FULL_INT_ST)
    {
        ETS_UART_INTR_DISABLE();
        while (READ_PERI_REG(UART_STATUS(1)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S)) {
            func(READ_PERI_REG(UART_FIFO(1)) & 0xFF);
        }
            //clear all interrupt
        WRITE_PERI_REG(UART_INT_CLR(1), UART_RXFIFO_FULL_INT_CLR);
        ETS_UART_INTR_ENABLE();
    }
}

template<uint32_t BAUD_RATE = 9600, FrameFormat FRAME_FORMAT = FrameFormat::_8N1, typename SizeUint = uint8_t>
class Uart0 {
public:
    static void start() {
        system_uart_de_swap();
        system_set_os_print(0);
        PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
    
        PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0RXD_U);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, 0);
        
       uart_div_modify(0, UART_CLK_FREQ / BAUD_RATE);

       WRITE_PERI_REG(UART_CONF0(0), existParity
            | parityEsp
            | (stopBit << UART_STOP_BIT_NUM_S)
            | (bitNumber << UART_BIT_NUM_S));

        //clear rx and tx fifo,not ready
        SET_PERI_REG_MASK(UART_CONF0(0), UART_RXFIFO_RST | UART_TXFIFO_RST);
        CLEAR_PERI_REG_MASK(UART_CONF0(0), UART_RXFIFO_RST | UART_TXFIFO_RST);
       
    }
    
    static void start(CharReceiver func) {
        start();
        ETS_UART_INTR_ATTACH((void *)intr_handler, (void *)func);
        WRITE_PERI_REG(UART_CONF1(0), (0x01 & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S);

        //clear all interrupt
        WRITE_PERI_REG(UART_INT_CLR(0), 0xffff);
        //enable rx_interrupt
        SET_PERI_REG_MASK(UART_INT_ENA(0), UART_RXFIFO_FULL_INT_ENA);
        ETS_UART_INTR_ENABLE();
     
    }

    static void write(SizeUint datum) {
        bool tx_fifo_len = (READ_PERI_REG(UART_STATUS(0)) >> UART_TXFIFO_CNT_S)&UART_TXFIFO_CNT >= MAX_FIFO_LENGHT;
        while (tx_fifo_len)
        {
            tx_fifo_len = (READ_PERI_REG(UART_STATUS(0)) >> UART_TXFIFO_CNT_S)&UART_TXFIFO_CNT >= MAX_FIFO_LENGHT;
        }
        WRITE_PERI_REG(UART_FIFO(0), datum);
    }


    static SizeUint read() {
        bool rx_fifo_len = READ_PERI_REG(UART_STATUS(0)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S) > 0;
        while (!rx_fifo_len)
        {
            rx_fifo_len = READ_PERI_REG(UART_STATUS(0)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S)  > 0;
        }
        return READ_PERI_REG(UART_FIFO(0)) & 0xFF;
    }

private:

    enum Parity { None, Even, Odd };
    enum StopBit { One, Two };
    enum BitNumber { Five , Six, Seven , Eight , Nine };

    static const auto bitNumberMask = 0b111000;
    static const auto stopBitMask = 0b000001;
    static const auto parityMask = 0b000110;
    
    static const auto MAX_FIFO_LENGHT = 126;

    static constexpr  auto parity = static_cast<Parity>((FRAME_FORMAT & parityMask) >> 1);
    static constexpr  auto stopBit = static_cast<StopBit>(FRAME_FORMAT & stopBitMask);
    static constexpr  auto bitNumber = static_cast<BitNumber>((FRAME_FORMAT & bitNumberMask) >> 3) ;

    static constexpr  auto existParity = (parity == None) ? 0 : BIT3 | BIT5;
    static constexpr  auto  parityEsp = (parity == None | parity == Odd) ? 0 : BIT4;
    
    
};
    
    
template<uint32_t BAUD_RATE = 9600, FrameFormat FRAME_FORMAT = FrameFormat::_8N1, typename SizeUint = uint8_t>
class Uart1 {
public:
    static void start() {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_U1TXD_BK);
        //PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA1_U, FUNC_U1RXD);
        
        uart_div_modify(1, UART_CLK_FREQ / BAUD_RATE);

        WRITE_PERI_REG(UART_CONF0(1),
            existParity
                | parityEsp
                | (stopBit << UART_STOP_BIT_NUM_S)
                | (bitNumber << UART_BIT_NUM_S));

                        //clear rx and tx fifo,not ready
        SET_PERI_REG_MASK(UART_CONF0(1),  UART_TXFIFO_RST);
        CLEAR_PERI_REG_MASK(UART_CONF0(1), UART_TXFIFO_RST);
    }
    
  /*  static void start(CharReceiver func) {
        start();
        ETS_UART_INTR_ATTACH((void *)intr_handler1, (void *)func);
        WRITE_PERI_REG(UART_CONF1(1), (0x01 & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S);

                //clear all interrupt
        WRITE_PERI_REG(UART_INT_CLR(1), 0xffff);
        //enable rx_interrupt
        SET_PERI_REG_MASK(UART_INT_ENA(1), UART_RXFIFO_FULL_INT_ENA);
        ETS_UART_INTR_ENABLE();
     
    }*/
   
    static void write(SizeUint datum) {
        bool tx_fifo_len = (READ_PERI_REG(UART_STATUS(1)) >> UART_TXFIFO_CNT_S)&UART_TXFIFO_CNT >= MAX_FIFO_LENGHT;
        while (tx_fifo_len)
        {
            tx_fifo_len = (READ_PERI_REG(UART_STATUS(1)) >> UART_TXFIFO_CNT_S)&UART_TXFIFO_CNT >= MAX_FIFO_LENGHT;
        }
        WRITE_PERI_REG(UART_FIFO(1), datum);
    }
    
   /* static SizeUint read() {
        bool rx_fifo_len = READ_PERI_REG(UART_STATUS(1)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S) > 0;
        while (!rx_fifo_len)
        {
            rx_fifo_len = READ_PERI_REG(UART_STATUS(1)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S)  > 0;
        }
        return READ_PERI_REG(UART_FIFO(1)) & 0xFF;
    }*/

private:

    enum Parity { None, Even, Odd };
    enum StopBit { One, Two };
    enum BitNumber { Five, Six, Seven, Eight, Nine };

    static const auto bitNumberMask = 0b111000;
    static const auto stopBitMask = 0b000001;
    static const auto parityMask = 0b000110;
    
    static const auto MAX_FIFO_LENGHT = 126;

    static constexpr  auto parity = static_cast<Parity>((FRAME_FORMAT & parityMask) >> 1);
    static constexpr  auto stopBit = static_cast<StopBit>(FRAME_FORMAT & stopBitMask);
    static constexpr  auto bitNumber = static_cast<BitNumber>((FRAME_FORMAT & bitNumberMask) >> 3);

    static constexpr  auto existParity = (parity == None) ? 0 : BIT3 | BIT5;
    static constexpr  auto  parityEsp = (parity == None | parity == Odd) ? 0 : BIT4;
    
    
};

    
template<uint32_t BAUD_RATE = 9600, FrameFormat FRAME_FORMAT = FrameFormat::_8N1, typename SizeUint = uint8_t>
class Uart2 {
public:
    static void start() {
        system_uart_swap();
        system_set_os_print(0);
        PIN_PULLUP_DIS(PERIPHS_IO_MUX_MTDO_U);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_U0RTS);
        
        PIN_PULLUP_DIS(PERIPHS_IO_MUX_MTCK_U);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, 4);
        
        uart_div_modify(0, UART_CLK_FREQ / BAUD_RATE);

        WRITE_PERI_REG(UART_CONF0(0),
            existParity
                | parityEsp
                | (stopBit << UART_STOP_BIT_NUM_S)
                | (bitNumber << UART_BIT_NUM_S));

                        //clear rx and tx fifo,not ready
        SET_PERI_REG_MASK(UART_CONF0(0), UART_RXFIFO_RST | UART_TXFIFO_RST);
       // SET_PERI_REG_MASK(UART_CONF0(0), UART_TX_FLOW_EN); 
        CLEAR_PERI_REG_MASK(UART_CONF0(0), UART_RXFIFO_RST | UART_TXFIFO_RST);
       
    }
    
    static void start(CharReceiver func) {
        start();
        ETS_UART_INTR_ATTACH((void *)intr_handler, (void *)func);
        WRITE_PERI_REG(UART_CONF1(0), (0x01 & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S);

        //clear all interrupt
        WRITE_PERI_REG(UART_INT_CLR(0), 0xffff);
        //enable rx_interrupt
        SET_PERI_REG_MASK(UART_INT_ENA(0), UART_RXFIFO_FULL_INT_ENA);
        ETS_UART_INTR_ENABLE();
     
    }

    static void write(SizeUint datum) {
        bool tx_fifo_len = (READ_PERI_REG(UART_STATUS(0)) >> UART_TXFIFO_CNT_S)&UART_TXFIFO_CNT >= MAX_FIFO_LENGHT;
        while (tx_fifo_len)
        {
            tx_fifo_len = (READ_PERI_REG(UART_STATUS(0)) >> UART_TXFIFO_CNT_S)&UART_TXFIFO_CNT >= MAX_FIFO_LENGHT;
        }
        WRITE_PERI_REG(UART_FIFO(0), datum);
    }


    static SizeUint read() {
        bool rx_fifo_len = READ_PERI_REG(UART_STATUS(0)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S) > 0;
        while (!rx_fifo_len)
        {
            rx_fifo_len = READ_PERI_REG(UART_STATUS(0)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S)  > 0;
        }
        return READ_PERI_REG(UART_FIFO(0)) & 0xFF;
    }


private:

    enum Parity { None, Even, Odd };
    enum StopBit { One, Two };
    enum BitNumber { Five, Six, Seven, Eight, Nine };

    static const auto bitNumberMask = 0b111000;
    static const auto stopBitMask = 0b000001;
    static const auto parityMask = 0b000110;
    
    static const auto MAX_FIFO_LENGHT = 126;

    static constexpr  auto parity = static_cast<Parity>((FRAME_FORMAT & parityMask) >> 1);
    static constexpr  auto stopBit = static_cast<StopBit>(FRAME_FORMAT & stopBitMask);
    static constexpr  auto bitNumber = static_cast<BitNumber>((FRAME_FORMAT & bitNumberMask) >> 3);

    static constexpr  auto existParity = (parity == None) ? 0 : BIT3 | BIT5;
    static constexpr  auto  parityEsp = (parity == None | parity == Odd) ? 0 : BIT4;
    
    
};

}