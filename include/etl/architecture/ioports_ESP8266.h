/// @file ioports_ESP8266.h
/// @date 30/08/17 17:34
/// @author Ambroise Leclerc and Cecile Thiebaut
/// @brief Espressif ESP 32-bit microcontrollers peripherals handling classes
//
// Copyright (c) 2017, Ambroise Leclerc and Cecile Thiebaut
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in
//     the documentation and/or other materials provided with the
//     distribution.
//   * Neither the name of the copyright holders nor the names of
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS' 
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <libstd/include/cstdint>
#include "ETLDevice_ESP8266.h"

extern "C" {
    #include "eagle_soc.h"
    #include "user_interface.h"
    #include "osapi.h"
int atoi(const char *nptr);
void ets_install_putc1(void *routine);
void ets_isr_attach(int intr, void *handler, void *arg);
void ets_isr_mask(unsigned intr);
void ets_isr_unmask(unsigned intr);
void ets_delay_us(unsigned delay);	//SAM
int ets_memcmp(const void *s1, const void *s2, size_t n);
void *ets_memcpy(void *dest, const void *src, size_t n);
void *ets_memset(void *s, int c, size_t n);
int ets_sprintf(char *str, const char *format, ...)  __attribute__ ((format (printf, 2, 3)));
int ets_str2macaddr(void *, void *);
int ets_strcmp(const char *s1, const char *s2);
char *ets_strcpy(char *dest, const char *src);
size_t ets_strlen(const char *s);
int ets_strncmp(const char *s1, const char *s2, int len);
char *ets_strncpy(char *dest, const char *src, size_t n);
char *ets_strstr(const char *haystack, const char *needle);
void ets_timer_arm_new(ETSTimer *a, int b, int c, int isMstimer);
void ets_timer_disarm(ETSTimer *a);
void ets_timer_setfn(ETSTimer *t, ETSTimerFunc *fn, void *parg);
int os_printf(const char *format, ...)  __attribute__ ((format (printf, 1, 2)));
int os_snprintf(char *str, size_t size, const char *format, ...) __attribute__ ((format (printf, 3, 4)));
void pvPortFree(void *ptr);
void *pvPortMalloc(size_t xWantedSize);
void *pvPortZalloc(size_t);
void uart_div_modify(int no, unsigned int freq);
void vPortFree(void *ptr);
void *vPortMalloc(size_t xWantedSize);
int os_printf_plus(const char *format, ...)  __attribute__ ((format (printf, 1, 2)));
}
static const uint32_t BASE_ADRR = 0x60000000;

static uint32_t* gpcAdress[16] = {
    (uint32_t *)(BASE_ADRR + 0x328), (uint32_t *)(BASE_ADRR + 0x32C), (uint32_t *)(BASE_ADRR + 0x330), (uint32_t *)(BASE_ADRR + 0x334),
    (uint32_t *)(BASE_ADRR + 0x338), (uint32_t *)(BASE_ADRR + 0x33C), (uint32_t *)(BASE_ADRR + 0x340), (uint32_t *)(BASE_ADRR + 0x344 ),
    (uint32_t *)(BASE_ADRR + 0x348), (uint32_t *)(BASE_ADRR +0x34C ), (uint32_t *)(BASE_ADRR + 0x350), (uint32_t *) (BASE_ADRR +0x354 ),
    (uint32_t *)(BASE_ADRR +0x358 ), (uint32_t *)(BASE_ADRR + 0x35C), (uint32_t *)(BASE_ADRR + 0x360), (uint32_t *)(BASE_ADRR +0x364 )        };

static uint32_t periphs[16] = {
    PERIPHS_IO_MUX_GPIO0_U, PERIPHS_IO_MUX_U0TXD_U, PERIPHS_IO_MUX_GPIO2_U, PERIPHS_IO_MUX_U0RXD_U,
    PERIPHS_IO_MUX_GPIO4_U, PERIPHS_IO_MUX_GPIO5_U, PERIPHS_IO_MUX_SD_CLK_U, PERIPHS_IO_MUX_SD_DATA0_U,
    PERIPHS_IO_MUX_SD_DATA1_U, PERIPHS_IO_MUX_SD_DATA2_U, PERIPHS_IO_MUX_SD_DATA3_U, PERIPHS_IO_MUX_SD_CMD_U,
    PERIPHS_IO_MUX_MTDI_U, PERIPHS_IO_MUX_MTCK_U, PERIPHS_IO_MUX_MTMS_U, PERIPHS_IO_MUX_MTDO_U       };

static uint8_t functionsGpio[16] = {
    FUNC_GPIO0, FUNC_GPIO1, FUNC_GPIO2, FUNC_GPIO3, FUNC_GPIO4, FUNC_GPIO5, 3, 3,
    3, FUNC_GPIO9, FUNC_GPIO10, 3, FUNC_GPIO12, FUNC_GPIO13, FUNC_GPIO14, FUNC_GPIO15 };

namespace etl {
class Device {
public:
 static ICACHE_FLASH_ATTR void delayTicks(uint32_t ticks) {
            os_delay_us(((ticks * 1000000) / McuFrequency));
        }
 static const uint32_t McuFrequency = SYS_CPU_80MHZ*1000000;
  static const auto architectureWidth = 32;
};

using clock_cycles = std::chrono::duration<unsigned long, std::ratio<1, Device::McuFrequency>>;
constexpr clock_cycles operator ""clks(unsigned long long c)     { return clock_cycles(static_cast<clock_cycles::rep>(c)); }

class Pin0 {
public:
    static void set()       { GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, (1 << 0)); }
    static void set(bool v) { v ? set() : clear(); }
    static void clear()     { GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << 0)); }
    static void toggle()    { (GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << 0)) == 0 ? set() : clear(); }
    static void setOutput() { PIN_FUNC_SELECT(periphs[0], functionsGpio[0]);
                              PIN_PULLUP_DIS(periphs[0]);
                              *gpcAdress[ 0] &= (*gpcAdress[0] & (0xF << 7));
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << 0);
    }
    static void setInput()  { PIN_FUNC_SELECT(periphs[0], functionsGpio[0]);
                              PIN_PULLUP_DIS(periphs[0]);
                              *gpcAdress[0] &= (*gpcAdress[0] & (0xF << 7)) | (1 << 0);
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << 0);
                              GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << 0);
    }
    static uint16_t read() { return (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << 0)) != 0; }
    static void pulseHigh(){ set(); clear(); }
    static void pulseLow() { clear(); set(); }
    static constexpr uint16_t bitmask() { return 1 << 0; }
};

class Pin1 {
public:
    static void set()       { GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, (1 << 1)); }
    static void set(bool v) { v ? set() : clear(); }
    static void clear()     { GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << 1)); }
    static void toggle()    { (GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << 1)) == 0 ? set() : clear(); }
    static void setOutput() { PIN_FUNC_SELECT(periphs[1], functionsGpio[1]);
                              PIN_PULLUP_DIS(periphs[1]);
                              *gpcAdress[ 1] &= (*gpcAdress[1] & (0xF << 7));
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << 1);
    }
    static void setInput()  { PIN_FUNC_SELECT(periphs[1], functionsGpio[1]);
                              PIN_PULLUP_DIS(periphs[1]);
                              *gpcAdress[1] &= (*gpcAdress[1] & (0xF << 7)) | (1 << 1);
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << 1);
                              GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << 1);
    }
    static uint16_t read() { return (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << 1)) != 0; }
    static void pulseHigh(){ set(); clear(); }
    static void pulseLow() { clear(); set(); }
    static constexpr uint16_t bitmask() { return 1 << 1; }
};

class Pin2 {
public:
    static void set()       { GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, (1 << 2)); }
    static void set(bool v) { v ? set() : clear(); }
    static void clear()     { GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << 2)); }
    static void toggle()    { (GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << 2)) == 0 ? set() : clear(); }
    static void setOutput() { PIN_FUNC_SELECT(periphs[2], functionsGpio[2]);
                              PIN_PULLUP_DIS(periphs[2]);
                              *gpcAdress[ 2] &= (*gpcAdress[2] & (0xF << 7));
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << 2);
    }
    static void setInput()  { PIN_FUNC_SELECT(periphs[2], functionsGpio[2]);
                              PIN_PULLUP_DIS(periphs[2]);
                              *gpcAdress[2] &= (*gpcAdress[2] & (0xF << 7)) | (1 << 2);
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << 2);
                              GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << 2);
    }
    static uint16_t read() { return (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << 2)) != 0; }
    static void pulseHigh(){ set(); clear(); }
    static void pulseLow() { clear(); set(); }
    static constexpr uint16_t bitmask() { return 1 << 2; }
};

class Pin3 {
public:
    static void set()       { GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, (1 << 3)); }
    static void set(bool v) { v ? set() : clear(); }
    static void clear()     { GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << 3)); }
    static void toggle()    { (GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << 3)) == 0 ? set() : clear(); }
    static void setOutput() { PIN_FUNC_SELECT(periphs[3], functionsGpio[3]);
                              PIN_PULLUP_DIS(periphs[3]);
                              *gpcAdress[ 3] &= (*gpcAdress[3] & (0xF << 7));
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << 3);
    }
    static void setInput()  { PIN_FUNC_SELECT(periphs[3], functionsGpio[3]);
                              PIN_PULLUP_DIS(periphs[3]);
                              *gpcAdress[3] &= (*gpcAdress[3] & (0xF << 7)) | (1 << 3);
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << 3);
                              GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << 3);
    }
    static uint16_t read() { return (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << 3)) != 0; }
    static void pulseHigh(){ set(); clear(); }
    static void pulseLow() { clear(); set(); }
    static constexpr uint16_t bitmask() { return 1 << 3; }
};

class Pin4 {
public:
    static void set()       { GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, (1 << 4)); }
    static void set(bool v) { v ? set() : clear(); }
    static void clear()     { GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << 4)); }
    static void toggle()    { (GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << 4)) == 0 ? set() : clear(); }
    static void setOutput() { PIN_FUNC_SELECT(periphs[4], functionsGpio[4]);
                              PIN_PULLUP_DIS(periphs[4]);
                              *gpcAdress[ 4] &= (*gpcAdress[4] & (0xF << 7));
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << 4);
    }
    static void setInput()  { PIN_FUNC_SELECT(periphs[4], functionsGpio[4]);
                              PIN_PULLUP_DIS(periphs[4]);
                              *gpcAdress[4] &= (*gpcAdress[4] & (0xF << 7)) | (1 << 4);
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << 4);
                              GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << 4);
    }
    static uint16_t read() { return (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << 4)) != 0; }
    static void pulseHigh(){ set(); clear(); }
    static void pulseLow() { clear(); set(); }
    static constexpr uint16_t bitmask() { return 1 << 4; }
};

class Pin5 {
public:
    static void set()       { GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, (1 << 5)); }
    static void set(bool v) { v ? set() : clear(); }
    static void clear()     { GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << 5)); }
    static void toggle()    { (GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << 5)) == 0 ? set() : clear(); }
    static void setOutput() { PIN_FUNC_SELECT(periphs[5], functionsGpio[5]);
                              PIN_PULLUP_DIS(periphs[5]);
                              *gpcAdress[ 5] &= (*gpcAdress[5] & (0xF << 7));
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << 5);
    }
    static void setInput()  { PIN_FUNC_SELECT(periphs[5], functionsGpio[5]);
                              PIN_PULLUP_DIS(periphs[5]);
                              *gpcAdress[5] &= (*gpcAdress[5] & (0xF << 7)) | (1 << 5);
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << 5);
                              GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << 5);
    }
    static uint16_t read() { return (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << 5)) != 0; }
    static void pulseHigh(){ set(); clear(); }
    static void pulseLow() { clear(); set(); }
    static constexpr uint16_t bitmask() { return 1 << 5; }
};

class Pin6 {
public:
    static void set()       { GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, (1 << 6)); }
    static void set(bool v) { v ? set() : clear(); }
    static void clear()     { GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << 6)); }
    static void toggle()    { (GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << 6)) == 0 ? set() : clear(); }
    static void setOutput() { PIN_FUNC_SELECT(periphs[6], functionsGpio[6]);
                              PIN_PULLUP_DIS(periphs[6]);
                              *gpcAdress[ 6] &= (*gpcAdress[6] & (0xF << 7));
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << 6);
    }
    static void setInput()  { PIN_FUNC_SELECT(periphs[6], functionsGpio[6]);
                              PIN_PULLUP_DIS(periphs[6]);
                              *gpcAdress[6] &= (*gpcAdress[6] & (0xF << 7)) | (1 << 6);
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << 6);
                              GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << 6);
    }
    static uint16_t read() { return (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << 6)) != 0; }
    static void pulseHigh(){ set(); clear(); }
    static void pulseLow() { clear(); set(); }
    static constexpr uint16_t bitmask() { return 1 << 6; }
};

class Pin7 {
public:
    static void set()       { GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, (1 << 7)); }
    static void set(bool v) { v ? set() : clear(); }
    static void clear()     { GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << 7)); }
    static void toggle()    { (GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << 7)) == 0 ? set() : clear(); }
    static void setOutput() { PIN_FUNC_SELECT(periphs[7], functionsGpio[7]);
                              PIN_PULLUP_DIS(periphs[7]);
                              *gpcAdress[ 7] &= (*gpcAdress[7] & (0xF << 7));
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << 7);
    }
    static void setInput()  { PIN_FUNC_SELECT(periphs[7], functionsGpio[7]);
                              PIN_PULLUP_DIS(periphs[7]);
                              *gpcAdress[7] &= (*gpcAdress[7] & (0xF << 7)) | (1 << 7);
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << 7);
                              GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << 7);
    }
    static uint16_t read() { return (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << 7)) != 0; }
    static void pulseHigh(){ set(); clear(); }
    static void pulseLow() { clear(); set(); }
    static constexpr uint16_t bitmask() { return 1 << 7; }
};

class Pin8 {
public:
    static void set()       { GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, (1 << 8)); }
    static void set(bool v) { v ? set() : clear(); }
    static void clear()     { GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << 8)); }
    static void toggle()    { (GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << 8)) == 0 ? set() : clear(); }
    static void setOutput() { PIN_FUNC_SELECT(periphs[8], functionsGpio[8]);
                              PIN_PULLUP_DIS(periphs[8]);
                              *gpcAdress[ 8] &= (*gpcAdress[8] & (0xF << 7));
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << 8);
    }
    static void setInput()  { PIN_FUNC_SELECT(periphs[8], functionsGpio[8]);
                              PIN_PULLUP_DIS(periphs[8]);
                              *gpcAdress[8] &= (*gpcAdress[8] & (0xF << 7)) | (1 << 8);
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << 8);
                              GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << 8);
    }
    static uint16_t read() { return (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << 8)) != 0; }
    static void pulseHigh(){ set(); clear(); }
    static void pulseLow() { clear(); set(); }
    static constexpr uint16_t bitmask() { return 1 << 8; }
};

class Pin9 {
public:
    static void set()       { GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, (1 << 9)); }
    static void set(bool v) { v ? set() : clear(); }
    static void clear()     { GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << 9)); }
    static void toggle()    { (GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << 9)) == 0 ? set() : clear(); }
    static void setOutput() { PIN_FUNC_SELECT(periphs[9], functionsGpio[9]);
                              PIN_PULLUP_DIS(periphs[9]);
                              *gpcAdress[ 9] &= (*gpcAdress[9] & (0xF << 7));
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << 9);
    }
    static void setInput()  { PIN_FUNC_SELECT(periphs[9], functionsGpio[9]);
                              PIN_PULLUP_DIS(periphs[9]);
                              *gpcAdress[9] &= (*gpcAdress[9] & (0xF << 7)) | (1 << 9);
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << 9);
                              GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << 9);
    }
    static uint16_t read() { return (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << 9)) != 0; }
    static void pulseHigh(){ set(); clear(); }
    static void pulseLow() { clear(); set(); }
    static constexpr uint16_t bitmask() { return 1 << 9; }
};

class Pin10 {
public:
    static void set()       { GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, (1 << 10)); }
    static void set(bool v) { v ? set() : clear(); }
    static void clear()     { GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << 10)); }
    static void toggle()    { (GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << 10)) == 0 ? set() : clear(); }
    static void setOutput() { PIN_FUNC_SELECT(periphs[10], functionsGpio[10]);
                              PIN_PULLUP_DIS(periphs[10]);
                              *gpcAdress[ 10] &= (*gpcAdress[10] & (0xF << 7));
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << 10);
    }
    static void setInput()  { PIN_FUNC_SELECT(periphs[10], functionsGpio[10]);
                              PIN_PULLUP_DIS(periphs[10]);
                              *gpcAdress[10] &= (*gpcAdress[10] & (0xF << 7)) | (1 << 10);
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << 10);
                              GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << 10);
    }
    static uint16_t read() { return (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << 10)) != 0; }
    static void pulseHigh(){ set(); clear(); }
    static void pulseLow() { clear(); set(); }
    static constexpr uint16_t bitmask() { return 1 << 10; }
};

class Pin11 {
public:
    static void set()       { GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, (1 << 11)); }
    static void set(bool v) { v ? set() : clear(); }
    static void clear()     { GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << 11)); }
    static void toggle()    { (GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << 11)) == 0 ? set() : clear(); }
    static void setOutput() { PIN_FUNC_SELECT(periphs[11], functionsGpio[11]);
                              PIN_PULLUP_DIS(periphs[11]);
                              *gpcAdress[ 11] &= (*gpcAdress[11] & (0xF << 7));
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << 11);
    }
    static void setInput()  { PIN_FUNC_SELECT(periphs[11], functionsGpio[11]);
                              PIN_PULLUP_DIS(periphs[11]);
                              *gpcAdress[11] &= (*gpcAdress[11] & (0xF << 7)) | (1 << 11);
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << 11);
                              GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << 11);
    }
    static uint16_t read() { return (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << 11)) != 0; }
    static void pulseHigh(){ set(); clear(); }
    static void pulseLow() { clear(); set(); }
    static constexpr uint16_t bitmask() { return 1 << 11; }
};

class Pin12 {
public:
    static void set()       { GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, (1 << 12)); }
    static void set(bool v) { v ? set() : clear(); }
    static void clear()     { GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << 12)); }
    static void toggle()    { (GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << 12)) == 0 ? set() : clear(); }
    static void setOutput() { PIN_FUNC_SELECT(periphs[12], functionsGpio[12]);
                              PIN_PULLUP_DIS(periphs[12]);
                              *gpcAdress[ 12] &= (*gpcAdress[12] & (0xF << 7));
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << 12);
    }
    static void setInput()  { PIN_FUNC_SELECT(periphs[12], functionsGpio[12]);
                              PIN_PULLUP_DIS(periphs[12]);
                              *gpcAdress[12] &= (*gpcAdress[12] & (0xF << 7)) | (1 << 12);
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << 12);
                              GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << 12);
    }
    static uint16_t read() { return (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << 12)) != 0; }
    static void pulseHigh(){ set(); clear(); }
    static void pulseLow() { clear(); set(); }
    static constexpr uint16_t bitmask() { return 1 << 12; }
};

class Pin13 {
public:
    static void set()       { GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, (1 << 13)); }
    static void set(bool v) { v ? set() : clear(); }
    static void clear()     { GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << 13)); }
    static void toggle()    { (GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << 13)) == 0 ? set() : clear(); }
    static void setOutput() { PIN_FUNC_SELECT(periphs[13], functionsGpio[13]);
                              PIN_PULLUP_DIS(periphs[13]);
                              *gpcAdress[ 13] &= (*gpcAdress[13] & (0xF << 7));
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << 13);
    }
    static void setInput()  { PIN_FUNC_SELECT(periphs[13], functionsGpio[13]);
                              PIN_PULLUP_DIS(periphs[13]);
                              *gpcAdress[13] &= (*gpcAdress[13] & (0xF << 7)) | (1 << 13);
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << 13);
                              GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << 13);
    }
    static uint16_t read() { return (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << 13)) != 0; }
    static void pulseHigh(){ set(); clear(); }
    static void pulseLow() { clear(); set(); }
    static constexpr uint16_t bitmask() { return 1 << 13; }
};

class Pin14 {
public:
    static void set()       { GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, (1 << 14)); }
    static void set(bool v) { v ? set() : clear(); }
    static void clear()     { GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << 14)); }
    static void toggle()    { (GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << 14)) == 0 ? set() : clear(); }
    static void setOutput() { PIN_FUNC_SELECT(periphs[14], functionsGpio[14]);
                              PIN_PULLUP_DIS(periphs[14]);
                              *gpcAdress[ 14] &= (*gpcAdress[14] & (0xF << 7));
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << 14);
    }
    static void setInput()  { PIN_FUNC_SELECT(periphs[14], functionsGpio[14]);
                              PIN_PULLUP_DIS(periphs[14]);
                              *gpcAdress[14] &= (*gpcAdress[14] & (0xF << 7)) | (1 << 14);
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << 14);
                              GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << 14);
    }
    static uint16_t read() { return (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << 14)) != 0; }
    static void pulseHigh(){ set(); clear(); }
    static void pulseLow() { clear(); set(); }
    static constexpr uint16_t bitmask() { return 1 << 14; }
};

class Pin15 {
public:
    static void set()       { GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, (1 << 15)); }
    static void set(bool v) { v ? set() : clear(); }
    static void clear()     { GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << 15)); }
    static void toggle()    { (GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << 15)) == 0 ? set() : clear(); }
    static void setOutput() { PIN_FUNC_SELECT(periphs[15], functionsGpio[15]);
                              PIN_PULLUP_DIS(periphs[15]);
                              *gpcAdress[ 15] &= (*gpcAdress[15] & (0xF << 7));
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << 15);
    }
    static void setInput()  { PIN_FUNC_SELECT(periphs[15], functionsGpio[15]);
                              PIN_PULLUP_DIS(periphs[15]);
                              *gpcAdress[15] &= (*gpcAdress[15] & (0xF << 7)) | (1 << 15);
                              GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << 15);
                              GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << 15);
    }
    static uint16_t read() { return (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << 15)) != 0; }
    static void pulseHigh(){ set(); clear(); }
    static void pulseLow() { clear(); set(); }
    static constexpr uint16_t bitmask() { return 1 << 15; }
};
} // namespace etl
