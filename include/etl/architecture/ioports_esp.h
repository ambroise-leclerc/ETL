#pragma once
#include <cstdint>

extern "C"
{
  #include "eagle_soc.h"
}

extern "C" void ets_delay_us(int us);
extern "C" void ets_timer_disarm(ETSTimer *a);
extern "C" void ets_timer_setfn(ETSTimer *t, ETSTimerFunc *pfunction, void *parg);
extern "C" void ets_timer_arm_new(ETSTimer *a, int b, int c, int isMstimer);
extern "C" void ets_isr_mask(uint32_t t);
extern "C" void ets_isr_attach(int intr, void *handler, void *arg); 
extern "C" void ets_isr_unmask(unsigned intr);





//DU C PAS REUSSI A FAIRE AUTREMENT
typedef void (*voidFuncPtr)(void);
typedef struct {
  void (*fn)(void);
} interrupt_handler_t;

static const uint32_t BASE_ADRR = 0x60000000;

static uint32_t* gpcAdress[16] = { 
	(uint32_t *)(BASE_ADRR + 0x328),
	 (uint32_t *)(BASE_ADRR + 0x32C),
	 (uint32_t *)(BASE_ADRR + 0x330),
	 (uint32_t *)(BASE_ADRR + 0x334),
	 (uint32_t *)(BASE_ADRR + 0x338),
	 (uint32_t *)(BASE_ADRR + 0x33C),
	  (uint32_t *)(BASE_ADRR + 0x340),
	  (uint32_t *)(BASE_ADRR + 0x344 ),
	  (uint32_t *)(BASE_ADRR + 0x348),
	  (uint32_t *)(BASE_ADRR +0x34C ),
	  (uint32_t *)(BASE_ADRR + 0x350),
	 (uint32_t *) (BASE_ADRR +0x354 ),
	  (uint32_t *)(BASE_ADRR +0x358 ),
	  (uint32_t *)(BASE_ADRR + 0x35C),
	  (uint32_t *)(BASE_ADRR + 0x360), 
	 (uint32_t *)(BASE_ADRR +0x364 )
 };
	
static uint32_t periphs[16] = { 
	PERIPHS_IO_MUX_GPIO0_U,
	PERIPHS_IO_MUX_U0TXD_U,
	PERIPHS_IO_MUX_GPIO2_U, 
	PERIPHS_IO_MUX_U0RXD_U,
	PERIPHS_IO_MUX_GPIO4_U,
	PERIPHS_IO_MUX_GPIO5_U,
	PERIPHS_IO_MUX_SD_CLK_U,
	PERIPHS_IO_MUX_SD_DATA0_U,
	PERIPHS_IO_MUX_SD_DATA1_U,
	PERIPHS_IO_MUX_SD_DATA2_U,
	PERIPHS_IO_MUX_SD_DATA3_U,
	PERIPHS_IO_MUX_SD_CMD_U,
	PERIPHS_IO_MUX_MTDI_U,
	PERIPHS_IO_MUX_MTCK_U,
	PERIPHS_IO_MUX_MTMS_U,
	PERIPHS_IO_MUX_MTDO_U 
};
	
static  uint8_t functionsGpio[16] = { 
	FUNC_GPIO0,
	FUNC_GPIO1,
	FUNC_GPIO2,
	FUNC_GPIO3,
	FUNC_GPIO4,
	FUNC_GPIO5,
	3,
	3,
	3,
	FUNC_GPIO9,
	FUNC_GPIO10,
	3,
	FUNC_GPIO12,
	FUNC_GPIO13,
	FUNC_GPIO14,
	FUNC_GPIO15
};




struct PinChangeIRQ0;
struct PinChangeIRQ1;

// TON CODE
struct Port0 {
	
 using PinChangeIRQ = PinChangeIRQ0;
  /// Assigns a value to PORT0.
  /// @param[in] value value affected to PORT0
  static void Assign(uint8_t value)   {  }

  /// Sets masked bits in PORT0.
  /// @param[in] mask bits to set
  static void SetBits(uint8_t mask)   { }

  /// Clears masked bits in PORT0.
  /// @param[in] mask bits to clear
  static void ClearBits(uint8_t mask) { } 

  /// Changes values of masked bits in PORT0.
  /// @param[in] mask bits to change
  /// @param[in] value new bits values
  static void ChangeBits(uint8_t mask, uint8_t value) { } 

  /// Toggles masked bits in PORT0.
  /// @param[in] mask bits to toggle
  static void ToggleBits(uint8_t mask) { } 

  /// Pulses masked bits in PORT0 with high state first.
  /// @param[in] mask bits to pulse
  static void PulseHigh(uint8_t mask) { }

  /// Pulses masked bits in PORT0 with low state first.
  /// @param[in] mask bits to pulse
  static void PulseLow(uint8_t mask)  { }

  /// Set corresponding masked bits of PORT0 to output direction.
  /// @param[in] mask bits
  static void SetDDR(uint8_t mask)    { }

  /// Set corresponding masked bits of PORT0 to input direction.
  /// @param[in] mask bits
  static void ClearDDR(uint8_t mask)  { }
};

struct Port1 {
	
    using PinChangeIRQ = PinChangeIRQ1;
     /// Assigns a value to PORT0.
     /// @param[in] value value affected to PORT0
    static void Assign(uint8_t value) {}

      /// Sets masked bits in PORT0.
      /// @param[in] mask bits to set
    static void SetBits(uint8_t mask) {}

      /// Clears masked bits in PORT0.
      /// @param[in] mask bits to clear
    static void ClearBits(uint8_t mask) {} 

      /// Changes values of masked bits in PORT0.
      /// @param[in] mask bits to change
      /// @param[in] value new bits values
    static void ChangeBits(uint8_t mask, uint8_t value) {} 

      /// Toggles masked bits in PORT0.
      /// @param[in] mask bits to toggle
    static void ToggleBits(uint8_t mask) {} 

      /// Pulses masked bits in PORT0 with high state first.
      /// @param[in] mask bits to pulse
    static void PulseHigh(uint8_t mask) {}

      /// Pulses masked bits in PORT0 with low state first.
      /// @param[in] mask bits to pulse
    static void PulseLow(uint8_t mask) {}

      /// Set corresponding masked bits of PORT0 to output direction.
      /// @param[in] mask bits
    static void SetDDR(uint8_t mask) {}

      /// Set corresponding masked bits of PORT0 to input direction.
      /// @param[in] mask bits
    static void ClearDDR(uint8_t mask) {}
};

template<uint8_t pinNumber>
	struct AbstractPin : public Pin<Port0> 
	{
		static void Set() { 
			GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, (1 << pinNumber));
		}
		
		static void Set(bool value) {
			if (value)
			{
				Set();
				
			}
			else
			{
				Clear();
				
			}
		}

		static void Clear() { 
			GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, (1 << pinNumber));
		}

		static void Toggle() { 
			if ((GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << pinNumber)) != 0)
			{
				Clear();
			}
			else {
				Set();
			}	
		}
	
		static void SetOutput() {
			PIN_FUNC_SELECT(periphs[pinNumber], functionsGpio[pinNumber]);
			PIN_PULLUP_DIS(periphs[pinNumber]);
			*gpcAdress[pinNumber] &= (*gpcAdress[pinNumber] & (0xF << 7));  
			GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << pinNumber);
		}
	
		static void SetInput() {
			PIN_FUNC_SELECT(periphs[pinNumber], functionsGpio[pinNumber]);
			PIN_PULLUP_DIS(periphs[pinNumber]);
			*gpcAdress[pinNumber] &= (*gpcAdress[pinNumber] & (0xF << 7)) | (1 << pinNumber);  
			GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << pinNumber);  
			GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << pinNumber);
		}
	
		static uint16_t  Read()
		{
			return (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << pinNumber)) != 0;
		}

		static void PulseHigh() {	
			Set();
			Clear();
		}

		static void PulseLow() {
			Clear();
			Set();
		} 

		static constexpr uint8_t bitmask() {
			return (1 << pinNumber); 
		}
		
		static void AttachHandler(voidFuncPtr userFunc) {
			//AbstractPin<2>::AttachHandler(userFunc);
		}
		
		static void Interrupt() {
		//AbstractPin<2>::AttachHandler(userFunc);
		}
	
	};

