#pragma once
//#include <string>
#include <../libstd/include/thread>
#include <../libstd/include/chrono>

using namespace std;
using namespace chrono_literals;

namespace etl {
//mode 4 pins pour le moment
template <typename RS, typename ENABLE, typename D4, typename D5, typename D6, typename D7, uint8_t lineNumber, uint8_t colSize>
class HD44780 {
public:

    HD44780() {
    }

    void init() {
        initAddresses();

        RS::setOutput();
        ENABLE::setOutput();
        D4::setOutput();
        D5::setOutput();
        D6::setOutput();
        D7::setOutput();
        std::this_thread::sleep_for(45ms);
			
        RS::clear();
        ENABLE::clear();

        // Only for 4 bits mode
        writeInit(0x30);
		std::this_thread::sleep_for(45ms);
        writeInit(0x30);
		std::this_thread::sleep_for(1ms);
        writeInit(0x30);
	    std::this_thread::sleep_for(150us);
        write(0x02);
			
			
        uint8_t functionSet = 0;
        if (lineNumber > 1) {
            functionSet |= Display_Function::D_TWO_LINES;
        }
        if ((lineNumber == 1) && (colSize > 10)) {
            functionSet |= Display_Function::D_MATRICE_5_11;
            dots5x11Matrice = true;
        }
        issueCommand(Mode::FUNCTION_SET, functionSet);
        clear();
        setDirection(true, false);
        setDisplay(true, true, false);
    }
		
    void setDisplay(bool display, bool visibleCursor, bool blinkCursor)	{
        if (display) {
            currentDisplay |= Display_Control::DISPLAY_ON;
        }
        else {
            currentDisplay &= ~Display_Control::DISPLAY_ON;
        }
        if (visibleCursor) {
            currentDisplay |= Display_Control::CURSOR_VISIBLE;
        }
        else {
            currentDisplay &= ~Display_Control::CURSOR_VISIBLE;
        }
        if (blinkCursor) {
            currentDisplay |= Display_Control::BLINK_CURSOR;
        }
        else {
            currentDisplay |= Display_Control::BLINK_CURSOR;
        }
        issueCommand(Commands::DISPLAY_CONTROL, currentDisplay);
    }

    void showText() {
        currentDisplay |= Display_Control::DISPLAY_ON;
        IssueCommand(Commands::DISPLAY_CONTROL, currentDisplay);
    }

    void hideText() {
        currentDisplay &= ~Display_Control::DISPLAY_ON;
        issueCommand(Commands::DISPLAY_CONTROL, currentDisplay);
    }

    void showCursor() {
        currentDisplay |= Display_Control::CURSOR_VISIBLE;
        issueCommand(Commands::DISPLAY_CONTROL, currentDisplay);
    }

    void hideCursor() {
        currentDisplay &= ~Display_Control::CURSOR_VISIBLE;
        issueCommand(Commands::DISPLAY_CONTROL, currentDisplay);
    }

    void blinkCursor() {
        currentDisplay |= Display_Control::BLINK_CURSOR;
        issueCommand(Commands::DISPLAY_CONTROL, currentDisplay);
    }

    void staticCursor() {
        currentDisplay &= ~Display_Control::BLINK_CURSOR;
        issueCommand(Commands::DISPLAY_CONTROL, currentDisplay);
    }
		
	void clear() {
		issueCommand(Commands::CLEAR_DISPLAY);
        currentCol = 0;
        currentRow = 0;
        ddRamddr = true;
	    std::this_thread::sleep_for(2000ms);
	}

    void home() {
        issueCommand(Commands::RETURN_HOME);
        currentCol = 0;
        currentRow = 0;
        ddRamddr = true;
		 std::this_thread::sleep_for(2000ms);
    }

    void setDirection(bool writeRight,bool shiftAdress) {
        uint8_t args = 0;
        if (writeRight) {
            args |= Display_Mode::DIRECTION;
        }
        if (shiftAdress) {
            args |=Display_Mode::SHIFT;
        }
           
        issueCommand(Commands::ENTRY_MODE_SET, args);
    }

    void display(char character) {
        if (currentCol >= colSize) {
            changeRow();
        }
        if (!ddRamddr) {
            setCursor(currentCol, currentRow);
        }
        RS::set();
        write(character);
        currentCol++;
    }


    void display(uint8_t colNumber, uint8_t rowNumber, char character) {
        setCursor(colNumber, rowNumber);
        RS::set();
        write(character);
        currentCol++;
    }

    void setCursor(uint8_t colNumber, uint8_t rowNumber) {
        uint8_t args = row_addresses[rowNumber];
        args += colNumber;
        issueCommand(Mode::SET_DDRAMADDR, args);
        currentCol = colNumber;
        currentRow = rowNumber;
        ddRamddr = true;
    }

    void addChar(uint8_t location, uint8_t matrice[]) {
        if (!dots5x11Matrice && location > 8) {
            location = 8;
        }
        else if (dots5x11Matrice && location > 4) {
            location = 4;
        }
        ddRamddr = false;
        issueCommand(Mode::SET_CGRAMADDR, location << 3);
        auto length = dots5x11Matrice ? 11 : 8;
        RS::set();
        for (auto i = 0; i < length; i++) {
            write(matrice[i]);
        }
    }
		
private:
	enum Commands:uint8_t {
		CLEAR_DISPLAY   = 0x01,
		RETURN_HOME     = 0x02,
		ENTRY_MODE_SET  = 0x04,
		DISPLAY_CONTROL = 0x08
			
	};
		
	enum Mode:uint8_t {
		FUNCTION_SET    = 0x20,
		SET_CGRAMADDR   = 0x40,
		SET_DDRAMADDR	= 0x80
	};
		
	enum Display_Function:uint8_t
	{
		D_8_BITS = 0x10,
		D_TWO_LINES = 0x08,
		D_MATRICE_5_11 = 0x04
			
	};
		
	enum Display_Control:uint8_t
	{
		DISPLAY_ON     = 0x04,
		CURSOR_VISIBLE = 0x02,
		BLINK_CURSOR   = 0x01
			
	};

    enum Display_Mode :uint8_t
    {
        DIRECTION = 0x02,
        SHIFT = 0x01
    };
		

    uint8_t currentCol = 0;
    uint8_t currentRow = 0;
    uint8_t currentDisplay = 0;
    uint8_t row_addresses[lineNumber];
    bool ddRamddr = false;
    bool dots5x11Matrice = false;

    void changeRow() {
        if ((currentRow+1) < lineNumber) {
            currentRow++;
            setCursor(0, currentRow );
        }
        else {
            setCursor(0, 0);
        }
    }

    void initAddresses() {
        for (auto i = 0; i < lineNumber; i++) {
            row_addresses[i] = colSize * (i / 2) + 0x40 * (i % 2);
        }
    }

		void issueCommand(Mode mode, uint8_t arg) {
        RS::clear();
		write(mode | arg);
		std::this_thread::sleep_for(2ms);
	}
		
		void issueCommand(Commands command) {
        RS::clear();
		write(command);
		std::this_thread::sleep_for(2ms);
	}
		
		void issueCommand(Commands command,uint8_t arg) {
        RS::clear();
		write(command | arg);
		std::this_thread::sleep_for(2ms);
	}

    void write(uint8_t data) {
		D4::set(((1 << 4)&data) != 0 );
		D5::set(((1 << 5)&data) != 0);
		D6::set(((1 << 6)&data) != 0);
		D7::set(((1 << 7)&data) != 0);
		pulseEnable();
		D4::set(((1 << 0)&data) != 0);
		D5::set(((1 << 1)&data) != 0);
		D6::set(((1 << 2)&data) != 0);
		D7::set(((1 << 3)&data) != 0);
		pulseEnable();
	}

    void writeInit(uint8_t data) {
        D4::set(((1 << 4)&data) != 0);
        D5::set(((1 << 5)&data) != 0);
        D6::set(((1 << 6)&data) != 0);
        D7::set(((1 << 7)&data) != 0);
        pulseEnable();
    }
		
    void pulseEnable() {
	    using namespace std::chrono;
		ENABLE::clear();
		std::this_thread::sleep_for(1us);
		ENABLE::set();
        std::this_thread::sleep_for(1us);
		ENABLE::clear();
        std::this_thread::sleep_for(1ms);
	}

   
};

template <typename HD44780>
class HD44780Printer {
public:
    static void print(HD44780& h,const char str[]) {
        for (auto i = 0; str[i] != '\0'; i++) {
            h.display(str[i]);
        }
    }
};
}