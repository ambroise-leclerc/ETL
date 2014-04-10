// JavaScript Document

function GenHeader(file) {
  file.WriteLine("/// @file ioports.h");
  file.WriteLine("/// @date 19/01/2014 22:28:16");
  file.WriteLine("/// @author Ambroise Leclerc");
  file.WriteLine("/// @brief AVR 8-bit microcontrollers ports handling classes");
  file.WriteLine("//");
  file.WriteLine("// Copyright (c) 2014, Ambroise Leclerc");
  file.WriteLine("//   All rights reserved.");
  file.WriteLine("//");
  file.WriteLine("//   Redistribution and use in source and binary forms, with or without");
  file.WriteLine("//   modification, are permitted provided that the following conditions are met:");
  file.WriteLine("//");
  file.WriteLine("//   * Redistributions of source code must retain the above copyright");
  file.WriteLine("//     notice, this list of conditions and the following disclaimer.");
  file.WriteLine("//   * Redistributions in binary form must reproduce the above copyright");
  file.WriteLine("//     notice, this list of conditions and the following disclaimer in");
  file.WriteLine("//     the documentation and/or other materials provided with the");
  file.WriteLine("//     distribution.");
  file.WriteLine("//   * Neither the name of the copyright holders nor the names of");
  file.WriteLine("//     contributors may be used to endorse or promote products derived");
  file.WriteLine("//     from this software without specific prior written permission.");
  file.WriteLine("//");
  file.WriteLine("//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS' ");
  file.WriteLine("//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE");
  file.WriteLine("//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE");
  file.WriteLine("//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE");
  file.WriteLine("//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR");
  file.WriteLine("//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF");
  file.WriteLine("//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS");
  file.WriteLine("//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN");
  file.WriteLine("//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)");
  file.WriteLine("//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE");
  file.WriteLine("//  POSSIBILITY OF SUCH DAMAGE.");
  file.WriteLine("");
  file.WriteLine("#ifndef ETL_IOPORTS_H_");
  file.WriteLine("#define ETL_IOPORTS_H_");
  file.WriteLine();
  file.WriteLine();
  file.WriteLine("namespace etl {");
}

function GenFooter(file) {
  file.WriteLine("} // namespace etl");
  file.WriteLine("#endif //ETL_IOPORTS_H_");
}

function GenPinWrapperClass(file, port, pin) {
  file.WriteLine("struct Pin"+port+pin+" {");
  file.WriteLine("  /// Sets Pin" + port + pin + " to HIGH." + port + ".");
  file.WriteLine("  static void Set()       { PORT" + port + " |= (1<<" + pin + "); }");
  file.WriteLine("");
  file.WriteLine("  /// Sets Pin" + port + pin + " to LOW." + port + ".");
  file.WriteLine("  static void Clear()     { PORT" + port + " &= (~(1<<" + pin + ")); }");
  file.WriteLine("");
  file.WriteLine("  /// Toggles Pin" + port + pin + " value." + port + ".");
  file.WriteLine("  static void Toggle()    { PIN" + port + " |= (1<<" + pin + "); }");
  file.WriteLine("");
  file.WriteLine("  /// Configures Pin" + port + pin + "  as an output pin.");
  file.WriteLine("  static void SetOutput() { DDR" + port + " |= (1<<" + pin + "); }");
  file.WriteLine("");
  file.WriteLine("  /// Configures Pin" + port + pin + "  as an input pin.");
  file.WriteLine("  static void SetInput()  { DDR" + port + " &= (~(1<<" + pin + ")); }");
  file.WriteLine("  static void PulseHigh() { PORT"+port+" |= (1<<"+pin+"); PORT"+port+" &= (~(1<<"+pin+")); }");
  file.WriteLine("  static void PulseLow()  { PORT" + port + " &= (~(1<<" + pin + ")); PORT" + port + " |= (1<<" + pin + "); }");
  file.WriteLine("");
  file.WriteLine("  /// Reads Pin" + port + pin + "  value.");
  file.WriteLine("  /// @return Port pin value.");
  file.WriteLine("  static bool Test()      { return PIN"+port+" & (1<<"+pin+"); }");
  file.WriteLine("");
  file.WriteLine('  /// Returns the native port #definition corresponding to Pin"+port+pin+" as defined in "avr/io.h" ');
  file.WriteLine("  /// @return PORT"+port);
  file.WriteLine("  static constexpr decltype(PORT" + port + ") GetNativePort() { return PORT" + port + "; }");
  file.WriteLine("");
  file.WriteLine("  /// Returns the bitmask corresponding to this pin.");
  file.WriteLine("  /// @return (1<<"+pin+")");
  file.WriteLine("  static constexpr uint8_t bitmask()               { return (1<<" + pin + "); }");
  file.WriteLine("");
  file.WriteLine("  /// Returns the bit corresponding to this pin.");
  file.WriteLine("  /// @return "+pin);
  file.WriteLine("  static constexpr uint8_t bit()                   { return " + pin + "; }");
  file.WriteLine("");
  file.WriteLine("  /// Port is defined as the Port object to which this pin belongs.")
  file.WriteLine("  using Port = Port"+port+";");
  file.WriteLine("};");
  file.WriteLine("");
}

function GenPortWrapperClass(file, port) {
  file.WriteLine("struct Port"+port+" {");
  file.WriteLine("  /// Assigns a value to PORT"+port+".");
  file.WriteLine("  /// @param[in] value value affected to PORT"+port);
  file.WriteLine("  static void Assign(uint8_t value)   { PORT"+port+" = value; }");
  file.WriteLine("");
  file.WriteLine("  /// Sets masked bits in PORT"+port+".");
  file.WriteLine("  /// @param[in] mask bits to set");
  file.WriteLine("  static void SetBits(uint8_t mask)   { PORT"+port+" |= mask;}");
  file.WriteLine("");
  file.WriteLine("  /// Clears masked bits in PORT"+port+".");
  file.WriteLine("  /// @param[in] mask bits to clear");
  file.WriteLine("  static void ClearBits(uint8_t mask) { PORT"+port+" &= ~mask;} ");
  file.WriteLine("");
  file.WriteLine("  /// Toggles masked bits in PORT"+port+".");
  file.WriteLine("  /// @param[in] mask bits to toggle");
  file.WriteLine("  static void ToggleBits(uint8_t mask) { PORT"+port+" ^= mask;} ");
  file.WriteLine("");
  file.WriteLine("  /// Pulses masked bits in PORT"+port+" with high state first.");
  file.WriteLine("  /// @param[in] mask bits to pulse");
  file.WriteLine("  static void PulseHigh(uint8_t mask) { PORT"+port+" |= mask; PORT"+port+" &= ~mask; }");
  file.WriteLine("");
  file.WriteLine("  /// Pulses masked bits in PORT"+port+" with low state first.");
  file.WriteLine("  /// @param[in] mask bits to pulse");
  file.WriteLine("  static void PulseLow(uint8_t mask)  { PORT"+port+" &= ~mask; PORT"+port+" |= mask; }");
  file.WriteLine("");
  file.WriteLine("  /// Set corresponding masked bits of PORT"+port+" to output direction.");
  file.WriteLine("  /// @param[in] mask bits");
  file.WriteLine("  static void SetDDR(uint8_t mask)    { DDR"+port+" |= mask; }");
  file.WriteLine("");
  file.WriteLine("  /// Set corresponding masked bits of PORT"+port+" to input direction.");
  file.WriteLine("  /// @param[in] mask bits");
  file.WriteLine("  static void ClearDDR(uint8_t mask)  { DDR"+port+" &= ~mask; }");
  file.WriteLine("");
  file.WriteLine("  /// Returns PIN"+port+" register.");
  file.WriteLine("  static uint8_t GetPIN()             { return PIN"+port+"; }");
  file.WriteLine("");
  file.WriteLine("  /// Tests masked bits of PORT"+port);
  file.WriteLine("  /// @param[in] mask bits");
  file.WriteLine("  /// @param[in] true if the corresponding bits are all set, false otherwise.");
  file.WriteLine("  static bool TestBits(uint8_t mask)  { return (PIN"+port+" & mask) == mask; }");
  file.WriteLine("");
  file.WriteLine("  /// Returns the value of the bit at the position pos.");
  file.WriteLine("  /// @param[in] position of the bit to return");
  file.WriteLine("  /// @return true if the requested bit is set, false otherwise.");
  file.WriteLine("  static bool Test(uint8_t bit) { return PIN"+port+" & (1<<bit); }");
  file.WriteLine("};");
  file.WriteLine("");
}

function GenGPIOWrapperClasses(file, port) {
  file.WriteLine("#ifdef PORT"+port);
  GenPortWrapperClass(file, port);
  for (bit=0; bit<8; bit++)
  {
    GenPinWrapperClass(file, port, bit);
  }
  file.WriteLine("#endif //PORT"+port);
  file.WriteLine();
}

function GenSPIWrapperClasses(file, module, register, bitlen) {
  var class_name = module + register.charAt(0) + register.substring(1).toLowerCase();
  file.WriteLine("struct "+class_name+" {");
  file.WriteLine("");
  file.WriteLine("  /// Assigns a value to "+register);
  file.WriteLine("  /// @param[in] value value affected to "+register);
  file.WriteLine("  static void Assign(uint"+bitlen+"_t value)  { "+register+" = value; }");
  file.WriteLine("");
  file.WriteLine("  /// Sets masked bits in "+register);
  file.WriteLine("  /// @param[in] mask bits to set");
  file.WriteLine("  static void Set(uint"+bitlen+"_t mask)      { "+register+" |= mask; }");
  file.WriteLine("");
  file.WriteLine("  /// Clears masked bits in "+register);
  file.WriteLine("  /// @param[in] mask bits to clear");
  file.WriteLine("  static void Clear(uint"+bitlen+"_t mask)    { "+register+" &= ~mask; }");
  file.WriteLine("  static uint8_t Get()               { return "+register+"; }");
  file.WriteLine("  static bool TestBits(uint"+bitlen+"_t mask) { return "+register+" & mask; }");
  file.WriteLine("  void operator=(uint8_t value)      { "+register+" = value; }");
  file.WriteLine("};");
  file.WriteLine("");
}

function GenTimerWrapperClasses(file, timernum, timerwidth) {
  file.WriteLine("#ifdef TCNT" + timernum);
  file.WriteLine("struct Timer" + timernum + " {");
  file.WriteLine("  typedef uint" + timerwidth + "_t value_type;");
  file.WriteLine("  ");
  file.WriteLine("  static value_type GetValue()            { return TCNT" + timernum + "; }");
  file.WriteLine("  static void SetValue(value_type value)  { TCNT" + timernum + " = value; }");
  file.WriteLine("  static void AddValue(value_type value)  { TCNT" + timernum + " += value; }");
  file.WriteLine("  static void SubValue(value_type value)  { TCNT" + timernum + " -= value; }");
  file.WriteLine("  static void SetCtrlRegA(uint8_t mask)   { TCCR" + timernum + "A |= mask; }");
  file.WriteLine("  static void ClearCtrlRegA(uint8_t mask) { TCCR" + timernum + "A &= ~mask; }");
  file.WriteLine("  static void SetCtrlRegB(uint8_t mask)   { TCCR" + timernum + "B |= mask; }");
  file.WriteLine("  static void ClearCtrlRegB(uint8_t mask) { TCCR" + timernum + "B &= ~mask; }");
  file.WriteLine("    ");
  file.WriteLine("  enum Prescaler : uint8_t {");
  file.WriteLine("    STOP_TIMER = 0, NO_PRESCALER = (1<<CS" + timernum + "0), CLK_DIV_8 = (1<<CS" + timernum + "1),");
  file.WriteLine("    CLK_DIV_64 = (1<<CS" + timernum + "1)|(1<<CS" + timernum + "0), CLK_DIV_256 = (1<<CS" + timernum + "2), CLK_DIV_1024 = (1<<CS" + timernum + "2)|(1<<CS" + timernum + "0),");
  file.WriteLine("    INC_ON_FALLING = (1<<CS" + timernum + "2)|(1<<CS" + timernum + "1), INC_ON_RISING = (1<<CS" + timernum + "2)|(1<<CS" + timernum + "1)|(1<<CS" + timernum + "0) };");
  file.WriteLine("  enum Constants : uint8_t { ALL_BITS = 0xFF };");
  file.WriteLine("};  ");
  file.WriteLine("#endif //TCNT" + timernum + "");
  file.WriteLine("");
}


var outputFileName = WScript.Arguments.Item(0);
var fso = new ActiveXObject("Scripting.FileSystemObject");
var file = fso.CreateTextFile(outputFileName, true);

GenHeader(file);
for (port=66; port<77; port++)
  GenGPIOWrapperClasses(file, String.fromCharCode(port));

GenSPIWrapperClasses(file, "Spi", "SPCR", 8);
GenSPIWrapperClasses(file, "Spi", "SPDR", 8);
GenSPIWrapperClasses(file, "Spi", "SPSR", 8);
GenSPIWrapperClasses(file, "Usart", "UBRR0", 16);
GenSPIWrapperClasses(file, "Usart", "UCSR0A", 8);
GenSPIWrapperClasses(file, "Usart", "UCSR0B", 8);
GenSPIWrapperClasses(file, "Usart", "UCSR0C", 8);
GenSPIWrapperClasses(file, "Usart", "UDR0", 8);

GenTimerWrapperClasses(file, 0, 8);
GenTimerWrapperClasses(file, 1, 16);
GenTimerWrapperClasses(file, 2, 8);
GenTimerWrapperClasses(file, 3, 16);

GenFooter(file);
file.Close();