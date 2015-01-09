// Arduino as Digital Delay Generator
//
// The program allows to use the Arduino as externally triggered delay generator 
// with a delay range of 4.5 us to 4092 us and an estimated precision of 180 ns.
// The current program offers 5 programmable output channels (D3-D7)
// Use via Serial Interface, 19200 baud; Send \n terminated commands.
//
// Delay offset from trigger is 58 ticks +-1 (3.625 us)
// Dead time after trigger is 71 ticks +-1 (4.4375 us) (offset+13)
// Need minimum of 62 ticks (about 4 us) between two delay events and before 
// accepting the next trigger (Probably the required time to rewrite the compare 
// match register OCRA)
// 
// Tried & Failed: 
// - Alternating OCRA and OCRB with asynchronous update of DOut to reduce
//   dead-time between delay events. This only gave ~300 ns improvement 
//   but increased the trigger dead time and complexity of the progam.
// - Count off ticks for delay differences <3.81 us. 
//   Any loop in the interrupt routine + value lookup in the arrays took 
//   too much time to significantly reduce the dead time between delay events
// Possibe improvements:
// - Use overflow interupt to access times beyond 4.096 ms 
//   (create special case to access times very close to 4.096 ms?)
// - Option to use other digital/analog pins
// - Utilize "Use Toggle OC1A/OC1B on Compare Match." to address delays < 4 us
//   (see ATMEL 328 datasheet, p. 126)
// - Add internal trigger option 
//   (via scaled separate timer - not very precise)
//   (or via overflow counter for timer 1 - needs some effort)
//
// Feel free to use, redistribute and/or modify this program under the terms of the 
// GNU Lesser General Public License as published by the Free Software Foundation 
// (<http://www.gnu.org/licenses/>). I offer no warranty or support.
//
// Thomas Schultz

#include <EEPROM.h>
#include <EEPROM_anything.h>
#include <SerialCommand.h>              // SerialCommand library to handle serial commands
                                        // This is identical to kroimon/Arduino-SerialCommand but ignores the case of commands
                                        // line 75: char inChar = tolower(Serial.read());
#define ProgString "Delay Generator"
#define ID "Arduino UNO"
#define clock 16000                     // Arduino clock rate in kHz
#define TimerOffset 58                  // Trigger offset in ticks
# define _EEPROM_save_position 0        // EEPROM position for saving of delays


unsigned int Delays[] = {13,        75,        149,       228,       0,0,0,0,0,0}; // Delays; intitialize 10 values
short int    DOut[]   = {B11111000, B00000000, B11111000, B00000000, 0,0,0,0,0,0}; // Digital pin values; intitialize 10 values
short int _imax=4;                      // Number of active delays
volatile short int i=0;                 // Counter for delays
char buf[17];                           // Buffer for text output

SerialCommand sCmd;  

void setup(){
  Serial.begin (19200);                             // Set up the Serial Connection
  Help();                                           // Print command list to serial
  LoadDelays();                                     // Load Delay values from EEPROM
  sCmd.addCommand("List", ListDelays);              // Print list of delay values
  sCmd.addCommand("C", ClearDelay);                 // Clear one or all delays
  sCmd.addCommand("A", SetDelay);                   // Set delay value
  sCmd.addCommand("Save", SaveDelays);              // Save delay values to EEPROM
  sCmd.addCommand("Load", LoadDelays);              // Load delay values from EEPROM
  sCmd.addCommand("Help", Help);                    // Print command list
  sCmd.addCommand("ID", IDN);                       // Print ID
  sCmd.setDefaultHandler(unrecognized);             // Handler for unexpected commands (says "What?")


  cli();                    // Stop interrupts
  DDRD |= B11111000;        // Set pins 3 to 7 as output
  DDRD &= ~B00000100;       // Set D2 (trigger pin) as input 
  
// Set up external trigger interrupt on pin D2 (INT0) (see: https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328)
  PORTD = B00000100;        // Turn on the Pull-up on D2 (INT0, trigger pin)
  EICRA |= B00000011;       // Trigger on rising edge of INT0
  EIMSK |= B00000001;       // Turn on external trigger INT0 (on pin 2)

// Set up timer1 interrupt (see: https://sites.google.com/site/qeewiki/books/avr-guide/timers-on-the-atmega328)
//  TCCR1A &= ~B00000011;   // Set timer 1 to normal mode (WGM01,WGM00 = 0)
  TCCR1A = 0;               // set entire TCCR1A register to 0 (no PWM here)
  TCCR1B = 0;               // same for TCCR1B
  TIMSK1 = B00000010;       // Enable timer 1 - compare interrupt A (set bit 1 - OCIE1A)
  
// Turn off  timers 0 and 2 to avoid interference //
  TCCR0A = 0;               // Set entire TCC0A register to 0
  TCCR0B = 0;               // same for TCCR0B
  TCCR2A = 0;               // set entire TCCR1A register to 0
  TCCR2B = 0;               // same for TCCR1B
  
  sei();                    // Activate interrupt (set register bit) 
}

void loop(){
  sCmd.readSerial();          // Process the serial commands
  TCNT1 = 0;                  // Reset timer 1 counter value  
  i=0;                        // Reset delay list pointer
  OCR1A = Delays[0];          // Reset compare match register for first delay
  EIFR = 1;                   // Clear external interrupt flag
  EIMSK = B00000001;          // Turn on external trigger INT0 (on pin 2)
  while (i<_imax){}           // Wait for timer delays
  TCCR1B = B00000000;         // Stop Timer1
  sei();                      // Activate external interrupt
}


ISR(TIMER1_COMPA_vect){   //--- INTERRUPT: TIMER 1 REACHED OCR1A VALUE ---//
// It takes about 600 ns to set the digital output values. This is twice more than 
// discussed in "http://www.billporter.info/2010/08/18/ready-set-oscillate-the-fastest-way-to-change-arduino-pins/'
// The speed must be reduced due to the variable look-up. Assigning a simple variable value 
// takes some 400 ns.
// The dead time between two interrupts is some 4 microseconds, close to 64 
// clock cycles. I don't know what takes so long.
  PORTD = DOut[i];            // Set digital output values
  OCR1A = Delays[++i];        // Set compare match register for next delay
}

ISR (INT0_vect){         //--- INTERRUPT: TRUGGER EVENT ON PIN 2 (INT0) ---//
// I get up to 180 ns (3 ticks) jitter: 1 tick (62.5 ns) due to trigger digitization,
// up to 2 ticks due to OCR1A value incrementing only on rising counter flag.
  EIMSK = B00000000;          // Turn off external trigger INT0 (on pin 2)
  TCCR1B = B00000001;          // Start timer 1
}

void ListDelays(){      //--- LIST DELAY VALUES ---//
  cli();                                     // Stop interrupt
  int j; 
  Serial.print(_rtxt(buf,"Delay", 11));      // List header 
  Serial.print(_rtxt(buf,"Ticks", 11));
  Serial.print(_rtxt(buf,"D-out", 10));
  Serial.println(_rtxt(buf,"D76543", 10));
  for (j=0; j<_imax; j++){                   // For each delay
    Serial.print(char(j+65));                // Label delay with capital letters
    Serial.print(_ftoa(buf,TicksToMicros(Delays[j]),4,10));  // Print delay in microseconds
    Serial.print(_itoa(buf,Delays[j],10));   // Print delay in clock ticks
    Serial.print(_itoa(buf, DOut[j],10));    // Print status of bits 3-7 (D3-D7) as integer
    Serial.println(_itob(buf,DOut[j],B11111000,10));  // Print status of bits 3-7 (D3-D7) as binary
  }
  Serial.println();
  sei();                                     // Activate interrupt
}
void ClearDelay(){      //--- CLEAR ONE OR ALL DELAY VALUES ---//
  cli();                                     // Stop interrupt
// Clear x: Clear a single delay value
  char *arg;
  arg = sCmd.next();                         // Read number string
  if (arg != NULL) {                         // Did the user give a number?
    short a = (char)*arg-65;                 // Letter A,B,... to number 1,2,...
    if ((a < _imax) && (a >= 0)){            // Value in range?
      for (int k=a; k<=_imax; k++){          // Loop over all remaining delays
        Delays[k] = Delays[k+1];             // Shift remaining values in array
        DOut[k]   = DOut[k+1];}               // --"--
    _imax--;                                 // New number of delays
    }
    else Serial.println("Not in range");
  }
// Clear: Clear all
  else {
     memset(DOut,0,9);                       // Flush array
     memset(Delays,0,9);                     // Flush array
    _imax = 0;}                              // Reset _imax
  ListDelays();                              // List remaining delays
  sei();                                     // Activate interrupt
}

void SetDelay(){       //--- SET A NEW DELAY VALUE ---//
  cli();                                     // Stop interrupt
  int j=0, k;
  char *arg;
  
  arg = sCmd.next();                         // Read next command
  if (arg != NULL) {                         // Did the user give a number?
    short _dout=0;                           // Variable for digital out values
    float _delay = atof(arg);                // Value for new delay  
    unsigned long _ldelay = MicrosToTicks(atof(arg));  // New delay in counter ticks 
    while (_ldelay > Delays[j]) {j++;}       // Find correct delay position in Delays[]
    if (j > _imax) j = _imax;                // Make sure position is in range
    for (k=_imax; k>=j; k--) Delays[k] = Delays[k-1];  // Make space for new _delay
    Delays[j] = _ldelay;                     // Insert _delay in Delays[]
    
    arg = sCmd.next();                       // Read next command
    if (arg != NULL) {                       // Did the user give a number?
      if (*arg == 'B'){                      // Binary digital out value
        for (k=7; k>=3; k--) _dout |= (*(++arg) != '0') << k;}  // Set bits in _dout
      else _dout += atoi(arg);               // Integer digital out value
    } 
    else _dout = 255 - DOut[j-1];            // No digital out value given, invert every channel
    for (k=_imax; k>=j; k--) DOut[k] = DOut[k-1];  // Make space for new Dout value
    _dout |= B00000100;                      // Pull-up D2 (trigger pin)
    DOut[j] = _dout;                         // Insert _dout in DOut[]
    _imax++;
  }
  
  else Serial.println("Missing delay parameter");
  ListDelays();                              // List delays
  sei();                                     // Activate interrupt
}

void SaveDelays(){      //--- SAVE DELAYS TO EEPROM ---//
  cli();                                     // Stop interrupt
  int pos = _EEPROM_save_position;
  EEPROM_writeAnything(pos, _imax); pos +=2; // Can only write ints, not shorts!
  for (int j=0; j<_imax; j++){               // For all Delays
      EEPROM_writeAnything(pos, Delays[j]); pos+=2;  // Save delay value
      EEPROM_writeAnything(pos, DOut[j]); pos+=2;    // save DOut value
  }
  sei();                                     // Activate interrupt
}

void LoadDelays(){      //--- READ DELAYS FROM EEPROM ---//
  cli();                                     // Stop interrupt
  int pos = _EEPROM_save_position;
  EEPROM_readAnything(pos, _imax); pos +=2;  // Can only write ints, not shorts!
  for (int j=0; j<_imax; j++){               // For every delay
      EEPROM_readAnything(pos, Delays[j]); pos+=2;  // Save delay value
      EEPROM_readAnything(pos, DOut[j]); pos+=2;    // Save DOut values
  }
  sei();                                     // Activate interrupt
}

char *_itoa(char *_buf, unsigned long i, int len){
/* Converts an integer number into a right-aligned 
   string with the length len.
   Precision cannot exceed 5 decimal places.
   Length cannot exceed 15 bytes.
   Requires char array buffer _buf with sufficient length. (char buf[len+1];)
   */
//  static char _buf[16];                   // Alternative local buffer for char array
  char *ptr = _buf;                         // ptr is pointer to _buf, *ptr is value
  int j=0;                                  // Position in char array
  if (len > 15) len = 15;                   // Limit memory
  ltoa(i, ptr, 10);                         // Convert integer to string, base 10 (decimal)
  while (*ptr != '\0') {ptr++; j++;}        // Move to end of integer
  int k = j-1;                              // Length of number
  while (j++ < len) *ptr++ = ' ';           // Add spaces as specified by len (left alignment)
  *(ptr--) = '\0';                          // Terminate char array
  for (k; k>=0; k--) *ptr-- = _buf[k];      // Shift number to right side of _buf
  for (j; j>=0; j--) *ptr-- = ' ';          // Add spaces to the left of the number
  return _buf;
}

char *_ftoa(char *_buf, double f, int precision, int len){
/* Converts a floating point number into a right-aligned 
   string with the specified precision and length len.
   Precision cannot exceed 5 decimal places.
   Length cannot exceed 15 bytes.
   Requires char array buffer _buf with sufficient length. (char buf[len+1];)
>> easier way is to use char * dtostrf(double __val, signed char __width, unsigned char __prec, char * __s); <<
   */
//  static char _buf[16];                   // Alternative local buffer for char array
  char *ptr = _buf;                         // ptr is pointer to _buf, *ptr is value
  long p[] = {0,10,100,1000,10000,100000};  // Multiplier for float precision
  int j=0, k;                                  // Position in char array
  if (precision > 5) precision = 5;         // Limited precision in float conversion
  long intpart = (long)f;                   // Extract integer part
  itoa(intpart, ptr, 10);                   // Convert integer part to string
  while (*ptr != '\0') {ptr++; j++;}        // Move to end of integer part
  if (precision){
    *ptr++ = '.'; j++;                      // Add decimal point
    long fract = abs((long)((f-intpart) * p[precision]));  // Fractional part of number
    itoa(fract, ptr, 10);                   // Convert fractional part to string
    while (*ptr != '\0') {ptr++; j++;}      // Move to end of fractional part
    if (fract == 0){                        // Missing zeros 
      for (k=1; k<precision; k++) {*ptr++ = '0'; j++;} // Add the missing zeros
    } 
  }
  k = j-1;                                  // Length of number
  while (j++ < len) *ptr++ = ' ';           // Add spaces as specified by len (left alignment)
  *(ptr--) = '\0';                          // Terminate char array
  for (k; k>=0; k--) *ptr-- = _buf[k];      // Shift number to right side of _buf
  for (j; j>0; j--) *ptr-- = ' ';           // Add spaces to the left of the number
  return _buf;
}

char *_itob(char *_buf, int integer, int bits, int len){
/* Converts 8 bits from an integer into a right-aligned string 
   with length len and containing the selected bits as 0/1.
   Requires char array buffer _buf with sufficient length. (char buf[len+1];)
   E.g.: Serial.print(_itob(buf,PORTD,B11111000,10)); // Print status of D0-D7 pins; buf must be char array of appropriate size.
   */
//  static char _buf[16];                   // Alternative local buffer for char array
  int j=0, k;
  char *ptr = _buf;                         // ptr points to start of _buf
  *ptr++ = 'B';                             // Character B to indicate binary number
  j++;                                      // Position in char array
  for (k=7; k>=0; k--) {                    // Loop over bits
    if (bits & (1<<k)){                     // Use desired bit-mask
      if (integer & (1<<k)) *ptr++ = '1';   // Bit is true
      else *ptr++ = '0';                    // Bit is false
      j++;                                  // Position in char array
    }
  }
  k = j-1;                                  // Length of char array
  while (j++ < len) *ptr++ = ' ';           // Add spaces as specified by len (left alignment)
  *(ptr--) = '\0';                          // Terminate char array
  for (k; k>=0; k--) *ptr-- = _buf[k];      // Shift number to right side of _buf
  for (j; j>0; j--) *ptr-- = ' ';           // Add spaces to the left of the number
  return _buf;
}

char *_rtxt(char *_buf, char *txt, int len){
/* Converts a string into a right-aligned string with length len. 
   Requires char array buffer _buf with sufficient length. (char buf[len+1];)
   */
  int i,j=0;
  char *ptr = _buf;                         // ptr points to start of _buf
  char *_txt = txt;                         // _txt points to start of txt
  while (*_txt++ |= '\0') {j++;}            // Find end  of txt
  for (i=0; i<len-j; i++) *ptr++ = ' ';     // Fill left side of _buf with spaces
  for (i; i<len; i++) *ptr++ = *txt++;      // Fill right side of _buf with txt
  *ptr = '\0';                              // Terminate char array
  return _buf;
}

float TicksToMicros(int ticks){             //--- CONVERT TIMER TICKS TO MICROSECONDS ---//
  return (float)(ticks + TimerOffset) * 1000.0/clock;
}

unsigned long MicrosToTicks(float micro){   //--- CONVERT MICROSECONDS TO TIMER TICKS ---//
  return (unsigned long)(micro * clock/1000.0 +0.5) - TimerOffset;
}

void IDN() {               // --- QUERY DEVICE IDENTIFICATION --- //
  Serial.print("ID: ");
  Serial.print(ID);
  Serial.print(" + ");
  Serial.println(ProgString);
}

void Help() {             // --- PRINT LIST OF COMMANDS --- //
  IDN();
  Serial.println("Delay pins: D7 - D3");
  Serial.println();
  Serial.println("Command list (* indicates argument type)");    // Instructions
  Serial.println("List:            List delays");
  Serial.println("A *float *int:   Add delay at *float microsec. with *int Dout values");
  Serial.println("A *float B*bool: Add delay at *float microsec. with boolean Dout values");
  Serial.println("A *float:        Add delay at *float microsec. with inverted Dout values");
  Serial.println("C *char:         Clear delay number *char");
  Serial.println("C:               Clear all delays");
  Serial.println("Save:            Save delays");
  Serial.println("Load:            Load delays");
  Serial.println("Help:            Print command list"); 
  Serial.println("ID:              Print device ID");
  Serial.println();
}

void unrecognized(const char *command) {    // --- NONEXISTING COMMAND --- //
  Serial.print("I don't understand the command '");
  Serial.print(command);
  Serial.println("'");
}

