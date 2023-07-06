/*
 * es_project.c
 *
 * Created: 5/2/2022 10:37:15 PM
 * Author : Emmanuel A
 */ 
//r3->RXD
//r2->TXD

#define F_CPU 16000000UL


#define SREG _SFR_IO8(0x3F)
#  define SREG_C  (0)
#  define SREG_Z  (1)
#  define SREG_N  (2)
#  define SREG_V  (3)
#  define SREG_S  (4)
#  define SREG_H  (5)
#  define SREG_T  (6)
#  define SREG_I  (7)

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "Arduino.h"
#include "binary.h"
//#include "Client.h"
//#include "HardwareSerial.h"
//#include "IPAddress.h"
//#include "new.h"
#include "PluggableUSB.h"
//#include "Print.h"
//#include "Printable.h"
//#include "Server.h"
//#include "Stream.h"
//#include "Udp.h"
#include "USBAPI.h"
#include "USBCore.h"
#include "USBDesc.h"
#include "WCharacter.h"
#include "wiring_private.h"
#include "WString.h"

//#include "TinyGPSPlus.h"
#include "hooks.c"
#include "wiring.c"
#include "WInterrupts.c"
#include "wiring_analog.c"
#include "wiring_digital.c"
#include "wiring_pulse.c"
#include "wiring_shift.c"



int mode=1; //mode 1 will be the microphone, mode 2 will be GPS
float dB=0.0;
//LCD ports/pins
#define lcd_D4_port     PORTC                   // lcd D4 connection
#define lcd_D4_bit      PORTC0
#define lcd_D4_ddr      DDRC

#define lcd_D5_port     PORTC                   // lcd D5 connection
#define lcd_D5_bit      PORTC1
#define lcd_D5_ddr      DDRC

#define lcd_D6_port     PORTC                  // lcd D6 connection
#define lcd_D6_bit      PORTC2
#define lcd_D6_ddr      DDRC

#define lcd_D7_port     PORTC                   // lcd D7 connection
#define lcd_D7_bit      PORTC3
#define lcd_D7_ddr      DDRC

#define lcd_E_port      PORTB                   // lcd Enable pin
#define lcd_E_bit       PORTB1
#define lcd_E_ddr       DDRB

#define lcd_RS_port     PORTB                   // lcd Register Select pin
#define lcd_RS_bit      PORTB0
#define lcd_RS_ddr      DDRB

//Sound detector pins/ports
#define envelope_port     PORTC                   // envelope pin
#define envelope_bit      PORTC4
#define envelope_ddr      DDRC

#define gate_port     PORTD                   // gate pin
#define gate_bit      PORTD2
#define gate_ddr      DDRD

//GPS pins/ports
#define gps_rxd_port     PORTD                   //R3/RXD Pin
#define gps_rxd_bit      PORTD3
#define gps_rxd_ddr      DDRD

#define gps_txd_port     PORTD                   //"R2"/TXD Pin
#define gps_txd_bit      PORTD4
#define gps_txd_ddr      DDRD

//push button ports/pins
#define push_button_pin     PORTD                   // button pin
#define push_button_bit     PIND0
#define push_button_ddr     DDRD

#define push_button_LED			PORTD               //led port (for testing)
#define push_button_LED_bit		PORTD1
#define push_button_LED_ddr     DDRD

// LCD module information
#define lcd_LineOne     0x00                    // start of line 1
#define lcd_LineTwo     0x40

// LCD instructions
#define lcd_Clear           0b00000001          // replace all characters with ASCII 'space'
#define lcd_Home            0b00000010          // return cursor to first position on first line
#define lcd_EntryMode       0b00000110          // shift cursor from left to right on read/write
#define lcd_DisplayOff      0b00001000          // turn display off
#define lcd_DisplayOn       0b00001100          // display on, cursor off, don't blink character
#define lcd_FunctionReset   0b00110000          // reset the LCD
#define lcd_FunctionSet4bit 0b00101000          // 4-bit data, 2-line display, 5 x 7 font
#define lcd_SetCursor       0b10000000          // set cursor position


float locations[4][4] = {{1.1,1.1},	{2.2, 2.2} };
float current_location[2] = {2.0000,1.0000};

//some AVR definitions
void sbi_led(void){
	push_button_LED |= (1<<push_button_LED_bit);
	}
	
void cbi_led(void){
	push_button_LED &= ~(1<<(push_button_LED_bit));
}
	
void LED_toggle(){
	push_button_LED ^= (1<<push_button_LED_bit);
	
	if(mode==1){
		mode=2;	
	}else if(mode==2){
		mode=1;
	}
	
	// Function Set instruction
	lcd_write_instruction_4d(lcd_FunctionSet4bit);   // set mode, lines, and font
	_delay_us(80);                                  // 40uS delay (min)

	// Display On/Off Control instruction
	lcd_write_instruction_4d(lcd_DisplayOff);        // turn display OFF
	_delay_us(80);                                  // 40uS delay (min)

	// Clear Display instruction
	lcd_write_instruction_4d(lcd_Clear);             // clear display RAM
	_delay_ms(4);                                   // 1.64 mS delay (min)

	// ; Entry Mode Set instruction
	lcd_write_instruction_4d(lcd_EntryMode);         // set desired shift characteristics
	_delay_us(80);                                  // 40uS delay (min)
		
	// Display On/Off Control instruction
	lcd_write_instruction_4d(lcd_DisplayOn);         // turn the display ON
	_delay_us(80);

}

bool SBIS_button(void){ //sbis=Skip if Bit is Set="if the 'button' is pressed, do the next line"
		//if (PIND & (1<<2))    // Test if bit set
		//{
			//your function
		//}
		if ((PIND & (1<<PIND0)) == 0){
			return true;
		}else return false;
}



//some arduino commands




float soundISR(){
	  int pin_val;

	  pin_val = digitalRead(gate_port);
	  digitalWrite(push_button_LED, pin_val);
	  
	  attachInterrupt(0, soundISR, CHANGE);
	  
	  float value;
	  value = analogRead(envelope_port);
	  float f = (26.0/38.0)*value;
	  return f;
	
}


//code courtesy of Donald Weiman  (weimandn@alfredstate.edu)
//***********************************************************************************
void lcd_write_4(uint8_t theByte)
{
	lcd_D7_port &= ~(1<<lcd_D7_bit);                        // assume that data is '0'
	if (theByte & 1<<7) lcd_D7_port |= (1<<lcd_D7_bit);     // make data = '1' if necessary

	lcd_D6_port &= ~(1<<lcd_D6_bit);                        // repeat for each data bit
	if (theByte & 1<<6) lcd_D6_port |= (1<<lcd_D6_bit);

	lcd_D5_port &= ~(1<<lcd_D5_bit);
	if (theByte & 1<<5) lcd_D5_port |= (1<<lcd_D5_bit);

	lcd_D4_port &= ~(1<<lcd_D4_bit);
	if (theByte & 1<<4) lcd_D4_port |= (1<<lcd_D4_bit);

	// write the data
	// 'Address set-up time' (40 nS)
	lcd_E_port |= (1<<lcd_E_bit);                   // Enable pin high
	_delay_us(1);                                   // implement 'Data set-up time' (80 nS) and 'Enable pulse width' (230 nS)
	lcd_E_port &= ~(1<<lcd_E_bit);                  // Enable pin low
	_delay_us(1);                                   // implement 'Data hold time' (10 nS) and 'Enable cycle time' (500 nS)
}

void lcd_write_instruction_4d(uint8_t theInstruction)
{
	lcd_RS_port &= ~(1<<lcd_RS_bit);                // select the Instruction Register (RS low)
	lcd_E_port &= ~(1<<lcd_E_bit);                  // make sure E is initially low
	lcd_write_4(theInstruction);                    // write the upper 4-bits of the data
	lcd_write_4(theInstruction << 4);               // write the lower 4-bits of the data
}

void lcd_write_character_4d(uint8_t theData)
{
	lcd_RS_port |= (1<<lcd_RS_bit);                 // select the Data Register (RS high)
	lcd_E_port &= ~(1<<lcd_E_bit);                  // make sure E is initially low
	lcd_write_4(theData);                           // write the upper 4-bits of the data
	lcd_write_4(theData << 4);                      // write the lower 4-bits of the data
}

void lcd_write_string_4d(uint8_t theString[])
{
	volatile int i = 0;                             // character counter*/
	while (theString[i] != 0)
	{
	lcd_write_character_4d(theString[i]);
	i++;
	_delay_us(80);                              // 40 uS delay (min)
	}
}

void lcd_init_4d(void)
{
	//lcd_D4_ddr |= (1<<lcd_D4_bit);//set RS
	//_delay_ms(100);				  //1) wait 100ms
	// Power-up delay
	_delay_ms(100);                                 // initial 40 mSec delay

	// Set up the RS and E lines for the 'lcd_write_4' subroutine.
	lcd_RS_port &= ~(1<<lcd_RS_bit);                // select the Instruction Register (RS low)
	lcd_E_port &= ~(1<<lcd_E_bit);                  // make sure E is initially low

	// Reset the LCD controller
	lcd_write_4(lcd_FunctionReset);                 // first part of reset sequence
	_delay_ms(10);                                  // 4.1 mS delay (min)

	lcd_write_4(lcd_FunctionReset);                 // second part of reset sequence
	_delay_us(200);                                 // 100uS delay (min)

	lcd_write_4(lcd_FunctionReset);                 // third part of reset sequence
	_delay_us(200);                                 // this delay is omitted in the data sheet
	
	lcd_write_4(lcd_FunctionSet4bit);               // set 4-bit mode
	_delay_us(80);                                  // 40uS delay (min)

	// Function Set instruction
	lcd_write_instruction_4d(lcd_FunctionSet4bit);   // set mode, lines, and font
	_delay_us(80);                                  // 40uS delay (min)

	// Display On/Off Control instruction
	lcd_write_instruction_4d(lcd_DisplayOff);        // turn display OFF
	_delay_us(80);                                  // 40uS delay (min)

	// Clear Display instruction
	lcd_write_instruction_4d(lcd_Clear);             // clear display RAM
	_delay_ms(4);                                   // 1.64 mS delay (min)

	// ; Entry Mode Set instruction
	lcd_write_instruction_4d(lcd_EntryMode);         // set desired shift characteristics
	_delay_us(80);                                  // 40uS delay (min)
	
	// Display On/Off Control instruction
	lcd_write_instruction_4d(lcd_DisplayOn);         // turn the display ON
	_delay_us(80);
	                                  
}
//***********************************************************************************
//code courtesy of GeeksForGeeks

void reverse(char* str, int len)
{
	int i = 0, j = len - 1, temp;
	while (i < j) {
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++;
		j--;
	}
}


int intToStr(int x, char str[], int d)
{
	int i = 0;
	while (x) {
		str[i++] = (x % 10) + '0';
		x = x / 10;
	}
	
	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d)
	str[i++] = '0';
	
	reverse(str, i);
	str[i] = '\0';
	return i;
}

// Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint)
{
	// Extract integer part
	int ipart = (int)n;
	
	// Extract floating part
	float fpart = n - (float)ipart;
	
	// convert integer part to string
	int i = intToStr(ipart, res, 0);
	
	// check for display option after point
	if (afterpoint != 0) {
		res[i] = '.'; // add dot
		
		fpart = fpart * pow(10, afterpoint);
		
		intToStr((int)fpart, res + i + 1, afterpoint);
	}
}
//**************************************************************************************
void button_wait(){
	if(SBIS_button()){ LED_toggle();}
		else{ //sbi_led(); 
		}	
}


int main(void)
{
	// configure the microprocessor pins for the data lines
	lcd_D7_ddr |= (1<<lcd_D7_bit);                  // 4 data lines - output
	lcd_D6_ddr |= (1<<lcd_D6_bit);
	lcd_D5_ddr |= (1<<lcd_D5_bit);
	lcd_D4_ddr |= (1<<lcd_D4_bit);

	// configure the microprocessor pins for the control lines
	lcd_E_ddr |= (1<<lcd_E_bit);                    // E line - output
	lcd_RS_ddr |= (1<<lcd_RS_bit);                  // RS line - output
	
	lcd_init_4d();
	lcd_write_string_4d("Welcome!");	//uint8_t program_author[]   = "Donald";
	_delay_ms(2000);
	lcd_init_4d();
	
    /* Replace with your application code */
    while (1) 
    {
		
			
		if(mode==1){ //decibel-meter mode		
			_delay_us(1000);
			dB=soundISR();
			char buf1[100];
			ftoa(dB, buf1, 3);
			
			lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
			lcd_write_string_4d(" ");
			lcd_write_string_4d(buf1);
			lcd_write_string_4d("dB");
			lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
			_delay_us(80);
			
			lcd_write_string_4d("Lvl:");
			if(dB<60.0){
				lcd_write_string_4d("Safe");
			}else if(dB<85.0){
				lcd_write_string_4d("Loud");
			}else if(dB<110.0){
				lcd_write_string_4d("VLoud");
			}else if(dB>=140.0){
				lcd_write_string_4d("Pain");
			}
			
			_delay_us(100);
			button_wait();
			//detector_wait();
			_delay_us(1000);

			}else if(mode==2){ //gps mode
				
			char buf2[100];
			ftoa(current_location[0], buf2, 6);
			
			lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
			lcd_write_string_4d(" La:");
			lcd_write_string_4d(buf2);
			
			
			ftoa(current_location[1], buf2, 6);
			
			lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
			lcd_write_string_4d(" Lo:");
			lcd_write_string_4d(buf2);
			
			_delay_us(100);
			button_wait();
			//detector_wait();
			_delay_us(1000);
		}
		
    }
}

