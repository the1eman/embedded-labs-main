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

#define FOSC 16000000 // Clock Speed
#define FCPU 16000000UL // Clock Speed
#define BAUD 9600
#define dac_addr 0b01011000

#define MYUBRR FOSC/16/BAUD-1
#define Dev24C02  0xA2      // device address of EEPROM 24C02, see datasheet


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
//#include "wiring_private.h"
#include "WString.h"

#include "TinyGPSPlus.h"
#include "hooks.c"
#include "wiring.c"
#include "WInterrupts.c"
#include "wiring_analog.c"
#include "wiring_digital.c"
//#include "wiring_pulse.c"
//#include "wiring_shift.c"

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
#define envelope_pin     PINC                   // envelope pin
#define envelope_bit      PINC4
#define envelope_ddr      DDRC

#define gate_pin	  PORTD                   // gate pin
#define gate_bit      PIND2
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
float current_location[2] = {41.6589,-91.5364};
float allowed_volume[4]={60,60,60,60};

TinyGPSPlus gps;


//some AVR definitions

/*void sbi_led(void){
	push_button_LED |= (1<<push_button_LED_bit);
	}
	
void cbi_led(void){
	push_button_LED &= ~(1<<(push_button_LED_bit));
}*/
	
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
void initADC(void){
	sei();
	//ADMUX = 0b11100000;
	//ADCSRA = 0b10001100;
	//ADCSRA = ADCSRA | (1<< ADSC);
	ADMUX |= (1<<REFS0);
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
	cli();
}

uint16_t adc_read(uint8_t adcx) {
	ADCSRA |= (1<<ADSC);
	while ( (ADCSRA & (1<<ADSC)) );
	return ADC;
}

uint16_t ReceiveADC(void){
	ADCSRA |= (1<<ADSC);
	while ( (ADCSRA & (1<<ADSC)) );
	return ADC;
}
void USART_Init(unsigned int ubrr)
{
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char) ubrr;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C |= (1<<USBS0)|(1<<UCSZ00);
}
void USART_Transmit( unsigned char data )
{
	while ( !( UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}
unsigned char USART_Receive( void )
{
	while ( !(UCSR0A & (1<<RXC0)) );
	return UDR0;
}


long my_map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float soundISR(){
	
	  _delay_ms(700);
	  int pin_val=gate_pin;

	  //pin_val = digitalRead(gate_pin);
	  //digitalWrite(push_button_LED, pin_val);
	  //pinMode(push_button_LED, OUTPUT);
	  //pinMode(gate_pin, INPUT);
	  
	  attachInterrupt(0, soundISR, CHANGE);
	  float value;
	  float sample;
	  value = analogRead(envelope_bit);
	  value=(value/1023.0)*5.0;
	  
	  float f; //= 10*log(value);//(200.0)*value;
	  	  
  	 value = analogRead(envelope_bit);
	 f=value;
	 if(f>200){
		 f=f*0.4;
	 }else if(f>100){
	 f=f*0.75;
	}else if(f<17){
		 f=f*1.2;
	 }
	 return f;
	  
	  /*
	  
	  //
	  
	     //float peakToPeak = 0;                                  // peak-to-peak level
	     
	     unsigned int signalMax = 0;                            //minimum value
	     unsigned int signalMin = 1024;                         //maximum value
	     int timing=0;
	     // collect data for 50 mS
	     while (timing<50)
	     {
		     sample = analogRead(envelope_bit);                             //get reading from microphone
		     if (sample < 1024)                                  // toss out spurious readings
		     {
			     if (sample > signalMax)
			     {
				     signalMax = sample;                           // save just the max levels
			     }
			     else if (sample < signalMin)
			     {
				     signalMin = sample;                           // save just the min levels
			     }
		     }
			 timing++;
			 _delay_ms(1);
	     }
		 
	  float peakToPeak = signalMax - signalMin;                    // max - min = peak-peak amplitude
	  //float f = my_map(peakToPeak,20,900,30.5,80);             //calibrate for deciBels
	  */

	  //return 1.0;
	  //return value;
	
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
	if(n<0.0){
		n=abs(n);
	}
	
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
	char str[]="-";
	
	//if(n<0.0){
		//for (int i=0;i<(strlen(res)-1);i++)
		//{
			//str[i+1]=res[i];
		//}
	//}
	//strcpy(res,str);
}
//**************************************************************************************
void button_wait(){
	if(SBIS_button()){
		mode=2;
		_delay_ms(100); 
		// Clear Display instruction
		lcd_write_instruction_4d(lcd_Clear);             // clear display RAM
		_delay_ms(4);                                   // 1.64 mS delay (min)

		// ; Entry Mode Set instruction
		lcd_write_instruction_4d(lcd_EntryMode);         // set desired shift characteristics
		_delay_us(80);                                  // 40uS delay (min)
			
		// Display On/Off Control instruction
		lcd_write_instruction_4d(lcd_DisplayOn);         // turn the display ON
		_delay_us(80);

		}else{
		mode=1;	
	}
	
	//if(SBIS_button()){ LED_toggle();}
		//else{ //sbi_led(); 
		//}	
}


int main(void)
{
	
	USART_Init(MYUBRR);
	initADC();


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
			
			_delay_ms(100);
			// Clear Display instruction
			lcd_write_instruction_4d(lcd_Clear);             // clear display RAM
			_delay_ms(4);                                   // 1.64 mS delay (min)

			// ; Entry Mode Set instruction
			lcd_write_instruction_4d(lcd_EntryMode);         // set desired shift characteristics
			_delay_us(80);                                  // 40uS delay (min)
				
			// Display On/Off Control instruction
			lcd_write_instruction_4d(lcd_DisplayOn);         // turn the display ON
			_delay_us(80);
	
			lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
			lcd_write_string_4d(" ");
			lcd_write_string_4d(buf1);
			lcd_write_string_4d("dB");
			lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
			_delay_us(80);
			
			lcd_write_string_4d("Lvl:");
			if(dB<60.0){
				lcd_write_string_4d("Safe |");
			}else if(dB<85.0){
				lcd_write_string_4d("Loud |");
			}else if(dB<110.0){
				lcd_write_string_4d("VLoud|");
			}else if(dB>=140.0){
				lcd_write_string_4d("Pain |");
			}
			
			
				
			if(dB<allowed_volume[0]){
				lcd_write_string_4d("OK");
				}else{
				lcd_write_string_4d("NotOK");
				
			}
			
			_delay_us(100);
			button_wait();

			}else if(mode==2){ //gps mode
				
			current_location[1]=(gps.location.lat(), 4);
			current_location[1]=(gps.location.lng(), 4);
			
			
			char buf2[100];
			ftoa(current_location[0], buf2, 4);
			
			lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
			lcd_write_string_4d(" La:");
			lcd_write_string_4d(buf2);
			
			
			ftoa(current_location[1], buf2, 4);
			
			
			lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
			lcd_write_string_4d(" Lo:");
			
			if(current_location[1]<0.0){
				lcd_write_string_4d("-");
			}
			lcd_write_string_4d(buf2);
			
			_delay_us(100);
			button_wait();
			//detector_wait();
			_delay_us(1000);
		}
		
    }
}

