/*
 * Lab5.c
 * Created: 4/12/2022 10:35:34 AM
 * Author : eakpan
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL //16MHz
#endif

#include<avr/io.h>
#include<util/delay.h>
#include<stdlib.h>
#include<stdio.h>

#include <avr/io.h>
#include <string.h>
#include<math.h>
#include <stdio.h>
#include <stdlib.h>
#define FOSC 16000000 // Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1
//be sure to select no line ending

//------------------------------------------ADC functions
//-------------------------------------------------------
void initADC(void){
	// Select v ref=AVcc
	ADMUX |= (1<<REFS0);
	//set pre-scaler to 128 and enable ADC
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
}

int readADC(float v){
	int n=0;
	return n;
}

uint16_t adc_read(uint8_t adcx) {
	//ADMUX	&=	0xf0;
	//ADMUX	|=	adcx;
	/* This starts the conversion. */
	ADCSRA |= (1<<ADSC);

	while ( (ADCSRA & (1<<ADSC)) );
	/* Finally, we return the converted value to the calling function. */
	//return Number2String(ADC);
	//printf(ADC);
	return ADC;
}

uint16_t ReceiveADC(void){
	//ADMUX = (1<<REFS0);
	ADCSRA |= (1<<ADSC);
	while ( (ADCSRA & (1<<ADSC)) );
	return ADC;
}


//------------------------------------------Analog Transmission Functions
//-------------------------------------------------------
void USART_Init(unsigned int ubrr)
{
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char) ubrr;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C |= (1<<USBS0)|(1<<UCSZ00);
}
void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)));
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) );
	/* Get and return received data from buffer */

	return UDR0;
}

//------------------------------------------my functions
//-------------------------------------------------------


//no arguments, The ATmega328P ADC must be used in 10-bit mode!

	
void write_string(char arr[]){
	int i=0;
	while(arr[i]!= NULL){
		USART_Transmit(arr[i]);
		i++;
	}
}

//------------------------------------------main method, below all
//---------------------------------------------------------------
int main(void)
{
	//unsigned char output_c;
	//ReceiveADC();
	USART_Init(MYUBRR);
	initADC();
	
	char arr2[20];
	float num;
	float num2;
	char answer;//=UDR0;
	//char answer_str[20];
	
	//input strings, used for later calculations
	char m_string1[20]="";
	char m_string2[20]="";
	char m_string3[20]="";
	
	char s_string1[20]="";
	char s_string2[20]="";
	char s_string3[20]="";
	
	//integers used to iterate through strings
	int j=0;
	int k=0;
	int p=0;
	//int counter=0;
	
	//specific numbers used for calculations
	int n, dt, c;
	float v;
	
	while (1)
	{
		//determine the voltage
		num = ReceiveADC(); //initial voltage
		num=(num/1023.0)*5.0; //voltage on 5 scale
		
		uint16_t temp = ReceiveADC(); //integer form of initial voltage
		temp=(temp/1023.0)*5.0;	//integer of initial voltage on 5 scale
		
		num2=((num-((float)temp))*1000); //used to obtain floating portion of the 5 scale voltage

		answer=USART_Receive(); //wait for an input
		USART_Transmit(answer);
		
		switch(answer){
			case 'G': //when input is G, display the voltage as a floating decimal
			case 'g':
				write_string("\n");
				write_string("v=");
				write_string(itoa(num,arr2,10));
				write_string(".");
				write_string(itoa(num2,arr2,10));
				write_string(" V \n");
				break;
					
			case 'M': //when input is M, take inputs n and dt. N is the number of measurements performed and dt is the time interval between them.
			case 'm':
				//initialize strings that will hold input values, later to become integers.
				m_string1[10]="";
				m_string2[10]="";
				m_string3[10]="";
				m_string1[0]=answer;
				j=1;
			
				while((answer!=NULL && answer!='\n')){	//turn input line into a string
					answer=USART_Receive();
					m_string1[j]=answer;
					j++;
				}
				write_string("\n");
				write_string(m_string1);
				
				if(m_string1[1]==','){ //turn the inputs into two separate strings, representing n and dt respectively
					k=2;
					while(m_string1[k]!=','){
						m_string2[k-2]=m_string1[k];
						k++;	
					}
					k++;
					p=k;
				
					while(m_string1[k]!='\n'){
						m_string3[k-p]=m_string1[k];
						k++;
					}	
				}else{
					write_string("No command \n");
					break;
				}
			
				n=atoi(m_string2); //turn the strings into integers
				dt=atoi(m_string3);
				write_string("\n");

				if( ((n>=2)&&(n<=20)) &&((dt>=1)&&(dt<=10)) ){//make sure inputs are in the ranges: 2<=n<=20, 1<=dt<=10
					for(int i1=0;i1<(n);i1++){ 
						arr2[5]="";//display the current time
						write_string("t=");
						write_string(itoa((i1*dt),arr2,10));
						write_string("s, ");
					
						arr2[10]=""; //perform the voltage retrieval calculation
						num = ReceiveADC();
						num=(num/1023.0)*5.0;
						uint16_t temp = ReceiveADC();
						temp=(temp/1023.0)*5.0;
						num2=((num-((float)temp))*1000);
						
						write_string("v="); //display the voltage as a float
						write_string(itoa(num,arr2,10));
						write_string(".");
						write_string(itoa(num2,arr2,10));
						write_string("V \n");
					
						for(int i2=0;i2<(dt);i2++){ //call the 1 second delay 'dt' times
							_delay_ms(1000);
						}
					}	
				}else{ //if one of the inputs isn't within the above range, perform no calculations.
					write_string("Bad bounds. \n");
				}
				//clear left over values
				m_string1[10]="";
				m_string2[10]="";
				m_string3[10]="";
				break;
					
			case 'S':
			case 's':
				//initialize strings that will hold input values, later to become integers.
				s_string1[10]="";
				s_string2[10]="";
				s_string3[10]="";
				s_string1[0]=answer;
				j=1;
				
				while((answer!='\0' && answer!='\n')){	//turn input line into a string
					answer=USART_Receive();
					s_string1[j]=answer;
					j++;
				}
				write_string("\n");
				write_string(s_string1);
				
				if(s_string1[1]==','){ //turn the inputs into two separate strings, representing c and v respectively
					k=2;
					while(s_string1[k]!=','){
						s_string2[k-2]=s_string1[k];
						k++;
					}
					k++;
					p=k;
					while(s_string1[k]!='\n'){
						s_string3[k-p]=s_string1[k];
						k++;
					}
				}else{
					write_string("No command \n");
					break;
			}
				
				c=atoi(s_string2); //turn the strings into integers
				v=atof(s_string3);
				write_string("\n");
				
				
				//clear values
				if((c!=1)&&(c!=0)){
					write_string("Bad value \n");
					break;
				}
				
				write_string("DAC channel ");
				write_string(s_string2);
				write_string(" set to ");
				write_string("_voltage here_");
				write_string("V (");
				write_string("_d here_");
				write_string("d)\n");
				
				s_string1[10]="";
				s_string2[10]="";
				s_string3[10]="";
				
				break;
			
			default:
				// do nothing	
				break;	
		
		}	
	}
}

//tmp=PORTC;
//asm volatile( “sei” ”\n\t”::);
	/*
		write_string("V: ");
		write_string(itoa(num,arr2,10));
		write_string(".");
		write_string(itoa(num2,arr2,10));
		write_string(" V \n");*/
