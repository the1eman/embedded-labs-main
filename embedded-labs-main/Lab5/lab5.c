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
	//sprintf(str,"v = %.3f V\n",v);
	
	char arr2[20];
	float num;
	float num2;
	char answer;//=UDR0;
	char answer_str[20];
	
	char m_string1[20]="";
	char m_string2[20]="";
	char m_string3[20]="";
	
	char s_string1[20]="";
	char s_string2[20]="";
	char s_string3[20]="";
	
	int j=0;
	int k=0;
	int p=0;
	
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
				//write_string("Choice is g \n");
				write_string("\n");
				write_string("v=");
				write_string(itoa(num,arr2,10));
				write_string(".");
				write_string(itoa(num2,arr2,10));
				write_string(" V \n");
				break;
					
			case 'M':
			case 'm':
			m_string1[10]="";
			m_string2[10]="";
			m_string3[10]="";
			j=0;
			
			while((answer!=NULL && answer!='\n')){
				answer=USART_Receive();
				m_string1[j]=answer;
				j++;
			}write_string(m_string1);
			//m,23,56
			if(m_string1[1]==','){
				k=2;
				while(m_string1[k]!=','){
					m_string2[k-2]=m_string1[k];
					k++;	
				}
				p=k;
				while(m_string1[k]!='\n'){
					m_string3[k-p]=m_string1[k];
					k++;
				}	
			}
			write_string(m_string2);
			write_string(m_string3);
		

				break;
					
			case 'S':
			case 's':
				break;
			
			default:
			// do nothing	
				break;	
		
		}	//write_string("\n");	
	}
}

//_delay_ms(1000);
//tmp=PORTC;
//asm volatile( “sei” ”\n\t”::);
	/*
		write_string("V: ");
		write_string(itoa(num,arr2,10));
		write_string(".");
		write_string(itoa(num2,arr2,10));
		write_string(" V \n");*/
