/*
    6-1-08
    Copyright Spark Fun Electronics© 2007
    Viliam Klein
    viliam at sparkfun.com
    
    Example I2C Interface to MCP4725 DAQ IC
*/

#include <stdio.h>
#include <avr/io.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "types.h"
#include "defs.h"
#include "i2c.h"

#define FOSC 8000000
#define BAUD 9600
#define MYUBRR 103//(((((FOSC * 10) / (16L * BAUD)) + 5) / 10) - 1)
//#define MYUBRR 6

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

#define STATUS_LED 5

#define Vout 3000//binary value going into register, 0-4095

#define SLA_W 0xC0//write address
#define SLA_R 0xC1//read address

//Define functions
//======================
void i2cSendStart(void);
void i2cSendStop(void);
void i2cWaitForComplete(void);
void i2cSendByte(unsigned char data);
void i2cInit(void);
void i2cHz(long uP_F, long scl_F);





void ioinit(void);      // initializes IO
static int uart_putchar(char c, FILE *stream);
uint8_t uart_getchar(void);

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void delay_ms(uint16_t x); // general purpose delay
//======================

int main (void)
{
	uint8_t x = 0;
	
	//ADC result vars
	long l;//low register
	long h;//high register
	
	long Vref = 5120;//reference V in mV
		
	
    ioinit(); //Setup IO pins and defaults

    /*while(x < 10)
    {
		x++;
	
		printf("Test it! x = %d\n", x);
		
		sbi(PORTC, STATUS_LED);
		delay_ms(50);

		cbi(PORTC, STATUS_LED);
		delay_ms(50);
    }*/
	
		
	//set control bytes
	char lVout = Vout;
	char hVout = (Vout>>8) & 0x0F;
	
	
	
	
	//initilize I2C hardware
	TWCR = 0x00;
	TWBR = 8;
	cbi(TWCR, TWEA);	
	sbi(TWCR, TWEN);
	printf("%x\n", TWCR);
	
	
	//Send start condition 
	i2cSendStart();	
    i2cWaitForComplete();
	
	
	// send slave device address with write
	i2cSendByte(SLA_W);	
	i2cWaitForComplete();	
	
	
	// send first byte to MCP
	TWDR = hVout;
	// begin send
	TWCR = (1<<TWINT)|(1<<TWEN);	
	i2cWaitForComplete();	
	
	
	
	// send second byte to MCP
	TWDR = lVout;
	// begin send
	TWCR = (1<<TWINT)|(1<<TWEN);	
	i2cWaitForComplete();
	
	
	//send stop condition
	i2cSendStop();
	
	TWCR = 0x00;//stop I2C
	
	//adc conversion
	
	
	while(1)
	{
	
		
		
		//ADCSRA = (1 << ADEN) | (1 << ADSC);
		//ADCSRA = (1 << ADEN) | (1 << ADSC);
		//ADCSRA = (1 << ADEN) | (1 << ADSC);
		//ADCSRA = (1 << ADEN) | (1 << ADSC);
		ADCSRA = (1 << ADEN) | (1 << ADSC);
		
		l = ADCL;
		h = ADCH & 0x03;
		h = h << 8;
		h = ((h + l)*Vref)/1024;
		
		printf("adc is: %d \n", h);
		
		delay_ms(1000);
	
	}
	
   
   
    return(0);
}

void ioinit (void)
{
    //1 = output, 0 = input
    DDRB = 0b11101111; //PB4 = MISO 
    DDRC = 0b11111110; //
    DDRD = 0b11111110; //PORTD (RX on PD0)

    //USART Baud rate: 9600
    UBRR0H = (MYUBRR >> 8);
    UBRR0L = MYUBRR;
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0A = (1<<U2X0);
    
    stdout = &mystdout; //Required for printf init
}

static int uart_putchar(char c, FILE *stream)
{
    if (c == '\n') uart_putchar('\r', stream);
  
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    
    return 0;
}

uint8_t uart_getchar(void)
{
    while( !(UCSR0A & (1<<RXC0)) );
    return(UDR0);
}

//General short delays
void delay_ms(uint16_t x)
{
  uint8_t y, z;
  for ( ; x > 0 ; x--){
    for ( y = 0 ; y < 80 ; y++){
      for ( z = 0 ; z < 40 ; z++){
        asm volatile ("nop");
      }
    }
  }
}


//==========================//
//
//
//
//
//
//
//
//
//I2C functions
//
//
//
//
//
//
//
//==========================//

void i2cInit(void)
{
	// set pull-up resistors on I2C bus pins
	//sbi(PORTC, 0);	// i2c SCL on ATmega163,323,16,32,etc
	//sbi(PORTC, 1);	// i2c SDA on ATmega163,323,16,32,etc
	//sbi(PORTD, 0);	// i2c SCL on ATmega128,64
	//sbi(PORTD, 1);	// i2c SDA on ATmega128,64

	// set i2c bit rate to 40KHz
	//i2cSetBitrate(100);
	// enable TWI (two-wire interface)
	sbi(TWCR, TWEN);
}

void i2cSetBitrate(unsigned short bitrateKHz)
{
	unsigned char bitrate_div;
	// set i2c bitrate
	// SCL freq = F_CPU/(16+2*TWBR))
	//#ifdef TWPS0
		// for processors with additional bitrate division (mega128)
		// SCL freq = F_CPU/(16+2*TWBR*4^TWPS)
		// set TWPS to zero
		cbi(TWSR, TWPS0);
		cbi(TWSR, TWPS1);
	//#endif
	// calculate bitrate division	
	bitrate_div = ((F_CPU/1000l)/bitrateKHz);
	if(bitrate_div >= 16)
		bitrate_div = (bitrate_div-16)/2;
	outb(TWBR, bitrate_div);
}

void i2cSendStart(void)
{
	// send start condition
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
}

void i2cSendStop(void)
{
	// transmit stop condition
        TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

void i2cWaitForComplete(void)
{
	// wait for i2c interface to complete operation
        while (!(TWCR & (1<<TWINT)));
}

void i2cSendByte(unsigned char data)
{
	// save data to the TWDR
	TWDR = data;
	// begin send
	TWCR = (1<<TWINT)|(1<<TWEN);
}

void i2cReceiveByte(unsigned char ackFlag)
{
	// begin receive over i2c
	if( ackFlag )
	{
		// ackFlag = TRUE: ACK the recevied data
		outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT)|BV(TWEA));
	}
	else
	{
		// ackFlag = FALSE: NACK the recevied data
		outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT));
	}
}

unsigned char i2cGetReceivedByte(void)
{
	// retieve received data byte from i2c TWDR
	return( inb(TWDR) );
}

unsigned char i2cGetStatus(void)
{
	// retieve current i2c status from i2c TWSR
	return( inb(TWSR) );
}



unsigned char i2cMasterSendNI(unsigned char deviceAddr, unsigned char length, unsigned char* data)
{
	unsigned char retval = I2C_OK;
	// send start condition
	i2cSendStart();
        i2cWaitForComplete();
	// send device address with write
	i2cSendByte( deviceAddr & 0xFE );
	i2cWaitForComplete();
	// check if device is present and live
	if( inb(TWSR) == TW_MT_SLA_ACK)
	{
		// send data
		while(length)
		{
			i2cSendByte( *data++ );
			i2cWaitForComplete();
			length--;
		}
	}
	else
	{
		// device did not ACK it's address,
		// data will not be transferred
		// return error
		retval = I2C_ERROR_NODEV;
	}

	// transmit stop condition
	// leave with TWEA on for slave receiving
	i2cSendStop();
	while( !(inb(TWCR) & BV(TWSTO)) );

	return retval;
}

unsigned char i2cMasterReceiveNI(unsigned char deviceAddr, unsigned char length, unsigned char *data)
{
	unsigned char retval = I2C_OK;

	// send start condition
	i2cSendStart();
	i2cWaitForComplete();

	// send device address with read
	i2cSendByte( deviceAddr | 0x01 );
	i2cWaitForComplete();

	// check if device is present and live
	if( inb(TWSR) == TW_MR_SLA_ACK)
	{
		// accept receive data and ack it
		while(length > 1)
		{
			i2cReceiveByte(TRUE);
			i2cWaitForComplete();
			*data++ = i2cGetReceivedByte();
			// decrement length
			length--;
		}

		// accept receive data and nack it (last-byte signal)
		i2cReceiveByte(FALSE);
		i2cWaitForComplete();
		*data++ = i2cGetReceivedByte();
	}
	else
	{
		// device did not ACK it's address,
		// data will not be transferred
		// return error
		retval = I2C_ERROR_NODEV;
	}

	// transmit stop condition
	// leave with TWEA on for slave receiving
	i2cSendStop();

	return retval;
}

