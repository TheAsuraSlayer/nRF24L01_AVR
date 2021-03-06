/*
 * nRF24_RX.c
 *
 * Created: 11-01-2014 23:23:06
 *  Author: Abhinav
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "nRF24L01.h"

#define CEPin  PINB0
#define CSNPin PINB1
#define SCKPin PINB7
#define MISOPin PINB6
#define MOSIPin PINB5

#define LEDPin PIND6


uint8_t *data;

/*** USART TO PC ***/
void USART_INIT(unsigned int baud)
{
	//Set Baud Rate
	UBRRH = (unsigned char)(baud<<8);
	UBRRL = (unsigned char)baud;
	//Enable RX and TX
	UCSRB = (1<<RXEN) | (1<<TXEN);
	//Frame format - 8DATA 2STOP BIT
	UCSRC = (1<<USBS) | (3<<UCSZ0);
}
void USART_TRANSMIT(unsigned char data)
{
	//Wait for empty BUFFER
	while(!(UCSRA & (1<<UDRE)))
	;
	//Put DATA into BUFFER
	UDR = data;
}
void USART_SENDSTRING(char* StringPtr)
{
	USART_TRANSMIT(10);
	while(*StringPtr != 0x00)
	{
		USART_TRANSMIT(*StringPtr);
		StringPtr++;
	}
}
/*******************/
void initSPI(void)
{
	//Intialise the SPI-USI Communication
	DDRB |= (1<<CEPin) | (1<<CSNPin) | (1<<SCKPin) | (1<<MISOPin);	// Set CE,CSN,SCK,MISO[MOSI of nRF] as Output
	DDRB &= ~(1<<MOSIPin);											//Set MOSI[MISO of nRF] as Input
	PORTB |= (1<<MOSIPin);											//Enable Internal PullUP
	
	USICR |= (1<<USIWM0) | (1<<USICS1) | (1<<USICLK);	//ENABLE USI-SPI 3 wire mode | External, Positive Edge Software clock Strobe
	
	SETBIT(PORTB,CSNPin);								//CSNPin To HIGH-But not sending any command
	CLEARBIT(PORTB,CEPin);								//CE Pin Low
}
uint8_t WriteReadByteSPI(uint8_t cData)
{
	//Load Byte to DATA REGISTER
	USIDR = cData;
	USISR |= (1<<USIOIF); //Clear 4-bit counter overflow flag to be able to receive new Data
	//Wait for completion of transmission
	while((USISR & (1<<USIOIF)) == 0)
	{
		USICR |= (1<<USITC); //Reset flag
	}
	return USIDR;
}
uint8_t GetReg(uint8_t reg)
{
	_delay_us(10);						//Delay for 10us
	CLEARBIT(PORTB,CSNPin);				//Set CSN Low - nRf starts listening for commands 10us after CSN Low
	_delay_us(12);						//Delay for 12us
	WriteReadByteSPI(R_REGISTER + reg);	//R_Register --> Set to Reading Mode, "reg" --> The registry which will be read
	_delay_us(12);						//Dealy 12us
	reg = WriteReadByteSPI(NOP);		//Send DUMMY BYTE[NOP] to receive first byte in 'reg' register
	_delay_us(12);						//Delay 12us
	SETBIT(PORTB,CSNPin);				//CSN High
	return reg;							//Return the registry read
}
uint8_t *WriteToNrf(uint8_t ReadWrite, uint8_t reg, uint8_t *val, uint8_t antVal)
{
	//ReadWrite --> "R" or "W", reg --> 'register', *val --> array with package, antVal --> number of int in array
	if(ReadWrite == W)//If it is in READMODE, then addr is already 0x00
	{
		reg = W_REGISTER + reg;
	}
	static uint8_t ret[32];	//Array to be returned in the end
	
	_delay_us(10);						//Delay for 10us
	CLEARBIT(PORTB,CSNPin);				//Set CSN Low - nRf starts listening for commands 10us after CSN Low
	_delay_us(12);						//Delay for 12us
	WriteReadByteSPI(reg);				//"reg" --> Set nRf to write or read mode
	_delay_us(10);
	
	for(int i = 0; i<antVal; i++)
	{
		if(ReadWrite == R && reg != W_TX_PAYLOAD)
		{
			//READ A REGISTRY
			ret[i] = WriteReadByteSPI(NOP);		//Send dummy Byte to read data
			_delay_us(10);
		}
		else
		{
			//Write to nRF
			WriteReadByteSPI(val[i]);			//Send command one at a time
			_delay_us(10);
		}
	}
	SETBIT(PORTB,CSNPin);		//nRf into IDLE with CSN HIGH
	return ret;					//Return the data read
}
void reset(void)
{
	_delay_us(10);
	CLEARBIT(PORTB,CSNPin);
	_delay_us(10);
	WriteReadByteSPI(W_REGISTER+STATUS);
	_delay_us(10);
	WriteReadByteSPI(0x70);
	_delay_us(10);
	SETBIT(PORTB, CSNPin);
}
uint8_t *receive_payload(void)
{
	SETBIT(PORTB,CEPin);//Start Active RX Mode
	_delay_ms(1000);
	USART_SENDSTRING("Waiting..");
	while ((GetReg(STATUS) & (1<<6)) == 0)
	{
		_delay_us(10);
	}
	CLEARBIT(PORTB,CEPin);//Enter Standby Mode
	_delay_ms(10);
	return WriteToNrf(R,R_RX_PAYLOAD,data,32);
}
void receive_init(void)
{
	_delay_ms(1000);
	uint8_t val[5];
	val[0] = 0x1D;
	WriteToNrf(W,CONFIG,val,1);//Set PRIM_RX bit in CONFIG reg to HIGH. But do not power up the Radio.
	_delay_ms(100);
	val[0] = 0x03;
	WriteToNrf(W,SETUP_AW,val,1);//Set Address Width as 5Bytes
	val[0] = 0x01;
	WriteToNrf(W,EN_RXADDR,val,1);//Enable Data Pipe 0
	for(int i = 0;i<5;i++)
	{
		val[i] = 0x12;
	}
	WriteToNrf(W,RX_ADDR_P0,val,5);//Set Data Pipe 1 Addr
	val[0] = 0x01;
	WriteToNrf(W,EN_AA,val,1);//Enable Auto Ack on Data Pipe 0
	val[0] = 5;
	WriteToNrf(W,RX_PW_P0,val,1);//Set correct payload width[5 bytes]
	val[0] = 0x32;
	WriteToNrf(W,RF_CH,val,1);//Set the CH
	val[0] = 0x07;
	WriteToNrf(W,RF_SETUP,val,1);//Set transfer rate
	
	val[0] = 0x1F;
	WriteToNrf(W,CONFIG,val,1);//Power up Radio
	_delay_ms(1000);
	USART_SENDSTRING("RADIO_INIT");
}

void receive_data(void)
{
	_delay_ms(10);
	SETBIT(PORTB,CEPin);
	_delay_ms(1000);
	CLEARBIT(PORTB,CEPin);
	_delay_ms(10);
}
int main(void)
{
	uint8_t *val;
	USART_INIT(51);
	USART_SENDSTRING("PROGRAM STARTED");
	initSPI();
	receive_init();
	DDRD |= (1<<LEDPin);		//Set LEDPin as Output
	while(1)
	{
		USART_TRANSMIT(GetReg(CONFIG));
		USART_TRANSMIT(GetReg(STATUS));
		receive_data();
		if(((GetReg(STATUS) & (1<<6)) != 0 ))
		{
			PORTD |= (1<<LEDPin);
			_delay_ms(100);
			PORTD &= ~(1<<LEDPin);
			val = WriteToNrf(R,R_RX_PAYLOAD,val,5);
			for(int i=0;i<5;i++)
			{
				USART_TRANSMIT(val[i]);
			}
		}
		reset();
	}
}
