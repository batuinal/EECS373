#include <stdio.h>
#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>

#define ADDR_DEV0 0xB4
#define ADDR_DEV1 0xB6
#define WRITE 0x0
#define READ 0x1

#define BAUD_SLOW 9600
#define BAUD_FAST 57600

void TWI_init_master(void) // Function to initialize master
{
    TWBR=0x0F;  // Bit rate
    TWSR=(1<<TWPS1)|(0<<TWPS0); // Setting prescalar bits
    // SCL freq= F_CPU/(16+2(TWBR).4^TWPS)
}

void TWI_start(void)
{
    // Clear TWI interrupt flag, Put start condition on SDA, Enable TWI
    TWCR= (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);  
    while(!(TWCR & (1<<TWINT))); // Wait till start condition is transmitted
    while((TWSR & 0xF8)!= 0x08); // Check for the acknowledgement
}

void TWI_repeated_start(void)
{
    // Clear TWI interrupt flag, Put start condition on SDA, Enable TWI
    TWCR= (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);  
    while(!(TWCR & (1<<TWINT))); // wait till restart condition is transmitted
    while((TWSR & 0xF8)!= 0x10); // Check for the acknowledgement
}

void TWI_write_address(unsigned char data)
{
    TWDR=data;  // Address and write instruction
    TWCR=(1<<TWINT)|(1<<TWEN);    // Clear TWI interrupt flag,Enable TWI
    while (!(TWCR & (1<<TWINT))); // Wait till complete TWDR byte transmitted
    while((TWSR & 0xF8)!= 0x18);  // Check for the acknowledgement
}

void TWI_read_address(unsigned char data)
{
    TWDR=data;  // Address and read instruction
    TWCR=(1<<TWINT)|(1<<TWEN);    // Clear TWI interrupt flag,Enable TWI
    while (!(TWCR & (1<<TWINT))); // Wait till complete TWDR byte received
    while((TWSR & 0xF8)!= 0x40);  // Check for the acknowledgement
}

void TWI_write_data(unsigned char data)
{
    TWDR=data;  // put data in TWDR
    TWCR=(1<<TWINT)|(1<<TWEN);    // Clear TWI interrupt flag,Enable TWI
    while (!(TWCR & (1<<TWINT))); // Wait till complete TWDR byte transmitted
    while((TWSR & 0xF8) != 0x28); // Check for the acknowledgement
}

unsigned char TWI_read_data(void)
{
    TWCR=(1<<TWINT)|(1<<TWEN);    // Clear TWI interrupt flag,Enable TWI
    while (!(TWCR & (1<<TWINT))); // Wait till complete TWDR byte transmitted
    while((TWSR & 0xF8) != 0x58); // Check for the acknowledgement
	unsigned char data=TWDR;
    PORTB=data;
	return data;
}

void TWI_stop(void)
{
    // Clear TWI interrupt flag, Put stop condition on SDA, Enable TWI
    TWCR= (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);  
    while(!(TWCR & (1<<TWSTO)));  // Wait till stop condition is transmitted
}

void USART_Init(unsigned int baud)
{
	/* Set baud rate */
	baud = ((F_CPU / (baud * 16UL))) - 1;
	
	UBRR1H = (unsigned char)(baud>>8);
	UBRR1L = (unsigned char) baud;
	/* Enable receiver and transmitter */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	/* Set frame format: 8data, 2stop bit */
	UCSR1C = (0<<USBS1)|(3<<UCSZ10);
}

void USART_Transmit(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR1A & (1<<UDRE1)) );
	/* Put data into buffer, sends the data */
	UDR1 = data;
}

// ===== ===== //

void delay(int factor){
	if (factor > 10)
		factor = 10;
	volatile int i = 0;
	for (i = 0; i < (1000 << factor); ++i){}
}

void write(uint8_t addr, uint8_t reg, uint8_t data){
	TWI_start();
	TWI_write_address(addr+WRITE);
	TWI_write_data(reg);
	TWI_write_data(data);
	TWI_stop();
	delay(0);
}

uint8_t read(uint8_t addr, uint8_t reg){
	TWI_start();
	TWI_write_address(addr+WRITE);
	TWI_write_data(reg);
	TWI_repeated_start();
	TWI_read_address(addr+READ);
	unsigned char data = TWI_read_data();
	TWI_stop();
	delay(0);
	return data;
}

int main(void)
{
	int i = 0;

	TWI_init_master();
	USART_Init(BAUD_SLOW);


	// configure thresholds for all registers.
	uint8_t a;
	for (a = 0x41; a < 0x5A; a += 2){
		write(ADDR_DEV0,a,0x0F);
		write(ADDR_DEV0,a+1,0x0A);
		write(ADDR_DEV1,a,0x0F);
		write(ADDR_DEV1,a+1,0x0A);
	}

	write(ADDR_DEV0,0x5E,0x0C); // Enable all electrodes.
	write(ADDR_DEV1,0x5E,0x0C);

	// enable auto-configuration.
	uint8_t autoaddr[5] = {0x7B, 0x7C, 0x7D, 0x7E, 0x7F};
	uint8_t autodata[5] = {0x32, 0x00, 0xCA, 0x82, 0xB4};
	for (i = 0; i < 5; ++i){
		write(ADDR_DEV0,autoaddr[i],autodata[i]);
		write(ADDR_DEV1,autoaddr[i],autodata[i]);
	}

	// Read the data.
	uint8_t addrbuf[2] = {0x00, 0x01};
	uint8_t dev0_databuf[2] = {0xFF, 0xFF};
	uint8_t dev1_databuf[2] = {0xFF, 0xFF};
	while (1){

		for (i = 0; i < 2; ++i){
			dev0_databuf[i] = read(ADDR_DEV0,addrbuf[i]);
			dev1_databuf[i] = read(ADDR_DEV1,addrbuf[i]);
		}

		uint8_t data[3];
		data[0] = dev0_databuf[0];
		data[1] = dev0_databuf[1] | (dev1_databuf[0] << 4);
		data[2] = (dev1_databuf[0] >> 4) | (dev1_databuf[1] << 4);
		
		for (i = 0; i < 3; ++i)
			USART_Transmit(data[i]);
			
		delay(3);
	}

	return(0);
}
