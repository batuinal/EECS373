#include <stdio.h>
#include <math.h>
#include <inttypes.h>
#include "drivers/mss_uart/mss_uart.h"
#include "drivers/CoreUARTapb/core_uart_apb.h"
int index = 0;

#define COREUARTAPB0_BASE_ADDR      0x40050000
#define COREUARTAPB1_BASE_ADDR		0x40050100
#define COREUARTAPB2_BASE_ADDR		0x40050200
#define COREUARTAPB3_BASE_ADDR		0x40050300
#define COREUARTAPB4_BASE_ADDR		0x40050400

#define BAUD_VALUE_31250    199
#define BAUD_VALUE_57600	108
#define BAUD_VALUE_9600 	650

// For a straight distance of 4 feet.
#define CAMERA_LODIST 70
#define CAMERA_HIDIST 120

#define IMU_RADPERDEG 3.14159265/180
#define IMU_GRAVITY 30
#define IMU_IDLETHRESH 15

#define MIDI_BASENOTE 36

UART_instance_t midi_uart, camera_uart, imu_uart, pcb_uart, lcd_uart;

// Effective range is 41 - 88.
uint8_t pitchmap[24] = {19,20,21,22,12,13,14,15,5,6,7,8,41,42,43,44,34,35,36,37,27,28,29,30};

uint8_t notemap[12][3];

void clear_disp(){
	uint8_t message[] = {254, 0x01};
	UART_send(&lcd_uart, message, sizeof(message));

	volatile int j = 0;
	for(; j < 100000; ++j){}
}

void cursor_move(int pos){

	uint8_t mes[] = {254, pos+128};
	UART_send( &lcd_uart, mes, sizeof(mes));

	volatile int j = 0;
	for(; j < 100000; ++j){}
}

void initialize(){

	notemap[0][0] = 'C';
	notemap[0][1] = '_';
	notemap[1][0] = 'C';
	notemap[1][1] = '#';
	notemap[2][0] = 'D';
	notemap[2][1] = '_';
	notemap[3][0] = 'D';
	notemap[3][1] = '#';
	notemap[4][0] = 'E';
	notemap[4][1] = '_';
	notemap[5][0] = 'F';
	notemap[5][1] = '_';
	notemap[6][0] = 'F';
	notemap[6][1] = '#';
	notemap[7][0] = 'G';
	notemap[7][1] = '_';
	notemap[8][0] = 'G';
	notemap[8][1] = '#';
	notemap[9][0] = 'A';
	notemap[9][1] = '_';
	notemap[10][0] = 'A';
	notemap[10][1] = '#';
	notemap[11][0] = 'B';
	notemap[11][1] = '_';


	UART_init(&midi_uart, COREUARTAPB0_BASE_ADDR, BAUD_VALUE_31250, DATA_8_BITS | NO_PARITY);
	UART_init(&camera_uart, COREUARTAPB1_BASE_ADDR, BAUD_VALUE_57600, DATA_8_BITS | NO_PARITY);
	UART_init(&imu_uart, COREUARTAPB2_BASE_ADDR, BAUD_VALUE_9600, DATA_8_BITS | NO_PARITY);
	UART_init(&pcb_uart, COREUARTAPB3_BASE_ADDR, BAUD_VALUE_9600, DATA_8_BITS | NO_PARITY);
	UART_init(&lcd_uart, COREUARTAPB4_BASE_ADDR, BAUD_VALUE_9600, DATA_8_BITS | NO_PARITY);

	volatile int i = 0;
	for(; i < 100000; ++i){}

	//Brightness
	uint8_t message0[] = {0x7C, 157};
	UART_send( &lcd_uart, message0, 2);

	volatile int x = 0;
	for(; x < 100000; ++x){}

		//Clear display
		clear_disp();

		//write(" EECS 373 AirGuitar", 20);
		uint8_t message[15] = "373 AirGuitar!";
		UART_send(&lcd_uart, message, 14);

		cursor_move(64);

		uint8_t message1[10] = "Batu Inal";
		UART_send(&lcd_uart, message1, 9);

		cursor_move(20);

		uint8_t message2[13] = "Madhav Achar";
		UART_send(&lcd_uart, message2, 12);

		cursor_move(84);

		uint8_t message3[13] = "Sinan Gunbay";
		UART_send( &lcd_uart, message3, 12);

		for (; x < 20000000; ++x){}

		clear_disp();
		cursor_move(0);

		uint8_t midibuf[] = {0xC0, 0x1B};
		UART_send(&midi_uart, midibuf, sizeof(midibuf));

}

uint8_t recvkeys(uint8_t *keys, uint8_t prevnumkeys){

	uint8_t numkeys = prevnumkeys;
	uint8_t keybuf[3] = {0};

	if(!UART_get_rx(&pcb_uart, keybuf, 1)){
		return prevnumkeys;
	}

	while(!UART_get_rx(&pcb_uart, keybuf+1, 1));

	while(!UART_get_rx(&pcb_uart, keybuf+2, 1));

	uint32_t keydata = keybuf[2];
	keydata = (keydata << 8) | keybuf[1];
	keydata = (keydata << 8) | keybuf[0];

	uint8_t temp[6] = {255, 255, 255, 255, 255, 255};

	int i;
	for (i = 23; i >= 0; --i){
		if (keydata & (1 << i)){
			temp[i/4] = i;
		}
	}

	numkeys = 0;
	for (i = 0; i < 6; ++i){
		if (temp[i] != 255)
			keys[numkeys++] = temp[i];
	}
	return numkeys;
}


uint8_t recvdist(uint8_t *prevdist){

	uint8_t buf;

	if(!UART_get_rx( &camera_uart, &buf, sizeof(buf)))
		buf = *prevdist;
	else
		*prevdist = buf;

	if (buf > CAMERA_HIDIST)
		return 2;
	else if (buf > CAMERA_LODIST)
		return 1;
	else
		return 0;
}

uint8_t recvstrum(uint8_t prevstrum){

	int8_t accdata[3];
	if (!UART_get_rx(&imu_uart, accdata, 1))
		return prevstrum;

	while(!UART_get_rx(&imu_uart, &accdata[1], 1));
	while(!UART_get_rx(&imu_uart, &accdata[2], 1));

	uint8_t magnitude = (uint8_t) sqrt((accdata[0] * accdata[0]) + (accdata[1] * accdata[1]) + (accdata[2] * accdata[2]));

	return magnitude;
}

void sendMIDI(uint8_t *keys, uint8_t numkeys, uint8_t distance, uint8_t imudata, uint8_t *pitches){

	uint8_t notes[15];

	uint16_t velocity = 37 + (2*imudata/3);
	if (velocity > 127)
		velocity = 127;

	notes[0] = 0xB0;
	notes[1] = 0x7B;
	notes[2] = 0x00;
	notes[3] = 0x90;

	uint32_t i = 0;
	for (i = 0; i < numkeys; ++i){
		pitches[i] = MIDI_BASENOTE + pitchmap[keys[i]] - 4*distance;
		notes[2*i+4] = pitches[i];
		notes[2*i+5] = (uint8_t) velocity;
	}

	UART_send(&midi_uart, notes, (4+numkeys*2));
	volatile int d = 0;
	for (d = 0; d < 600000; ++d){}

}

void sendmsg(uint8_t *str, uint8_t size){
	if(index == 0){
		clear_disp();
		cursor_move(2);
	}
	else if(index == 1)
		cursor_move(66);
	else if(index == 2)
		cursor_move(22);
	else
		cursor_move(86);
	index = (index+1)%4;
	UART_send(&lcd_uart, str, size);
}

void sendLCD(uint8_t *pitches, uint8_t numkeys){

	uint8_t message0[16] = "=== Played: ===";
	sendmsg(message0,15);

	uint8_t message[] = " ___  ___  ___  ___  ___  ___ ";
	uint32_t i = 0;
	for (i = 0; i < numkeys; ++i){
		message[5*i+1] = notemap[pitches[i]%12][0];
		message[5*i+2] = notemap[pitches[i]%12][1];
		message[5*i+3] = ('0' + pitches[i]/12);
	}

	uint8_t remaining = 5*numkeys;
	uint8_t send;
	for (i = 0; i < 3; ++i){
		if (!remaining)
			send = 1;
		else {
			send = remaining;
			if (remaining > 15)
				send = 15;
			remaining -= send;
		}
		sendmsg(message+(5*i),send);
	}
}

int main() {

	uint8_t playing = 0;
	// For IMU Data
	uint8_t previmudata = 0;
	uint8_t imudata = 0;

	// For Camera Data
	uint8_t distance = 0;
	uint8_t prevdist = 0;

	// For PCB Data
	uint8_t keys[6];
	uint8_t numkeys = 0;

	// For LCD
	uint8_t pitches[6];

	initialize();
	uint8_t dummy;
	while(!UART_get_rx(&pcb_uart, &dummy, 1));

	printf("-- Initialized! --\n\r");

	while(1) {

		previmudata = imudata;

		numkeys = recvkeys(keys, numkeys);
		distance = recvdist(&prevdist);
		imudata = recvstrum(previmudata);

		if (playing){
			playing = (imudata > (IMU_GRAVITY + IMU_IDLETHRESH));
			continue;
		}

		if (!numkeys || (imudata < (IMU_GRAVITY + IMU_IDLETHRESH)))
			continue;

		playing = 1;
		sendMIDI(keys, numkeys, distance, imudata, pitches);
		sendLCD(pitches, numkeys);


	}
	return 0;
}
