#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <RF24/RF24.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>

#define IPCKEY 0x261303

using namespace std;

typedef struct{
	int value[10];
	unsigned long timestamp;
	bool shouldServerExit=false;
} rc_data;

RF24 radio(22,0);

const uint64_t addresses[2] = { 0xABCDABCD71LL, 0x544d52687CLL };

uint8_t payload[22];
char pong_back[10];
unsigned long rxTimer=0;
int counter=0;
int shm_id;
rc_data *p_map;

int main(int argc, char** argv){

	if (!strcmp(argv[1],"start")){
		printf("start\n");
		shm_id = shmget(IPCKEY,0, 0);
		if(shm_id != -1)
		{
			perror("already running.\n");
			return -1;
		}
		shm_id=shmget(IPCKEY,4096,IPC_CREAT|IPC_EXCL|0600);
		if(shm_id==-1)
		{
			perror("shmget error\n");
			return -1;
		}

		p_map=(rc_data*)shmat(shm_id,NULL,0);

		radio.begin();                           // Setup and configure rf radio
		radio.setChannel(1);
		radio.setPALevel(RF24_PA_MAX);
		radio.setDataRate(RF24_1MBPS);
		radio.setAutoAck(0);                     // Ensure autoACK is enabled
		radio.setCRCLength(RF24_CRC_8);          // Use 8-bit CRC for performance
/***********************************/
		radio.openWritingPipe(addresses[0]);
		radio.openReadingPipe(1,addresses[1]);
		radio.startListening();
		int thr=0,yaw=0,roll=0,pitch=0,ch5=0,ch6=0,ch7=0,ch8=0,ch9=0,ch10=0;
		while (true){
			if((*p_map).shouldServerExit)
			{
				printf("stopped.\n");
				radio.powerDown();
				shmdt(p_map) ;
				shmctl(shm_id, IPC_RMID, 0);
				return 0;
			}
			while(radio.available()){
				radio.read(&payload,22);
				thr=payload[2]*256+payload[3];
				yaw=payload[4]*256+payload[5];
				roll=payload[6]*256+payload[7];
				pitch=payload[8]*256+payload[9];
				ch5=payload[10]*256+payload[11];
				ch6=payload[12]*256+payload[13];
				ch7=payload[14]*256+payload[15];
				ch8=payload[16]*256+payload[17];
				ch9=payload[18]*256+payload[19];
				ch10=payload[20]*256+payload[21];
				counter++;

				(*p_map).value[0]=thr;
				(*p_map).value[1]=yaw;
				(*p_map).value[2]=roll;
				(*p_map).value[3]=pitch;
				(*p_map).value[4]=ch5;
				(*p_map).value[5]=ch6;
				(*p_map).value[6]=ch7;
				(*p_map).value[7]=ch8;
				(*p_map).value[8]=ch9;
				(*p_map).value[9]=ch10;

				radio.stopListening();
				pong_back[0]='P';
				pong_back[1]='A';
				radio.writeFast(&pong_back, 10);
				radio.txStandBy();
				radio.startListening();
			}
			if(millis() - rxTimer > 1000){
				rxTimer = millis();
				printf("Count: %d\n",counter);
				printf("%c %c %d %d %d %d %d %d %d %d %d %d\n",payload[0],payload[1],thr,yaw,roll,pitch,ch5,ch6,ch7,ch8,ch9,ch10);
				counter=0;
			}
		} // loop
	}
	else if(!strcmp(argv[1],"stop"))
	{
		printf("stopping...\n");
		shm_id = shmget(IPCKEY,0, 0);
		if (shm_id==-1){
			perror("not running\n");
			return -1;
		}
		p_map=(rc_data*)shmat(shm_id,NULL,0);
		(*p_map).shouldServerExit=true;
		shmdt(p_map) ;
		if ((argc>=3)&&(!strcmp(argv[2],"-f"))){
			shmctl(shm_id, IPC_RMID, 0);
			printf("stopped.\n");
		}
		return 0;
	}
} // main
