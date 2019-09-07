#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringSerial.h>
#include <wiringPi.h>


#define IBUS_BUFFERSIZE 32  //Max iBus paket s recivera
#define IBUS_MAXCHANNELS 7


//gcc c.c -o program -Wall -lwiringPi -llsm9ds1 -l
static int ibusIndex = 0;
static char ibus[IBUS_BUFFERSIZE] = {0};
static short rcValue[IBUS_MAXCHANNELS];
static float ref[4];
static int rxFrameDone;
int fds;
int i,f1,f2,f3,f4;
float acc[3] = {0};
float gyr[3] = {0};



void readRx(); //fun za citanje sa serijske

int main()
{
	wiringPiSetup();
//	init_imu();

	pinMode(0, OUTPUT); digitalWrite(0, LOW); //na WiringPi Pin 0-->LED CRVENA, ne radi serija
	pinMode(1, OUTPUT); digitalWrite(0, LOW); //   -||-         1-->LED ZUTA, ugasene komande sredina joysticka 

	if((fds = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
	{
		fprintf(stderr, "ne dela serija: %s\n",strerror (errno));
		digitalWrite(0, HIGH); delay(500);
		digitalWrite(0, LOW); delay(500);
		return 1;
	}
	if((fd = pca9685Setup(300,0x40,50)) < 0)
	{
		fprintf(stderr,"i2c do pwm ne dela");
		digitalWrite(1, HIGH); delay(500);
		digitalWrite(1, LOW); delay(500);
	}

   for(;;)
    {
	readRx();
	if(rxFrameDone == 1 && rcValue[5] == 2000)
	{
		for(i=0;i<4;i++){    //Skaliranje ulaza s rc-a na referentne brzine +/- 0.5m/s(rad/s)
			ref[i] = ((float)(rcValue[i] - 1500)) / 1000 ;
			}
//		get_imu_reading(acc, gyr); //citanje senzora, 
//		v=v+acc*t, complementarni
		//zakon upravljanja, U1,U2,U3,U4
		//racuna sile i pwm 
		pca9685PWMWrite(fd,0,0,f1); //iBus output za motore
		pca9685PWMWrite(fd,1,0,f2);
		pca9685PWMWrite(fd,2,0,f2);
		pca9685PWMWrite(fd,3,0,f3);
	printf(" akc:xyz %f , %f,  %f, gyro %f,  %f,   %f\n",acc[0], acc[1],acc[2], gyr[0],gyr[1],gyr[3]);
	fflush(stdout);
	}
    }
}


//FUNKCIJE


void readRx()
{
	rxFrameDone = 0;
	char val = serialGetchar(fds);
	if(ibusIndex == 0 && val != 0x20){
		ibusIndex = 0;
		return;
	}
	if (ibusIndex == 1 && val != 0x40){
		ibusIndex = 0;
		return;
	}
	if(ibusIndex == IBUS_BUFFERSIZE){
		rxFrameDone = 1;
		ibusIndex = 0;
		rcValue[0] = (ibus[3] << 8) + ibus[2];
		rcValue[1] = (ibus[5] << 8) + ibus[4];
		rcValue[2] = (ibus[7] << 8) + ibus[6];
		rcValue[3] = (ibus[9] << 8) + ibus[8];
		rcValue[4] = (ibus[11] << 8) + ibus[10];
		rcValue[5] = (ibus[13] << 8) + ibus[12];
		rcValue[6] = (ibus[15] << 8) + ibus[14];
	}
	else{
		ibus[ibusIndex] = val;
		ibusIndex++;
	}
}
