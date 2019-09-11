#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <pca9685.h>
#include <lsm9ds1.h>
#include <time.h>
#include <math.h>

#define IBUS_BUFFERSIZE 32  //Max iBus paket s recivera
#define IBUS_MAXCHANNELS 7
#define KP 8
#define KK0 60
#define KK1 47
#define KK2 12


//gcc c.c -o program -Wall -lwiringPi -llsm9ds1 -lwiringPiPca9685 -llrt
static int ibusIndex = 0;
static char ibus[IBUS_BUFFERSIZE] = {0};
static short rcValue[IBUS_MAXCHANNELS];
static float ref[4];
static int rxFrameDone;
int fds,fd,i;
float f1,f2,f3,f4,c,b;
float acc[3] = {0};
float gyr[3] = {0};
float bax=0.0474, bay=-0.1295, baz=0.3044, bgx=1.2099, bgy=1.1932, bgz=0.2329; //biasi senzora
float vx=0,vy=0,vz=0;
float u1,u2,u3,u4;
float phi=0, theta=0, psi=0;
long int time_senzor;
long int dt,tpocetak;
struct timespec gettime_now;
float Phi=0,Theta=0,Psi=0;
float m = 1.7, g=9.81, ix=0.72,iy=0.69;


void readRx(); //fun za citanje sa serijske

int main()
{
	wiringPiSetup();
	init_imu();

	pinMode(0, OUTPUT); digitalWrite(0, LOW); //na WiringPi Pin 0-->LED CRVENA, ne radi serija
	pinMode(1, OUTPUT); digitalWrite(0, LOW); //   -||-         1-->LED ZUTA, ne radi PWMi2c 

	if((fds = serialOpen ("/dev/serial0", 115200)) < 0)
	{
		fprintf(stderr, "ne dela serija: %s\n",strerror (errno));
		digitalWrite(0, HIGH); delay(500);
		digitalWrite(0, LOW); delay(500);
		return 1;
	}
	init_imu();
	if((fd = pca9685Setup(300,0x40,50)) < 0)
	{
		fprintf(stderr,"i2c do pwm ne dela");
		digitalWrite(1, HIGH); delay(500);
		digitalWrite(1, LOW); delay(500);
	}
	clock_gettime(CLOCK_REALTIME, &gettime_now);
	time_senzor = gettime_now.tv_nsec;


   for(;;)
    {
	readRx();
        if(rxFrameDone == 1 && rcValue[5] == 1000){
                pca9685PWMWrite(fd,0,0,0x700);
                pca9685PWMWrite(fd,1,0,0x700);
                pca9685PWMWrite(fd,2,0,0x700);
                pca9685PWMWrite(fd,3,0,0x700);
		rxFrameDone = 0;
		                }


	if(rxFrameDone == 1 && rcValue[5]== 2000)
	{         clock_gettime(CLOCK_REALTIME,&gettime_now);
		 tpocetak=gettime_now.tv_nsec; //da vidim kolko traje
		for(i=0;i<4;i++){    //Skaliranje ulaza s rc-a na referentne brzine +/- 0.5m/s(rad/s)
			ref[i] = ((float)(rcValue[i] - 1500)) / 1000 ;
			}
		get_imu_reading(acc, gyr); //citanje senzora
		clock_gettime(CLOCK_REALTIME,&gettime_now);
		dt= (gettime_now.tv_nsec - time_senzor)/1000000000;printf("%ld\n",dt);
		if(dt<0){
			dt=8588888/1000000000;
			}
		time_senzor = gettime_now.tv_nsec;
	//	acc[0] =acc[0]-bax; acc[1]= acc[1]-bay; acc[2]= acc[2]-baz; gyr[0]= gyr[0]-bgx; gyr[1]=gyr[1]-bgy; gyr[2]=gyr[2]-bgz;
		vx = vx+acc[0]*dt; vy = vy+acc[1]*dt; vz = vz+acc[2]*dt; //integriranje akceleracija
		Phi = 0.9*(Phi+(gyr[0]*dt*3.14/180))+0.1*acc[1]/acc[3]; //complementarni za kuteve
		Theta = 0.9*(Theta + gyr[1]*dt*3.14/180)+ 0.1*(-1)*acc[0]/sqrt(pow(acc[1],2)+pow(acc[2],2));
		Psi = psi + gyr[2]*dt*3.14/180;
		u1 = -KP*(vz-ref[2])+ m*g;//zakon upravljanja, U1,U2,U3,U4
		u2 = -ix/g +(-KK2*(-g*gyr[0])-KK1*(-g*Phi)-KK0*(vy-ref[0]));
		u3 = iy/g +(-KK2*(g*gyr[1])-KK1*(g*Theta)-KK0*(vx-ref[1]));
		u4 = -1*(KP*(gyr[3]-ref[3]));
		f1 = 0.25*u1-0.0016*u3+2.5+u4;
		c = f1/2.5/100000;//racuna sile i pwm
		b = sqrt(c);
		//f2=(sqrt((0.25*u1-0.0016*u2-2.5+u4)/2.5/100000))/7814.1;
 		//f3=(sqrt((0.25*u1+0.0016*u3+2.5+u4)/2.5/100000))/7814.1;
		//f4=(sqrt((0.25*u1+0.0016*u2-2.5+u4)/2.5/100000))/7814.4;
		pca9685PWMWrite(fd,0,0,(0x800+0x7FF*(int)(f1))); //iBus output za motore
		//pca9685PWMWrite(fd,1,0,(int)(0x800+0x7FF*f2));
		//pca9685PWMWrite(fd,2,0,(int)(0x800+0x7FF*f3));
		//pca9685PWMWrite(fd,3,0,(int)(0x800+0x7FF*f4));
	clock_gettime(CLOCK_REALTIME,&gettime_now);
	printf("u1,u2,u3,u4 %f,%f,%f,%f\n#f1= %f\n %f\n %f\n\n ",u1,u2,u3,u4,f1,c,b );
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
	if(val < 0){
		printf("neprimam");}
	else{
		ibus[ibusIndex] = val;
		ibusIndex++;
	}
}
