#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <pca9685.h>
#include <lsm9ds1.h>
#include <time.h>
#include <math.h>

#define IBUS_BUFFERSIZE 32 //Max iBus paket s recivera
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
long double t;
int fds, fd, i, j = 0;
float c, b;
float acc[3] = {0};
float gyr[3] = {0};
float u1, u2, u3, u4;
long int tpocetak;
struct timespec gettime_now;
long int time_senzor;

void readRx(); //fun za citanje sa serijske
void min_max(float rp, float *prp);
//void rpm_PWM(float rpm1, float rpm2, float rpm3, float rpm4, int *p1, int *p2, int *p3, int *p4);
void uprav_u_PWM(float u1, float u2, float u3, float u4, int *r1, int *r2, int *r3, int *r4);
void chill();

int main()
{
	float Phi = 0, Theta = 0, Psi = 0;
	//	float bax=0.0474, bay=-0.1295, baz=0.3044, bgx=1.2099, bgy=1.1932, bgz=0.2329; //biasi senzora
	float vx = 0, vy = 0, vz = 0;
	//float rpm1, rpm2, rpm3, rpm4;
	float m = 1.6, g = 9.81, ix = 0.72, iy = 0.69;
	int pwm1, pwm2, pwm3, pwm4;
	long double dt_long_double;

	wiringPiSetup();
	init_imu();

	pinMode(0, OUTPUT);
	digitalWrite(0, LOW); //na WiringPi Pin 0-->LED CRVENA, ne radi serija
	pinMode(1, OUTPUT);
	digitalWrite(0, LOW); //   -||-         1-->LED ZUTA, ne radi PWMi2c

	if ((fds = serialOpen("/dev/serial0", 115200)) < 0)
	{
		fprintf(stderr, "ne dela serija: %s\n", strerror(errno));
		digitalWrite(0, HIGH);
		delay(500);
		digitalWrite(0, LOW);
		delay(500);
		return 1;
	}
	init_imu();
	if ((fd = pca9685Setup(300, 0x40, 50)) < 0)
	{
		fprintf(stderr, "i2c do pwm ne dela");
		digitalWrite(1, HIGH);
		delay(500);
		digitalWrite(1, LOW);
		delay(500);
	}
	clock_gettime(CLOCK_REALTIME, &gettime_now);
	time_senzor = gettime_now.tv_nsec;

	for (;;)
	{
		readRx();
		if (rxFrameDone == 1 && rcValue[5] == 1000)
		{
			chill();
		}

		if (rxFrameDone == 1 && rcValue[5] == 2000)
		{ // clock_gettime(CLOCK_REALTIME,&gettime_now);
			// tpocetak=gettime_now.tv_nsec; //da vidim kolko traje
			for (i = 0; i < 4; i++)
			{ //Skaliranje ulaza s rc-a na referentne brzine +/- 0.5m/s(rad/s)
				ref[i] = ((float)(rcValue[i] - 1500)) / 1000;
			}
			get_imu_reading(acc, gyr); //citanje senzora

			clock_gettime(CLOCK_REALTIME, &gettime_now);
			/*
				dt ti je bio tipa long int tj. long samo
				gettime_now.tv_nsec je isti tip kao i time_senzor
				oni mogu drzati brojeve vece od 10^9 ali primjeti da razlika ta dva broja koju
				spremas u dt varijablu rijetko kada bude veca od 10^9 (vjerojatno nikada)
				tada imas broj koji je tipa long int i manji od 10^9 i dijelis ga sa 10^9
				to je u stvarnosti 0.nest ali takav rezultat se u intu zaokruzuje u 0
				tek nakon toga svega si pretvarao taj rezultat u double sto je opet 0

				prvo oduzmes, razliku pretvoris u float/double i tek onda dijelis
			*/
			dt_long_double = gettime_now.tv_nsec -time_senzor; // oduzimaju se kao long int i pri dodjeljivanju postaju double (automatski cast)
			// bar bi tako trebalo biti
			if (dt_long_double < 0)
			{
				dt_long_double += 1000000000;
			}
		
		time_senzor = gettime_now.tv_nsec;

		t = dt_long_double / 1000000000;
		//printf("vrijeme u s %Lf\n", t);
		//	acc[0] =acc[0]-bax; acc[1]= acc[1]-bay; acc[2]= acc[2]-baz; gyr[0]= gyr[0]-bgx; gyr[1]=gyr[1]-bgy; gyr[2]=gyr[2]-bgz; //ako micem bias

		vx = vx + acc[0] * t;
		vy = vy + acc[1] * t;
		vz = vz + (acc[2]-9.9) * t;												   //integriranje akceleracija
		//printf("\n brzina po Z: %f",vz);
		Phi = 0.95 * (Phi + (gyr[0] * t * M_PI / 180)) + 0.15 * acc[1] / acc[3]; //complementarni za kuteve
		Theta = 0.85 * (Theta + gyr[1] * t * M_PI / 180) + 0.15 * (-1) * acc[0] / sqrt(pow(acc[1], 2) + pow(acc[2], 2));
		Psi = Psi + gyr[2] * t * M_PI / 180;
		//printf("kutevi: %f  ,%f,  %f",Phi,Theta,Psi);
		u1 = -KP * (vz - ref[2]) + m * g; //zakon upravljanja, U1,U2,U3,U4
		u2 = -ix / g * (-KK2 * (-g * gyr[0]) - KK1 * (-g * Phi) - KK0 * (vy - ref[0]));
		u3 = iy / g * (-KK2 * (g * gyr[1]) - KK1 * (g * Theta) - KK0 * (vx - ref[1]));
		u4 = -1 * (KP * (gyr[3] - ref[3]));
		//printf("\nupravljacke: U1_%f  ,U2_%f  ,U3%f  ,U4 %f",u1,u2,u3,u4);
		uprav_u_PWM(u1, u2, u3, u4, &pwm1, &pwm2, &pwm3, &pwm4); //racuna U u PWM
		//printf("\nRPMzeljeni: rpm1_%f  ,2_ %f  ,3  %f  ,4 %f",rpm1,rpm2,rpm3,rpm4);
		//rpm_PWM(rpm1, rpm2, rpm3, rpm4, &pwm1, &pwm2, &pwm3, &pwm4);
		
	//	printf("\npwm za ibus %d %d %d %d", pwm1, pwm2, pwm3, pwm4);
		pca9685PWMWrite(fd, 0, 0, 205 + pwm1); //iBus output za motore
		pca9685PWMWrite(fd, 1, 0, 205 + pwm2);
		pca9685PWMWrite(fd, 2, 0, 205 + pwm3);
		pca9685PWMWrite(fd, 3, 0, 205 + pwm4);
		//clock_gettime(CLOCK_REALTIME,&gettime_now);
		//printf("u1,u2,u3,u4 %f,%f,%f,%f\n#f1= %f\n %f\n %f\n\n ",u1,u2,u3,u4,f1,c,b );
		//fflush(stdout);
		}
	}
}

//FUNKCIJE
void min_max(float rp, float *prp)
{
	if (rp <= 1)
	{
		*prp = 1;
	}
	if (rp >= 10)
	{
		*prp = 10;
	}
	else
	{
		*prp = rp;
	}
}
/*void rpm_PWM(float rpm1, float rpm2, float rpm3, float rpm4, int *p1, int *p2, int *p3, int *p4)
{
	min_max(rpm1, &rpm1);
	*p1 = (int)((rpm1 - 100) * 4.2);
	*p2 = (int)((rpm2 - 100) * 4.2);
	*p3 = (int)((rpm3 - 100) * 4.2);
	*p4 = (int)((rpm4 - 100) * 4.2);
}*/

void uprav_u_PWM(float u1, float u2, float u3, float u4, int *r1, int *r2, int *r3, int *r4)
{
	float f1, f2, f3, f4;
	//double fc1, fc2, fc3, fc4;
	f1 = 0.25 * u1 - 0.0016 * u3 + 2.5 * u4;
	f2 = 0.25 * u1 - 0.0016 * u2 - 2.5 * u4;
	f3 = 0.25 * u1 + 0.0016 * u3 + 2.5 * u4;
	f4 = 0.25 * u1 + 0.0016 * u2 - 2.5 * u4;
	min_max(f1,&f1);
        min_max(f2,&f2);
        min_max(f3,&f3);
        min_max(f4,&f4);
	*r1 = f1*20.4;
        *r2 = f2*20.4;
        *r3 = f3*20.4;
        *r4 = f4*20.4;

	/*fc1 = f1 /8 * 1000000;
	fc2 = f2 /8 * 1000000;
	fc3 = f3 /8* 1000000;
	fc4 = f4 /8 * 1000000;
	 printf("\nomeg21 %f\n", fc1); printf("omeg23 %f\n", fc3);
	 printf("Sila3 %f\n", f3); printf("Sila1 %f\n", f1); printf("Sila2 %f\n", f2); printf("Sila4 %f\n", f4);
	*r1 = sqrt(fc1);
	*r2 = sqrt(fc2);
	*r3 = sqrt(fc3);
	*r4 = sqrt(fc4);*/
}

void chill()
{
	pca9685PWMWrite(fd, 0, 0, 2040);
	pca9685PWMWrite(fd, 1, 0, 2040);
	pca9685PWMWrite(fd, 2, 0, 2040);
	pca9685PWMWrite(fd, 3, 0, 2040);
	rxFrameDone = 0;
}

void readRx()
{
	rxFrameDone = 0;
	char val = serialGetchar(fds);
	if (ibusIndex == 0 && val != 0x20)
	{
		ibusIndex = 0;
		return;
	}
	if (ibusIndex == 1 && val != 0x40)
	{
		ibusIndex = 0;
		return;
	}
	if (ibusIndex == IBUS_BUFFERSIZE)
	{
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
	if (val < 0)
	{
		printf("neprimam");
	}
	else
	{
		ibus[ibusIndex] = val;
		ibusIndex++;
	}
}
