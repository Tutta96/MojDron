#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringSerial.h>
#include <string.h>
#include <pca9685.h>
#define IBUS_BUFFSIZE 32 // Max iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
#define IBUS_MAXCHANNELS 7



static int ibusIndex = 0;
static char ibus[IBUS_BUFFSIZE] = {0};
static short rcValue[IBUS_MAXCHANNELS];
const int pinBase = 300;
const int i2caddres = 0x40;
float freq = 50;
static int rxFrameDone;
int fd;
int fdd ;
int br;
int bro;





void readRx()
{
  rxFrameDone = 0;

  if (1)
  {
    char val = serialGetchar(fdd);
    // Look for 0x2040 as start of packet
    if (ibusIndex == 0 && val != 0x20) {
      ibusIndex = 0;
      return;
    }
    if (ibusIndex == 1 && val != 0x40) {
      ibusIndex = 0;
      return;
    }

    if ( ibusIndex == IBUS_BUFFSIZE ){
      rxFrameDone = 1;
       ibusIndex = 0;
      rcValue[0] = (ibus[ 3] << 8) + ibus[ 2];
      rcValue[1] = (ibus[ 5] << 8) + ibus[ 4];
      rcValue[2] = (ibus[ 7] << 8) + ibus[ 6];
      rcValue[3] = (ibus[ 9] << 8) + ibus[ 8];
      rcValue[4] = (ibus[11] << 8) + ibus[10];
      rcValue[5] = (ibus[13] << 8) + ibus[12];
      rcValue[6] = (ibus[15] << 8) + ibus[14];
       }else{
      ibus[ibusIndex] = val;
      ibusIndex++;
    }

    
    return;
  }
}


int main ()
{
  wiringPiSetup();
 if((fd = pca9685Setup(300,0x40,50)) < 0)
 {   printf("Ne dela");
    return fd;
}
pca9685PWMReset(fd);
  if ((fdd = serialOpen ("/dev/serial0", 115200)) < 0)
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }

// Loop, getting and printing characters

  for (;;)
  {
	  readRx();
  if ( rxFrameDone ){
    printf("%d   ",rcValue[2]);
    br = (rcValue[2]-1000);
    bro = (br*204)/1000;
    printf("%d \n", bro);
    fflush(stdout);
    pca9685PWMWrite(fd,3,0,(204+bro));
    delay(2);
    }
  }
}
