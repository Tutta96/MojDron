#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <wiringSerial.h>
#include <string.h>

#define IBUS_BUFFSIZE 32 // Max iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
#define IBUS_MAXCHANNELS 10

#define THROT_CHANNEL 2
#define RUDDER_CHANNEL 3

#define LEFT_PWM_PIN 5
#define RIGHT_PWM_PIN 6
#define LEFT_DIR_PIN 7
#define RIGHT_DIR_PIN 8

static int ibusIndex = 0;
static char ibus[IBUS_BUFFSIZE] = {0};
static short rcValue[IBUS_MAXCHANNELS];

static int rxFrameDone;

int ThrotLeft = 0;
int ThrotRight = 0;
int DirLeft = 1;
int DirRight = 1;


  int fd ;


void readRx()
{
  rxFrameDone = 0;

  if (1)
  {
    char val = serialGetchar(fd);
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
      rcValue[7] = (ibus[17] << 8) + ibus[16];
      rcValue[8] = (ibus[19] << 8) + ibus[18];
      rcValue[9] = (ibus[21] << 8) + ibus[20];
    }else{
      ibus[ibusIndex] = val;
      ibusIndex++;
    }

    
    return;
  }
}
int main ()
{

  if ((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }

// Loop, getting and printing characters

  for (;;)
  {
	  readRx();
  if ( rxFrameDone ){
    printf("%d\n",rcValue[3]);
    fflush (stdout) ;
  }
  }
}
