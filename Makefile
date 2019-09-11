CC=gcc
CFLAGS=-Wall
LLSM=-DSERIES1 -llsm9ds1
RPI=-lwiringPi -lwiringPiPca9685 -lrt -lm

allinone: allinone.c
		$(CC) -o $@ $(CFLAGS) $^ $(RPI) $(LLSM)

.PHONY: clean

clean:
		rm -rf allinone
