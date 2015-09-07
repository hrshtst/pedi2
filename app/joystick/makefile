include ../../config

ROOTDIR=../..

INCLUDE=`pedi2-config -I`
LIB=`pedi2-config -L`
LINK=`pedi2-config -l` `rokigl-config -l` -laviator

CC=gcc
CFLAGS=-Wall -O3 $(LIB) $(INCLUDE) -funroll-loops

TARGET=joystick_ctrl

all: $(TARGET)

joystick_ctrl: joystick_ctrl.c
	$(CC) $(CFLAGS) -o $@ $^ $(LINK)
clean:
	rm -f *.o *~ core $(TARGET)
