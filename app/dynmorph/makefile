include ../../config

ROOTDIR=../..

INCLUDE=`pedi2-config -I`
LIB=`pedi2-config -L`
LINK=`pedi2-config -l` `rokigl-config -l`

CC=gcc
CFLAGS=-Wall -O3 $(LIB) $(INCLUDE) -funroll-loops

OBJS=dm_scene.o dm_console.o
TARGET=dynmorph

VPATH=util
all: $(TARGET)

%.o: %.c %.h
	$(CC) $(CFLAGS) -c $<
dynmorph: dynmorph.c $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LINK)
clean :
	rm -f *.o *~ core $(TARGET)
