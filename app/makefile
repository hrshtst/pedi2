include ../config

ROOTDIR=..

INCLUDE=-I$(ROOTDIR)/include -I$(PREFIX)/include
LIB=-L$(ROOTDIR)/lib -L$(PREFIX)/lib
LINK=-lpedi2 `glroki-config`

CC=gcc
CFLAGS=-Wall -O3 $(LIB) $(INCLUDE) -funroll-loops

OBJS=
TARGET=dynmorph

VPATH=util
all: $(TARGET)

%.o: %.c %.h
	$(CC) $(CFLAGS) -c $<
dynmorph: dynmorph.c $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LINK)
clean :
	rm -f *.o *~ core $(TARGET)
