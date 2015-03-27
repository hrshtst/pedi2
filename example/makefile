include ../config

ROOTDIR=..

INCLUDE=-I$(ROOTDIR)/include -I$(PREFIX)/include
LIB=-L$(ROOTDIR)/lib -L$(PREFIX)/lib
LINK=-lpedi2 -lm `glroki-config`

CC=gcc
CFLAGS=-Wall -O3 $(LIB) $(INCLUDE) -funroll-loops

VPATH=util
%: %.c
	$(CC) $(CFLAGS) -o $@ $< $(LINK)
%.o: %.c
	$(CC) $(CFLAGS) -c $<
clean :
	rm -f *.o *~ core *test
