include ../config

ROOTDIR=..

INCLUDE=-I$(ROOTDIR)/include -I$(PREFIX)/include
LIB=-L$(ROOTDIR)/lib -L$(PREFIX)/lib
LINK=-lpedi2 `glroki-config`

CC=gcc
CFLAGS=-Wall -O3 $(LIB) $(INCLUDE) -funroll-loops

VPATH=util
ctrl_view_test: ctrl_view_test.c scene.o
	$(CC) $(CFLAGS) -o $@ $^ $(LINK)
%: %.c
	$(CC) $(CFLAGS) -o $@ $< $(LINK)
%.o: %.c
	$(CC) $(CFLAGS) -c $<
clean :
	rm -f *.o *~ core *test
