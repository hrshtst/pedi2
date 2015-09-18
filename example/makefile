include ../config

ROOTDIR=..

INCLUDE=`pedi2-config -I`
LIB=`pedi2-config -L`
LINK=`pedi2-config -l` `rokigl-config -l` `dzco-config -l`

CC=gcc
CFLAGS=-Wall -O3 $(LIB) $(INCLUDE) -funroll-loops

%: %.c
	$(CC) $(CFLAGS) -o $@ $< $(LINK)
clean :
	rm -f *.o *~ core *test
