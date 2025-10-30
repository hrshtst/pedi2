CFLAGS=`pedi2-config --cflags`
LINK=`pedi2-config -l`
LINK_ROKI_GL=`roki-gl-config -l`

TARGET=$(shell ls *.c | xargs -i basename {} .c | tr -s "\n" " ")

CC=gcc
CFLAGS+=-Wall -O3 -funroll-loops

all: $(TARGET)
pd_cz_view_test: pd_cz_view_test.c
	$(CC) $(CFLAGS) -o $@ $< $(LINK) $(LINK_ROKI_GL)
%: %.c
	$(CC) $(CFLAGS) -o $@ $< $(LINK)
clean :
	rm -f *.o *~ core *test $(TARGET)
