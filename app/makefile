include ../config

INCLUDE=-I. -I../include -I$(PREFIX)/include
LIB=-L../lib -L$(PREFIX)/lib
BINDIR=$(PREFIX)/bin

CC=gcc
CFLAGS=-ansi -Wall -O3 $(INCLUDE) $(LIB)

LINK=-lpedi2 -ldzco `roki-config -l`

COMPILE=$(CC) $(CFLAGS) -o $@ $< $(LINK)

INSTALL=install -m 755

CONFIGGEN=cure-config-gen
CONFIG=pedi2-config
TARGET=$(CONFIG)

#####################################################################
all : $(TARGET)

$(CONFIG):
	@echo " GENERATE" $@
	-@rm -f $@
	@$(CONFIGGEN) -I $(PREFIX)/include -L $(PREFIX)/lib -l "$(LINK)" -v $(VERSION) > $@

%: %.c
	@echo " CC	" $<
	@$(COMPILE)
install: $(TARGET)
	@echo " INSTALL" $^
	@$(INSTALL) $^ $(BINDIR)

clean :
	@echo " CLEAN"
	-@rm -f *.o *~ core $(TARGET)
