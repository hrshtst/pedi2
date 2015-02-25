include config

ROOTDIR=.
INCDIR=$(ROOTDIR)/include/pedi2
SRCDIR=$(ROOTDIR)/src
LIBDIR=$(ROOTDIR)/lib

all:
	@cd $(SRCDIR); make

clean:
	-@rm -f $(ROOTDIR)/*~ $(INCDIR)/*~
	@cd $(SRCDIR); make clean
	-@rm -f $(LIBDIR)/*.so
install:
	-install -m 755 $(LIBDIR)/*.so $(PREFIX)/lib/
	-install -m 755 -d $(PREFIX)/include/$(PROJNAME)
	-install -m 644 $(INCDIR)/*.h $(PREFIX)/include/$(PROJNAME)/
