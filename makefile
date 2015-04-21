CONFIG:=$(shell test -e config || cp config.org config; echo config)
include $(CONFIG)

ROOTDIR:=.
INCDIR:=$(ROOTDIR)/include/$(PROJNAME)
SRCDIR:=$(ROOTDIR)/src
LIBDIR:=$(ROOTDIR)/lib
APPDIR:=$(ROOTDIR)/app

all:
	@cd $(SRCDIR); make
	@cd $(APPDIR); make

clean:
	-@rm -f $(ROOTDIR)/*~ $(INCDIR)/*~
	@cd $(SRCDIR); make clean
	-@rm -f $(LIBDIR)/*.so
	@cd $(APPDIR); make clean
install:
	@echo " INSTALL library"
	-@install -m 755 $(LIBDIR)/*.so $(PREFIX)/lib/
	@echo " INSTALL header files"
	-@install -m 755 -d $(PREFIX)/include/$(PROJNAME)
	-@install -m 644 $(INCDIR)/*.h $(PREFIX)/include/$(PROJNAME)/
	@echo " INSTALL applications"
	@cd $(APPDIR); make install
