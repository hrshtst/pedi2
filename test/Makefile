include ../config

ROOTDIR=..
INCDIR=$(ROOTDIR)/include
SRCDIR=$(ROOTDIR)/src
LIBDIR=$(ROOTDIR)/lib
INCLUDE=-I$(INCDIR) -I$(PREFIX)/include
LIBRARY=-L$(LIBDIR)/lib -L$(PREFIX)/lib
LINK = `roki-config`

TESTDIR=.
MOCKDIR=$(TESTDIR)/mocks

GTESTDIR=$(TESTDIR)/gtest
GTEST_SRC_DIR = $(GTESTDIR)/src
GTEST_INC_DIR = $(GTESTDIR)/include

CXX=g++
CPPFLAGS += -isystem $(GTEST_INC_DIR)
CXXFLAGS += -g -Wall -Wextra -pthread

# Edit below when you add a new test
TESTS=pdVrtTest pdHrzTest pdCtrlTest
# Add depending objects (make sure $(SRCDIR)/makefile)
OBJ=pd_vrt.o pd_hrz.o pd_ctrl.o

TESTOBJ=$(addsuffix .o, $(TESTS))
LIBOBJ=$(addprefix $(SRCDIR)/, $(OBJ))

GTEST_HEADERS =\
	$(GTEST_INC_DIR)/gtest/*.h\
	$(GTEST_INC_DIR)/gtest/internal/*.h

.PHONY: all
all: $(TESTS)

.PHONY: test
test: $(TESTS)
	@./test_runner.sh

.PHONY: debug
debug:
	@echo $(addprefix $(SRCDIR)/, $(OBJ))
	@echo $(addsuffix .o, $(TESTS))

.PHONY: clean
clean :
	rm -f $(TESTS) gtest.a gtest_main.a *.o

GTEST_SRCS_ = \
	$(GTEST_SRC_DIR)/*.cc\
	$(GTEST_SRC_DIR)/*.h\
	$(GTEST_HEADERS)
gtest-all.o : $(GTEST_SRCS_)
	$(CXX) $(CPPFLAGS) -I$(GTESTDIR) $(CXXFLAGS) -c \
	    $(GTEST_SRC_DIR)/gtest-all.cc

gtest_main.o : $(GTEST_SRCS_)
	$(CXX) $(CPPFLAGS) -I$(GTESTDIR) $(CXXFLAGS) -c \
	    $(GTEST_SRC_DIR)/gtest_main.cc

gtest.a : gtest-all.o
	$(AR) $(ARFLAGS) $@ $^

gtest_main.a : gtest-all.o gtest_main.o
	$(AR) $(ARFLAGS) $@ $^

LIBOBJ_SRCS_=$(INCDIR)/pedi2/*.h $(SRCDIR)/*.c
$(LIBOBJ): $(LIBOBJ_SRCS_)
	@cd $(SRCDIR); make

$(TESTS): $(TESTOBJ) $(LIBOBJ) gtest_main.a
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LIBRARY) $@.o $(LIBOBJ) gtest_main.a -o $@ -lpthread $(LINK)

%.o: %.cpp
	$(CXX) $(CPPFLAGS) $(INCLUDE) $(CXXFLAGS) -c $<
