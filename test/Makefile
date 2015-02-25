PREFIX = $(HOME)/usr
INC_DIR = $(PREFIX)/include
LIB_DIR = $(PREFIX)/lib
LINK = `roki-config`

PROJ_ROOT = ..
PROJ_SRC_DIR = $(PROJ_ROOT)/src
PROJ_INC_DIR = $(PROJ_ROOT)/include

TEST_ROOT = .
TEST_SRC_DIR = $(TEST_ROOT)
TEST_INC_DIR = $(TEST_ROOT)

MOCK_ROOT = $(TEST_ROOT)/mocks
MOCK_SRC_DIR = $(MOCK_ROOT)
MOCK_INC_DIR = $(MOCK_ROOT)

GTEST_ROOT = $(TEST_ROOT)/gtest
GTEST_SRC_DIR = $(GTEST_ROOT)/src
GTEST_INC_DIR = $(GTEST_ROOT)/include

CPPFLAGS += -isystem $(GTEST_INC_DIR)
CXXFLAGS += -g -Wall -Wextra -pthread

TESTS = pedi2CtrlTest

GTEST_HEADERS =\
	$(GTEST_INC_DIR)/gtest/*.h\
	$(GTEST_INC_DIR)/gtest/internal/*.h

all : $(TESTS)

clean :
	rm -f $(TESTS) gtest.a gtest_main.a *.o

GTEST_SRCS_ = \
	$(GTEST_SRC_DIR)/*.cc\
	$(GTEST_SRC_DIR)/*.h\
	$(GTEST_HEADERS)
gtest-all.o : $(GTEST_SRCS_)
	$(CXX) $(CPPFLAGS) -I$(GTEST_ROOT) $(CXXFLAGS) -c \
	    $(GTEST_SRC_DIR)/gtest-all.cc

gtest_main.o : $(GTEST_SRCS_)
	$(CXX) $(CPPFLAGS) -I$(GTEST_ROOT) $(CXXFLAGS) -c \
	    $(GTEST_SRC_DIR)/gtest_main.cc

gtest.a : gtest-all.o
	$(AR) $(ARFLAGS) $@ $^

gtest_main.a : gtest-all.o gtest_main.o
	$(AR) $(ARFLAGS) $@ $^

pedi2_ctrl.o : $(PROJ_SRC_DIR)/pedi2_ctrl.c\
	     $(PROJ_INC_DIR)/pedi2/pedi2_ctrl.h
	$(CXX) $(CPPFLAGS) -I$(INC_DIR) -I$(PROJ_INC_DIR) -L$(LIB_DIR) $(CXXFLAGS) -c $(PROJ_SRC_DIR)/pedi2_ctrl.c

pedi2CtrlTest.o : $(TEST_SRC_DIR)/pedi2CtrlTest.cpp\
		$(PROJ_INC_DIR)/pedi2/pedi2_ctrl.h\
		$(GTEST_HEADERS)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -I$(INC_DIR) -I$(PROJ_INC_DIR) -L$(LIB_DIR) -c $(TEST_SRC_DIR)/pedi2CtrlTest.cpp

pedi2CtrlTest : pedi2_ctrl.o pedi2CtrlTest.o gtest_main.a
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -L$(LIB_DIR) -lpthread $^ -o $@ $(LINK)

# LedDriver.o :	$(PROJ_SRC_DIR)/LedDriver.c\
#		$(PROJ_INC_DIR)/LedDriver.h\
#		$(PROJ_INC_DIR)/RuntimeError.h
#	$(CXX) $(CPPFLAGS) -I$(PROJ_INC_DIR) $(CXXFLAGS) -c $(PROJ_SRC_DIR)/LedDriver.c

# LedDriverTest.o : $(TEST_SRC_DIR)/LedDriverTest.cc\
#		$(PROJ_INC_DIR)/LedDriver.h\
#		$(MOCK_INC_DIR)/RuntimeErrorStub.h\
#		$(MOCK_SRC_DIR)/RuntimeErrorStub.c\
#		$(GTEST_HEADERS)
#	$(CXX) $(CPPFLAGS) -I$(PROJ_INC_DIR) -I$(MOCK_INC_DIR) $(CXXFLAGS) -c $(TEST_SRC_DIR)/LedDriverTest.cc

# RuntimeErrorStub.o : $(MOCK_SRC_DIR)/RuntimeErrorStub.c\
#		$(MOCK_INC_DIR)/RuntimeErrorStub.h
#	$(CXX) $(CPPFLAGS) -I$(MOCK_INC_DIR) $(CXXFLAGS) -c $(MOCK_SRC_DIR)/RuntimeErrorStub.c

# LedDriverTest : LedDriver.o RuntimeErrorStub.o LedDriverTest.o gtest_main.a
#	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -lpthread $^ -o $@
