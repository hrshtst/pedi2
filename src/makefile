include ../config

ROOTDIR=..
LIBDIR=$(ROOTDIR)/lib
INCLUDE=-I$(ROOTDIR)/include -I$(PREFIX)/include

CC=gcc
CFLAGS=-ansi -Wall -fPIC -O3 $(INCLUDE) -funroll-loops
LD=gcc
LDFLAGS=-shared

OBJ=pd_cz_vrt.o \
    pd_cz_hrz_u.o pd_cz_hrz_w.o pd_cz_hrz_uw.o pd_cz_hrz.o \
    pd_cz.o \
    pd_foot_uw.o pd_foot_z.o pd_foot.o \
    pd_state.o \
    pd_cmd.o \
    pd_mode.o pd_biped.o \
    pd_filter.o pd_filter_none.o pd_filter_bw.o \
    pd_sensor.o pd_sensor_dummy.o pd_sensor_6ft.o \
    pd_est_zmp.o \
    pd_joint.o pd_joint_pd_trq.o pd_joint_pid_trq.o \
    pd_robot.o
DLIB=libpedi2.so

TARGET=$(DLIB)

all: $(TARGET)

$(DLIB): $(OBJ)
	@echo " LD	" $^
	@$(LD) $(LDFLAGS) -o $@ $^ > /dev/null
	-@mv $@ $(LIBDIR)
%.o: %.c
	@echo " CC	" $<
	@$(CC) $(CFLAGS) -c $< > /dev/null
clean:
	@echo " CLEAN"
	-@rm -f *.o *~ $(TARGET)
