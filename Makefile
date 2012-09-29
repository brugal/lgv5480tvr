CC = gcc
INCLUDES = -I/usr/src/kernel-headers-2.4.17/include
CFLAGS = -Wall -D__KERNEL__ -DMODULE -O $(INCLUDES)
OBJS = lgv_5480tvr_v4l.o

all: $(OBJS)

clean:
	rm -f core $(OBJS)

