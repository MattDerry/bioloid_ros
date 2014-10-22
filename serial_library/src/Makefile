TARGETS		= libzgb.a libzgb.so
OBJS		= zgb_hal.o zigbee.o
SRCS		= $(OBJS:.o=.c)
INCLUDEDIRS	+= -I../include
LIBDIRS		+= 
CFLAGS		= $(INCLUDEDIRS) -W -Wall -O2

CC			= gcc
AR			= ar

all: $(TARGETS)

libzgb.a: $(OBJS)
	$(AR) rs $@ $^
	cp libzgb.a ../lib

libzgb.so: $(OBJS)
	gcc -fPIC -shared -o $@ $^
	cp libzgb.so ../lib
	
.c.o:
	$(CC) -fPIC -c $< $(CFLAGS)
	
clean:
	rm -f $(OBJS) $(TARGET) libzgb.so
	@echo "file deleted."

dep:
	gccmakedep $(SRCS)

