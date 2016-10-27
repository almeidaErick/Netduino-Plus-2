# Add additional project sources like this:
# SRCS += X.c
#
# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)
MYLIB=$(HOME)/repo/mylib
NRF24L01PLUS_PATH=$(NP2_ROOT)/src/peripherals/nrf24l01plus
#location of freertos source files
FREERTOS_PATH=$(NP2_ROOT)/src/FreeRTOS

#Name of project
PROJ_NAME=main

#Your local source files
SRCS = $(PROJ_NAME).c


#Include your mylib header files
CFLAGS = -I$(MYLIB)
CFLAGS += -DENABLE_VCP	#enable USB Virtual Com Port
CFLAGS += -I$(NRF24L01PLUS_PATH)
CFLAGS += -I$(FREERTOS_PATH)
###################################################
ROOT=$(shell pwd)

OBJS=$(SRCS:.c=.o)

#Extra files from the src folder must ONLY be added to LIBSRCS
LIBSRCS += $(FREERTOS_PATH)/portable/MemMang/heap_1.c
LIBSRCS += $(wildcard $(NRF24L01PLUS_PATH)/*.c)
###################################################

.PHONY: $(shell mkdir -p obj) lib proj all depend
all: proj

#Include freertos makefile
include $(NP2_ROOT)/src/FreeRTOS/Makefile.freertos


#include NP2 makefile
include $(NP2_ROOT)/src/boards/netduinoplus2/Makefile.np2

#include radio makefile

lib:
	$(MAKE) -C lib

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf:	$(NP2OBJ) $(OBJS)
	$(CC) $(LDFLAGS) -o $@ -Llib $(OBJS) $(NP2OBJ) $(LD_SYS_LIBS)
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

%.o:
	$(CC) $(CFLAGS) -c $< -o $@

depend: .depend

.depend: $(SRCS)
	rm -f ./.depend
	mkdir -p obj
	$(CC) $(CFLAGS) -MM $^>>./.depend;

prog:
	sudo dfu-util -d 0483:df11 -c 1 -i 0 -a 0 -s 0x08000000 -D $(PROJ_NAME).bin		#Program NP2

pron:
	sudo ~/gitdev/stlink/st-flash write $(PROJ_NAME).bin 0x8000000	#Program Nucleo

qemu:
	np2-qemu $(PROJ_NAME).elf

gdb:
	np2-gdb-qemu $(PROJ_NAME).elf

clean:
	rm -f *.o
	rm -f $(OBJDIR)/*.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
	rm -f $(LIBNP2)
	rm -rf .depend
	rmdir $(OBJDIR)


include .depend
