# ======================================================================
#  Builds the boot code memory file for Xilinx Vivado EDA tools.
#
#  Chun-Jen Tsai, Nov/29/2021
# ======================================================================

CROSS = riscv64-unknown-elf
CCPATH = $(RV64)/bin

CC = $(CCPATH)/$(CROSS)-gcc
LD = $(CCPATH)/$(CROSS)-ld
OD = $(CCPATH)/$(CROSS)-objdump
OC = $(CCPATH)/$(CROSS)-objcopy

CCFLAGS = -Wall -O2 -fomit-frame-pointer -fno-builtin -march=rv64ima_zicsr_zifencei -mstrict-align -mcmodel=medany
LDFLAGS = -T$(PROJ).ld -Map=$(PROJ).map

OCFLAGS = -O binary
ODFLAGS = -d

PROJ = uartboot
OBJS = boot.o $(PROJ).o io_uart.o

all:
	make $(PROJ).mem

clean:
	rm -f *.mem *.objdump *.map *.o

%.o: %.c $(PROJ).ld
	$(CC) $(CCFLAGS) -c $< -o $@

$(PROJ).elf: $(OBJS)
	$(LD) $(LDFLAGS) $(OBJS) -o $@
	rm $^

$(PROJ).bin: $(PROJ).elf
	$(OC) $(OCFLAGS) $< $@
	$(OD) $(ODFLAGS) $< > $(PROJ).objdump
	rm $<

$(PROJ).mem: $(PROJ).bin
	$(eval BIN_SIZE = $(shell stat -c %s $(PROJ).bin))
	hexdump -ve '1/4 "%08x\n"' -n $(BIN_SIZE) $(PROJ).bin > $@
	./convert2mem < $(PROJ).mem > $(PROJ)_64.mem
	rm $<
