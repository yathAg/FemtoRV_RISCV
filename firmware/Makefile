ARCH = rv32i
ABI = ilp32
RVASFLAGS= -march=$(ARCH) -mabi=$(ABI) 



RAM_SIZE=6144

LIBOBJECTS=putchar.o wait.o perf.o # print.o memcpy.o errno.o

RVAS= riscv64-unknown-elf-as
RVLD= riscv64-unknown-elf-ld
RVOBJCOPY= riscv64-unknown-elf-objcopy
RVOBJDUMP= riscv64-unknown-elf-objdump
RVGCC= riscv64-unknown-elf-gcc
RVGPP= riscv64-unknown-elf-g++
RVAR= riscv64-unknown-elf-ar
RVRANLIB= riscv64-unknown-elf-ranlib

# RV_BINARIES=$(RVAS) $(RVLD) $(RVOBJCOPY) $(RVOBJDUMP) $(RVGCC) \
#             $(RVAR) $(RVRANLIB)

# #Rule to compile ASM objects
.S.o: $<
	riscv64-unknown-elf-as -march=rv32i -mabi=ilp32 $< -o $@ 

%.bram.elf: %.o start.o $(LIBOBJECTS) 
	$(RVLD) -T bram.ld -m elf32lriscv -nostdlib -norelax $< $(LIBOBJECTS) ./libgcc.a -o $@


%.hex: %.elf 
	/home/yatharth/vlsi_projects/learn-fpga/FemtoRV/FIRMWARE//TOOLS/firmware_words $< -ram $(RAM_SIZE) -max_addr $(RAM_SIZE) -out $@
	cp $@ ../firmware.hex
	mkdir -p ../obj_dir
	cp $@ ../obj_dir/firmware.hex
	echo $@ > ../firmware.txt


echo:
	$(RVAS) $(RVASFLAGS)
