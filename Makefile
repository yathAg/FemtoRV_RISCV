current_dir := ${CURDIR}
TOP := SOC
SOURCES := ${current_dir}/step22.v

#picocom variables
PICO_DEVICE=/dev/ttyUSB1   # replace by the terminal used by your device
BAUDS=1000000


ifeq ($(TARGET),arty_35)
ifdef F4PGA_USE_DEPRECATED
  XDC := ${current_dir}/arty.xdc
else
build:clean
	f4pga -vv build --flow ./flow.json
endif
else ifeq ($(TARGET),arty_100)
  XDC := ${current_dir}/arty.xdc
else ifeq ($(TARGET),nexys4ddr)
  XDC := ${current_dir}/nexys4ddr.xdc
else ifeq ($(TARGET),zybo)
  XDC := ${current_dir}/zybo.xdc
  SOURCES:=${current_dir}/counter_zynq.v
else ifeq ($(TARGET),nexys_video)
  XDC := ${current_dir}/nexys_video.xdc
else ifeq ($(TARGET),basys3)
  XDC := ${current_dir}/basys3.xdc
endif

include ${current_dir}/common.mk

bench:
	iverilog -DBENCH -DSIM -DPASSTHROUGH_PLL -DBOARD_FREQ=10 -DCPU_FREQ=10 -I ./includes/ bench_iverilog.v ${SOURCES}
	vvp a.out

terminal:
	sudo picocom -b $(BAUDS) $(PICO_DEVICE) --imap lfcrlf,crcrlf --omap delbs,crlf --send-cmd "ascii-xfr -s -l 30 -n"

clean_all:
	rm -rf ./build 
	rm -rf ./obj_dir 
	rm -rf *.log 
	rm -rf a.out
	rm -rf .f4cache

clean_verilate:
	rm -rf ./obj_dir

run_verilate: femtoRV.v sim_main.cpp
	verilator -DBENCH -DBOARD_FREQ=12 -Wno-fatal --top-module SOC -cc -exe sim_main.cpp femtoRV.v

build_verilate: run_verilate
	cd obj_dir && make -f VSOC.mk

verilate: build_verilate
	./obj_dir/VSOC