current_dir := ${CURDIR}
TOP := SOC
SOURCES := ${current_dir}/${file}
XDC := ${current_dir}/arty.xdc
TARGET:= arty_35

include ~/f4pga-examples/common/common.mk

# .PHONY: clean
# clean:
# 	rm -rf ./build
