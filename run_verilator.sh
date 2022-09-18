(cd obj_dir; rm -f *.cpp *.o *.a VSOC; make -f VSOC.mk)
verilator -DBENCH -DBOARD_FREQ=12 -Wno-fatal --top-module SOC -cc -exe sim_main.cpp femtoRV.v
(cd obj_dir; make -f VSOC.mk)
./VSOC
