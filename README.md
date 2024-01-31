# FemtoRV - RISCV Processor on Artix 35T

RISC-V is a free and open ISA enabling a new era of processor innovation through open standard collaboration. Recently it has gained popularity with major players like Intel, Apple, and NASA implementing RISC-V solutions.

The work in the repository wouldn't be possible without @BrunoLevy and his work at [BrunoLevy/learn-fpga](https://github.com/BrunoLevy/learn-fpga)
The project by him provides an excellent step-wise explanation of developing a RISC-V and serves as the base on which this project is created.

The project implementes FemtoRV on the ARTIX7 35T FPGA using the F4PGA toolchain.

## Required Software

### F4PGA

*F4PGA is an end-to-end FPGA synthesis toolchain, because of that it provides all the necessary tools to convert input Hardware Description Language (HDL) sources into a final bitstream*

![F4PGA Toolchain design flow](Docs/toolchain-flow.svg)

- [F4PGA Home Page](https://f4pga.readthedocs.io/en/latest/getting-started.html)
- [Installation guide](https://f4pga-examples.readthedocs.io/en/latest/getting.html#getting)
- [Basic Tutorial](https://f4pga-examples.readthedocs.io/en/latest/personal-designs.html)

| Tool name           | Description                                                                                                      | Link                                                      |
|---------------------|------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------|
| Icarus Verilog      | Icarus Verilog is a Verilog simulation and synthesis tool. It operates as a compiler, compiling source code written in Verilog (IEEE-1364) into some target format. For batch simulation, the compiler can generate an intermediate form called vvp assembly | [Icarus Verilog](https://steveicarus.github.io/iverilog/index.html)              |
| Verilator           | Verilator is a free and open-source software tool which converts Verilog to a cycle-accurate behavioral model in C++ or SystemC. | [Verilator](https://www.veripool.org/verilator/)          |
| Picocom             | picocom is a minimal dumb-terminal emulation program                                                            | [Picocom](https://linux.die.net/man/8/picocom)            |
| Riscv GNU toolchain | RISC-V C and C++ cross-compiler                                                                                 | [Riscv GNU toolchain](https://github.com/riscv-collab/riscv-gnu-toolchain) |

## Activate F$PGA env

```bash
export F4PGA_INSTALL_DIR=~/opt/f4pga
export FPGA_FAM="xc7"
export TARGET="arty_35"
source "$F4PGA_INSTALL_DIR/$FPGA_FAM/conda/etc/profile.d/conda.sh"
conda activate $FPGA_FAM
```

## makefile commands

- Compile bitsteam

```bash
make -C .
```

- Clean build

```bash
make clean
```

- Upload bitstream

```bash
make download
```

- Start Terminal

```bash
make terminal
```

- Write to flash

```bash
make download_flash
```

### Compiling C code

### SPI FLASH

