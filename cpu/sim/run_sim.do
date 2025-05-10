#[Library]
vlib work
vmap work work

#[vcom]

#[vlog]
vlog +acc -work work -incr -f file.list

#[vsim]
vsim -lib work -t 1ps +instr_init=./testcases/I-ADD-01/I-ADD-01.elf.mem riscv_tb

set IterationLimit 1000

do wave.do

run -all
