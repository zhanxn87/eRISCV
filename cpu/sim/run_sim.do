#[Library]
vlib work
vmap work work

#[vcom]

#[vlog]
vlog +acc -work work -incr -f file.list

#[vsim]
vsim -lib work -t 1ps +instr_init=instr_mem_tc1.hex riscv_tb

do wave.do

run -all
