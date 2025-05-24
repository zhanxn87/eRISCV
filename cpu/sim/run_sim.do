#[Library]
vlib work
vmap work work

#[vcom]

#[vlog]
vlog +acc -work work -incr -f file.list

#[vsim]
vsim -lib work -t 1ps +tc=I-BGE-01 riscv_tb

set IterationLimit 1000

do wave.do

run -all
