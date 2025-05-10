onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /riscv_tb/dut/riscv_core_i/if_stage_i/pc_set_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/if_stage_i/pc_mux_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/if_stage_i/exc_pc_mux_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/if_stage_i/halt_if_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/if_stage_i/pc_if_o
add wave -noupdate /riscv_tb/dut/riscv_core_i/if_stage_i/pc_id_o
add wave -noupdate /riscv_tb/dut/riscv_core_i/if_stage_i/instr_valid_id_o
add wave -noupdate /riscv_tb/dut/riscv_core_i/if_stage_i/instr_rdata_id_o
add wave -noupdate /riscv_tb/dut/riscv_core_i/id_stage_i/is_decoding_i
add wave -noupdate -color Magenta -itemcolor Magenta /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/opcode
add wave -noupdate /riscv_tb/dut/riscv_core_i/id_stage_i/rega_used_dec
add wave -noupdate /riscv_tb/dut/riscv_core_i/id_stage_i/regb_used_dec
add wave -noupdate -radix unsigned /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_addr_ra_id
add wave -noupdate -radix unsigned /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_addr_rb_id
add wave -noupdate /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_alu_we_id
add wave -noupdate /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_lsu_we_id
add wave -noupdate -radix unsigned /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_waddr_id
add wave -noupdate /riscv_tb/dut/riscv_core_i/csr_access
add wave -noupdate /riscv_tb/dut/riscv_core_i/csr_addr_int
add wave -noupdate -color Magenta -itemcolor Magenta /riscv_tb/dut/riscv_core_i/csr_addr
add wave -noupdate /riscv_tb/dut/riscv_core_i/csr_wdata
add wave -noupdate -color Magenta -itemcolor Magenta /riscv_tb/dut/riscv_core_i/csr_op
add wave -noupdate /riscv_tb/dut/riscv_core_i/csr_rdata
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/clk
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/rst_n
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/hart_id_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mtvec_o
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/utvec_o
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mtvec_mode_o
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/utvec_mode_o
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mtvec_addr_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/csr_mtvec_init_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/csr_addr_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/csr_wdata_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/csr_op_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/csr_rdata_o
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mie_bypass_o
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mip_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/m_irq_enable_o
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/u_irq_enable_o
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mepc_o
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/uepc_o
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/priv_lvl_o
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/pc_if_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/pc_id_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/pc_ex_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/csr_save_if_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/csr_save_id_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/csr_save_ex_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/csr_restore_mret_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/csr_restore_uret_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/csr_cause_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/csr_save_cause_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmevent_minstret_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmevent_load_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmevent_store_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmevent_jump_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmevent_branch_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmevent_branch_taken_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmevent_compressed_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmevent_jr_stall_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmevent_imiss_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmevent_ld_stall_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmevent_pipe_stall_i
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/csr_wdata_int
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/csr_rdata_int
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/csr_we_int
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mepc_q
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mepc_n
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/uepc_q
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/uepc_n
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/exception_pc
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mstatus_q
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mstatus_n
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mstatus_we_int
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mcause_q
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mcause_n
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/ucause_q
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/ucause_n
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mtvec_n
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mtvec_q
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/utvec_n
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/utvec_q
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mtvec_mode_n
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mtvec_mode_q
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/utvec_mode_n
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/utvec_mode_q
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mscratch_q
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mscratch_n
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mip
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mie_q
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mie_n
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/csr_mie_wdata
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/csr_mie_we
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/is_irq
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/priv_lvl_n
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/priv_lvl_q
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmcounter_q
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmevent_q
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmevent_n
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mcounteren_q
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mcounteren_n
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mcountinhibit_q
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mcountinhibit_n
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/hpm_events
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmcounter_increment
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmcounter_write_lower
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmcounter_write_upper
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmcounter_write_increment
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mcounteren_we
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mcountinhibit_we
add wave -noupdate -group csr /riscv_tb/dut/riscv_core_i/cs_registers_i/mhpmevent_we
add wave -noupdate -expand -group alu -color Magenta -itemcolor Magenta /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_i/i_alu_op
add wave -noupdate -expand -group alu /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_i/i_operand_a
add wave -noupdate -expand -group alu /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_i/i_operand_b
add wave -noupdate -expand -group alu /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_i/o_result
add wave -noupdate -expand -group alu /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_i/o_compare_result
add wave -noupdate -expand -group alu /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_i/o_zero_flag
add wave -noupdate /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_alu_we_ex_o
add wave -noupdate /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_alu_waddr_ex_o
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_alu_we_fw_o
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_alu_waddr_fw_o
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_alu_wdata_fw_o
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_req_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_we_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_type_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_wdata_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_addr
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_reg_offset_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_sign_ext_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_rdata
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_we_i
add wave -noupdate -radix unsigned /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_waddr_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_we_wb_o
add wave -noupdate -radix unsigned /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_waddr_wb_o
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_wdata_wb_o
add wave -noupdate -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/clk
add wave -noupdate -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/rst_n
add wave -noupdate -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/test_en_i
add wave -noupdate -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/raddr_a_i
add wave -noupdate -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/rdata_a_o
add wave -noupdate -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/raddr_b_i
add wave -noupdate -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/rdata_b_o
add wave -noupdate -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/we_a_i
add wave -noupdate -group regfile -radix unsigned /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/waddr_a_i
add wave -noupdate -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/wdata_a_i
add wave -noupdate -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/we_b_i
add wave -noupdate -group regfile -radix unsigned /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/waddr_b_i
add wave -noupdate -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/wdata_b_i
add wave -noupdate -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/rf_reg
add wave -noupdate -group data_mem_if /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_req_o
add wave -noupdate -group data_mem_if /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_addr_o
add wave -noupdate -group data_mem_if /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_we_o
add wave -noupdate -group data_mem_if /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_be_o
add wave -noupdate -group data_mem_if /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_wdata_o
add wave -noupdate -group data_mem_if /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_rvalid_i
add wave -noupdate -group data_mem_if /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_rdata_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/jump_mode_id
add wave -noupdate /riscv_tb/dut/riscv_core_i/jump_target_id
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/branch_in_ex_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/branch_decision_o
add wave -noupdate /riscv_tb/dut/riscv_core_i/branch_target_ex
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/clk
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/rst_n
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/fetch_enable_i
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/ctrl_busy_o
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/is_decoding_o
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/illegal_insn_i
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/eret_insn_i
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/pipe_flush_i
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/jump_mode_id_i
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/branch_taken_ex_i
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/instr_req_o
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/pc_set_o
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/pc_mux_o
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/instr_valid_i
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/exc_req_i
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/exc_ack_o
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/exc_save_if_o
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/exc_save_id_o
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/exc_restore_id_o
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/halt_if_o
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/halt_id_o
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/jr_stall_i
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/load_stall_i
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/id_ready_i
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/ex_valid_i
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/perf_jump_o
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/perf_jr_stall_o
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/perf_ld_stall_o
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/ctrl_fsm_cs
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/ctrl_fsm_ns
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/jump_done
add wave -noupdate -group controller /riscv_tb/dut/riscv_core_i/controller_i/jump_done_q
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/clk
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/rst_n
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/boot_addr_i
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/req_i
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/pc_set_i
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/pc_mux_i
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/exc_pc_mux_i
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/exc_vec_pc_mux_i
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/exception_pc_reg_i
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/jump_target_id_i
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/branch_target_ex_i
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/halt_if_i
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/id_ready_i
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/clear_instr_valid_i
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/instr_req_o
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/instr_addr_o
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/instr_rvalid_i
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/instr_rdata_i
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/pc_if_o
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/pc_id_o
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/instr_valid_id_o
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/instr_rdata_id_o
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/if_ready_o
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/if_valid_o
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/if_busy_o
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/perf_imiss_o
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/illegal_c_insn_id_o
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/offset_fsm_ns
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/offset_fsm_cs
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/illegal_c_insn
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/branch_req
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/fetch_addr_n
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/fetch_valid
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/fetch_addr
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/exc_pc
add wave -noupdate -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/pc_reg
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/clk
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/rst_n
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/test_en_i
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/halt_id_i
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/illegal_insn_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/ebrk_insn_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/eret_insn_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/ecall_insn_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/pipe_flush_o
add wave -noupdate -group id_stage -color Magenta -itemcolor Magenta /riscv_tb/dut/riscv_core_i/id_stage_i/pc_id_i
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/instr_valid_i
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/instr_rdata_i
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/clear_instr_valid_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/id_ready_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/ex_ready_i
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/id_valid_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/branch_in_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/branch_target_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/jump_mode_id_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/jump_target_id_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_jump_offset
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_branch_offset
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/pc_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/alu_operator_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/alu_operand_a_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/alu_operand_b_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/alu_operand_c_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_alu_waddr_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_alu_we_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_alu_waddr_fw_i
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_alu_we_fw_i
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_alu_wdata_fw_i
add wave -noupdate -group id_stage -radix unsigned /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_lsu_waddr_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_lsu_we_ex_o
add wave -noupdate -group id_stage -radix unsigned /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_lsu_waddr_wb_i
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_lsu_we_wb_i
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_lsu_wdata_wb_i
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/csr_access_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/csr_op_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/data_req_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/data_we_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/data_type_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/data_sign_ext_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/data_reg_offset_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/data_misaligned_i
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/data_misaligned_ex_o
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/mult_multicycle_i
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/is_decoding_i
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/i_deassert_we
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/i_is_compressed
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_illegal_inst
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_ebrk_inst
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_ecall_inst
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_eret_inst
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_pipe_flush
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/i_inst
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/i_pc
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_jump_mode
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_branch_mode
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_jump_offset
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_branch_offset
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_alu_op
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_alu_op_a_sel
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_alu_op_b_sel
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_imma_extend
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_immb_extend
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/mult_operator_o
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/mult_int_en_o
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/mult_signed_mode_o
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/mult_sel_subword_o
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/mult_imm_mux_o
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/mult_dot_en_o
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/mult_dot_signed_o
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_rs1_en
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_rs2_en
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_rd_alu_we
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_rd_mem_we
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_rs1_addr
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_rs2_addr
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_rd_addr
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_mem_wr_en
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_mem_req
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_mem_dtype
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_load_data_sign_ext
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_csr_access
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_csr_op
add wave -noupdate -group id_stage -group decoder {/riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/instr[14]}
add wave -noupdate -group id_stage -group decoder {/riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/instr[13]}
add wave -noupdate -group id_stage -group decoder {/riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/instr[12]}
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/instr
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/imm_i_type
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/imm_iz_type
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/imm_s_type
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/imm_b_type
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/imm_u_type
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/imm_j_type
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/imm_z_type
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/imm_pc_incr
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/rd_mem_we
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/rd_alu_we
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/mem_wr_en
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/mem_req
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/branch_mode
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/jump_mode
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/csr_op
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/ebrk_inst
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/eret_inst
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/pipe_flush
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/opcode
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/funct_7
add wave -noupdate -group id_stage -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/funct_3
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/rega_used_dec
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/regb_used_dec
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/imm_a
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/imm_b
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/imm_jump_offset
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/jump_base
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/jump_target
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_addr_ra_id
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_addr_rb_id
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_waddr_id
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_alu_waddr_id
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_alu_we_id
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_data_ra_id
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_data_rb_id
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/alu_operator
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/alu_op_a_mux_sel
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/alu_op_b_mux_sel
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/mult_operator
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/mult_en
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/mult_int_en
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/mult_sel_subword
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/mult_signed_mode
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/mult_dot_en
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/mult_dot_signed
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/data_req_id
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/data_we_id
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/data_type_id
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/data_sign_ext_id
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/data_reg_offset_id
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/csr_access
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/csr_op
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/operand_a_fw_id
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/operand_b_fw_id
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/alu_operand_a
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/alu_operand_b
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/alu_operand_c
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/mult_imm_mux
add wave -noupdate -group id_stage /riscv_tb/dut/riscv_core_i/id_stage_i/mult_imm_id
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/clk
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/rst_n
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_operator_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_operand_a_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_operand_b_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_operator_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_operand_a_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_operand_b_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_en_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_sel_subword_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_signed_mode_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_imm_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_multicycle_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/branch_in_ex_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/branch_decision_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_alu_we_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_alu_waddr_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_alu_we_fw_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_alu_waddr_fw_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_alu_wdata_fw_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_we_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_waddr_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_we_wb_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_waddr_wb_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_wdata_wb_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/csr_access_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/csr_rdata_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_req_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_we_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_type_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_wdata_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_reg_offset_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_addr
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_rdata
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_sign_ext_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_misaligned_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_misaligned_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_load_err_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_store_err_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_busy_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_req_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_addr_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_we_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_be_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_wdata_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_rvalid_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_rdata_i
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/ex_ready_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/ex_valid_o
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_result
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_csr_result
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_result
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_cmp_result
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_ready
add wave -noupdate -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_ready
add wave -noupdate /riscv_tb/dut/riscv_core_i/id_stage_i/illegal_insn
add wave -noupdate /riscv_tb/dut/riscv_core_i/id_stage_i/load_stall
add wave -noupdate -expand -group data_mem /riscv_tb/dut/data_mem_i/clk
add wave -noupdate -expand -group data_mem /riscv_tb/dut/data_mem_i/rst_n
add wave -noupdate -expand -group data_mem /riscv_tb/dut/data_mem_i/req
add wave -noupdate -expand -group data_mem /riscv_tb/dut/data_mem_i/we
add wave -noupdate -expand -group data_mem /riscv_tb/dut/data_mem_i/be
add wave -noupdate -expand -group data_mem /riscv_tb/dut/data_mem_i/addr
add wave -noupdate -expand -group data_mem /riscv_tb/dut/data_mem_i/wdata
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[255]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[254]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[253]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[252]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[251]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[250]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[249]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[248]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[247]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[246]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[245]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[244]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[243]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[242]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[241]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[240]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[239]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[238]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[237]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[236]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[235]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[234]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[233]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[232]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[231]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[230]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[229]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[228]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[227]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[226]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[225]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[224]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[223]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[222]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[221]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[220]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[219]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[218]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[217]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[216]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[215]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[214]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[213]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[212]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[211]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[210]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[209]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[208]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[207]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[206]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[205]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[204]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[203]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[202]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[201]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[200]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[199]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[198]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[197]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[196]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[195]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[194]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[193]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[192]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[191]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[190]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[189]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[188]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[187]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[186]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[185]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[184]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[183]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[182]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[181]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[180]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[179]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[178]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[177]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[176]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[175]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[174]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[173]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[172]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[171]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[170]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[169]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[168]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[167]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[166]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[165]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[164]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[163]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[162]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[161]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[160]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[159]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[158]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[157]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[156]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[155]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[154]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[153]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[152]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[151]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[150]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[149]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[148]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[147]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[146]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[145]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[144]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[143]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[142]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[141]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[140]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[139]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[138]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[137]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[136]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[135]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[134]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[133]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[132]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[131]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[130]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[129]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[128]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[127]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[126]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[125]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[124]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[123]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[122]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[121]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[120]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[119]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[118]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[117]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[116]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[115]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[114]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[113]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[112]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[111]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[110]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[109]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[108]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[107]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[106]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[105]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[104]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[103]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[102]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[101]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[100]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[99]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[98]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[97]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[96]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[95]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[94]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[93]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[92]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[91]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[90]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[89]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[88]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[87]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[86]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[85]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[84]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[83]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[82]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[81]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[80]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[79]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[78]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[77]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[76]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[75]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[74]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[73]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[72]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[71]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[70]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[69]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[68]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[67]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[66]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[65]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[64]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[63]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[62]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[61]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[60]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[59]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[58]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[57]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[56]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[55]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[54]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[53]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[52]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[51]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[50]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[49]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[48]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[47]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[46]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[45]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[44]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[43]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[42]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[41]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[40]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[39]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[38]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[37]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[36]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[35]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[34]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[33]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[32]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[31]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[30]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[29]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[28]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[27]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[26]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[25]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[24]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[23]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[22]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[21]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[20]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[19]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[18]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[17]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[16]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[15]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[14]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[13]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[12]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[11]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[10]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[9]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[8]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[7]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[6]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[5]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[4]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[3]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[2]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[1]}
add wave -noupdate -expand -group data_mem -group mem256 {/riscv_tb/dut/data_mem_i/mem[0]}
add wave -noupdate -expand -group data_mem /riscv_tb/dut/data_mem_i/rvalid
add wave -noupdate -expand -group data_mem /riscv_tb/dut/data_mem_i/rdata
add wave -noupdate /riscv_tb/dut/riscv_core_i/id_stage_i/jr_stall
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {2068051 ps} 0} {{Cursor 2} {1922907 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 204
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {0 ps} {2133358 ps}
