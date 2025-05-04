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
add wave -noupdate -color Magenta -itemcolor Magenta /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/opcode
add wave -noupdate -expand -group alu -color Magenta -itemcolor Magenta /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_i/i_alu_op
add wave -noupdate -expand -group alu /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_i/i_operand_a
add wave -noupdate -expand -group alu /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_i/i_operand_b
add wave -noupdate -expand -group alu /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_i/o_result
add wave -noupdate -expand -group alu /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_i/o_compare_result
add wave -noupdate -expand -group alu /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_i/o_zero_flag
add wave -noupdate -expand -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/clk
add wave -noupdate -expand -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/rst_n
add wave -noupdate -expand -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/test_en_i
add wave -noupdate -expand -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/raddr_a_i
add wave -noupdate -expand -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/rdata_a_o
add wave -noupdate -expand -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/raddr_b_i
add wave -noupdate -expand -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/rdata_b_o
add wave -noupdate -expand -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/we_a_i
add wave -noupdate -expand -group regfile -radix unsigned /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/waddr_a_i
add wave -noupdate -expand -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/wdata_a_i
add wave -noupdate -expand -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/we_b_i
add wave -noupdate -expand -group regfile -radix unsigned /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/waddr_b_i
add wave -noupdate -expand -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/wdata_b_i
add wave -noupdate -expand -group regfile /riscv_tb/dut/riscv_core_i/id_stage_i/registers_i/rf_reg
add wave -noupdate /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_alu_waddr_ex_o
add wave -noupdate /riscv_tb/dut/riscv_core_i/id_stage_i/regfile_alu_we_ex_o
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_alu_we_fw_o
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_alu_waddr_fw_o
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_alu_wdata_fw_o
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_req_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_we_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_type_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_wdata_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_reg_offset_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_sign_ext_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_we_i
add wave -noupdate -radix unsigned /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_waddr_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_we_wb_o
add wave -noupdate -radix unsigned /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_waddr_wb_o
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_wdata_wb_o
add wave -noupdate -group data_mem_if /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_req_o
add wave -noupdate -group data_mem_if /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_addr_o
add wave -noupdate -group data_mem_if /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_we_o
add wave -noupdate -group data_mem_if /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_be_o
add wave -noupdate -group data_mem_if /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_wdata_o
add wave -noupdate -group data_mem_if /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_rvalid_i
add wave -noupdate -group data_mem_if /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_rdata_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/branch_in_ex_i
add wave -noupdate /riscv_tb/dut/riscv_core_i/ex_stage_i/branch_decision_o
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/clk
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/rst_n
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/boot_addr_i
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/req_i
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/pc_set_i
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/pc_mux_i
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/exc_pc_mux_i
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/exc_vec_pc_mux_i
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/exception_pc_reg_i
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/jump_target_id_i
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/halt_if_i
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/id_ready_i
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/clear_instr_valid_i
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/instr_req_o
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/instr_addr_o
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/instr_rvalid_i
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/instr_rdata_i
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/pc_if_o
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/pc_id_o
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/instr_valid_id_o
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/instr_rdata_id_o
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/if_ready_o
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/if_valid_o
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/if_busy_o
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/perf_imiss_o
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/illegal_c_insn_id_o
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/offset_fsm_ns
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/offset_fsm_cs
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/illegal_c_insn
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/branch_req
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/fetch_addr_n
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/fetch_valid
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/fetch_addr
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/exc_pc
add wave -noupdate -expand -group if_stage /riscv_tb/dut/riscv_core_i/if_stage_i/pc_reg
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
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/i_deassert_we
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/i_is_compressed
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_illegal_inst
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_ebrk_inst
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_ecall_inst
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_eret_inst
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_pipe_flush
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/i_inst
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/i_pc
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_alu_op
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_alu_op_a_sel
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_alu_op_b_sel
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_imma_extend
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_immb_extend
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/mult_operator_o
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/mult_int_en_o
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/mult_signed_mode_o
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/mult_sel_subword_o
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/mult_imm_mux_o
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/mult_dot_en_o
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/mult_dot_signed_o
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_rs1_en
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_rs2_en
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_rd_alu_we
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_rd_mem_we
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_rs1_addr
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_rs2_addr
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_rd_addr
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_mem_wr_en
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_mem_dtype
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_load_data_sign_ext
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_csr_access
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/o_csr_op
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/instr
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/imm_i_type
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/imm_iz_type
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/imm_s_type
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/imm_b_type
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/imm_u_type
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/imm_j_type
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/imm_z_type
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/imm_pc_incr
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/rd_mem_we
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/rd_alu_we
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/mem_wr_en
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/csr_op
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/ebrk_inst
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/eret_inst
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/pipe_flush
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/opcode
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/funct_7
add wave -noupdate -group decoder /riscv_tb/dut/riscv_core_i/id_stage_i/decoder_i/funct_3
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/clk
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/rst_n
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_operator_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_operand_a_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_operand_b_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_operator_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_operand_a_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_operand_b_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_en_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_sel_subword_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_signed_mode_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_imm_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_multicycle_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/branch_in_ex_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/branch_decision_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_alu_we_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_alu_waddr_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_alu_we_fw_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_alu_waddr_fw_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_alu_wdata_fw_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_we_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_waddr_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_we_wb_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_waddr_wb_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/regfile_lsu_wdata_wb_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/csr_access_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/csr_rdata_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_req_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_we_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_type_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_wdata_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_reg_offset_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_data_sign_ext_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_misaligned_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_misaligned_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_load_err_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_store_err_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/lsu_busy_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_req_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_addr_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_we_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_be_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_wdata_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_rvalid_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/data_mem_rdata_i
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/ex_ready_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/ex_valid_o
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_result
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_csr_result
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_result
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_cmp_result
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/alu_ready
add wave -noupdate -expand -group ex_stage /riscv_tb/dut/riscv_core_i/ex_stage_i/mult_ready
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {115000 ps} 0} {{Cursor 2} {134877 ps} 0}
quietly wave cursor active 2
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
WaveRestoreZoom {15574 ps} {174127 ps}
