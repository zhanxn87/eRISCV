// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Renzo Andri - andrire@student.ethz.ch                      //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Andreas Traber - atraber@student.ethz.ch                   //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                                                                            //
// Design Name:    Instruction Decode Stage                                   //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Decode stage of the core. It decodes the instructions      //
//                 and hosts the register file.                               //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module id_stage
import riscv_defines::*;
(
    input  logic        clk,
    input  logic        rst_n,

    input  logic        test_en_i,

    // controller requests
    input  logic        halt_id_i,
    input  logic        is_decoding_i,
    output logic        illegal_insn_o,
    output logic        ebrk_insn_o,
    output logic        eret_insn_o,
    output logic        ecall_insn_o,
    output logic        pipe_flush_o,

    // Interface to IF stage
    input  logic        instr_valid_i,
    input  logic [31:0] instr_rdata_i,      // comes from pipeline of IF stage

    // Jumps and branches
    output logic        branch_in_ex_o,
    output logic [31:0] branch_target_ex_o,
    output jump_t       jump_mode_id_o,     // jump mode for the instruction
    output logic [31:0] jump_target_id_o,

    // IF and ID stage signals
    output logic        clear_instr_valid_o,

    input  logic [31:0] pc_id_i,

    output logic        id_ready_o,     // ID stage is ready for the next instruction
    input  logic        ex_ready_i,     // EX stage is ready for the next instruction
    output logic        id_valid_o,     // ID stage is done

    // Pipeline ID/EX
    output logic [31:0] pc_ex_o,

    output alu_op_t     alu_operator_ex_o,
    output logic [31:0] alu_operand_a_ex_o,
    output logic [31:0] alu_operand_b_ex_o,
    output logic [31:0] alu_operand_c_ex_o,

    output logic [4:0]  regfile_lsu_waddr_ex_o,
    output logic        regfile_lsu_we_ex_o,

    output logic [4:0]  regfile_alu_waddr_ex_o,
    output logic        regfile_alu_we_ex_o,

    // MUL
    output mul_op_t     mult_operator_ex_o,
    output logic [31:0] mult_operand_a_ex_o,
    output logic [31:0] mult_operand_b_ex_o,
    output logic [31:0] mult_operand_c_ex_o,
    output logic        mult_en_ex_o,
    output logic        mult_sel_subword_ex_o,
    output logic [ 1:0] mult_signed_mode_ex_o,
    output logic [ 4:0] mult_imm_ex_o,

    // CSR ID/EX
    output logic        csr_access_ex_o,
    output csr_op_t     csr_op_ex_o,

    // Interface to load store unit
    output logic        data_req_ex_o,
    output logic        data_we_ex_o,
    output logic [31:0] data_wdata_ex_o,
    output logic [1:0]  data_type_ex_o,
    output logic        data_sign_ext_ex_o,
    output logic [1:0]  data_reg_offset_ex_o,

    input  logic        data_misaligned_i,
    output logic        data_misaligned_ex_o,

    // Forward Signals
    input  logic [4:0]  regfile_lsu_waddr_wb_i,
    input  logic        regfile_lsu_we_wb_i,
    input  logic [31:0] regfile_lsu_wdata_wb_i, // From wb_stage: selects data from data memory, ex_stage result and sp rdata

    input  logic [4:0]  regfile_alu_waddr_fw_i, // From ex_stage ALU writeback
    input  logic        regfile_alu_we_fw_i,
    input  logic [31:0] regfile_alu_wdata_fw_i,

    output logic        jr_stall_o,
    output logic        load_stall_o,

    // from ALU
    input  logic        mult_multicycle_i     // when we need multiple cycles in the multiplier and use op c as storage
);

  // Decoder/Controller ID stage internal signals

  branch_t     branch_mode_dec; //signal from decoder without deassert_we mask
  jump_t       jump_mode_dec;
  logic        illegal_insn_dec;
  branch_t     branch_mode; //signals with deassert_we mask
  jump_t       jump_mode;
  logic        illegal_insn;

  logic [31:0] imm_a;       // contains the immediate for operand a
  logic [31:0] imm_b;       // contains the immediate for operand b

  logic [31:0] imm_jump_offset; // contains the immediate for jump instructions
  logic [31:0] imm_branch_offset; // contains the immediate for branch instructions
  logic [31:0] jump_base;       // contains the base address for jump instructions
  logic [31:0] jump_target;     // calculated jump target (-> EX -> IF)
  logic [31:0] branch_target;   // calculated branch target (-> EX -> IF)

  // Register file interface
  logic        rega_used_dec;
  logic        regb_used_dec;
  logic [4:0]  regfile_addr_ra_id; // extracted source register addresses from decoder
  logic [4:0]  regfile_addr_rb_id; // extracted source register addresses from decoder

  // Register file writeback control signals
  logic [4:0]  regfile_waddr_id;   // extracted destination register address from decoder
  logic        regfile_lsu_we_id;
  logic [4:0]  regfile_lsu_waddr_id;
  logic        regfile_alu_we_id;
  logic [4:0]  regfile_alu_waddr_id;

  logic [31:0] regfile_data_ra_id; // data from register file
  logic [31:0] regfile_data_rb_id; // data from register file

  // Forwarding control signals
  logic        reg_d_lsu_ex_is_reg_a_id;
  logic        reg_d_lsu_ex_is_reg_b_id;
  logic        reg_d_lsu_wb_is_reg_a_id;
  logic        reg_d_lsu_wb_is_reg_b_id;
  logic        reg_d_alu_fw_is_reg_a_id;
  logic        reg_d_alu_fw_is_reg_b_id;

  logic [1:0]  operand_a_fw_mux_sel; // Forwarding mux select for operand A
  logic [1:0]  operand_b_fw_mux_sel; // Forwarding mux select for operand B

  // ALU Control
  alu_op_t     alu_operator;
  op_a_sel_t   alu_op_a_mux_sel;
  op_b_sel_t   alu_op_b_mux_sel;

  // Multiplier Control
  mul_op_t     mult_operator;    // multiplication operation selection
  logic        mult_en;          // multiplication is used instead of ALU
  logic        mult_int_en;      // use integer multiplier
  logic        mult_sel_subword; // Select a subword when doing multiplications
  logic [1:0]  mult_signed_mode; // Signed mode multiplication at the output of the controller, and before the pipe registers
  logic        mult_dot_en;      // use dot product
  logic [1:0]  mult_dot_signed;  // Signed mode dot products (can be mixed types)

  // Data Memory Control
  logic        data_req_id; // data read request to data memory
  logic        data_we_id;  // data write request to data memory
  logic [1:0]  data_type_id; // data type of load/store
  logic        data_sign_ext_id; // sign extend from load data
  logic [1:0]  data_reg_offset_id = 2'b00; // offset for load/store

  // CSR control
  logic        csr_access;
  csr_op_t     csr_op;

  // Forwarding
  logic [31:0] operand_a_fw_id;
  logic [31:0] operand_b_fw_id;

  logic [31:0] alu_operand_a;
  logic [31:0] alu_operand_b;
  logic [31:0] alu_operand_c;

  // Immediates for ID
  logic [0:0]  mult_imm_mux;
  logic [ 4:0] mult_imm_id;

  //
  logic        load_stall;
  logic        jr_stall;
  logic        deassert_we;

  // Second Register Write Adress Selection
  // Used for prepost load/store and multiplier
  assign regfile_alu_waddr_id = regfile_waddr_id;// : regfile_addr_ra_id;
  assign regfile_lsu_waddr_id = regfile_waddr_id;// : regfile_addr_ra_id;

  // kill instruction in the IF/ID stage by setting the instr_valid_id control
  // signal to 0 for instructions that are done
  assign clear_instr_valid_o = id_ready_o | halt_id_i;

  assign illegal_insn_o = illegal_insn & instr_valid_i;

  assign mult_en = mult_int_en | mult_dot_en;

  //////////////////////////////////////////////////////////////////
  // Jump and branch target calculation
  //////////////////////////////////////////////////////////////////
  assign jump_base     = jump_mode_dec == JT_JALR ? regfile_data_ra_id : pc_id_i;
  assign jump_target   = jump_base + imm_jump_offset;
  assign branch_target = pc_id_i + imm_branch_offset;

  assign jump_mode_id_o   = jump_mode;
  assign jump_target_id_o = jump_target;

  ///////////////////////////////////////////////
  // Decoder
  ///////////////////////////////////////////////
  decoder decoder_i
  (
    // controller related signals
    .i_deassert_we                   ( deassert_we               ),
    .i_is_compressed                 ( 1'b0                      ),

    .o_illegal_inst                  ( illegal_insn_dec          ),
    .o_ebrk_inst                     ( ebrk_insn_o               ),
    .o_eret_inst                     ( eret_insn_o               ),
    .o_ecall_inst                    ( ecall_insn_o              ),
    .o_pipe_flush                    ( pipe_flush_o              ),

    // from IF/ID pipeline
    .i_inst                          ( instr_rdata_i             ), // from IF stage
    .i_pc                            ( pc_id_i                   ), // from IF stage

    // Jump and branch decoded control signals
    .o_jump_mode                     ( jump_mode_dec             ), 
    .o_branch_mode                   ( branch_mode_dec           ),
    .o_jump_offset                   ( imm_jump_offset           ),
    .o_branch_offset                 ( imm_branch_offset         ),

    // ALU signals
    .o_alu_op                        ( alu_operator              ),
    .o_alu_op_a_sel                  ( alu_op_a_mux_sel          ),
    .o_alu_op_b_sel                  ( alu_op_b_mux_sel          ),
    .o_imma_extend                   ( imm_a                     ),
    .o_immb_extend                   ( imm_b                     ),

    // MUL signals
    .mult_operator_o                 ( mult_operator             ),
    .mult_int_en_o                   ( mult_int_en               ),
    .mult_sel_subword_o              ( mult_sel_subword          ),
    .mult_signed_mode_o              ( mult_signed_mode          ),
    .mult_imm_mux_o                  ( mult_imm_mux              ),
    .mult_dot_en_o                   ( mult_dot_en               ),
    .mult_dot_signed_o               ( mult_dot_signed           ),

    // Register file control signals
    .o_rs1_en                        ( rega_used_dec             ),
    .o_rs2_en                        ( regb_used_dec             ),
    .o_rs1_addr                      ( regfile_addr_ra_id        ),
    .o_rs2_addr                      ( regfile_addr_rb_id        ),
    .o_rd_alu_we                     ( regfile_alu_we_id         ),
    .o_rd_mem_we                     ( regfile_lsu_we_id         ),
    .o_rd_addr                       ( regfile_waddr_id          ),

    // Data bus interface
    .o_mem_req                       ( data_req_id               ),
    .o_mem_wr_en                     ( data_we_id                ),
    .o_mem_dtype                     ( data_type_id              ), // data type of load/store
    .o_load_data_sign_ext            ( data_sign_ext_id          ), // sign extend from load data

    // CSR control signals
    .o_csr_access                    ( csr_access                ),
    .o_csr_op                        ( csr_op                    )

  );

  /////////////////////////////////////////////////////////
  // Register file
  /////////////////////////////////////////////////////////
  regfile  registers_i
  (
    .clk          ( clk                ),
    .rst_n        ( rst_n              ),

    .test_en_i    ( test_en_i          ),

    // Read port a
    .raddr_a_i    ( regfile_addr_ra_id ),
    .rdata_a_o    ( regfile_data_ra_id ),

    // Read port b
    .raddr_b_i    ( regfile_addr_rb_id ),
    .rdata_b_o    ( regfile_data_rb_id ),

    // Write port a
    .waddr_a_i    ( regfile_lsu_waddr_wb_i ),
    .wdata_a_i    ( regfile_lsu_wdata_wb_i ),
    .we_a_i       ( regfile_lsu_we_wb_i    ),

    // Write port b
    .waddr_b_i    ( regfile_alu_waddr_fw_i ),
    .wdata_b_i    ( regfile_alu_wdata_fw_i ),
    .we_b_i       ( regfile_alu_we_fw_i    )
  );

  //////////////////////////////////////////////////////////////////
  // Forwarding control signals
  //////////////////////////////////////////////////////////////////
  assign reg_d_lsu_ex_is_reg_a_id = (regfile_lsu_waddr_ex_o == regfile_addr_ra_id) && (rega_used_dec == 1'b1) && (regfile_addr_ra_id != '0); //used for stall_load detection in ex_stage
  assign reg_d_lsu_ex_is_reg_b_id = (regfile_lsu_waddr_ex_o == regfile_addr_rb_id) && (regb_used_dec == 1'b1) && (regfile_addr_rb_id != '0); //used for stall_load detection in ex_stage
  assign reg_d_lsu_wb_is_reg_a_id = (regfile_lsu_waddr_wb_i == regfile_addr_ra_id) && (rega_used_dec == 1'b1) && (regfile_addr_ra_id != '0);
  assign reg_d_lsu_wb_is_reg_b_id = (regfile_lsu_waddr_wb_i == regfile_addr_rb_id) && (regb_used_dec == 1'b1) && (regfile_addr_rb_id != '0);
  assign reg_d_alu_fw_is_reg_a_id = (regfile_alu_waddr_fw_i == regfile_addr_ra_id) && (rega_used_dec == 1'b1) && (regfile_addr_ra_id != '0);
  assign reg_d_alu_fw_is_reg_b_id = (regfile_alu_waddr_fw_i == regfile_addr_rb_id) && (regb_used_dec == 1'b1) && (regfile_addr_rb_id != '0);

  always_comb
  begin
    // default assignements
    operand_a_fw_mux_sel = SEL_REGFILE;
    operand_b_fw_mux_sel = SEL_REGFILE;

    // Forwarding WB -> ID
    if (regfile_lsu_we_wb_i == 1'b1)
    begin
      if (reg_d_lsu_wb_is_reg_a_id == 1'b1)
        operand_a_fw_mux_sel = SEL_FW_WB;
      if (reg_d_lsu_wb_is_reg_b_id == 1'b1)
        operand_b_fw_mux_sel = SEL_FW_WB;
    end

    // Forwarding EX -> ID
    if (regfile_alu_we_fw_i == 1'b1)
    begin
     if (reg_d_alu_fw_is_reg_a_id == 1'b1)
       operand_a_fw_mux_sel = SEL_FW_EX;
     if (reg_d_alu_fw_is_reg_b_id == 1'b1)
       operand_b_fw_mux_sel = SEL_FW_EX;
    end
  end

  ////////////////////////////////////////////////////////
  // Operand A Mux
  ////////////////////////////////////////////////////////

  // ALU_Op_a Mux
  always_comb
  begin : alu_operand_a_mux
    case (alu_op_a_mux_sel)
      OP_A_REGA_OR_FWD:  alu_operand_a = operand_a_fw_id;
      OP_A_REGB_OR_FWD:  alu_operand_a = operand_b_fw_id;
      OP_A_CURRPC:       alu_operand_a = pc_id_i;
      OP_A_IMM:          alu_operand_a = imm_a; //from decoder
      default:           alu_operand_a = operand_a_fw_id;
    endcase; // case (alu_op_a_mux_sel)
  end

  // Operand a forwarding mux
  always_comb
  begin : operand_a_fw_mux
    case (operand_a_fw_mux_sel) // from controller
      SEL_FW_EX:    operand_a_fw_id = regfile_alu_wdata_fw_i;
      SEL_FW_WB:    operand_a_fw_id = regfile_lsu_wdata_wb_i;
      SEL_REGFILE:  operand_a_fw_id = regfile_data_ra_id;
      default:      operand_a_fw_id = regfile_data_ra_id;
    endcase; // case (operand_a_fw_mux_sel)
  end

  //////////////////////////////////////////////////////
  // Operand B Mux
  //////////////////////////////////////////////////////

  // ALU_Op_b Mux
  always_comb
  begin : alu_operand_b_mux
    case (alu_op_b_mux_sel)
      OP_B_REGB_OR_FWD:  alu_operand_b = operand_b_fw_id;
      OP_B_IMM:          alu_operand_b = imm_b; //from decoder
      default:           alu_operand_b = operand_b_fw_id;
    endcase // case (alu_op_b_mux_sel)
  end

  // Operand b forwarding mux
  always_comb
  begin : operand_b_fw_mux
    case (operand_b_fw_mux_sel) // from controller
      SEL_FW_EX:    operand_b_fw_id = regfile_alu_wdata_fw_i;
      SEL_FW_WB:    operand_b_fw_id = regfile_lsu_wdata_wb_i;
      SEL_REGFILE:  operand_b_fw_id = regfile_data_rb_id;
      default:      operand_b_fw_id = regfile_data_rb_id;
    endcase; // case (operand_b_fw_mux_sel)
  end

  assign alu_operand_c = 32'h0; // Not used in the current implementation

  /////////////////////////////////////////////////////////////////////////////////
  // ID-EX pipeline registers
  /////////////////////////////////////////////////////////////////////////////////

  always_ff @(posedge clk, negedge rst_n)
  begin : ID_EX_PIPE_REGISTERS
    if (rst_n == 1'b0)
    begin
      alu_operator_ex_o           <= ALU_NOP;
      alu_operand_a_ex_o          <= '0;
      alu_operand_b_ex_o          <= '0;
      alu_operand_c_ex_o          <= '0;

      mult_operator_ex_o          <= MUL_I;
      mult_operand_a_ex_o         <= '0;
      mult_operand_b_ex_o         <= '0;
      mult_operand_c_ex_o         <= '0;
      mult_en_ex_o                <= 1'b0;
      mult_sel_subword_ex_o       <= 1'b0;
      mult_signed_mode_ex_o       <= 2'b00;
      mult_imm_ex_o               <= '0;

      regfile_lsu_waddr_ex_o      <= 5'b0;
      regfile_lsu_we_ex_o         <= 1'b0;

      regfile_alu_waddr_ex_o      <= 5'b0;
      regfile_alu_we_ex_o         <= 1'b0;

      csr_access_ex_o             <= 1'b0;
      csr_op_ex_o                 <= CSR_OP_READ;

      data_we_ex_o                <= 1'b0;
      data_wdata_ex_o             <= '0;
      data_type_ex_o              <= 2'b0;
      data_sign_ext_ex_o          <= 1'b0;
      data_reg_offset_ex_o        <= 2'b0;
      data_req_ex_o               <= 1'b0;

      data_misaligned_ex_o        <= 1'b0;

      pc_ex_o                     <= '0;

      branch_in_ex_o              <= BRANCH_NONE;
      branch_target_ex_o          <= '0;

    end
    else if (data_misaligned_i) begin
      // misaligned data access case
      if (ex_ready_i)
      begin // misaligned access case, only unstall alu operands

        // if we are using post increments, then we have to use the
        // original value of the register for the second memory access
        // => keep it stalled

        //  alu_operand_a_ex_o        <= alu_operand_a;

        alu_operand_b_ex_o          <= alu_operand_b;
        regfile_alu_we_ex_o         <= regfile_alu_we_id;

        data_misaligned_ex_o        <= 1'b1;
      end
    end 
    else if (mult_multicycle_i) begin
      mult_operand_c_ex_o <= alu_operand_c;
    end
    else begin
      // normal pipeline unstall case

      if (id_valid_o)
      begin // unstall the whole pipeline
        //-----------------------------------------------------------
        // ALU operands
        //-----------------------------------------------------------
        if (~mult_en)
        begin // only change those registers when we actually need to
          alu_operator_ex_o         <= alu_operator;
          alu_operand_a_ex_o        <= alu_operand_a;
          alu_operand_b_ex_o        <= alu_operand_b;
          alu_operand_c_ex_o        <= alu_operand_c;
        end

        //-----------------------------------------------------------
        // MUL operands
        //-----------------------------------------------------------
        mult_en_ex_o                <= mult_en;
        if (mult_int_en) begin  // when we are multiplying we don't need the ALU
          mult_operator_ex_o        <= mult_operator;
          mult_sel_subword_ex_o     <= mult_sel_subword;
          mult_signed_mode_ex_o     <= mult_signed_mode;
          mult_operand_a_ex_o       <= alu_operand_a;
          mult_operand_b_ex_o       <= alu_operand_b;
          mult_operand_c_ex_o       <= alu_operand_c;
          mult_imm_ex_o             <= mult_imm_id;
        end

        //-----------------------------------------------------------
        // regfile writeback operands
        //-----------------------------------------------------------
        regfile_lsu_we_ex_o         <= regfile_lsu_we_id;
        if (regfile_lsu_we_id) begin
          regfile_lsu_waddr_ex_o    <= regfile_waddr_id;
        end

        regfile_alu_we_ex_o         <= regfile_alu_we_id;
        if (regfile_alu_we_id) begin
          regfile_alu_waddr_ex_o    <= regfile_alu_waddr_id;
        end

        //-----------------------------------------------------------
        // CSR operands
        //-----------------------------------------------------------
        csr_access_ex_o             <= csr_access;
        csr_op_ex_o                 <= csr_op;

        //-----------------------------------------------------------
        // load/store operands
        //-----------------------------------------------------------
        data_req_ex_o               <= data_req_id;
        data_we_ex_o                <= data_we_id;
        if (data_req_id)
        begin // only needed for LSU when there is an active request
          data_wdata_ex_o           <= operand_b_fw_id; // data to be written to memory, 2025-05-10: fixed bug: use operand_b_fw_id instead of regfile_data_rb_id
          data_type_ex_o            <= data_type_id;
          data_sign_ext_ex_o        <= data_sign_ext_id;
          data_reg_offset_ex_o      <= data_reg_offset_id;
        end

        data_misaligned_ex_o        <= 1'b0;

        if ((branch_mode == BRANCH_COND)) begin
          pc_ex_o                   <= pc_id_i;
        end

        branch_in_ex_o              <= branch_mode == BRANCH_COND;
        branch_target_ex_o          <= branch_target;
      end else if(ex_ready_i) begin
        // EX stage is ready but we don't have a new instruction for it,
        // so we set all write enables to 0, but unstall the pipe

        regfile_lsu_we_ex_o         <= 1'b0;

        regfile_alu_we_ex_o         <= 1'b0;

        csr_op_ex_o                 <= CSR_OP_READ;

        data_req_ex_o               <= 1'b0;

        data_misaligned_ex_o        <= 1'b0;

        branch_in_ex_o              <= 1'b0;
      end
    end
  end

  // Stall because of load operation
  assign load_stall = (data_req_ex_o && regfile_lsu_we_ex_o && (reg_d_lsu_ex_is_reg_a_id || reg_d_lsu_ex_is_reg_b_id));

  // Stall because of jr path
  // - always stall if a result is to be forwarded to the PC
  // we don't care about in which state the ctrl_fsm is as we deassert_we
  // anyway when we are not in DECODE
  assign jr_stall   = (jump_mode_dec == JT_JALR) &&
                      ((regfile_lsu_we_wb_i && reg_d_lsu_wb_is_reg_a_id) ||
                       (regfile_lsu_we_ex_o && reg_d_lsu_ex_is_reg_a_id) ||
                       (regfile_alu_we_fw_i && reg_d_alu_fw_is_reg_a_id));

  assign deassert_we = (~is_decoding_i || illegal_insn_dec || load_stall || jr_stall);

  assign branch_mode  = deassert_we ? BRANCH_NONE : branch_mode_dec;
  assign jump_mode    = jump_mode_dec;
  assign illegal_insn = illegal_insn_dec;

  assign id_ready_o = ((~jr_stall) & (~load_stall) & ex_ready_i);
  assign id_valid_o = (~halt_id_i) & id_ready_o;

  assign jr_stall_o = jr_stall;
  assign load_stall_o = load_stall;

  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------

  // the instruction delivered to the ID stage should always be valid
  assert property (
    @(posedge clk) (instr_valid_i) |-> (!$isunknown(instr_rdata_i)) ) else $display("Instruction is valid, but has at least one X");

endmodule