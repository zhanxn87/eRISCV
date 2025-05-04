////////////////////////////////////////////////////////////////////////////////
// Copyright 2025 @zhanxn87
// Author:         Xianning Zhan - zhanxn@gmail.com                           //
//                                                                            //
// Design Name:    Excecute stage                                             //
// Project Name:   eRISCV                                                     //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Execution stage: Hosts ALU, MulDiv unit and LSU            //
//                 ALU   : computes additions/subtractions/comparisons        //
//                 MULDIV: Multiplier and Divider                             //
//                 LSU   : Load/Store Unit, handles memory access             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module ex_stage
import riscv_defines::*;
(
  input  logic        clk,
  input  logic        rst_n,

  // branch decoded from ID stage
  input  logic        branch_in_ex_i, // is_branch instruction
  // To IF: Jump and branch target and decision
  output logic        branch_decision_o,

  /////////////////////////////////////////////////
  // ALU
  /////////////////////////////////////////////////
  // ALU signals from ID stage
  input  alu_op_t     alu_operator_i,
  input  logic [31:0] alu_operand_a_i,
  input  logic [31:0] alu_operand_b_i,

  // Multiplier signals
  input  mul_op_t     mult_operator_i,
  input  logic [31:0] mult_operand_a_i,
  input  logic [31:0] mult_operand_b_i,
  input  logic [31:0] mult_operand_c_i,
  input  logic        mult_en_i,
  input  logic        mult_sel_subword_i,
  input  logic [ 1:0] mult_signed_mode_i,
  input  logic [ 4:0] mult_imm_i,

  output logic        mult_multicycle_o,

  // CSR access results to be forwarded to ID stage
  input  logic        csr_access_i,
  input  logic [31:0] csr_rdata_i,

  // input from ID stage
  input  logic [4:0]  regfile_alu_waddr_i,
  input  logic        regfile_alu_we_i,

  // Forwarding ports :  forward ALU/MULT/CSR results to ID stage
  output logic  [4:0] regfile_alu_waddr_fw_o, //=regfile_alu_waddr_i
  output logic        regfile_alu_we_fw_o,    //=regfile_alu_we_i
  output logic [31:0] regfile_alu_wdata_fw_o, // forward to RegFile and ID/EX pipe, ALU & MUL

  /////////////////////////////////////////////////
  // Load request and writeback signals
  /////////////////////////////////////////////////
  // directly passed through to WB stage, not used in EX
  input  logic        regfile_lsu_we_i, // for load data writeback from data memory
  input  logic [4:0]  regfile_lsu_waddr_i,

  // Output of EX stage pipeline
  output logic        regfile_lsu_we_wb_o,
  output logic [4:0]  regfile_lsu_waddr_wb_o,
  output logic [31:0] regfile_lsu_wdata_wb_o,

  ///////////////////////////////////////////////
  // LSU signals from ID/EX stage
  ///////////////////////////////////////////////
  // LSU signals from id/ex pipeline 
  input  logic        lsu_data_req_i,        // data request                      -> from id/ex stage
  input  logic        lsu_data_we_i,         // write enable                      -> from id/ex stage
  input  logic [1:0]  lsu_data_type_i,       // Data type word, halfword, byte    -> from id/ex stage
  input  logic [31:0] lsu_data_wdata_i,      // data to write to memory           -> from id/ex stage
  input  logic [1:0]  lsu_reg_offset_i,      // offset inside register for stores -> from id/ex stage
  input  logic        lsu_data_sign_ext_i,   // sign extension                    -> from id/ex stage

  input  logic        data_misaligned_i,    // misaligned access in last ld/st    -> from ID/EX pipeline
  output logic        data_misaligned_o,    // misaligned access was detected     -> to controller
  output logic        lsu_load_err_o,       // load error                         -> to exc_controller
  output logic        lsu_store_err_o,      // store error                        -> to exc_controller  
  output logic        lsu_busy_o,           // LSU is busy

  // interface to data memory
  output logic        data_mem_req_o,
  output logic [31:0] data_mem_addr_o,
  output logic        data_mem_we_o,
  output logic [3:0]  data_mem_be_o,
  output logic [31:0] data_mem_wdata_o,
  input  logic        data_mem_rvalid_i,
  input  logic [31:0] data_mem_rdata_i,

  output logic        ex_ready_o, // EX stage ready for new data
  output logic        ex_valid_o, // EX stage gets new data
  output logic        wb_ready_o  // WB stage ready for new data
);

  logic [31:0] alu_result;
  logic [31:0] alu_csr_result;
  logic [31:0] mult_result;
  logic        alu_cmp_result;

  logic        alu_ready;
  logic        mult_ready;

  logic [31:0] lsu_data_addr; // address to memory
  logic [31:0] lsu_data_rdata; // data read from memory,

  logic        lsu_ready_ex; // = 1'b1
  logic        lsu_ready_wb; // = 1'b1
  logic        wb_ready; // = 1'b1

  assign lsu_data_addr = alu_result; // address to memory is the result of the ALU operation

  // EX stage result mux (ALU, MAC unit, CSR)
  assign alu_csr_result         = csr_access_i ? csr_rdata_i : alu_result;

  assign regfile_alu_wdata_fw_o = mult_en_i ? mult_result : alu_csr_result;
  assign regfile_alu_we_fw_o    = regfile_alu_we_i;
  assign regfile_alu_waddr_fw_o = regfile_alu_waddr_i;

  // branch handling
  assign branch_decision_o = branch_in_ex_i ? alu_cmp_result : 1'b0;

  ////////////////////////////////////////////////////////////////
  // ALU
  ////////////////////////////////////////////////////////////////

  alu alu_i
  (
    .i_alu_op           ( alu_operator_i   ),
    .i_operand_a        ( alu_operand_a_i  ),
    .i_operand_b        ( alu_operand_b_i  ),

    .o_result           ( alu_result       ),
    .o_compare_result   ( alu_cmp_result   ),
    .o_zero_flag        (                  )
  );

  assign alu_ready = 1'b1;

  ////////////////////////////////////////////////////////////////
  // Multiplier Unit : from Pulpino project                                         
  ////////////////////////////////////////////////////////////////
  riscv_mult mult_i
  (
    .clk             ( clk                  ),
    .rst_n           ( rst_n                ),

    .enable_i        ( mult_en_i            ),
    .operator_i      ( mult_operator_i      ),

    .short_subword_i ( mult_sel_subword_i   ),
    .short_signed_i  ( mult_signed_mode_i   ),

    .op_a_i          ( mult_operand_a_i     ),
    .op_b_i          ( mult_operand_b_i     ),
    .op_c_i          ( mult_operand_c_i     ),
    .imm_i           ( mult_imm_i           ),

    // dot multiplier
    .dot_signed_i    (2'b0),
    .dot_op_a_i      (32'b0),
    .dot_op_b_i      (32'b0),
    .dot_op_c_i      (32'b0),

    .result_o        ( mult_result          ),

    .multicycle_o    ( mult_multicycle_o    ),
    .ready_o         ( mult_ready           ),
    .ex_ready_i      ( ex_ready_o           )
  );

  /////////////////////////////////////////////////////////////////////
  // Load/Store Unit (LSU) : from Pulpino project
  /////////////////////////////////////////////////////////////////////
  load_store_unit  load_store_unit_i
  (
    .clk                   ( clk                  ),
    .rst_n                 ( rst_n                ),

    //output to data memory
    .data_req_o            ( data_mem_req_o       ),
    .data_addr_o           ( data_mem_addr_o      ),
    .data_we_o             ( data_mem_we_o        ),
    .data_be_o             ( data_mem_be_o        ),
    .data_wdata_o          ( data_mem_wdata_o     ),
    .data_rvalid_i         ( data_mem_rvalid_i    ), // from data memory
    .data_rdata_i          ( data_mem_rdata_i     ),

    // signal from ID/ex pipeline
    .data_we_ex_i          ( lsu_data_we_i        ), // from ID/EX pipeline
    .data_type_ex_i        ( lsu_data_type_i      ), // from ID/EX pipeline
    .data_wdata_ex_i       ( lsu_data_wdata_i     ), // from ID/EX pipeline
    .data_reg_offset_ex_i  ( lsu_reg_offset_i     ), // from ID/EX pipeline
    .data_sign_ext_ex_i    ( lsu_data_sign_ext_i  ), // from ID/EX pipeline

    .data_req_ex_i         ( lsu_data_req_i       ),
    .data_addr_i           ( lsu_data_addr        ), // operand a from RF for address -> from ex stage
    .data_rdata_ex_o       ( lsu_data_rdata       ),

    .data_misaligned_ex_i  ( data_misaligned_i    ), // from ID/EX pipeline
    .data_misaligned_o     ( data_misaligned_o    ),

    // exception signals
    .load_err_o            ( lsu_load_err_o       ),
    .store_err_o           ( lsu_store_err_o      ),

    // control signals
    .lsu_ready_ex_o        ( lsu_ready_ex         ), // EX part of LSU is done
    .lsu_ready_wb_o        ( lsu_ready_wb         ),
    .busy_o                ( lsu_busy_o           )
  );

  assign wb_ready = lsu_ready_wb;
  assign wb_ready_o = wb_ready;

  ///////////////////////////////////////
  // EX/WB Pipeline Register           //
  ///////////////////////////////////////
  // One cycle delay from data memory
  always_ff @(posedge clk, negedge rst_n)
  begin : EX_WB_Pipeline_Register
    if (~rst_n)
    begin
      regfile_lsu_waddr_wb_o   <= '0;
      regfile_lsu_we_wb_o      <= 1'b0;
    end
    else
    begin
      if (ex_valid_o) // wb_ready is implied
      begin
        regfile_lsu_we_wb_o    <= regfile_lsu_we_i;
        if (regfile_lsu_we_i) begin
          regfile_lsu_waddr_wb_o <= regfile_lsu_waddr_i;
        end
      end else if (wb_ready) begin
        // we are ready for a new instruction, but there is none available,
        // so we just flush the current one out of the pipe
        regfile_lsu_we_wb_o    <= 1'b0;
      end
    end
  end

  assign regfile_lsu_wdata_wb_o = lsu_data_rdata;

  // As valid always goes to the right and ready to the left, and we are able
  // to finish branches without going to the WB stage, ex_valid does not
  // depend on ex_ready.
  assign ex_ready_o = (alu_ready & mult_ready & lsu_ready_ex & wb_ready) | branch_in_ex_i;
  assign ex_valid_o = (alu_ready & mult_ready & lsu_ready_ex & wb_ready);

endmodule
