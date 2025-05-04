// Description: RISC-V Core Top Level Module

////////////////////////////////////////////////////////////////////////////////
// Copyright 2025 @zhanxn87
// Author:         Xianning Zhan - zhanxn@gmail.com                           //
//                                                                            //
// Design Name:    eriscv_core                                                //
// Project Name:   eRISCV                                                     //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    eRISCV core containing IF_stage, ID_stage and EX_stage     //
//                 Exception controller                                       //
//                 4-stage pipeline architecture.                             //
//                 RV32IM ISA compliant.                                      //
//                 Modified from PULPIno RISC-V core.                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module riscv_core 
import riscv_defines::*;
(
  input  logic            clk,
  input  logic            rst_n,

  // CPU control signals
  input  logic            fetch_enable_i,
  output logic            ctrl_busy_o,

  // Instruction memory interface
  output logic            instr_req_o, // instruction request
  output logic [31:0]     instr_addr_o,
  input  logic            instr_rvalid_i, // instruction read valid
  input  logic [31:0]     instr_rdata_i,

  // Data memory interface
  output logic            data_req_o,
  output logic [31:0]     data_addr_o,
  output logic [31:0]     data_wdata_o,
  output logic            data_we_o,
  output logic [3:0]      data_be_o,
  input  logic            data_rvalid_i,
  input  logic [31:0]     data_rdata_i,

  // Interrupt interface
  input logic [31:0]      irq_i
);

localparam boot_addr = 32'h0000_0000; // Boot address for the processor

//from controller
logic        is_decoding; //from controller
logic        misaligned_stall;
logic        jr_stall;
logic        load_stall;

//from decoder
logic        illegal_insn_id;
logic        ebrk_insn_id ;
logic        eret_insn_id ;
logic        ecall_insn_id ;
logic        pipe_flush_id ;

logic        instr_req_int; // Instruction request signal
logic        pc_set;
pc_mux_t     pc_mux;
exc_pc_mux_t exc_pc_mux;
logic [4:0]  exc_vec_pc_mux;

logic        if_ready;     // IF stage is done
logic        if_valid;     // IF stage is done
logic        ex_valid;     // EX stage is done
logic        wb_valid;     // WB stage is done

// Interrupt signals
logic        irq_enable;

// Signals running between controller and exception controller
logic        exc_req, exc_ack;  // handshake

logic [5:0]  exc_cause;
logic        save_exc_cause;

logic        exc_save_if;
logic        exc_save_id;
logic        exc_restore_id;

logic        lsu_load_err;
logic        lsu_store_err;

// Missing signal declarations
logic        instr_valid_id;       // 
logic [31:0] instr_rdata_id;       // Instruction data for ID stage
logic [31:0] pc_if;                // Program counter for IF stage
logic [31:0] pc_id;                // Program counter for ID stage
logic [31:0] pc_ex;                // Program counter for EX stage

alu_op_t     alu_operator_ex;
logic [31:0] alu_operand_a_ex;     // ALU operand A from EX stage
logic [31:0] alu_operand_b_ex;     // ALU operand B from EX stage
logic [31:0] alu_operand_c_ex;     // ALU operand C from EX stage

mul_op_t     mult_operator_ex;    // Multiplier operator from EX stage
logic [31:0] mult_operand_a_ex;    // Multiplier operand A from EX stage
logic [31:0] mult_operand_b_ex;    // Multiplier operand B from EX stage
logic [31:0] mult_operand_c_ex;    // Multiplier operand C from EX stage
logic        mult_en_ex;          // Multiplier enable signal from EX stage
logic        mult_sel_subword_ex; // Multiplier subword selection from EX stage
logic [1:0]  mult_signed_mode_ex; // Multiplier signed mode from EX stage
logic [4:0]  mult_imm_ex;          // Multiplier immediate value from EX stage

logic [31:0] mepc;                 // Machine exception program counter

logic        data_req_ex;          // Data request signal from EX stage
logic        data_we_ex;           // Data write enable signal from EX stage  
logic [31:0] data_wdata_ex;        // Data type from EX stage
logic [1:0]  data_type_ex;         // Data type from EX stage
logic [1:0]  data_reg_offset_ex;   // Data register offset from EX stage
logic        data_sign_ext_ex;     // Data sign extension from EX stage

logic [1:0]  operand_a_fw_mux_sel_id; // Forwarding mux select for operand A
logic [1:0]  operand_b_fw_mux_sel_id; // Forwarding mux select for operand B

jump_t       jump_mode_id;         // Jump signal from ID stage
logic        branch_in_ex;         // Branch signal from ID to EX stage
logic        branch_decision;      // Branch decision signal
logic [31:0] jump_target_id;       // Jump target from ID stage
logic [31:0] branch_target_ex;     // branch target from EX stage

logic [31:0] ext_perf_counters_i;  // External performance counters
logic        perf_imiss;           // Performance counter for instruction misses
logic        perf_jump;            // Performance counter for jumps
logic        perf_jr_stall;        // Performance counter for JR stalls
logic        perf_ld_stall;        // Performance counter for load stalls

logic        clear_instr_valid_id;    // Clear instruction valid signal

logic        lsu_busy;             // Load/store unit busy signal

// Signal declarations for CSR interface
logic [11:0] csr_addr_int;
logic        csr_access;
logic [11:0] csr_addr;
logic [31:0] csr_wdata;
csr_op_t     csr_op;
logic [31:0] csr_rdata;            // CSR read data

csr_op_t     csr_op_ex;
logic        csr_access_ex;

//ID/EX pipeline registers
logic        regfile_lsu_we_ex;
logic [4:0]  regfile_lsu_waddr_ex; //write address for register file from id/ex pipeline registers
logic        regfile_alu_we_ex;
logic [4:0]  regfile_alu_waddr_ex; //write address for register file from id/ex pipeline registers

// writeback signals from EX stage to EX stage
logic        regfile_lsu_we_wb;
logic [4:0]  regfile_lsu_waddr_wb; //write address for register file from memory load
logic [31:0] regfile_lsu_wdata_wb; // Write data for register file from memory load
logic [4:0]  regfile_alu_waddr_fw; //write address for register file from id/ex pipeline registers
logic        regfile_alu_we_fw;
logic [31:0] regfile_alu_wdata_fw;

// Forwarding detection signals
logic        reg_d_ex_is_reg_a_id;
logic        reg_d_ex_is_reg_b_id;
logic        reg_d_wb_is_reg_a_id;
logic        reg_d_wb_is_reg_b_id;
logic        reg_d_alu_is_reg_a_id;
logic        reg_d_alu_is_reg_b_id;

// Stall
logic        halt_if;      // controller requests a halt of the IF stage
logic        halt_id;      // controller requests a halt of the ID stage

//////////////////////////////////////////////////
//   ___ _____   ____ _____  _    ____ _____    //
//  |_ _|  ___| / ___|_   _|/ \  / ___| ____|   //
//   | || |_    \___ \ | | / _ \| |  _|  _|     //
//   | ||  _|    ___) || |/ ___ \ |_| | |___    //
//  |___|_|     |____/ |_/_/   \_\____|_____|   //
//                                              //
//////////////////////////////////////////////////
if_stage if_stage_i
(
  .clk                 ( clk               ),
  .rst_n               ( rst_n             ),

  // boot address (trap vector location)
  .boot_addr_i         ( boot_addr         ),

  // instruction request control
  .req_i               ( instr_req_int     ),

  // instruction cache interface
  .instr_req_o         ( instr_req_o       ),
  .instr_addr_o        ( instr_addr_o      ),
  .instr_rvalid_i      ( instr_rvalid_i    ),
  .instr_rdata_i       ( instr_rdata_i     ),

  // outputs to ID stage
  .instr_valid_id_o    ( instr_valid_id    ),
  .instr_rdata_id_o    ( instr_rdata_id    ),
  .pc_if_o             ( pc_if             ), // to controller
  .pc_id_o             ( pc_id             ), // to ID stage

  // control signals
  .clear_instr_valid_i ( clear_instr_valid_id ), // from id stage
  .pc_set_i            ( pc_set            ),
  .pc_mux_i            ( pc_mux            ), // sel for pc multiplexer
  .exception_pc_reg_i  ( mepc              ), // exception return address
  .exc_pc_mux_i        ( exc_pc_mux        ),
  .exc_vec_pc_mux_i    ( exc_vec_pc_mux    ),

  // Jump targets
  .jump_target_id_i    ( jump_target_id    ),
  .branch_target_ex_i  ( branch_target_ex  ),

  // pipeline stalls
  .halt_if_i           ( halt_if           ),
  .if_ready_o          ( if_ready          ),
  .id_ready_i          ( id_ready          ), //from id_stage
  .if_valid_o          ( if_valid          ),

  .if_busy_o           ( if_busy           ),
  .perf_imiss_o        ( perf_imiss        )
);

/////////////////////////////////////////////////
//   ___ ____    ____ _____  _    ____ _____   //
//  |_ _|  _ \  / ___|_   _|/ \  / ___| ____|  //
//   | || | | | \___ \ | | / _ \| |  _|  _|    //
//   | || |_| |  ___) || |/ ___ \ |_| | |___   //
//  |___|____/  |____/ |_/_/   \_\____|_____|  //
//                                             //
/////////////////////////////////////////////////
id_stage id_stage_i
(
  .clk                         ( clk                  ),
  .rst_n                       ( rst_n                ),

  .test_en_i                   ( 1'b0   ), // Enable all clock for test mode

  // controller requests
  .halt_id_i                   ( halt_id              ), // controller requests a halt of the ID stage 
  .is_decoding_i               ( is_decoding          ), // controller requests a decode of the instruction
  .illegal_insn_o              ( illegal_insn_id      ), // illegal instruction detected
  .ebrk_insn_o                 ( ebrk_insn_id         ), // 
  .eret_insn_o                 ( eret_insn_id         ), // 
  .ecall_insn_o                ( ecall_insn_id        ), //
  .pipe_flush_o                ( pipe_flush_id        ), // WFI instruction detected

  // Interface with IF stage
  .instr_valid_i               ( instr_valid_id       ),
  .instr_rdata_i               ( instr_rdata_id       ),
  .pc_id_i                     ( pc_id                ),
  .clear_instr_valid_o         ( clear_instr_valid_id ), // to IF stage

  // Jumps and branches
  .branch_in_ex_o              ( branch_in_ex         ),
  .branch_target_ex_o          ( branch_target_ex     ), //jump_target from id/ex pipeline registers
  .jump_mode_id_o              ( jump_mode_id         ), //jump_mode from decoder
  .jump_target_id_o            ( jump_target_id       ), //jump_target from decoder

  .ex_ready_i                  ( ex_ready             ),
  .id_ready_o                  ( id_ready             ), // to IF
  .id_valid_o                  ( id_valid             ),

  // From the Pipeline ID/EX
  .pc_ex_o                     ( pc_ex                ),

  .alu_operator_ex_o           ( alu_operator_ex      ),
  .alu_operand_a_ex_o          ( alu_operand_a_ex     ),
  .alu_operand_b_ex_o          ( alu_operand_b_ex     ),
  .alu_operand_c_ex_o          ( alu_operand_c_ex     ),

  .regfile_lsu_we_ex_o         ( regfile_lsu_we_ex    ),
  .regfile_lsu_waddr_ex_o      ( regfile_lsu_waddr_ex ),

  .regfile_alu_we_ex_o         ( regfile_alu_we_ex    ),
  .regfile_alu_waddr_ex_o      ( regfile_alu_waddr_ex ),

  // MUL
  .mult_operator_ex_o          ( mult_operator_ex     ), // from ID to EX stage
  .mult_operand_a_ex_o         ( mult_operand_a_ex    ), // from ID to EX stage
  .mult_operand_b_ex_o         ( mult_operand_b_ex    ), // from ID to EX stage
  .mult_operand_c_ex_o         ( mult_operand_c_ex    ), // from ID to EX stage
  .mult_en_ex_o                ( mult_en_ex           ), // from ID to EX stage
  .mult_sel_subword_ex_o       ( mult_sel_subword_ex  ), // from ID to EX stage
  .mult_signed_mode_ex_o       ( mult_signed_mode_ex  ), // from ID to EX stage
  .mult_imm_ex_o               ( mult_imm_ex          ), // from ID to EX stage

  // CSR ID/EX
  .csr_access_ex_o             ( csr_access_ex        ),
  .csr_op_ex_o                 ( csr_op_ex            ),

  // LSU
  .data_req_ex_o               ( data_req_ex          ), // to load store unit
  .data_we_ex_o                ( data_we_ex           ), // to load store unit
  .data_wdata_ex_o             ( data_wdata_ex        ), // to load store unit
  .data_type_ex_o              ( data_type_ex         ), // to load store unit
  .data_sign_ext_ex_o          ( data_sign_ext_ex     ), // to load store unit
  .data_reg_offset_ex_o        ( data_reg_offset_ex   ), // to load store unit

  .data_misaligned_ex_o        ( data_misaligned_ex   ), // to load store unit
  .data_misaligned_i           ( data_misaligned      ), // from LSU

  // Forward/writeback Signals
  .regfile_lsu_waddr_wb_i      ( regfile_lsu_waddr_wb ),  // Write address ex-wb pipeline
  .regfile_lsu_we_wb_i         ( regfile_lsu_we_wb    ),  // write enable for the register file
  .regfile_lsu_wdata_wb_i      ( regfile_lsu_wdata_wb ),  // write data to commit in the register file

  .regfile_alu_waddr_fw_i      ( regfile_alu_waddr_fw ),
  .regfile_alu_we_fw_i         ( regfile_alu_we_fw    ),
  .regfile_alu_wdata_fw_i      ( regfile_alu_wdata_fw ),

  .jr_stall_o                  ( jr_stall             ), // controller requests a stall for jump register instruction
  .load_stall_o                ( load_stall           ), // controller requests a stall for load instruction

  // from ALU
  .mult_multicycle_i           ( mult_multicycle      )
);


/////////////////////////////////////////////////////
//   _______  __  ____ _____  _    ____ _____      //
//  | ____\ \/ / / ___|_   _|/ \  / ___| ____|     //
//  |  _|  \  /  \___ \ | | / _ \| |  _|  _|       //
//  | |___ /  \   ___) || |/ ___ \ |_| | |___      //
//  |_____/_/\_\ |____/ |_/_/   \_\____|_____|     //
//                                                 //
/////////////////////////////////////////////////////
ex_stage  ex_stage_i
(
  // Global signals: Clock and active low asynchronous reset
  .clk                        ( clk                          ),
  .rst_n                      ( rst_n                        ),

  // From ID Stage: Regfile control signals
  .branch_in_ex_i             ( branch_in_ex                 ),
  .branch_decision_o          ( branch_decision              ),

  /////////////////////////////////////////////////
  // ALU Operation and writeback signals
  /////////////////////////////////////////////////
  // Alu signals from ID stage
  .alu_operator_i             ( alu_operator_ex              ), // from ID/EX pipe registers
  .alu_operand_a_i            ( alu_operand_a_ex             ), // from ID/EX pipe registers
  .alu_operand_b_i            ( alu_operand_b_ex             ), // from ID/EX pipe registers

  // Multipler
  .mult_operator_i            ( mult_operator_ex             ), // from ID/EX pipe registers
  .mult_operand_a_i           ( mult_operand_a_ex            ), // from ID/EX pipe registers
  .mult_operand_b_i           ( mult_operand_b_ex            ), // from ID/EX pipe registers
  .mult_operand_c_i           ( mult_operand_c_ex            ), // from ID/EX pipe registers
  .mult_en_i                  ( mult_en_ex                   ), // from ID/EX pipe registers
  .mult_sel_subword_i         ( mult_sel_subword_ex          ), // from ID/EX pipe registers
  .mult_signed_mode_i         ( mult_signed_mode_ex          ), // from ID/EX pipe registers
  .mult_imm_i                 ( mult_imm_ex                  ), // from ID/EX pipe registers

  .mult_multicycle_o          ( mult_multicycle              ), // to ID/EX pipe registers

  // interface with CSRs
  .csr_access_i               ( csr_access_ex                ),
  .csr_rdata_i                ( csr_rdata                    ),

  .regfile_alu_waddr_i        ( regfile_alu_waddr_ex         ), //from ID/EX pipeline registers
  .regfile_alu_we_i           ( regfile_alu_we_ex            ),

  // Back To ID stage: Forwarding signals
  .regfile_alu_waddr_fw_o     ( regfile_alu_waddr_fw         ), //from ID/EX pipeline registers
  .regfile_alu_we_fw_o        ( regfile_alu_we_fw            ),
  .regfile_alu_wdata_fw_o     ( regfile_alu_wdata_fw         ),

  /////////////////////////////////////////////////
  // Load request and writeback signals
  /////////////////////////////////////////////////
  .regfile_lsu_we_i           ( regfile_lsu_we_ex            ),
  .regfile_lsu_waddr_i        ( regfile_lsu_waddr_ex         ), // from ID/EX pipeline registers

  // Output of ex stage pipeline
  .regfile_lsu_we_wb_o        ( regfile_lsu_we_wb            ),
  .regfile_lsu_waddr_wb_o     ( regfile_lsu_waddr_wb         ),
  .regfile_lsu_wdata_wb_o     ( regfile_lsu_wdata_wb         ), // from ID/EX pipeline registers

  // signal from ID/ex pipeline for memory store
  // lsu_addr is calculated in EX stage
  .lsu_data_req_i             ( data_req_ex                 ), // from ID/EX pipeline
  .lsu_data_we_i              ( data_we_ex                  ), // from ID/EX pipeline
  .lsu_data_type_i            ( data_type_ex                ), // from ID/EX pipeline
  .lsu_data_wdata_i           ( data_wdata_ex               ), // rs2 from ID/EX pipeline
  .lsu_reg_offset_i           ( data_reg_offset_ex          ), // from ID/EX pipeline
  .lsu_data_sign_ext_i        ( data_sign_ext_ex            ), // from ID/EX pipeline

  //lsu misaligned signal
  .data_misaligned_i          ( data_misaligned_ex          ), // from ID/EX pipeline
  .data_misaligned_o          ( data_misaligned             ),
  .lsu_load_err_o             ( lsu_load_err                ),
  .lsu_store_err_o            ( lsu_store_err               ),
  .lsu_busy_o                 ( lsu_busy                    ),

  //interface to data memory
  .data_mem_req_o             ( data_req_o                  ),
  .data_mem_addr_o            ( data_addr_o                 ),
  .data_mem_we_o              ( data_we_o                   ),
  .data_mem_be_o              ( data_be_o                   ),
  .data_mem_wdata_o           ( data_wdata_o                ),
  .data_mem_rvalid_i          ( data_rvalid_i               ), // from data memory
  .data_mem_rdata_i           ( data_rdata_i                ),

  .ex_ready_o                 ( ex_ready                    ),
  .ex_valid_o                 ( ex_valid                    ),
  .wb_ready_o                 ( wb_valid                    )
);

//////////////////////////////////////
//        ____ ____  ____           //
//       / ___/ ___||  _ \ ___      //
//      | |   \___ \| |_) / __|     //
//      | |___ ___) |  _ <\__ \     //
//       \____|____/|_| \_\___/     //
//                                  //
//   Control and Status Registers   //
//////////////////////////////////////

// Mux for CSR access through Debug Unit
assign csr_addr_int = csr_access_ex ? alu_operand_b_ex[11:0] : '0;
assign csr_access   = csr_access_ex;
assign csr_addr     = csr_addr_int;
assign csr_wdata    = alu_operand_a_ex;
assign csr_op       = csr_op_ex;

riscv_cs_registers
#(
  .N_EXT_CNT       ( 2)
)
cs_registers_i
(
  .clk                     ( clk                ),
  .rst_n                   ( rst_n              ),

  // Core and Cluster ID from outside
  .core_id_i               ( 4'b0),
  .cluster_id_i            ( 6'd0),

  // Interface to CSRs (SRAM like)
  .csr_access_i            ( csr_access         ),
  .csr_addr_i              ( csr_addr           ),
  .csr_wdata_i             ( csr_wdata          ),
  .csr_op_i                ( csr_op             ),
  .csr_rdata_o             ( csr_rdata          ),

  // Interrupt related control signals
  .irq_enable_o            ( irq_enable         ),
  .mepc_o                  ( mepc               ),

  .pc_if_i                 ( pc_if              ),
  .pc_id_i                 ( pc_id              ), // from IF stage
  .pc_ex_i                 ( pc_ex              ), // from ID/EX pipeline
  .data_load_event_ex_i    ( 1'b0 ), // from ID/EX pipeline
  .exc_save_if_i           ( exc_save_if        ),
  .exc_save_id_i           ( exc_save_id        ),
  .exc_restore_i           ( exc_restore_id     ),

  .exc_cause_i             ( exc_cause          ),
  .save_exc_cause_i        ( save_exc_cause     ),

  // performance counter related signals
  .id_valid_i              ( id_valid           ),
  .is_compressed_i         ( 1'b0 ),
  .is_decoding_i           ( is_decoding        ),

  .imiss_i                 ( perf_imiss         ),
  .pc_set_i                ( pc_set             ),
  .jump_i                  ( perf_jump          ),
  .branch_i                ( branch_in_ex       ),
  .branch_taken_i          ( branch_decision    ),
  .ld_stall_i              ( perf_ld_stall      ),
  .jr_stall_i              ( perf_jr_stall      ),

  .mem_load_i              ( data_req_o & (~data_we_o) ),
  .mem_store_i             ( data_req_o & data_we_o    ),

  .ext_counters_i          ( 2'b0)
);

////////////////////////////////////////////////////////////////////
//    ____ ___  _   _ _____ ____   ___  _     _     _____ ____    //
//   / ___/ _ \| \ | |_   _|  _ \ / _ \| |   | |   | ____|  _ \   //
//  | |  | | | |  \| | | | | |_) | | | | |   | |   |  _| | |_) |  //
//  | |__| |_| | |\  | | | |  _ <| |_| | |___| |___| |___|  _ <   //
//   \____\___/|_| \_| |_| |_| \_\\___/|_____|_____|_____|_| \_\  //
//                                                                //
////////////////////////////////////////////////////////////////////

controller controller_i
(
  .clk                            ( clk                    ),
  .rst_n                          ( rst_n                  ),

  .fetch_enable_i                 ( fetch_enable_i         ), // from top level
  .ctrl_busy_o                    ( ctrl_busy_o            ),
  .is_decoding_o                  ( is_decoding            ),

  // decoder related signals
  .illegal_insn_i                 ( illegal_insn_id        ),
  .eret_insn_i                    ( eret_insn_id           ),
  .pipe_flush_i                   ( pipe_flush_id          ),

  // jump/branch control
  .branch_taken_ex_i              ( branch_decision        ),
  .jump_mode_id_i                 ( jump_mode_id           ), //from ID stage

  // to prefetcher
  .instr_req_o                    ( instr_req_int          ),
  .pc_set_o                       ( pc_set                 ),
  .pc_mux_o                       ( pc_mux                 ),

  // from IF/ID pipeline
  .instr_valid_i                  ( instr_valid_id         ),

  // Exception Controller Signals
  .exc_req_i                      ( exc_req                ),
  .exc_ack_o                      ( exc_ack                ),

  .exc_save_if_o                  ( exc_save_if            ),
  .exc_save_id_o                  ( exc_save_id            ),
  .exc_restore_id_o               ( exc_restore_id         ),

  // Stall signals
  .halt_if_o                      ( halt_if                ),
  .halt_id_o                      ( halt_id                ),

  .jr_stall_i                     ( jr_stall               ),
  .load_stall_i                   ( load_stall             ),

  .id_ready_i                     ( id_ready               ),
  .ex_valid_i                     ( ex_valid               ),

  // Performance Counters
  .perf_jump_o                    ( perf_jump              ),
  .perf_jr_stall_o                ( perf_jr_stall          ),
  .perf_ld_stall_o                ( perf_ld_stall          )
);

///////////////////////////////////////////////////////////////////////
//  _____               ____            _             _ _            //
// | ____|_  _____     / ___|___  _ __ | |_ _ __ ___ | | | ___ _ __  //
// |  _| \ \/ / __|   | |   / _ \| '_ \| __| '__/ _ \| | |/ _ \ '__| //
// | |___ >  < (__ _  | |__| (_) | | | | |_| | | (_) | | |  __/ |    //
// |_____/_/\_\___(_)  \____\___/|_| |_|\__|_|  \___/|_|_|\___|_|    //
//                                                                   //
///////////////////////////////////////////////////////////////////////

exc_controller exc_controller_i
(
  .clk                  ( clk              ),
  .rst_n                ( rst_n            ),

  // to controller
  .req_o                ( exc_req          ),
  .ack_i                ( exc_ack          ),

  .trap_o               ( dbg_trap         ),

  // to IF stage
  .pc_mux_o             ( exc_pc_mux       ),
  .vec_pc_mux_o         ( exc_vec_pc_mux   ),

  // Interrupt signals
  .irq_i                ( irq_i            ),
  .irq_enable_i         ( irq_enable       ),

  .ebrk_insn_i          ( is_decoding & ebrk_insn_id     ),
  .illegal_insn_i       ( is_decoding & illegal_insn_id  ),
  .ecall_insn_i         ( is_decoding & ecall_insn_id    ),
  .eret_insn_i          ( is_decoding & eret_insn_id     ),

  .lsu_load_err_i       ( lsu_load_err     ),
  .lsu_store_err_i      ( lsu_store_err    ),

  .cause_o              ( exc_cause        ),
  .save_cause_o         ( save_exc_cause   ),

  .dbg_settings_i       ( 6'b0 ) // Debug settings from outside
);

endmodule