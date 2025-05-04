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
// Design Name:    Instruction Fetch Stage                                    //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Instruction fetch unit: Selection of the next PC, and      //
//                 buffering (sampling) of the read instruction               //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module if_stage
import riscv_defines::*;
(
    input  logic        clk,
    input  logic        rst_n,

    // the boot address is used to calculate the exception offsets
    input  logic [31:0] boot_addr_i,

    // instruction request control
    input  logic        req_i, //from controller

    // instruction memory interface
    output logic        instr_req_o, // instruction request
    output logic [31:0] instr_addr_o,
    input  logic        instr_rvalid_i, // instruction read valid
    input  logic [31:0] instr_rdata_i, //1-cycle read latency is assumed

    // Output of IF Pipeline stage
    output logic        instr_valid_id_o,      // instruction in IF/ID pipeline is valid
    output logic [31:0] instr_rdata_id_o,      // read instruction is sampled and sent to ID stage for decoding
    output logic [31:0] pc_if_o,
    output logic [31:0] pc_id_o,

    // Forwarding ports - control signals
    input  logic        clear_instr_valid_i,   // clear instruction valid bit in IF/ID pipe
    input  logic        pc_set_i,              // set the program counter to a new value
    input  pc_mux_t     pc_mux_i,              // sel for pc multiplexer
    input  exc_pc_mux_t exc_pc_mux_i,          // selects ISR address
    input  logic [31:0] exception_pc_reg_i,    // address used to restore PC when the interrupt/exception is served
    input  logic  [4:0] exc_vec_pc_mux_i,      // selects ISR address for vectorized interrupt lines

    // jump and branch target and decision
    input  logic [31:0] jump_target_id_i,      // jump target address from id_stage (jal or jalr)
    input  logic [31:0] branch_target_ex_i,      // jump target address from ex_stage (branch jump)

    // pipeline stall
    input  logic        halt_if_i,
    output logic        if_ready_o,
    input  logic        id_ready_i, //always ready?
    output logic        if_valid_o,

    // misc signals
    output logic        if_busy_o,             // is the IF stage busy fetching instructions?
    output logic        perf_imiss_o           // Instruction Fetch Miss
);

  localparam instr_nop = 32'h00000013; // nop instruction

  // offset FSM
  enum logic[0:0] {WAIT, IDLE } offset_fsm_cs, offset_fsm_ns;

  logic               illegal_c_insn = 1'b0; // illegal compressed instruction

  // prefetch buffer related signals
  logic               branch_req;
  logic [31:0]        fetch_addr_n;
               
  logic               fetch_valid;
  logic [31:0]        fetch_addr;
               
  logic [31:0]        exc_pc;

  logic [31:0]        pc_reg;           // Program counter register
  logic               illegal_c_insn_id_o; // Illegal compressed instruction flag for ID stage

  // exception PC selection mux
  always_comb
  begin : EXC_PC_MUX
    exc_pc = 'x;

    unique case (exc_pc_mux_i)
      EXC_PC_ILLINSN: exc_pc = { boot_addr_i[31:8], EXC_OFF_ILLINSN };
      EXC_PC_ECALL:   exc_pc = { boot_addr_i[31:8], EXC_OFF_ECALL   };
      EXC_PC_LOAD:    exc_pc = { boot_addr_i[31:8], EXC_OFF_LSUERR  };
      EXC_PC_IRQ:     exc_pc = { boot_addr_i[31:8], 1'b0, exc_vec_pc_mux_i[4:0], 2'b0 };
      EXC_PC_EBREAK:  exc_pc = { boot_addr_i[31:8], EXC_OFF_EBREAK  };
      // TODO: Add case for EXC_PC_STORE as soon as it differs from load

      default:;
    endcase
  end

  // fetch address selection
  always_comb
  begin
    fetch_addr_n = 'x;

    unique case (pc_mux_i)
      PC_BOOT:      fetch_addr_n = {boot_addr_i[31:8], EXC_OFF_RST}; //0x80
      PC_JUMP:      fetch_addr_n = jump_target_id_i;
      PC_BRANCH:    fetch_addr_n = branch_target_ex_i;
      PC_EXCEPTION: fetch_addr_n = exc_pc;             // set PC to exception handler
      PC_ERET:      fetch_addr_n = exception_pc_reg_i; // PC is restored when returning from IRQ/exception
      PC_DBG_NPC:   fetch_addr_n = '0; //dbg_jump_addr_i;    // PC is taken from debug unit

      default:;
    endcase
  end

  // offset FSM state
  always_ff @(posedge clk, negedge rst_n)
  begin
    if (rst_n == 1'b0) begin
      offset_fsm_cs     <= IDLE;
    end else begin
      offset_fsm_cs     <= offset_fsm_ns;
    end
  end

  // offset FSM state transition logic
  always_comb
  begin
    offset_fsm_ns = offset_fsm_cs;

    unique case (offset_fsm_cs)
      // no valid instruction data for ID stage
      // assume aligned
      IDLE: begin
        if (req_i) begin
          offset_fsm_ns = WAIT;
        end
      end

      WAIT: begin
      end

      default: begin
        offset_fsm_ns = IDLE;
      end
    endcase
  end

  assign branch_req = pc_set_i;

  assign fetch_addr   = branch_req ? fetch_addr_n : (pc_reg + 4);
  assign instr_addr_o = fetch_addr;
  assign instr_req_o  = req_i;

  always_ff @(posedge clk or posedge rst_n) begin
    if (!rst_n) begin
      pc_reg <= 32'h0;
    end else begin
      pc_reg <= fetch_addr;
    end
  end

  assign fetch_valid  = instr_rvalid_i;

  // IF-ID pipeline registers, frozen when the ID stage is stalled
  always_ff @(posedge clk, negedge rst_n)
  begin : IF_ID_PIPE_REGISTERS
    if (rst_n == 1'b0)
    begin
      instr_valid_id_o      <= 1'b0;
      instr_rdata_id_o      <= '0;
      illegal_c_insn_id_o   <= 1'b0;
      pc_if_o               <= '0;
      pc_id_o               <= '0;
    end else begin
      pc_if_o               <= fetch_addr;

      if (if_valid_o)
      begin
        instr_valid_id_o    <= fetch_valid & (!pc_set_i);
        instr_rdata_id_o    <= pc_set_i ? instr_nop : instr_rdata_i;
        illegal_c_insn_id_o <= illegal_c_insn;
        pc_id_o             <= pc_if_o;
      end
      else 
      begin
        instr_valid_id_o    <= 1'b0;
      end
    end
  end

  assign if_busy_o    = 1'b0; //never busy
  assign perf_imiss_o = (~fetch_valid) | branch_req;

  assign if_ready_o = fetch_valid & id_ready_i; //id_ready_i is always 1?
  assign if_valid_o = (~halt_if_i) & if_ready_o;

  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------

  // make sure LSB of fetch_addr_n is always 0
  assert property (
    @(posedge clk) (req_i) |-> (~fetch_addr_n[0]) )
    else $warning("There was a request while the fetch_addr_n LSB is set");

endmodule
