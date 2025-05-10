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
// Engineer:       Matthias Baer - baermatt@student.ethz.ch                   //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Andreas Traber - atraber@student.ethz.ch                   //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                                                                            //
// Design Name:    Main controller                                            //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Main CPU controller of the processor                       //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module controller
import riscv_defines::*;
(
  input  logic        clk,
  input  logic        rst_n,

  input  logic        fetch_enable_i,             // Start the decoding
  output logic        ctrl_busy_o,                // Core is busy processing instructions
  output logic        is_decoding_o,              // Core is in decoding state

  // decoder related signals
  input  logic        illegal_insn_i,             // decoder encountered an invalid instruction
  input  logic        eret_insn_i,                // decoder encountered an eret instruction
  input  logic        pipe_flush_i,               // decoder wants to do a pipe flush

  // jump/branch signals
  input  jump_t       jump_mode_dec_i,            // jump is being calculated in ALU
  input  logic        branch_taken_ex_i,          // branch taken signal from EX ALU

  // to IF_stage.prefetcher
  output logic        instr_req_o,                // Start fetching instructions
  output logic        pc_set_o,                   // jump to address set by pc_mux
  output pc_mux_t     pc_mux_o,                   // Selector in the Fetch stage to select the rigth PC (normal, jump ...)

  // from IF/ID pipeline
  input  logic        instr_valid_i,              // instruction coming from IF/ID pipeline is valid

  // Exception Controller Signals
  input  logic        exc_req_i,
  output logic        exc_ack_o,

  output logic        exc_save_if_o,
  output logic        exc_save_id_o,
  output logic        exc_restore_id_o,

  // stall signals
  output logic        halt_if_o,
  output logic        halt_id_o,

  input  logic        jr_stall_i,
  input  logic        load_stall_i,

  input  logic        id_ready_i,                 // ID stage is ready
  input  logic        ex_valid_i,                 // EX stage is done

  // Performance Counters
  output logic        perf_jump_o,                // we are executing a jump instruction   (j, jr, jal, jalr)
  output logic        perf_jr_stall_o,            // stall due to jump-register-hazard
  output logic        perf_ld_stall_o             // stall due to load-use-hazard
);

  // FSM state encoding
  enum  logic [3:0] { RESET, BOOT_SET, SLEEP, FIRST_FETCH,
                      DECODE, FLUSH_EX, FLUSH_WB} ctrl_fsm_cs, ctrl_fsm_ns;

  logic jump_done, jump_done_q;

`ifndef SYNTHESIS
  // synopsys translate_off
  // make sure we are called later so that we do not generate messages for
  // glitches
  always_ff @(negedge clk)
  begin
    // print warning in case of decoding errors
    if (is_decoding_o && illegal_insn_i) begin
      $display("%t: Illegal instruction at PC 0x%h:", $time, riscv_tb.dut.riscv_core_i.pc_id);
    end
  end
  // synopsys translate_on
`endif

  ////////////////////////////////////////////////////////////////////////////////////////////
  //   ____ ___  ____  _____    ____ ___  _   _ _____ ____   ___  _     _     _____ ____    //
  //  / ___/ _ \|  _ \| ____|  / ___/ _ \| \ | |_   _|  _ \ / _ \| |   | |   | ____|  _ \   //
  // | |  | | | | |_) |  _|   | |  | | | |  \| | | | | |_) | | | | |   | |   |  _| | |_) |  //
  // | |__| |_| |  _ <| |___  | |__| |_| | |\  | | | |  _ <| |_| | |___| |___| |___|  _ <   //
  //  \____\___/|_| \_\_____|  \____\___/|_| \_| |_| |_| \_\\___/|_____|_____|_____|_| \_\  //
  //                                                                                        //
  ////////////////////////////////////////////////////////////////////////////////////////////
  always_comb
  begin
    // Default values
    instr_req_o      = 1'b1;

    exc_ack_o        = 1'b0;
    exc_save_if_o    = 1'b0;
    exc_save_id_o    = 1'b0;
    exc_restore_id_o = 1'b0;

    pc_set_o         = 1'b0;
    pc_mux_o         = PC_BOOT;
    jump_done        = jump_done_q;

    ctrl_fsm_ns      = ctrl_fsm_cs;

    ctrl_busy_o      = 1'b1;
    is_decoding_o    = 1'b0;

    halt_if_o        = 1'b0;
    halt_id_o        = 1'b0;

    unique case (ctrl_fsm_cs)
      // We were just reset, wait for fetch_enable
      RESET:
      begin
        ctrl_busy_o   = 1'b0;
        instr_req_o   = 1'b0;

        if (fetch_enable_i == 1'b1)
          ctrl_fsm_ns = BOOT_SET;
      end

      // copy boot address to instr fetch address
      BOOT_SET:
      begin
        instr_req_o   = 1'b1;
        pc_mux_o      = PC_BOOT;
        pc_set_o      = 1'b1;

        ctrl_fsm_ns   = FIRST_FETCH;
      end

      FIRST_FETCH:
      begin
        if (id_ready_i == 1'b1)
        begin
          ctrl_fsm_ns = DECODE;
        end

        // handle exceptions
        if (exc_req_i) begin
          pc_mux_o     = PC_EXCEPTION;
          pc_set_o     = 1'b1;
          exc_ack_o    = 1'b1;

          // TODO: This assumes that the pipeline is always flushed before
          //       going to sleep.
          exc_save_if_o = 1'b1;
        end
      end

      DECODE:
      begin
        is_decoding_o = 1'b0;

        // decode and execute instructions only if the current conditional
        // branch in the EX stage is either not taken, or there is no
        // conditional branch in the EX stage
        if (instr_valid_i && (~branch_taken_ex_i))
        begin // now analyze the current instruction in the ID stage
          is_decoding_o = 1'b1;

          // handle unconditional jumps
          // we can jump directly since we know the address already
          // we don't need to worry about conditional branches here as they
          // will be evaluated in the EX stage
          if (jump_mode_dec_i == JT_JALR || jump_mode_dec_i == JT_JAL) begin
            pc_mux_o = PC_JUMP;

            // if there is a jr stall, wait for it to be gone
            if ((~jr_stall_i) && (~jump_done_q)) begin
              pc_set_o    = 1'b1;
              jump_done   = 1'b1;
            end

            // we don't have to change our current state here as the prefetch
            // buffer is automatically invalidated, thus the next instruction
            // that is served to the ID stage is the one of the jump target
          end else begin
            // handle exceptions
            if (exc_req_i) begin
              pc_mux_o      = PC_EXCEPTION;
              pc_set_o      = 1'b1;
              exc_ack_o     = 1'b1;

              halt_id_o     = 1'b1; // we don't want to propagate this instruction to EX
              exc_save_id_o = 1'b1;

              // we don't have to change our current state here as the prefetch
              // buffer is automatically invalidated, thus the next instruction
              // that is served to the ID stage is the one of the jump to the
              // exception handler
            end
          end

          if (eret_insn_i) begin
            pc_mux_o         = PC_ERET;
            exc_restore_id_o = 1'b1;

            if ((~jump_done_q)) begin
              pc_set_o    = 1'b1;
              jump_done   = 1'b1;
            end
          end

          // handle WFI instruction, flush pipeline and (potentially) go to
          // sleep
          // also handles eret when the core should go back to sleep
          if (pipe_flush_i || (eret_insn_i && (~fetch_enable_i)))
          begin
            halt_if_o = 1'b1;
            halt_id_o = 1'b1;

            ctrl_fsm_ns = FLUSH_EX;
          end
        end

        // TODO: make sure this is not done multiple times in a row!!!
        //       maybe with an assertion?
        // handle conditional branches
        if (branch_taken_ex_i) begin
          // there is a branch in the EX stage that is taken
          pc_mux_o      = PC_BRANCH;
          pc_set_o      = 1'b1;

          is_decoding_o = 1'b0; // we are not decoding the current instruction in the ID stage
        end
      end

      // flush the pipeline, insert NOP into EX stage
      FLUSH_EX:
      begin
        halt_if_o = 1'b1;
        halt_id_o = 1'b1;

        if (ex_valid_i)
          ctrl_fsm_ns = FLUSH_WB;
        else
          ctrl_fsm_ns = FLUSH_EX;
      end

      // flush the pipeline, insert NOP into EX and WB stage
      FLUSH_WB:
      begin
        halt_if_o = 1'b1;
        halt_id_o = 1'b1;

        if(fetch_enable_i) begin
          ctrl_fsm_ns = DECODE;
          halt_if_o   = 1'b0;
        end else begin
          ctrl_fsm_ns = SLEEP;
        end
      end

      // instruction in if_stage is already valid
      SLEEP:
      begin
        // we begin execution when either fetch_enable is high or an
        // interrupt has arrived
        ctrl_busy_o   = 1'b0;
        instr_req_o   = 1'b0;
        halt_if_o     = 1'b1;
        halt_id_o     = 1'b1;

        // no debug request incoming, normal execution flow
        if (fetch_enable_i || exc_req_i)
        begin
          ctrl_fsm_ns  = FIRST_FETCH;
        end
      end

      default: begin
        instr_req_o = 1'b0;
        ctrl_fsm_ns = RESET;
      end
    endcase
  end

  // update registers
  always_ff @(posedge clk , negedge rst_n)
  begin : UPDATE_REGS
    if ( rst_n == 1'b0 )
    begin
      ctrl_fsm_cs <= RESET;
      jump_done_q <= 1'b0;
    end
    else
    begin
      ctrl_fsm_cs <= ctrl_fsm_ns;

      // clear when id is valid (no instruction incoming)
      jump_done_q <= jump_done & (~id_ready_i);
    end
  end

  // Performance Counters
  assign perf_jump_o      = (jump_mode_dec_i == JT_JAL || jump_mode_dec_i == JT_JALR);
  assign perf_jr_stall_o  = jr_stall_i;
  assign perf_ld_stall_o  = load_stall_i;

  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------

  // make sure that taken branches do not happen back-to-back, as this is not
  // possible without branch prediction in the IF stage
  assert property (
    @(posedge clk) (branch_taken_ex_i) |=> (~branch_taken_ex_i) ) else $warning("Two branches back-to-back are taken");

endmodule // controller
