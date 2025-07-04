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
// Engineer:       Igor Loi - igor.loi@unibo.it                               //
//                                                                            //
// Additional contributions by:                                               //
//                 Andreas Traber - atraber@iis.ee.ethz.ch                    //
//                                                                            //
// Design Name:    Load Store Unit                                            //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Load Store Unit, used to eliminate multiple access during  //
//                 processor stalls, and to align bytes and halfwords         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module load_store_unit
(
    input  logic         clk,
    input  logic         rst_n,

    // output to data memory
    output logic         data_req_o,
    output logic [31:0]  data_addr_o,
    output logic         data_we_o,
    output logic [3:0]   data_be_o,
    output logic [31:0]  data_wdata_o,
    input  logic         data_rvalid_i,
    input  logic [31:0]  data_rdata_i,

    // signals from id/ex pipeline 
    input  logic         data_we_ex_i,         // write enable                      -> from ex stage
    input  logic [1:0]   data_type_ex_i,       // Data type word, halfword, byte    -> from ex stage
    input  logic [31:0]  data_wdata_ex_i,      // data to write to memory           -> from ex stage
    input  logic [1:0]   data_reg_offset_ex_i, // offset inside register for stores -> from ex stage
    input  logic         data_sign_ext_ex_i,   // sign extension                    -> from ex stage

    output logic [31:0]  data_rdata_ex_o,      // requested data                    -> to ex stage
    input  logic         data_req_ex_i,        // data request                      -> from ex stage
    input  logic [31:0]  data_addr_i,          // operand a from RF for address     -> from ex stage

    input  logic         data_misaligned_ex_i, // misaligned access in last ld/st   -> from ID/EX pipeline
    output logic         data_misaligned_o,    // misaligned access was detected    -> to controller

    // exception signals
    output logic         load_err_o,
    output logic         store_err_o,

    // stall signal
    output logic         lsu_ready_ex_o, // LSU ready for new data in EX stage
    output logic         lsu_ready_wb_o, // LSU ready for new data in WB stage

    output logic         busy_o
);

  logic [31:0]  data_addr_int;

  // registers for data_rdata alignment and sign extension
  logic [1:0]   data_type_q;
  logic [1:0]   rdata_offset_q;
  logic         data_sign_ext_q;
  logic         data_we_q;

  logic [1:0]   wdata_offset;   // mux control for data to be written to memory

  logic [3:0]   data_be;
  logic [31:0]  data_wdata;

  logic         misaligned_st;   // high if we are currently performing the second part of a misaligned store

  //enum logic [1:0]  { IDLE, WAIT_RVALID, WAIT_RVALID_EX_STALL, IDLE_EX_STALL } CS, NS;

  logic [31:0]  rdata_q;

  ///////////////////////////////// BE generation ////////////////////////////////
  always_comb
  begin
    case (data_type_ex_i) // Data type 00 Word, 01 Half word, 11,10 byte
      2'b00:
      begin // Writing a word
        if (misaligned_st == 1'b0)
        begin // non-misaligned case
          case (data_addr_int[1:0])
            2'b00: data_be = 4'b1111;
            2'b01: data_be = 4'b1110;
            2'b10: data_be = 4'b1100;
            2'b11: data_be = 4'b1000;
          endcase; // case (data_addr_int[1:0])
        end
        else
        begin // misaligned case
          case (data_addr_int[1:0])
            2'b00: data_be = 4'b0000; // this is not used, but included for completeness
            2'b01: data_be = 4'b0001;
            2'b10: data_be = 4'b0011;
            2'b11: data_be = 4'b0111;
          endcase; // case (data_addr_int[1:0])
        end
      end

      2'b01:
      begin // Writing a half word
        if (misaligned_st == 1'b0)
        begin // non-misaligned case
          case (data_addr_int[1:0])
            2'b00: data_be = 4'b0011;
            2'b01: data_be = 4'b0110;
            2'b10: data_be = 4'b1100;
            2'b11: data_be = 4'b1000;
          endcase; // case (data_addr_int[1:0])
        end
        else
        begin // misaligned case
          data_be = 4'b0001;
        end
      end

      2'b10,
      2'b11: begin // Writing a byte
        case (data_addr_int[1:0])
          2'b00: data_be = 4'b0001;
          2'b01: data_be = 4'b0010;
          2'b10: data_be = 4'b0100;
          2'b11: data_be = 4'b1000;
        endcase; // case (data_addr_int[1:0])
      end
    endcase; // case (data_type_ex_i)
  end

  // prepare data to be written to the memory
  // we handle misaligned accesses, half word and byte accesses and
  // register offsets here
  assign wdata_offset = data_addr_int[1:0] - data_reg_offset_ex_i[1:0];
  always_comb
  begin
    case (wdata_offset)
      2'b00: data_wdata = data_wdata_ex_i[31:0];
      2'b01: data_wdata = {data_wdata_ex_i[23:0], data_wdata_ex_i[31:24]};
      2'b10: data_wdata = {data_wdata_ex_i[15:0], data_wdata_ex_i[31:16]};
      2'b11: data_wdata = {data_wdata_ex_i[ 7:0], data_wdata_ex_i[31: 8]};
    endcase; // case (wdata_offset)
  end

  // FF for rdata alignment and sign-extension
  always_ff @(posedge clk, negedge rst_n)
  begin
    if(rst_n == 1'b0)
    begin
      data_type_q     <= '0;
      rdata_offset_q  <= '0;
      data_sign_ext_q <= '0;
      data_we_q       <= 1'b0;
    end
    else// if (data_gnt_i == 1'b1) // request was granted, we wait for rvalid and can continue to WB
    begin
      data_type_q     <= data_type_ex_i;
      rdata_offset_q  <= data_addr_int[1:0];
      data_sign_ext_q <= data_sign_ext_ex_i;
      data_we_q       <= data_we_ex_i;
    end
  end

  ////////////////////////////////////////////////////////////////////////
  //  ____  _               _____      _                 _              //
  // / ___|(_) __ _ _ __   | ____|_  _| |_ ___ _ __  ___(_) ___  _ __   //
  // \___ \| |/ _` | '_ \  |  _| \ \/ / __/ _ \ '_ \/ __| |/ _ \| '_ \  //
  //  ___) | | (_| | | | | | |___ >  <| ||  __/ | | \__ \ | (_) | | | | //
  // |____/|_|\__, |_| |_| |_____/_/\_\\__\___|_| |_|___/_|\___/|_| |_| //
  //          |___/                                                     //
  ////////////////////////////////////////////////////////////////////////

  logic [31:0] data_rdata_ext;

  logic [31:0] rdata_w_ext; // sign extension for words, actually only misaligned assembly
  logic [31:0] rdata_h_ext; // sign extension for half words
  logic [31:0] rdata_b_ext; // sign extension for bytes

  // take care of misaligned words
  always_comb
  begin
    case (rdata_offset_q)
      2'b00: rdata_w_ext = data_rdata_i[31:0];
      2'b01: rdata_w_ext = {data_rdata_i[ 7:0], rdata_q[31:8]};
      2'b10: rdata_w_ext = {data_rdata_i[15:0], rdata_q[31:16]};
      2'b11: rdata_w_ext = {data_rdata_i[23:0], rdata_q[31:24]};
    endcase
  end

  // sign extension for half words
  always_comb
  begin
    case (rdata_offset_q)
      2'b00:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_h_ext = {16'h0000, data_rdata_i[15:0]};
        else
          rdata_h_ext = {{16{data_rdata_i[15]}}, data_rdata_i[15:0]};
      end

      2'b01:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_h_ext = {16'h0000, data_rdata_i[23:8]};
        else
          rdata_h_ext = {{16{data_rdata_i[23]}}, data_rdata_i[23:8]};
      end

      2'b10:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_h_ext = {16'h0000, data_rdata_i[31:16]};
        else
          rdata_h_ext = {{16{data_rdata_i[31]}}, data_rdata_i[31:16]};
      end

      2'b11:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_h_ext = {16'h0000, data_rdata_i[7:0], rdata_q[31:24]};
        else
          rdata_h_ext = {{16{data_rdata_i[7]}}, data_rdata_i[7:0], rdata_q[31:24]};
      end
    endcase // case (rdata_offset_q)
  end

  // sign extension for bytes
  always_comb
  begin
    case (rdata_offset_q)
      2'b00:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_b_ext = {24'h00_0000, data_rdata_i[7:0]};
        else
          rdata_b_ext = {{24{data_rdata_i[7]}}, data_rdata_i[7:0]};
      end

      2'b01: begin
        if (data_sign_ext_q == 1'b0)
          rdata_b_ext = {24'h00_0000, data_rdata_i[15:8]};
        else
          rdata_b_ext = {{24{data_rdata_i[15]}}, data_rdata_i[15:8]};
      end

      2'b10:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_b_ext = {24'h00_0000, data_rdata_i[23:16]};
        else
          rdata_b_ext = {{24{data_rdata_i[23]}}, data_rdata_i[23:16]};
      end

      2'b11:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_b_ext = {24'h00_0000, data_rdata_i[31:24]};
        else
          rdata_b_ext = {{24{data_rdata_i[31]}}, data_rdata_i[31:24]};
      end
    endcase // case (rdata_offset_q)
  end

  // select word, half word or byte sign extended version
  always_comb
  begin
    case (data_type_q)
      2'b00:       data_rdata_ext = rdata_w_ext;
      2'b01:       data_rdata_ext = rdata_h_ext;
      2'b10,2'b11: data_rdata_ext = rdata_b_ext;
    endcase //~case(rdata_type_q)
  end

  always_ff @(posedge clk, negedge rst_n)
  begin
    if(rst_n == 1'b0)
    begin
      //CS            <= IDLE;
      rdata_q       <= '0;
    end
    else
    begin
      //CS            <= NS;

      if (data_rvalid_i && (~data_we_q))
      begin
        // if we have detected a misaligned access, and we are
        // currently doing the first part of this access, then
        // store the data coming from memory in rdata_q.
        // In all other cases, rdata_q gets the value that we are
        // writing to the register file
        if ((data_misaligned_ex_i == 1'b1) || (data_misaligned_o == 1'b1))
          rdata_q  <= data_rdata_i;
        else
          rdata_q  <= data_rdata_ext;
      end
    end
  end

  // output to register file
  assign data_rdata_ex_o = (data_rvalid_i == 1'b1) ? data_rdata_ext : rdata_q;

  // output to data-mem interface
  assign data_addr_o   = data_addr_int;
  assign data_wdata_o  = data_wdata;
  assign data_we_o     = data_we_ex_i;
  assign data_be_o     = data_be;

  assign data_req_o    = data_req_ex_i;

  assign misaligned_st = data_misaligned_ex_i;

  assign load_err_o    = 1'b0;
  assign store_err_o   = 1'b0;
  assign lsu_ready_ex_o = 1'b1;
  assign lsu_ready_wb_o = 1'b1;

  // check for misaligned accesses that need a second memory access
  // If one is detected, this is signaled with data_misaligned_o to
  // the controller which selectively stalls the pipeline
  always_comb
  begin
    data_misaligned_o = 1'b0;

    if((data_req_ex_i == 1'b1) && (data_misaligned_ex_i == 1'b0))
    begin
      case (data_type_ex_i)
        2'b00: // word
        begin
          if(data_addr_int[1:0] != 2'b00)
            data_misaligned_o = 1'b1;
        end
        2'b01: // half word
        begin
          if(data_addr_int[1:0] == 2'b11)
            data_misaligned_o = 1'b1;
        end
      endcase // case (data_type_ex_i)
    end
  end

  // generate address from operands
  assign data_addr_int = data_addr_i;

  //assign busy_o = (CS == WAIT_RVALID) || (CS == WAIT_RVALID_EX_STALL) || (CS == IDLE_EX_STALL) || (data_req_o == 1'b1);
  assign busy_o = 1'b0; //(data_req_o == 1'b1);

  //////////////////////////////////////////////////////////////////////////////
  // Assertions
  //////////////////////////////////////////////////////////////////////////////

  // make sure there is no new request when the old one is not yet completely done
  // i.e. it should not be possible to get a grant without an rvalid for the
  // last request
  //assert property (
  //  @(posedge clk) ((CS == WAIT_RVALID)) |-> (data_rvalid_i == 1'b1) );

  //// there should be no rvalid when we are in IDLE
  //assert property (
  //  @(posedge clk) (CS == IDLE) |-> (data_rvalid_i == 1'b0) );

  // assert that the address does not contain X when request is sent
  assert property ( @(posedge clk) (data_req_o) |-> (!$isunknown(data_addr_o)) );

endmodule
