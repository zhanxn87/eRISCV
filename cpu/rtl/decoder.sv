// decode.sv
// Decode block for RISC-V CPU

module decoder 
  import riscv_defines::*; // Import RISC-V package for constants and definitions
  //#(parameter NUM_ = 32) // Number of registers
  (
  // controller interface
  input  logic                        i_deassert_we, // deassert write enable signals
  input  logic                        i_is_compressed, // compressed instruction

  output logic                        o_illegal_inst,
  output logic                        o_ebrk_inst,  // ebreak instruction
  output logic                        o_ecall_inst, // ecall instruction
  output logic                        o_eret_inst,  // eret instruction
  output logic                        o_pipe_flush, // flush pipeline

  input  logic [INST_WIDTH-1:0]       i_inst  ,
  input  logic [31:0]                 i_pc    , // current PC in id_stage

  // Jump and branch decoded control signals
  output jump_t                       o_jump_mode,
  output branch_t                     o_branch_mode,

  output logic [31:0]                 o_jump_offset,  // jump target
  output logic [31:0]                 o_branch_offset, // branch offset

  // ALU interface
  output alu_op_t                     o_alu_op,
  output op_a_sel_t                   o_alu_op_a_sel, //
  output op_b_sel_t                   o_alu_op_b_sel, // 
  output logic [31:0]                 o_imma_extend,  // extended immediate for ALU operand A
  output logic [31:0]                 o_immb_extend,  // extended immediate for ALU operand B

  // Multiplier interface
  output mul_op_t                     mult_operator_o,
  output logic                        mult_int_en_o, // enable integer multiplier
  output logic [1:0]                  mult_signed_mode_o, // signed mode for multiplier
  output logic                        mult_sel_subword_o, // subword selection for multiplier
  output logic [0:0]                  mult_imm_mux_o, // immediate selection for multiplier
  output logic                        mult_dot_en_o, // enable dot product multiplier
  output logic [1:0]                  mult_dot_signed_o, // signed mode for dot product multiplier

  // Register file read/write decoded control signals
  output logic                        o_rs1_en,
  output logic                        o_rs2_en,
  output logic                        o_rd_alu_we,    // Enables writing to the register file from ALU
  output logic                        o_rd_mem_we,   // Enables writing to the register file from memory
  output logic [$clog2(NUM_REGS)-1:0] o_rs1_addr,
  output logic [$clog2(NUM_REGS)-1:0] o_rs2_addr,
  output logic [$clog2(NUM_REGS)-1:0] o_rd_addr,

  // memory write/load decoded control signals
  output logic                        o_mem_wr_en,
  output logic                        o_mem_req, // memory request signal both for load and store
  output logic [1:0]                  o_mem_dtype, // 2'b00 = SW, 2'b01 = SH, 2'b10 = SB
  output logic                        o_load_data_sign_ext, // 1'b0 = zero extend, 1'b1 = sign extend

  // CSR interface
  output logic                        o_csr_access,
  csr_op_t                            o_csr_op  // CSR operation type
);

  logic [INST_WIDTH-1:0]    instr;
  logic [INST_WIDTH-1:0]    imm_i_type;
  logic [INST_WIDTH-1:0]    imm_iz_type; //zero extended immediate for I-type instructions
  logic [INST_WIDTH-1:0]    imm_s_type;
  logic [INST_WIDTH-1:0]    imm_b_type;
  logic [INST_WIDTH-1:0]    imm_u_type;
  logic [INST_WIDTH-1:0]    imm_j_type; //
  logic [INST_WIDTH-1:0]    imm_z_type; //zero extended immediate for CSR instructions
  logic [31:0]              imm_pc_incr; // immediate for PC increment

  logic                     rd_mem_we;
  logic                     rd_alu_we;

  logic                     mem_wr_en;
  logic                     mem_req;

  branch_t                  branch_mode   ;
  jump_t                    jump_mode     ;

  csr_op_t                  csr_op;

  logic                     ebrk_inst ;
  logic                     eret_inst ;
  logic                     pipe_flush;

  // Decode the opcode and funct fields
  opcode_t                 opcode  ;
  logic [6:0]              funct_7 ;
  logic [2:0]              funct_3 ;

  assign opcode  = opcode_t'(i_inst[OPCODE_WIDTH-1:0]);
  assign funct_7 = i_inst[INST_WIDTH-1:INST_WIDTH-7];
  assign funct_3 = i_inst[14:12];

  // immediate extraction and sign extension
  assign instr = i_inst;
  assign imm_i_type  = { {20 {instr[31]}}, instr[31:20] }; //immediate for I-type instructions
  assign imm_iz_type = {            20'b0, instr[31:20] }; //immediate for I-type instructions (zero extended)
  assign imm_s_type  = { {20 {instr[31]}}, instr[31:25], instr[11:7] }; //immediate for S-type instructions
  assign imm_b_type  = { {19 {instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0 }; //immediate for B-type instructions
  assign imm_u_type  = {      instr[31:12], 12'b0 }; //immediate for U-type instructions
  assign imm_j_type  = { {12 {instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0 }; //immediate for UJ-type instructions

  // immediate for CSR manipulatin (zero extended)
  assign imm_z_type  = { 27'b0, instr[19:15] }; //rs1 index

  // 
  assign imm_pc_incr = (i_is_compressed) ? 32'h2 : 32'h4;

  // Decoder
  always_comb
  begin
    branch_mode              = BRANCH_NONE;
    jump_mode                = JT_NONE;

    o_branch_offset          = 32'b0;
    o_jump_offset            = 32'b0;

    o_alu_op                    = ALU_NOP; // default ALU operation
    o_alu_op_a_sel              = OP_A_REGA_OR_FWD;
    o_alu_op_b_sel              = OP_B_REGB_OR_FWD;
    o_imma_extend               = 32'b0;
    o_immb_extend               = 32'b0;

    mult_operator_o             = MUL_I;
    mult_int_en_o               = 1'b0;
    mult_dot_en_o               = 1'b0;
    mult_imm_mux_o              = 1'b0;//MIMM_ZERO;
    mult_signed_mode_o          = 2'b00;
    mult_sel_subword_o          = 1'b0;
    mult_dot_signed_o           = 2'b00;

    o_rs1_en                    = 1'b0;
    o_rs2_en                    = 1'b0;
    rd_mem_we                   = 1'b0; // Enables writing to the register file from memory
    rd_alu_we                   = 1'b0; // Enables writing to the register file from ALU

    o_csr_access                = 1'b0;
    csr_op                      = CSR_OP_NONE;

    mem_wr_en                   = 1'b0;
    mem_req                     = 1'b0;
    o_mem_dtype                 = 2'b00;
    o_load_data_sign_ext        = 1'b0;

    o_illegal_inst              = 1'b0;
    o_ecall_inst                = 1'b0;
    ebrk_inst                   = 1'b0;
    eret_inst                   = 1'b0;
    pipe_flush                  = 1'b0;

    unique case (opcode)

      //////////////////////////////////
      // Jump and Branch instructions
      //////////////////////////////////
      // JAL: Jump and Link
      // rd = PC + 4; --> ALU_ADD
      // PC = PC + imm --> Jump target
      OPCODE_JAL: begin   // Jump and Link
        jump_mode           = JT_JAL;
        // Calculate and store PC+4 to rd
        o_alu_op_a_sel      = OP_A_IMM;
        o_alu_op_b_sel      = OP_B_IMM;
        o_imma_extend       = i_pc ; // current PC
        o_immb_extend       = imm_pc_incr;// 32'h4; // aligned to 4 bytes
        o_alu_op            = ALU_ADD;
        rd_alu_we           = 1'b1;  // store PC+4 in rd
        // Calculate jump target (= PC + UJ imm)
        o_jump_offset       = imm_j_type;
      end

      // JALR: Jump and Link Register
      // rd = PC + 4; --> ALU_ADD
      // PC = RS1 + imm; --> Jump target
      OPCODE_JALR: begin  // Jump and Link Register
        jump_mode         = JT_JALR;
        // Calculate and store PC+4 to rd
        o_alu_op_a_sel    = OP_A_IMM;
        o_alu_op_b_sel    = OP_B_IMM;
        o_imma_extend     = i_pc; // current PC
        o_immb_extend     = imm_pc_incr; //32'h4; // aligned to 4 bytes
        o_alu_op          = ALU_ADD;
        rd_alu_we         = 1'b1;

        // Calculate jump target (= RS1 + I imm)
        o_rs1_en          = 1'b1;
        o_jump_offset     = imm_i_type;

        if (funct_3 != 3'b0) begin
          branch_mode     = BRANCH_NONE;
          rd_alu_we       = 1'b0;
          o_illegal_inst  = 1'b1;
        end
      end

      OPCODE_BRANCH: begin // Branch
        branch_mode       = BRANCH_COND;
        o_branch_offset   = imm_i_type;
        o_rs1_en          = 1'b1;
        o_rs2_en          = 1'b1;
        o_alu_op_a_sel    = OP_A_REGA_OR_FWD;
        o_alu_op_b_sel    = OP_B_REGB_OR_FWD;

        unique case (funct_3)
          3'b000: o_alu_op = ALU_EQ;
          3'b001: o_alu_op = ALU_NE;
          3'b100: o_alu_op = ALU_LTS;
          3'b101: o_alu_op = ALU_GES;
          3'b110: o_alu_op = ALU_LTU;
          3'b111: o_alu_op = ALU_GEU;
          default: begin
            o_alu_op = ALU_SLTU;
            o_illegal_inst = 1'b1;
          end
        endcase
      end

      //////////////////////////////////
      //Load and Store
      //////////////////////////////////
      //SW: Mem[rs1 + imm] = rs2
      OPCODE_STORE: begin
        o_rs1_en      = 1'b1;
        o_rs2_en      = 1'b1;
        mem_req       = 1'b1;
        mem_wr_en     = 1'b1;

        // offset from immediate
        o_alu_op_a_sel = OP_A_REGA_OR_FWD;
        o_alu_op_b_sel = OP_B_IMM;
        o_immb_extend  = imm_s_type;
        o_alu_op       = ALU_ADD;

        // store size
        unique case (funct_3)
          3'b000: o_mem_dtype = 2'b10; // SB
          3'b001: o_mem_dtype = 2'b01; // SH
          3'b010: o_mem_dtype = 2'b00; // SW
          default: begin
            mem_wr_en    = 1'b0;
            o_mem_dtype    = 2'b00; // SW
            o_illegal_inst = 1'b1;
          end
        endcase
      end

      //LW: rd = Mem[rs1 + imm]
      OPCODE_LOAD: begin
        rd_mem_we    = 1'b1;
        o_rs1_en     = 1'b1;
        o_rs2_en     = 1'b0;
        mem_req      = 1'b1;

        // offset from immediate
        o_alu_op_a_sel  = OP_A_REGA_OR_FWD;
        o_alu_op_b_sel  = OP_B_IMM;
        o_immb_extend   = imm_i_type;
        o_alu_op        = ALU_ADD;

        // sign/zero extension
        o_load_data_sign_ext = ~funct_3[2];

        // load size
        unique case (funct_3[1:0])
          2'b00:   o_mem_dtype = 2'b10; // LB
          2'b01:   o_mem_dtype = 2'b01; // LH
          2'b10:   o_mem_dtype = 2'b00; // LW
          default: o_mem_dtype = 2'b00; // illegal or reg-reg
        endcase
      end

      //////////////////////////
      // U-type instructions
      //////////////////////////
      //LUI: rd = 0 + (imm << 12)
      OPCODE_LUI: begin  // Load Upper Immediate
        o_alu_op_a_sel    = OP_A_IMM;
        o_alu_op_b_sel    = OP_B_IMM;
        o_imma_extend     = 32'h0; // LUI is always 0
        o_immb_extend     = imm_u_type;
        o_alu_op          = ALU_ADD;
        rd_alu_we         = 1'b1; // store LUI in rd
      end

      //AUIPC: rd = PC + (imm << 12)
      OPCODE_AUIPC: begin  // Add Upper Immediate to PC
        o_alu_op_a_sel    = OP_A_IMM;
        o_alu_op_b_sel    = OP_B_IMM;
        o_imma_extend     = i_pc;
        o_immb_extend     = imm_u_type;
        o_alu_op          = ALU_ADD;
        rd_alu_we         = 1'b1; // store AUIPC in rd
      end

      /////////////////////////////////////////////////
      // Register-Immediate ALU Operations
      /////////////////////////////////////////////////
      OPCODE_OPIMM: begin // Register-Immediate ALU Operations
        o_alu_op_a_sel    = OP_A_REGA_OR_FWD;
        o_alu_op_b_sel    = OP_B_IMM;
        o_immb_extend     = imm_i_type;
        o_rs1_en          = 1'b1;
        o_rs2_en          = 1'b0;
        rd_alu_we         = 1'b1;

        unique case (funct_3)
          3'b000: o_alu_op = ALU_ADD;  // Add Immediate
          3'b010: o_alu_op = ALU_SLTS; // Set to one if Lower Than Immediate
          3'b011: o_alu_op = ALU_SLTU; // Set to one if Lower Than Immediate Unsigned
          3'b100: o_alu_op = ALU_XOR;  // Exclusive Or with Immediate
          3'b110: o_alu_op = ALU_OR;   // Or with Immediate
          3'b111: o_alu_op = ALU_AND;  // And with Immediate

          3'b001: begin
            o_alu_op       = ALU_SLL;  // Shift Left Logical by Immediate
            if (i_inst[31:25] != 7'b0)
              o_illegal_inst = 1'b1;
          end

          3'b101: begin
            if (i_inst[31:25] == 7'b0)
              o_alu_op       = ALU_SRL;  // Shift Right Logical by Immediate
            else if (i_inst[31:25] == 7'b010_0000)
              o_alu_op       = ALU_SRA;  // Shift Right Arithmetically by Immediate
            else
              o_illegal_inst = 1'b1;
          end

          default: o_illegal_inst = 1'b1;
        endcase
      end

      /////////////////////////////////////////////////
      // Register-Register ALU Operations
      /////////////////////////////////////////////////
      OPCODE_OP: begin  // Register-Register ALU operation
        rd_alu_we   = 1'b1;
        o_rs1_en    = 1'b1;

        if (~i_inst[28]) // funct_7[5] == 0
          o_rs2_en     = 1'b1;

        o_alu_op_a_sel  = OP_A_REGA_OR_FWD;
        o_alu_op_b_sel  = OP_B_REGB_OR_FWD;
        o_rs1_en        = 1'b1;
        o_rs2_en        = 1'b1;

        unique case ({funct_7, funct_3})
          // RV32I ALU operations
          {7'b000_0000, 3'b000}: o_alu_op = ALU_ADD;   // Add
          {7'b010_0000, 3'b000}: o_alu_op = ALU_SUB;   // Sub
          {7'b000_0000, 3'b010}: o_alu_op = ALU_SLTS;  // Set Lower Than
          {7'b000_0000, 3'b011}: o_alu_op = ALU_SLTU;  // Set Lower Than Unsigned
          {7'b000_0000, 3'b100}: o_alu_op = ALU_XOR;   // Xor
          {7'b000_0000, 3'b110}: o_alu_op = ALU_OR;    // Or
          {7'b000_0000, 3'b111}: o_alu_op = ALU_AND;   // And
          {7'b000_0000, 3'b001}: o_alu_op = ALU_SLL;   // Shift Left Logical
          {7'b000_0000, 3'b101}: o_alu_op = ALU_SRL;   // Shift Right Logical
          {7'b010_0000, 3'b101}: o_alu_op = ALU_SRA;   // Shift Right Arithmetic

          // supported RV32M instructions
          {FUNCT7_ME, 3'b000}: begin // mul: rd = (rs1 * rs2)[31:0]
            mult_int_en_o      = 1'b1;
            mult_operator_o    = MUL_I;
          end
          {FUNCT7_ME, 3'b001}: begin // mulh: rd = (rs1 * rs2)[63:32]
            mult_signed_mode_o = 2'b11;
            mult_int_en_o      = 1'b1;
            mult_operator_o    = MUL_H;
          end
          {FUNCT7_ME, 3'b010}: begin // mulhsu: rd = (rs1 * rs2)[63:32] (signed * unsigned)
            mult_signed_mode_o = 2'b01;
            mult_int_en_o      = 1'b1;
            mult_operator_o    = MUL_H;
          end
          {FUNCT7_ME, 3'b011}: begin // mulhu: rd = (rs1 * rs2)[63:32] (unsigned * unsigned)
            mult_signed_mode_o = 2'b00;
            mult_int_en_o      = 1'b1;
            mult_operator_o    = MUL_H;
          end

          // Move div/rem instructions to multdiv block
          //{FUNCT7_ME, 3'b100}: begin // div: rd = (rs1 / rs2)[31:0]
          //  o_alu_op           = ALU_DIV;
          //end
          //{FUNCT7_ME, 3'b101}: begin // divu: rd = (rs1 / rs2)[31:0] (unsigned)
          //  o_alu_op           = ALU_DIVU;
          //end
          //{FUNCT7_ME, 3'b110}: begin // rem: rd = (rs1 % rs2)[31:0]
          //  o_alu_op           = ALU_REM;
          //end
          //{FUNCT7_ME, 3'b111}: begin // remu: rd = (rs1 % rs2)[31:0] (unsigned)
          //  o_alu_op           = ALU_REMU;
          //end

          default: begin
            o_illegal_inst = 1'b1;
          end
        endcase
      end // End of OPCODE_OP

      ////////////////////////////////////////////////
      // System instructions
      ////////////////////////////////////////////////
      OPCODE_SYSTEM: begin
        if (funct_3 == 3'b000)
        begin
          // non CSR related SYSTEM instructions
          unique case (i_inst[31:0])
            32'h00_00_00_73:  // ECALL
            begin
              // environment (system) call
              o_ecall_inst = 1'b1;
            end

            32'h00_10_00_73:  // ebreak
            begin
              // debugger trap
              ebrk_inst = 1'b1;
            end

            32'h10_00_00_73:  // eret
            begin
              eret_inst = 1'b1;
            end

            32'h10_20_00_73:  // wfi: wait for interrupt
            begin
              // flush pipeline
              pipe_flush = 1'b1;
            end

            default:
            begin
              o_illegal_inst = 1'b1;
            end
          endcase
        end
        else
        begin
          // instruction to read/modify CSR
          o_csr_access       = 1'b1;
          rd_alu_we          = 1'b1;

          if (i_inst[14] == 1'b1) begin
            // rs1 field is used as immediate
            o_alu_op_a_sel = OP_A_IMM;
            o_imma_extend  = imm_z_type; // rs1 index
          end else begin
            o_rs1_en       = 1'b1;
            o_alu_op_a_sel = OP_A_REGA_OR_FWD;
          end

          o_alu_op_b_sel = OP_B_IMM;
          o_immb_extend  = imm_i_type; //

          unique case (i_inst[13:12])
            2'b01:   csr_op   = CSR_OP_WRITE;
            2'b10:   csr_op   = CSR_OP_SET;
            2'b11:   csr_op   = CSR_OP_CLEAR;
            default: o_illegal_inst = 1'b1;
          endcase
        end
      end // End of OPCODE_SYSTEM

      //************************************
      // Illegal instructions
      //************************************
      default: begin
        o_illegal_inst = 1'b1;
      end
    endcase
  end // End of unique case

  // address for the rs2
  assign o_rs2_addr = i_inst[24:20];

  // LUI is (rd <= immb0 << 12), which is the same as (rd <= x0 + immb0 << 12)
  assign o_rs1_addr = (OPCODE_LUI == opcode) ? 5'b00000 : i_inst[19:15];

  // rd_addr is the destination register for the instruction
  assign o_rd_addr = i_inst[11:7];

  // deassert we signals (in case of stalls)
  assign o_rd_mem_we    = (i_deassert_we) ? 1'b0          : rd_mem_we;
  assign o_rd_alu_we    = (i_deassert_we) ? 1'b0          : rd_alu_we;
  assign o_mem_wr_en    = (i_deassert_we) ? 1'b0          : mem_wr_en;
  assign o_mem_req      = (i_deassert_we) ? 1'b0          : mem_req;
  assign o_csr_op       = (i_deassert_we) ? CSR_OP_NONE   : csr_op;
  assign o_branch_mode  = (i_deassert_we) ? BRANCH_NONE   : branch_mode;
  assign o_jump_mode    = (i_deassert_we) ? JT_NONE       : jump_mode;
  assign o_ebrk_inst    = (i_deassert_we) ? 1'b0          : ebrk_inst;
  assign o_eret_inst    = (i_deassert_we) ? 1'b0          : eret_inst;
  assign o_pipe_flush   = (i_deassert_we) ? 1'b0          : pipe_flush;


endmodule