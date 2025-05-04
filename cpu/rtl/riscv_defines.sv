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
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                                                                            //
//                                                                            //
// Design Name:    RISC-V processor core                                      //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Defines for various constants used by the processor core.  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

package riscv_defines;

////////////////////////////////////////////////////////////////////////////////
// Constants for instruction width, opcode, funct, etc.
////////////////////////////////////////////////////////////////////////////////
parameter INST_WIDTH   = 32;
parameter OPCODE_WIDTH = 7;

parameter NUM_REGS = 32;

//////////////////////////////////////////////////////////////
// Instruction format
//////////////////////////////////////////////////////////////
typedef enum logic [6:0] {
    OPCODE_SYSTEM  = 7'h73, // SYSTEM instructions (ECALL, EBREAK, WFI, CSR*)
    OPCODE_FENCE   = 7'h0f, // FENCE, FENCE.I
    OPCODE_OP      = 7'h33, // R-type (ADD, SUB, AND, OR, etc.)
    OPCODE_OPIMM   = 7'h13, // I-type (ADDI, ANDI, ORI, etc.)
    OPCODE_STORE   = 7'h23, // S-type (SW, SH, SB)
    OPCODE_LOAD    = 7'h03, // I-type (LW, LH, LHU, LB, LBU)
    OPCODE_BRANCH  = 7'h63, // B-type (BEQ, BNE, BLT, BGE, etc.)
    OPCODE_JALR    = 7'h67, // I-type (JALR)
    OPCODE_JAL     = 7'h6f, // J-type (JAL)
    OPCODE_AUIPC   = 7'h17, // U-type (AUIPC)
    OPCODE_LUI     = 7'h37  // U-type (LUI)
} opcode_t;

parameter FUNCT7_ME        = 7'h01; // RV32M Multiply Extension (OPCODE must be R-type)


//////////////////////////////////////////////////////////////
//ALU operations
//////////////////////////////////////////////////////////////
//parameter ALU_OP_WIDTH = 6;

typedef enum logic [4:0] {
    // ----------------------------
    // Arithmetic operations
    // ----------------------------
    ALU_NOP    = 'h00,  // No operation
    ALU_ADD    = 'h01,
    ALU_SUB    = 'h02,

    // ----------------------------
    // Bitwise logical operations
    // ----------------------------
    ALU_XOR    = 'h03,
    ALU_OR     = 'h04,
    ALU_AND    = 'h05,

    // ----------------------------
    // Shift operations
    // ----------------------------
    ALU_SRA    = 'h06,  // Arithmetic right shift
    ALU_SRL    = 'h07,  // Logical right shift
    ALU_SLL    = 'h08,  // Logical left shift

    // ----------------------------
    // Comparisons
    // ----------------------------
    ALU_EQ     = 'h09,  // Equal
    ALU_NE     = 'h0A,  // Not equal
    ALU_LTS    = 'h0B,  // Less than signed
    ALU_LTU    = 'h0C,  // Less than unsigned
    ALU_GES    = 'h0D,  // Greater than or equal signed
    ALU_GEU    = 'h0E,  // Greater than or equal unsigned
    ALU_SLTS   = 'h0F,  // Set less than signed
    ALU_SLTU   = 'h10   // Set less than unsigned

  //ALU_LEU    = 'h000101,  // Less than or equal unsigned
  //ALU_GTS    = 'h001000,  // Greater than signed
  //ALU_GTU    = 'h001001,  // Greater than unsigned
  //ALU_SLETS  = 'h000110,  // Set less or equal signed
  //ALU_SLETU  = 'h000111   // Set less or equal unsigned
} alu_op_t;

typedef enum logic [2:0] {
    MUL_MAC32 = 3'b000, // 32-bit multiply-accumulate
    MUL_MSU32 = 3'b001, // 32-bit multiply-subtract
    MUL_I     = 3'b010, // Integer multiply
    MUL_IR    = 3'b011, // Integer multiply with rounding
    MUL_DOT8  = 3'b100, // 8-bit dot product
    MUL_DOT16 = 3'b101, // 16-bit dot product
    MUL_H     = 3'b110  // High-part multiply
} mul_op_t;


/////////////////////////////////////////////////////////
//    ____ ____    ____            _     _             //
//   / ___/ ___|  |  _ \ ___  __ _(_)___| |_ ___ _ __  //
//  | |   \___ \  | |_) / _ \/ _` | / __| __/ _ \ '__| //
//  | |___ ___) | |  _ <  __/ (_| | \__ \ ||  __/ |    //
//   \____|____/  |_| \_\___|\__, |_|___/\__\___|_|    //
//                           |___/                     //
/////////////////////////////////////////////////////////

// CSR operations
typedef enum logic [1:0] {
    CSR_OP_NONE  = 2'b00,  // No CSR operation
    CSR_OP_WRITE = 2'b01,  // Write CSR = rs1
    CSR_OP_SET   = 2'b10,  // CSR |= rs1
    CSR_OP_CLEAR = 2'b11   // CSR &= ~rs1
} csr_op_t;

///////////////////////////////////////////////
//   ___ ____    ____  _                     //
//  |_ _|  _ \  / ___|| |_ __ _  __ _  ___   //
//   | || | | | \___ \| __/ _` |/ _` |/ _ \  //
//   | || |_| |  ___) | || (_| | (_| |  __/  //
//  |___|____/  |____/ \__\__,_|\__, |\___|  //
//                              |___/        //
///////////////////////////////////////////////

typedef enum logic [1:0] {
    SEL_REGFILE = 2'b00, // Select value from register file
    SEL_FW_EX   = 2'b01, // Forward from EX stage
    SEL_FW_WB   = 2'b10  // Forward from WB stage
} fw_sel_t;

typedef enum logic [1:0] {
    OP_A_REGA_OR_FWD = 2'b00, // Select value from register A or forward
    OP_A_CURRPC      = 2'b01, // Select current PC value
    OP_A_IMM         = 2'b10, // Select immediate value
    OP_A_REGB_OR_FWD = 2'b11  // Select value from register B or forward
} op_a_sel_t;

typedef enum logic [1:0] {
    OP_B_REGB_OR_FWD = 2'b00, // Select value from register B or forward
    OP_B_REGC_OR_FWD = 2'b01, // Select value from register C or forward
    OP_B_IMM         = 2'b10  // Select immediate value
} op_b_sel_t;

typedef enum logic [3:0] {
    IMMB_I      = 4'b0000, // Immediate type I
    IMMB_S      = 4'b0001, // Immediate type S
    IMMB_U      = 4'b0010, // Immediate type U
    IMMB_PCINCR = 4'b0011  // Immediate type PC increment
} imm_b_sel_t;

typedef enum logic [0:0] {
    OP_SRC_REG = 1'b0, // Select value from register
    OP_SRC_IMM = 1'b1  // Select immediate value
} op_src_sel_t;

typedef enum logic [0:0] {
    IMMA_Z    = 1'b0, // Zero-extended immediate A
    IMMA_ZERO = 1'b1  // Zero-extended immediate A
} imm_a_sel_t;

typedef enum logic [0:0] {
    BRANCH_NONE = 1'b0, // No branch
    BRANCH_COND = 1'b1  // Conditional branch
} branch_t;

typedef enum logic [1:0] {
    JT_NONE = 2'b00, // No jump
    JT_JAL  = 2'b01, // Jump and link
    JT_JALR = 2'b10  // Jump and link register
} jump_t;

///////////////////////////////////////////////
//   ___ _____   ____  _                     //
//  |_ _|  ___| / ___|| |_ __ _  __ _  ___   //
//   | || |_    \___ \| __/ _` |/ _` |/ _ \  //
//   | ||  _|    ___) | || (_| | (_| |  __/  //
//  |___|_|     |____/ \__\__,_|\__, |\___|  //
//                              |___/        //
///////////////////////////////////////////////

// PC mux selector defines
typedef enum logic [2:0] {
    PC_BOOT      = 3'b000,
    PC_JUMP      = 3'b010,
    PC_BRANCH    = 3'b011,
    PC_EXCEPTION = 3'b100,
    PC_ERET      = 3'b101,
    PC_DBG_NPC   = 3'b111
} pc_mux_t;

// Exception PC mux selector defines
typedef enum logic [2:0] {
    EXC_PC_ILLINSN = 3'b000,
    EXC_PC_ECALL   = 3'b001,
    EXC_PC_EBREAK  = 3'b010,
    EXC_PC_LOAD    = 3'b011,
    EXC_PC_STORE   = 3'b100,
    EXC_PC_IRQ     = 3'b101
} exc_pc_mux_t;

// Exceptions offsets
// target address = {boot_addr[31:8], EXC_OFF} (boot_addr must be 32 BYTE aligned!)
// offset 00 to 7e is used for external interrupts
parameter EXC_OFF_RST      = 8'h80;
parameter EXC_OFF_ILLINSN  = 8'h84;
parameter EXC_OFF_ECALL    = 8'h88;
parameter EXC_OFF_LSUERR   = 8'h8c;
parameter EXC_OFF_EBREAK   = 8'h88;

endpackage
