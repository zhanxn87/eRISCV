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
  parameter CSR_OP_WIDTH = 2;

  // CSR operations
  typedef enum logic [1:0] {
      CSR_OP_READ  = 2'b00,  // READ CSR operation
      CSR_OP_WRITE = 2'b01,  // Write CSR = rs1
      CSR_OP_SET   = 2'b10,  // CSR |= rs1
      CSR_OP_CLEAR = 2'b11   // CSR &= ~rs1
  } csr_op_t;

  // CSRs mnemonics
  // imported form IBEX, some regs may be still not implemented
  typedef enum logic [11:0] {

    ///////////////////////////////////////////////////////
    // User CSRs
    ///////////////////////////////////////////////////////

    // User trap setup
    CSR_USTATUS = 12'h000,  // Not included (PULP_SECURE = 0)

    // Floating Point
    CSR_FFLAGS = 12'h001,  // Included if FPU = 1
    CSR_FRM    = 12'h002,  // Included if FPU = 1
    CSR_FCSR   = 12'h003,  // Included if FPU = 1

    // User trap setup
    CSR_UTVEC = 12'h005,  // Not included (PULP_SECURE = 0)

    // User trap handling
    CSR_UEPC   = 12'h041,  // Not included (PULP_SECURE = 0)
    CSR_UCAUSE = 12'h042,  // Not included (PULP_SECURE = 0)

    ///////////////////////////////////////////////////////
    // User Custom CSRs
    ///////////////////////////////////////////////////////

    // Hardware Loop
    CSR_LPSTART0 = 12'hCC0,  // Custom CSR. Included if PULP_HWLP = 1
    CSR_LPEND0   = 12'hCC1,  // Custom CSR. Included if PULP_HWLP = 1
    CSR_LPCOUNT0 = 12'hCC2,  // Custom CSR. Included if PULP_HWLP = 1
    CSR_LPSTART1 = 12'hCC4,  // Custom CSR. Included if PULP_HWLP = 1
    CSR_LPEND1   = 12'hCC5,  // Custom CSR. Included if PULP_HWLP = 1
    CSR_LPCOUNT1 = 12'hCC6,  // Custom CSR. Included if PULP_HWLP = 1

    // User Hart ID
    CSR_UHARTID = 12'hCD0,  // Custom CSR. User Hart ID

    // Privilege
    CSR_PRIVLV = 12'hCD1,  // Custom CSR. Privilege Level

    // ZFINX
    CSR_ZFINX = 12'hCD2,  // Custom CSR. ZFINX

    ///////////////////////////////////////////////////////
    // Machine CSRs
    ///////////////////////////////////////////////////////

    // Machine trap setup
    CSR_MSTATUS = 12'h300,
    CSR_MISA    = 12'h301,
    CSR_MIE     = 12'h304,
    CSR_MTVEC   = 12'h305,

    // Performance counters
    CSR_MCOUNTEREN    = 12'h306,
    CSR_MCOUNTINHIBIT = 12'h320,
    CSR_MHPMEVENT3    = 12'h323,
    CSR_MHPMEVENT4    = 12'h324,
    CSR_MHPMEVENT5    = 12'h325,
    CSR_MHPMEVENT6    = 12'h326,
    CSR_MHPMEVENT7    = 12'h327,
    CSR_MHPMEVENT8    = 12'h328,
    CSR_MHPMEVENT9    = 12'h329,
    CSR_MHPMEVENT10   = 12'h32A,
    CSR_MHPMEVENT11   = 12'h32B,
    CSR_MHPMEVENT12   = 12'h32C,
    CSR_MHPMEVENT13   = 12'h32D,
    CSR_MHPMEVENT14   = 12'h32E,
    CSR_MHPMEVENT15   = 12'h32F,
    CSR_MHPMEVENT16   = 12'h330,
    CSR_MHPMEVENT17   = 12'h331,
    CSR_MHPMEVENT18   = 12'h332,
    CSR_MHPMEVENT19   = 12'h333,
    CSR_MHPMEVENT20   = 12'h334,
    CSR_MHPMEVENT21   = 12'h335,
    CSR_MHPMEVENT22   = 12'h336,
    CSR_MHPMEVENT23   = 12'h337,
    CSR_MHPMEVENT24   = 12'h338,
    CSR_MHPMEVENT25   = 12'h339,
    CSR_MHPMEVENT26   = 12'h33A,
    CSR_MHPMEVENT27   = 12'h33B,
    CSR_MHPMEVENT28   = 12'h33C,
    CSR_MHPMEVENT29   = 12'h33D,
    CSR_MHPMEVENT30   = 12'h33E,
    CSR_MHPMEVENT31   = 12'h33F,

    // Machine trap handling
    CSR_MSCRATCH = 12'h340,
    CSR_MEPC     = 12'h341,
    CSR_MCAUSE   = 12'h342,
    CSR_MTVAL    = 12'h343,
    CSR_MIP      = 12'h344,

    // Physical memory protection (PMP)
    CSR_PMPCFG0   = 12'h3A0,  // Not included (USE_PMP = 0)
    CSR_PMPCFG1   = 12'h3A1,  // Not included (USE_PMP = 0)
    CSR_PMPCFG2   = 12'h3A2,  // Not included (USE_PMP = 0)
    CSR_PMPCFG3   = 12'h3A3,  // Not included (USE_PMP = 0)
    CSR_PMPADDR0  = 12'h3B0,  // Not included (USE_PMP = 0)
    CSR_PMPADDR1  = 12'h3B1,  // Not included (USE_PMP = 0)
    CSR_PMPADDR2  = 12'h3B2,  // Not included (USE_PMP = 0)
    CSR_PMPADDR3  = 12'h3B3,  // Not included (USE_PMP = 0)
    CSR_PMPADDR4  = 12'h3B4,  // Not included (USE_PMP = 0)
    CSR_PMPADDR5  = 12'h3B5,  // Not included (USE_PMP = 0)
    CSR_PMPADDR6  = 12'h3B6,  // Not included (USE_PMP = 0)
    CSR_PMPADDR7  = 12'h3B7,  // Not included (USE_PMP = 0)
    CSR_PMPADDR8  = 12'h3B8,  // Not included (USE_PMP = 0)
    CSR_PMPADDR9  = 12'h3B9,  // Not included (USE_PMP = 0)
    CSR_PMPADDR10 = 12'h3BA,  // Not included (USE_PMP = 0)
    CSR_PMPADDR11 = 12'h3BB,  // Not included (USE_PMP = 0)
    CSR_PMPADDR12 = 12'h3BC,  // Not included (USE_PMP = 0)
    CSR_PMPADDR13 = 12'h3BD,  // Not included (USE_PMP = 0)
    CSR_PMPADDR14 = 12'h3BE,  // Not included (USE_PMP = 0)
    CSR_PMPADDR15 = 12'h3BF,  // Not included (USE_PMP = 0)

    // Trigger
    CSR_TSELECT  = 12'h7A0,
    CSR_TDATA1   = 12'h7A1,
    CSR_TDATA2   = 12'h7A2,
    CSR_TDATA3   = 12'h7A3,
    CSR_TINFO    = 12'h7A4,
    CSR_MCONTEXT = 12'h7A8,
    CSR_SCONTEXT = 12'h7AA,

    // Debug/trace
    CSR_DCSR = 12'h7B0,
    CSR_DPC  = 12'h7B1,

    // Debug
    CSR_DSCRATCH0 = 12'h7B2,
    CSR_DSCRATCH1 = 12'h7B3,

    // Hardware Performance Monitor
    CSR_MCYCLE        = 12'hB00,
    CSR_MINSTRET      = 12'hB02,
    CSR_MHPMCOUNTER3  = 12'hB03,
    CSR_MHPMCOUNTER4  = 12'hB04,
    CSR_MHPMCOUNTER5  = 12'hB05,
    CSR_MHPMCOUNTER6  = 12'hB06,
    CSR_MHPMCOUNTER7  = 12'hB07,
    CSR_MHPMCOUNTER8  = 12'hB08,
    CSR_MHPMCOUNTER9  = 12'hB09,
    CSR_MHPMCOUNTER10 = 12'hB0A,
    CSR_MHPMCOUNTER11 = 12'hB0B,
    CSR_MHPMCOUNTER12 = 12'hB0C,
    CSR_MHPMCOUNTER13 = 12'hB0D,
    CSR_MHPMCOUNTER14 = 12'hB0E,
    CSR_MHPMCOUNTER15 = 12'hB0F,
    CSR_MHPMCOUNTER16 = 12'hB10,
    CSR_MHPMCOUNTER17 = 12'hB11,
    CSR_MHPMCOUNTER18 = 12'hB12,
    CSR_MHPMCOUNTER19 = 12'hB13,
    CSR_MHPMCOUNTER20 = 12'hB14,
    CSR_MHPMCOUNTER21 = 12'hB15,
    CSR_MHPMCOUNTER22 = 12'hB16,
    CSR_MHPMCOUNTER23 = 12'hB17,
    CSR_MHPMCOUNTER24 = 12'hB18,
    CSR_MHPMCOUNTER25 = 12'hB19,
    CSR_MHPMCOUNTER26 = 12'hB1A,
    CSR_MHPMCOUNTER27 = 12'hB1B,
    CSR_MHPMCOUNTER28 = 12'hB1C,
    CSR_MHPMCOUNTER29 = 12'hB1D,
    CSR_MHPMCOUNTER30 = 12'hB1E,
    CSR_MHPMCOUNTER31 = 12'hB1F,

    CSR_MCYCLEH        = 12'hB80,
    CSR_MINSTRETH      = 12'hB82,
    CSR_MHPMCOUNTER3H  = 12'hB83,
    CSR_MHPMCOUNTER4H  = 12'hB84,
    CSR_MHPMCOUNTER5H  = 12'hB85,
    CSR_MHPMCOUNTER6H  = 12'hB86,
    CSR_MHPMCOUNTER7H  = 12'hB87,
    CSR_MHPMCOUNTER8H  = 12'hB88,
    CSR_MHPMCOUNTER9H  = 12'hB89,
    CSR_MHPMCOUNTER10H = 12'hB8A,
    CSR_MHPMCOUNTER11H = 12'hB8B,
    CSR_MHPMCOUNTER12H = 12'hB8C,
    CSR_MHPMCOUNTER13H = 12'hB8D,
    CSR_MHPMCOUNTER14H = 12'hB8E,
    CSR_MHPMCOUNTER15H = 12'hB8F,
    CSR_MHPMCOUNTER16H = 12'hB90,
    CSR_MHPMCOUNTER17H = 12'hB91,
    CSR_MHPMCOUNTER18H = 12'hB92,
    CSR_MHPMCOUNTER19H = 12'hB93,
    CSR_MHPMCOUNTER20H = 12'hB94,
    CSR_MHPMCOUNTER21H = 12'hB95,
    CSR_MHPMCOUNTER22H = 12'hB96,
    CSR_MHPMCOUNTER23H = 12'hB97,
    CSR_MHPMCOUNTER24H = 12'hB98,
    CSR_MHPMCOUNTER25H = 12'hB99,
    CSR_MHPMCOUNTER26H = 12'hB9A,
    CSR_MHPMCOUNTER27H = 12'hB9B,
    CSR_MHPMCOUNTER28H = 12'hB9C,
    CSR_MHPMCOUNTER29H = 12'hB9D,
    CSR_MHPMCOUNTER30H = 12'hB9E,
    CSR_MHPMCOUNTER31H = 12'hB9F,

    CSR_CYCLE        = 12'hC00,
    CSR_INSTRET      = 12'hC02,
    CSR_HPMCOUNTER3  = 12'hC03,
    CSR_HPMCOUNTER4  = 12'hC04,
    CSR_HPMCOUNTER5  = 12'hC05,
    CSR_HPMCOUNTER6  = 12'hC06,
    CSR_HPMCOUNTER7  = 12'hC07,
    CSR_HPMCOUNTER8  = 12'hC08,
    CSR_HPMCOUNTER9  = 12'hC09,
    CSR_HPMCOUNTER10 = 12'hC0A,
    CSR_HPMCOUNTER11 = 12'hC0B,
    CSR_HPMCOUNTER12 = 12'hC0C,
    CSR_HPMCOUNTER13 = 12'hC0D,
    CSR_HPMCOUNTER14 = 12'hC0E,
    CSR_HPMCOUNTER15 = 12'hC0F,
    CSR_HPMCOUNTER16 = 12'hC10,
    CSR_HPMCOUNTER17 = 12'hC11,
    CSR_HPMCOUNTER18 = 12'hC12,
    CSR_HPMCOUNTER19 = 12'hC13,
    CSR_HPMCOUNTER20 = 12'hC14,
    CSR_HPMCOUNTER21 = 12'hC15,
    CSR_HPMCOUNTER22 = 12'hC16,
    CSR_HPMCOUNTER23 = 12'hC17,
    CSR_HPMCOUNTER24 = 12'hC18,
    CSR_HPMCOUNTER25 = 12'hC19,
    CSR_HPMCOUNTER26 = 12'hC1A,
    CSR_HPMCOUNTER27 = 12'hC1B,
    CSR_HPMCOUNTER28 = 12'hC1C,
    CSR_HPMCOUNTER29 = 12'hC1D,
    CSR_HPMCOUNTER30 = 12'hC1E,
    CSR_HPMCOUNTER31 = 12'hC1F,

    CSR_CYCLEH        = 12'hC80,
    CSR_INSTRETH      = 12'hC82,
    CSR_HPMCOUNTER3H  = 12'hC83,
    CSR_HPMCOUNTER4H  = 12'hC84,
    CSR_HPMCOUNTER5H  = 12'hC85,
    CSR_HPMCOUNTER6H  = 12'hC86,
    CSR_HPMCOUNTER7H  = 12'hC87,
    CSR_HPMCOUNTER8H  = 12'hC88,
    CSR_HPMCOUNTER9H  = 12'hC89,
    CSR_HPMCOUNTER10H = 12'hC8A,
    CSR_HPMCOUNTER11H = 12'hC8B,
    CSR_HPMCOUNTER12H = 12'hC8C,
    CSR_HPMCOUNTER13H = 12'hC8D,
    CSR_HPMCOUNTER14H = 12'hC8E,
    CSR_HPMCOUNTER15H = 12'hC8F,
    CSR_HPMCOUNTER16H = 12'hC90,
    CSR_HPMCOUNTER17H = 12'hC91,
    CSR_HPMCOUNTER18H = 12'hC92,
    CSR_HPMCOUNTER19H = 12'hC93,
    CSR_HPMCOUNTER20H = 12'hC94,
    CSR_HPMCOUNTER21H = 12'hC95,
    CSR_HPMCOUNTER22H = 12'hC96,
    CSR_HPMCOUNTER23H = 12'hC97,
    CSR_HPMCOUNTER24H = 12'hC98,
    CSR_HPMCOUNTER25H = 12'hC99,
    CSR_HPMCOUNTER26H = 12'hC9A,
    CSR_HPMCOUNTER27H = 12'hC9B,
    CSR_HPMCOUNTER28H = 12'hC9C,
    CSR_HPMCOUNTER29H = 12'hC9D,
    CSR_HPMCOUNTER30H = 12'hC9E,
    CSR_HPMCOUNTER31H = 12'hC9F,

    // Machine information
    CSR_MVENDORID = 12'hF11,
    CSR_MARCHID   = 12'hF12,
    CSR_MIMPID    = 12'hF13,
    CSR_MHARTID   = 12'hF14
  } csr_num_e;

  // CSR interrupt pending/enable bits
  parameter int unsigned CSR_MSIX_BIT = 3;
  parameter int unsigned CSR_MTIX_BIT = 7;
  parameter int unsigned CSR_MEIX_BIT = 11;
  parameter int unsigned CSR_MFIX_BIT_LOW = 16;
  parameter int unsigned CSR_MFIX_BIT_HIGH = 31;

  // SPR for debugger, not accessible by CPU
  parameter SP_DVR0 = 16'h3000;
  parameter SP_DCR0 = 16'h3008;
  parameter SP_DMR1 = 16'h3010;
  parameter SP_DMR2 = 16'h3011;

  parameter SP_DVR_MSB = 8'h00;
  parameter SP_DCR_MSB = 8'h01;
  parameter SP_DMR_MSB = 8'h02;
  parameter SP_DSR_MSB = 8'h04;

  // Privileged mode
  typedef enum logic [1:0] {
    PRIV_LVL_M = 2'b11,
    PRIV_LVL_H = 2'b10,
    PRIV_LVL_S = 2'b01,
    PRIV_LVL_U = 2'b00
  } PrivLvl_t;

  typedef struct packed {
    logic uie;
    // logic sie;      - unimplemented, hardwired to '0
    // logic hie;      - unimplemented, hardwired to '0
    logic mie;
    logic upie;
    // logic spie;     - unimplemented, hardwired to '0
    // logic hpie;     - unimplemented, hardwired to '0
    logic mpie;
    // logic spp;      - unimplemented, hardwired to '0
    // logic[1:0] hpp; - unimplemented, hardwired to '0
    PrivLvl_t mpp;
    logic mprv;
  } Status_t;

  // Machine Vendor ID - OpenHW JEDEC ID is '2 decimal (bank 13)'
  parameter MVENDORID_OFFSET = 7'h2;  // Final byte without parity bit
  parameter MVENDORID_BANK = 25'hC;  // Number of continuation codes

  // Machine Architecture ID (https://github.com/riscv/riscv-isa-manual/blob/master/marchid.md)
  parameter MARCHID = 32'h4;

  parameter MHPMCOUNTER_WIDTH = 64;

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
parameter EXC_OFF_ILLINSN  = 8'h58;
parameter EXC_OFF_ECALL    = 8'h58;
parameter EXC_OFF_LSUERR   = 8'h58;
parameter EXC_OFF_EBREAK   = 8'h58;

// Interrupt mask
parameter IRQ_MASK = 32'hFFFF0888;

// Debug module
parameter DBG_SETS_W = 6;

parameter DBG_SETS_IRQ    = 5;
parameter DBG_SETS_ECALL  = 4;
parameter DBG_SETS_EILL   = 3;
parameter DBG_SETS_ELSU   = 2;
parameter DBG_SETS_EBRK   = 1;
parameter DBG_SETS_SSTE   = 0;

parameter DBG_CAUSE_HALT   = 6'h1F;

endpackage
