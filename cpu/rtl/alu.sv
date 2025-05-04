////////////////////////////////////////////////////////////////////////////////
// Copyright 2025 @zhanxn87
// Author:         Xianning Zhan - zhanxn@gmail.com                           //
//                                                                            //
// Design Name:    ALU (Arithmetic Logic Unit)                                //
// Project Name:   eRISCV                                                     //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    RV32I ALU module for RISC-V CPU.                           //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module alu 
import riscv_defines::*; // Import RISC-V package for ALU operation codes
#(
  parameter DATA_WIDTH = 32
)  
(
  input  alu_op_t                 i_alu_op,     // ALU operation selector
  input  logic [DATA_WIDTH-1:0]   i_operand_a,  // First operand
  input  logic [DATA_WIDTH-1:0]   i_operand_b,  // Second operand

  output logic [DATA_WIDTH-1:0]   o_result,     // ALU result
  output logic                    o_compare_result, // Comparison result
  output logic                    o_zero_flag   // Zero flag
);

  logic [DATA_WIDTH-1:0] operand_a;  // First operand
  logic [DATA_WIDTH-1:0] operand_b;  // Second operand
  logic [DATA_WIDTH-1:0] result; 

  assign operand_a = i_operand_a; // Assign input operand A to internal signal
  assign operand_b = i_operand_b; // Assign input operand B to internal signal

  always_comb begin
    case (i_alu_op)
        ALU_NOP:   result = {DATA_WIDTH{1'b0}}; // No operation, result is zero
        ALU_ADD:   result = operand_a + operand_b;
        ALU_SUB:   result = operand_a - operand_b;

        ALU_AND:   result = operand_a & operand_b;
        ALU_OR:    result = operand_a | operand_b;
        ALU_XOR:   result = operand_a ^ operand_b;

        ALU_SRA:   result = $signed(operand_a) >>> operand_b[4:0];
        ALU_SRL:   result = operand_a >> operand_b[4:0];
        ALU_SLL:   result = operand_a << operand_b[4:0];
        //ALU_ROR:   result = (operand_a >> operand_b[4:0]) | (operand_a << (DATA_WIDTH - operand_b[4:0]));

        ALU_GES  : result = ($signed(operand_a) >= $signed(operand_b)) ? 1 : 0; //for bge
        ALU_LTS, 
        ALU_SLTS : result = ($signed(operand_a)  < $signed(operand_b)) ? 1 : 0; //for blt, slt, slti
        ALU_LTU, 
        ALU_SLTU : result = (operand_a  < operand_b) ? 1 : 0; //for bltu, sltiu, sltu
        ALU_GEU  : result = (operand_a >= operand_b) ? 1 : 0; //for bgeu
        ALU_EQ   : result = (operand_a == operand_b) ? 1 : 0; //for beq
        ALU_NE   : result = (operand_a != operand_b) ? 1 : 0; //for bne

        //ALU_MIN:   result = ($signed(operand_a) < $signed(operand_b)) ? operand_a : operand_b;
        //ALU_MINU:  result = (operand_a < operand_b) ? operand_a : operand_b;
        //ALU_MAX:   result = ($signed(operand_a) > $signed(operand_b)) ? operand_a : operand_b;
        //ALU_MAXU:  result = (operand_a > operand_b) ? operand_a : operand_b;

        default:   result = {DATA_WIDTH{1'b0}};
    endcase

  end

  assign o_zero_flag      = (result == 0);
  assign o_result         = result;
  assign o_compare_result = result[0];

endmodule