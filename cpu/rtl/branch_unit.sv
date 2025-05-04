module branch_unit (
    // INPUT
    input wire i_branch,                  // Signal to indicate if this is a branch instruction
    input wire [2:0] i_branch_op,          // Branch operation type (BEQ, BNE, etc.)
    input wire [`DATA_WIDTH-1:0] i_a,      // First operand (source register)
    input wire [`DATA_WIDTH-1:0] i_b,      // Second operand (second register or immediate)
    
    // OUTPUT
    output reg o_branch_decision                      // Flag to indicate if the branch should be taken
);

    always_comb begin
        if (i_branch) begin
            case (i_branch_op)
                `BRANCH_BEQ:   o_branch_decision = (i_a == i_b);              // Branch if Equal
                `BRANCH_BNE:   o_branch_decision = (i_a != i_b);              // Branch if Not Equal
                `BRANCH_BLT:   o_branch_decision = ($signed(i_a) < $signed(i_b)); // Branch if Less Than (signed)
                `BRANCH_BGE:   o_branch_decision = ($signed(i_a) >= $signed(i_b)); // Branch if Greater Than or Equal (signed)
                `BRANCH_BLTU:  o_branch_decision = (i_a < i_b);              // Branch if Less Than Unsigned
                `BRANCH_BGEU:  o_branch_decision = (i_a >= i_b);             // Branch if Greater Than or Equal Unsigned
                `BRANCH_JAL_JALR: o_branch_decision = 1'b1;                   // Always take for JAL/JALR (unconditional jump)
                default:          o_branch_decision = 1'b0;                      // Default to not take
            endcase
        end else begin
            o_branch_decision = 1'b0;  // If not a branch, don't take
        end
    end

endmodule
