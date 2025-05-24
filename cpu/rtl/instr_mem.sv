// Instruction Memory Module
module instr_mem #(
  parameter ADDR_WIDTH = 10, // Address width
  parameter DATA_WIDTH = 32  // Data width
) (
  input  logic                  clk,       // Clock signal
  input  logic                  rd_req,    // Read request signal
  input  logic [ADDR_WIDTH-1:0] addr,      // Address input
  output logic                  rvalid_o,  // Read valid output signal
  output logic [DATA_WIDTH-1:0] instr_o    // Instruction output
);

  // Memory array
  logic [DATA_WIDTH-1:0] mem [(2**ADDR_WIDTH)-1:0];

  // Read instruction from memory
  always_ff @(posedge clk) begin
    instr_o <= mem[addr];
    rvalid_o <= rd_req; // Set read valid signal when read request is made
  end

  // Optional: Initialize memory with a file
  string tc_name;
  string file_name;
  initial begin
    if (!$value$plusargs("tc=%s", tc_name)) begin
        $fatal("No tc specified!");
        tc_name = "I-ADD-01"; // Default tc_name
    end
    file_name = {"./testcases/", tc_name, ".mem"};
    $display("Instr_mem using file: %s", file_name);
    $readmemh(file_name, mem);
  end

endmodule