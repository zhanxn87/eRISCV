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
  string filename;
  initial begin
    if (!$value$plusargs("instr_init=%s", filename)) begin
        $fatal("No instr_init filename specified!");
        filename = "instr_mem_init.hex"; // Default filename
    end
    $display("Using file: %s", filename);
    $readmemh(filename, mem);
  end

endmodule