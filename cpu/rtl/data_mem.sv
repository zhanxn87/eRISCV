module data_mem #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 10
) (
  input  logic                  clk,
  input  logic                  rst_n,
  input  logic                  req,   // Memory request signal
  input  logic                  we,        // Write enable
  input  logic [3:0]            be,        // Byte enable
  input  logic [ADDR_WIDTH-1:0] addr,      // Address
  input  logic [DATA_WIDTH-1:0] wdata,     // Write data
  output logic                  rvalid,    // Read valid signal
  output logic [DATA_WIDTH-1:0] rdata      // Read data
);

  // Memory array
  logic [DATA_WIDTH-1:0] mem [(2**ADDR_WIDTH)-1:0];

  // Read operation
  always_ff @(posedge clk) begin
    if (req && !we) begin
      rvalid <= 1'b1; // Indicate that read data is valid
      rdata  <= mem[addr];
    end else begin
      rvalid <= 1'b0; // Clear read valid signal
    end
  end

  // Write operation
  always_ff @(posedge clk) begin
    if (req && we) begin
      if (be[0]) mem[addr][ 7: 0] <= wdata[ 7: 0]; // Write byte 0
      if (be[1]) mem[addr][15: 8] <= wdata[15: 8]; // Write byte 1 
      if (be[2]) mem[addr][23:16] <= wdata[23:16]; // Write byte 0
      if (be[3]) mem[addr][31:24] <= wdata[31:24]; // Write byte 1 
    end
  end

endmodule