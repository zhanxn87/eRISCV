// Description: RISC-V Core Top Level Module

module riscv (
    input  logic            clk,
    input  logic            rst_n,

    // CPU control signals
    input  logic            fetch_enable_i,
    output logic            core_busy_o,

    // Interrupt interface
    input  logic [31:0]     irq_i
);

// Instruction memory interface signals
logic [31:0] instr_addr;
logic [31:0] instr_rdata;

// Data memory interface signals
logic           mem_req;
logic [31:0]    mem_addr;
logic [31:0]    mem_wdata;
logic           mem_we;
logic [3:0]     mem_be;
logic           mem_rvalid;
logic [31:0]    mem_rdata;

riscv_core riscv_core_i(
  .clk              (clk            ),       // Clock signal
  .rst_n            (rst_n          ),

  // CPU control signals
  .fetch_enable_i   (fetch_enable_i ),
  .ctrl_busy_o      (core_busy_o    ),

  // Instruction memory interface
  .instr_req_o      (instr_req      ),
  .instr_addr_o     (instr_addr     ),
  .instr_rvalid_i   (instr_rvalid   ),
  .instr_rdata_i    (instr_rdata    ),

  // Data memory interface
  .data_req_o       (mem_req        ),
  .data_addr_o      (mem_addr       ), 
  .data_wdata_o     (mem_wdata      ),
  .data_we_o        (mem_we         ), 
  .data_be_o        (mem_be         ), 
  .data_rvalid_i    (mem_rvalid     ),
  .data_rdata_i     (mem_rdata      ),

  // Interrupt interface
  .irq_i            (irq_i          )
);

instr_mem #(
  .ADDR_WIDTH ( 10), // Address width
  .DATA_WIDTH ( 32)  // Data width
) instr_mem_i(
  .clk        (clk                ),       // Clock signal
  .rd_req     (instr_req          ),       // Read request signal
  .addr       (instr_addr[11:2]   ),      // Address input
  .rvalid_o   (instr_rvalid       ),
  .instr_o    (instr_rdata        )      // Instruction output
);

data_mem #(
  .ADDR_WIDTH ( 10 ),
  .DATA_WIDTH ( 32 )
) data_mem_i(
  .clk        (clk          ),     // Clock signal
  .rst_n      (rst_n        ),     // Reset signal
  .req        (mem_req      ),     // Memory request signal
  .addr       (mem_addr[11:2]     ),     // Address
  .we         (mem_we       ),     // Write enable
  .be         (mem_be       ),     // Write enable
  .wdata      (mem_wdata    ),     // Write data
  .rvalid     (mem_rvalid   ),     // Read valid
  .rdata      (mem_rdata    )      // Read data
);

endmodule
