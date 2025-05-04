`timescale 1ns/1ps

module riscv_tb;

  // Parameters
  localparam CLK_PERIOD = 10; // 100 MHz

  // DUT signals
  logic clk;
  logic rst_n;
  logic fetch_enable_i;
  logic [31:0] irq_i;

  // Instantiate DUT
  riscv dut (
    .clk            (clk),
    .rst_n          (rst_n),
    .fetch_enable_i (fetch_enable_i),
    .core_busy_o    (),
    .irq_i          (irq_i)
  );

  // Clock generation
  initial clk = 0;
  always #(CLK_PERIOD / 2) clk = ~clk;

  // Test sequence
  initial begin
    $display("=== Starting RISC-V CPU Testbench ===");

    // Initial values
    rst_n = 0;
    fetch_enable_i = 0;
    irq_i = 32'd0;

    // Hold reset
    #20;
    rst_n = 1;

    // Wait a bit, then enable instruction fetch
    #10;
    fetch_enable_i = 1;

    // Simulate a basic interrupt signal toggle (optional)
    #100;
    irq_i = 32'h00000001;
    #20;
    irq_i = 32'h00000000;

    // Let it run for a while
    #200;

    $display("=== Simulation complete ===");
    $finish;
  end

endmodule
