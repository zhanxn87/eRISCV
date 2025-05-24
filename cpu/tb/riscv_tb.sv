`timescale 1ns/1ps

module riscv_tb;

  // Parameters
  localparam CLK_PERIOD = 10; // 100 MHz

  // DUT signals
  logic clk;
  logic rst_n;
  logic fetch_enable_i;
  logic [31:0] irq_i;

  logic ecall_reached;

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

  assign ecall_reached = dut.riscv_core_i.pc_set==1'b1 && dut.riscv_core_i.pc_mux==3'b100 && dut.riscv_core_i.exc_pc_mux==3'b001;

  string tc_name;

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

    wait (ecall_reached); // Wait for ecall to be reached

    #40;

    // Call the task
    // Optional: Initialize memory with a file
    if (!$value$plusargs("tc=%s", tc_name)) begin
        $fatal("No tc specified!");
        tc_name = "I-ADD-01"; // Default tc_name
    end
    $display("TB using reference file: %s.reference_output", tc_name);
    compare_memory_with_reference({"./testcases/", tc_name, ".reference_output"});

    $display("=== Simulation completed ===");
    $stop;
  end

  // Task to compare memory contents with reference file
  task compare_memory_with_reference;
    input string ref_file;
    int file;
    int status;
    int i;
    logic [31:0] ref_data;
    logic [31:0] mem_data;

    begin
    file = $fopen(ref_file, "r");
    if (file == 0) begin
      $display("TB ERROR: Unable to open reference file: %s", ref_file);
      $stop;
    end

    // Compare memory contents
    i = 0;
    while (!$feof(file)) begin
      status = $fscanf(file, "%h\n", ref_data);
      if (status != 1) begin
        $display("TB ERROR: Failed to read data from reference file at line %0d", i + 1);
        $stop;
      end

      mem_data = dut.data_mem_i.mem[8'h80 + i];

      if (i < 32) begin
        $display("TB INFO: Address 0x%0h: Expected 0x%8h, Got 0x%8h", 8'h80 + i, ref_data, mem_data);
      end
      
      if (i<32 && mem_data !== ref_data) begin
        $display("TB ERROR: Mismatch at address 0x%0h: Expected 0x%0h, Got 0x%0h", 8'h80 + i, ref_data, mem_data);
        $stop;
      end

      i++;
    end

    $fclose(file);
    $display(">>>>>>>>>>PASS: Memory contents (0~%0d) match reference file.", ((i-1)>=32 ? 31 : (i-1)));
    end
  endtask

endmodule
