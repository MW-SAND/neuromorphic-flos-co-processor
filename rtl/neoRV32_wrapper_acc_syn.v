module neoRV32_wrapper_acc_syn #(
  parameter CLOCK_FREQUENCY = 500000000,
  parameter MEM_INT_IMEM_EN = 1'b1,     		// boolean, represented as 1'b0 for false
  parameter MEM_INT_IMEM_SIZE = 1024 * 32, 		// size in bytes
  parameter MEM_INT_DMEM_EN = 1'b1,     		// boolean, represented as 1'b0 for false
  parameter MEM_INT_DMEM_SIZE = 1024 * 256, 		// size in bytes
  parameter CFS_EN = 1'b1
)
(
	input clk_i,
	input rstn_i,
	input mtime_irq_i,
	input msw_irq_i,
	input mext_irq_i,
 // GPIO
  output wire [31:0]  gpio_o,   // parallel output
  input  wire [31:0]  gpio_i,   // parallel input

  // XBUS interface
  output wire [31:0]  xbus_adr_o, // address
  output wire [31:0]  xbus_dat_o, // write data
  output wire [2:0]   xbus_tag_o, // access tag
  output wire         xbus_we_o,  // read/write
  output wire [3:0]   xbus_sel_o, // byte enable
  output wire         xbus_stb_o, // strobe
  output wire         xbus_cyc_o, // valid cycle
  input  wire [31:0]  xbus_dat_i, // read data
  input  wire         xbus_ack_i, // transfer acknowledge
  input  wire         xbus_err_i // transfer error
); 

  // Instantiate the neorv32_top module
  neorv32_top_acc #(
    .CLOCK_FREQUENCY(CLOCK_FREQUENCY),
    .BOOT_MODE_SELECT(2), 
    // 
    .MEM_INT_IMEM_EN(MEM_INT_IMEM_EN),
    .MEM_INT_IMEM_SIZE(MEM_INT_IMEM_SIZE),
    // 
    .MEM_INT_DMEM_EN(MEM_INT_DMEM_EN),
    .MEM_INT_DMEM_SIZE(MEM_INT_DMEM_SIZE),
  
    // 
    .RISCV_ISA_C(1'b0), 
    .RISCV_ISA_M(1'b1),
    .CPU_FAST_SHIFT_EN(1'b1),
    .CPU_FAST_MUL_EN(1'b1),
    //
    .IO_CFS_IN_SIZE(1),
    .IO_CFS_OUT_SIZE(1),
    .IO_CFS_EN(CFS_EN),
    .IO_CFS_CONFIG(32'b00000000000000000000000000001000) // 8
  ) u_neorv32_top_acc (
    .clk_i(clk_i),
    .rstn_i(rstn_i),
    .mtime_irq_i(mtime_irq_i),
    .msw_irq_i(msw_irq_i),
    .mext_irq_i(mext_irq_i),
    // GPIO
    .gpio_o       (gpio_o),
    .gpio_i       (gpio_i),
    // External bus interface
    .xbus_adr_o   (xbus_adr_o),
    .xbus_dat_o   (xbus_dat_o),
    .xbus_tag_o   (xbus_tag_o),
    .xbus_we_o    (xbus_we_o),
    .xbus_sel_o   (xbus_sel_o),
    .xbus_stb_o   (xbus_stb_o),
    .xbus_cyc_o   (xbus_cyc_o),
    .xbus_dat_i   (xbus_dat_i),
    .xbus_ack_i   (xbus_ack_i),
    .xbus_err_i   (xbus_err_i),
	.cfs_in_i(1'b0)
  );

  // Additional logic, clock generation, etc., can be added here

endmodule
