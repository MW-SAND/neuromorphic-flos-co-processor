module neoRV32_wrapper_acc #(
	parameter CLOCK_FREQUENCY = 500000000,
	parameter MEM_INT_IMEM_EN = 1'b1,     		// boolean, represented as 1'b0 for false
  parameter MEM_INT_IMEM_SIZE = 1024 * 32, 		// size in bytes
  parameter MEM_INT_DMEM_EN = 1'b1,     		// boolean, represented as 1'b0 for false
  parameter MEM_INT_DMEM_SIZE = 1024 * 512, 		// size in bytes
  parameter CFS_EN = 1'b1
)
(
	input clk_i,
	input rstn_i,
	input mtime_irq_i,
	input msw_irq_i,
	input mext_irq_i
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
    .IO_UART0_EN(1'b1),  
    .IO_UART0_RX_FIFO(32),  
    .IO_UART0_TX_FIFO(32),  
    // 
    .RISCV_ISA_C(1'b0), 
    .RISCV_ISA_M(1'b1),
    // HPM
    .RISCV_ISA_Zicntr(1'b1),
    .RISCV_ISA_Zihpm(1'b1),
    .HPM_NUM_CNTS(13),
    .HPM_CNT_WIDTH(40),
    //
    .IO_CFS_IN_SIZE(0),
    .IO_CFS_OUT_SIZE(0),
    .IO_CFS_EN(CFS_EN),
    .IO_CFS_CONFIG(32'b00000000000000000000000000001000), // 8

    .CPU_FAST_MUL_EN(1'b1),
    .CPU_FAST_SHIFT_EN(1'b1)
  ) u_neorv32_top_acc (
    .clk_i(clk_i),
    .rstn_i(rstn_i),
    .mtime_irq_i(mtime_irq_i),
    .msw_irq_i(msw_irq_i),
    .mext_irq_i(mext_irq_i)
  );

  // Additional logic, clock generation, etc., can be added here

endmodule
