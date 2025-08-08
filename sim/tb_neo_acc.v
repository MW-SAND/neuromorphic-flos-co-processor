module tb_neo_acc;

reg clk_i;
reg rstn_i;
reg mtime_irq_i = 1'b0; 	//time interrupt
reg msw_irq_i = 1'b0;		//software interrupt
reg mext_irq_i = 1'b0;		//external interrupt

neoRV32_wrapper_acc #( 
	.CLOCK_FREQUENCY(500000000)
) 
uut
(
	.clk_i(clk_i),
	.rstn_i(rstn_i),
	.mtime_irq_i(mtime_irq_i),
	.msw_irq_i(msw_irq_i),
	.mext_irq_i(mext_irq_i)
);

initial begin
	clk_i = 1'b0;
	forever #(1) clk_i = ~clk_i;
end

initial
begin
	rstn_i = 1'b0;
	repeat (10) 
		@(posedge  clk_i);
	rstn_i = 1'b1;

	repeat (10000) 
		@(posedge  clk_i);
end

endmodule

