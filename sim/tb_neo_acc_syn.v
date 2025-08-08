module tb_neo_acc_syn;

reg clk_i;
reg rstn_i;
reg mtime_irq_i = 1'b0; 	//time interrupt
reg msw_irq_i = 1'b0;		//software interrupt
reg mext_irq_i = 1'b0;		//external interrupt

reg[31:0] gpio_i = 32'b0;
wire[31:0] gpio_o;

wire[31:0] xbus_adr_o, xbus_dat_o;
reg[31:0] xbus_dat_i = 32'b0;
wire[2:0] xbus_tag_o;
wire[3:0] xbus_sel_o;
wire xbus_we_o, xbus_stb_o, xbus_cyc_o;
reg xbus_ack_i, xbus_err_i = 1'b0;

neoRV32_wrapper_acc_syn #( 
	.CLOCK_FREQUENCY(500000000)
) 
uut
(
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
  .xbus_err_i   (xbus_err_i)
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

