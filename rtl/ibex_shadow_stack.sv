module ibex_shadow_stack #(
	parameter bit RV32E = 1'b0,
	parameter int unsigned ADDR_WIDTH = 8
)
(
	input clk_i,    // Clock
	input rst_ni,  // Asynchronous reset active low

	input logic [31:0] pointer_wr_i, // pointer to store in the stack
	input logic [31:0] pointer_rd_i, // pointer to validate in the stack and remove
	input logic write_indication_i,  // valid for pointer_wr_i 
	input logic read_indication_i,    // valid for pointer_rd_i

	output error_o
);

import ibex_pkg::*;


logic [ADDR_WIDTH - 1:0]  stack_top_addr;
logic [ADDR_WIDTH - 1:0]  rf_waddr_wb;
logic [ADDR_WIDTH - 1:0]  rf_raddr_a;
logic [31:0] rf_wdata_wb;
logic [31:0] rf_rdata_a;
logic 		 rf_we_wb;
logic        error_int;


// Manages the stack top pointer and error
always_ff @(posedge clk_i or negedge rst_ni) begin
	if(~rst_ni) begin
		stack_top_addr <= 0;
		error_o <= 1'b0;
	end else begin
		error_o <= error_int;
		if(write_indication_i & stack_top_addr != '1) begin
			stack_top_addr <= stack_top_addr + 1;
		end else if(read_indication_i & stack_top_addr != 0) begin
			stack_top_addr <= stack_top_addr - 1;
		end
	end
end

// Manages read and writes from the internal register file.
assign rf_raddr_a = stack_top_addr;
assign rf_wdata_wb = pointer_wr_i;
assign rf_waddr_wb = stack_top_addr + 1;


assign rf_we_wb = write_indication_i & stack_top_addr != '1;
assign error_int = (write_indication_i & stack_top_addr == '1 ) | // error when the stack is full
		(read_indication_i & (stack_top_addr == 0 |  // error when the stack is empty
			(rf_rdata_a != pointer_rd_i))); // mismatch between expected address and actual one

// always_comb begin	
	// if(write_indication_i) begin
// 
		// Check if the stack is full
		// if(stack_top_addr == '1) begin
			// error_o = 1'b1;
			// rf_we_wb = 1'b0;
		// end else begin
			// rf_we_wb = 1'b1;
			// error_o = 1'b0;
		// end
// 
	// end else if(read_indication_i) begin
		// rf_we_wb = 1'b0;
// 
			// Check if the stack is empty
		// if(stack_top_addr == 0) begin
			// error_o = 1'b1;
		// end else begin
			// if(rf_rdata_a != pointer_rd_i) begin
				// error_o = 1'b1;
			// end
		// end
	// end else begin
		// error_o     = 1'b0;
		// rf_we_wb    = 1'b0;
	// end
// end


ibex_register_file #(
      .DataWidth(32),
      .AddrWidth(8)
  ) register_file_i (
      .clk_i        ( clk_i        ),
      .rst_ni       ( rst_ni       ),

      .test_en_i    ( 1'b0    ),

      // Read port a
      .raddr_a_i    ( rf_raddr_a   ),
      .rdata_a_o    ( rf_rdata_a   ),
      // Read port b
      .raddr_b_i    ( '0   ),
      .rdata_b_o    (),
      // write port
      .waddr_a_i    ( rf_waddr_wb  ),
      .wdata_a_i    ( rf_wdata_wb  ),
      .we_a_i       ( rf_we_wb     )
  );


endmodule