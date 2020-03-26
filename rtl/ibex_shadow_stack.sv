module ibex_shadow_stack #(
	parameter bit RV32E = 1'b0
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

logic [4:0]  stack_top_addr;
logic [4:0]  rf_waddr_wb;
logic [4:0]  rf_raddr_a;
logic [31:0] rf_wdata_wb;
logic [31:0] rf_rdata_a;
logic 		 rf_we_wb;


// Manages the stack top pointer
always_ff @(posedge clk_i or negedge rst_ni) begin
	if(~rst_ni) begin
		stack_top_addr <= 0;
	end else begin
		if(write_indication_i & stack_top_addr != '1) begin
			stack_top_addr <= stack_top_addr + 1;
		end else if(read_indication_i & stack_top_addr != 0) begin
			stack_top_addr <= stack_top_addr - 1;
		end
	end
end

// Manages read and writes from the internal register file.
always_comb begin
	if(write_indication_i) begin

		// Check if the stack is full
		if(stack_top_addr == '1) begin
			error_o = 1'b1;
		end else begin
			rf_we_wb = 1'b1;
			rf_wdata_wb = pointer_wr_i;
			rf_waddr_wb = stack_top_addr + 1;
		end

	end else if(read_indication_i) begin

		//	Check if the stack is empty
		if(stack_top_addr == 0) begin
			error_o = 1;
		end else begin
			rf_raddr_a = stack_top_addr;
			if(rf_rdata_a != pointer_rd_i) begin
				error_o = 1'b1;
			end
		end
	end else begin
		error_o     = 1'b0;
		rf_raddr_a  = '0;
		rf_we_wb    = 1'b0;
		rf_wdata_wb = '0;
		rf_waddr_wb = '0;
	end
end


ibex_register_file #(
      .RV32E(RV32E),
      .DataWidth(32)
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