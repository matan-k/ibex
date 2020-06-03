module trigger_clock_enable 
#(
 parameter int unsigned DELAY = 128,
 parameter logic TRIGGER_EDGE = 1'b1,
 parameter logic RELEASE_EDGE = 1'b1
)
(
	input logic clk,    // Clock
	input logic rst_n,  // Asynchronous reset active low
	input logic trigger_i,
	input logic release_i,

	output logic clock_enable
);

logic [31:0] delay_counter; // Delays the CE by DELAY clocks 

typedef enum logic [1:0]{
    CE_HIGH = 2'b00,
    DELAYED_CE =2'b01,
    WAIT_FOR_RELEASE = 2'b10
  } ce_fsm_state;
ce_fsm_state current_state;

always_ff @(posedge clk or negedge rst_n) begin 
	if(~rst_n) begin
		current_state <= CE_HIGH;
		clock_enable <= 1'b1;
		delay_counter <= '0;
	end else begin
		case (current_state)
		 	CE_HIGH : begin
		 		clock_enable <= 1'b1;
		 		if(trigger_i == TRIGGER_EDGE) begin
		 			current_state <= DELAYED_CE;
		 			delay_counter <= DELAY-1;
		 		end
		 	end
		 	DELAYED_CE : begin
		 		if(delay_counter > 0) begin
		 			clock_enable  <= 1'b1;
		 			delay_counter <= delay_counter - 1;
		 		end else begin
		 			clock_enable  <= 1'b0;
		 			current_state <= WAIT_FOR_RELEASE;
		 		end
		 	end
		 	WAIT_FOR_RELEASE : begin
		 		if(release_i == RELEASE_EDGE) begin
		 			clock_enable  <= 1'b1;
		 			current_state <= CE_HIGH;
		 		end
		 	end
		 endcase;
	end
end

endmodule