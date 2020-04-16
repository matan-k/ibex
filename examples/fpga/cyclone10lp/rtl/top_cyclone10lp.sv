// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module top_cyclone10lp (
    input               IO_CLK,
    input               IO_RST_N,
    output [3:0]        LED_N
);

  parameter int          MEM_SIZE  = 64 * 1024; // 64 kB
  parameter logic [31:0] MEM_START = 32'h00000000;
  parameter logic [31:0] MEM_MASK  = MEM_SIZE-1;

  // logic IO_CLK, IO_RST_N;

  // Instruction connection to SRAM
  logic        instr_req;
  logic        instr_gnt;
  logic        instr_rvalid;
  logic [31:0] instr_addr;
  logic [31:0] instr_rdata;

  // Data connection to SRAM
  logic        data_req;
  logic        data_gnt;
  logic        data_rvalid;
  logic        data_we;
  logic  [3:0] data_be;
  logic [31:0] data_addr;
  logic [31:0] data_wdata;
  logic [31:0] data_rdata;

  // SRAM arbiter
  logic [31:0] mem_addr;
  logic        mem_req;
  logic        mem_write;
  logic  [3:0] mem_be;
  logic [31:0] mem_wdata;
  logic        mem_rvalid;
  logic [31:0] mem_rdata;
  logic        mem_read;



  // ibex_core #(
  //    .DmHaltAddr(32'h00000000),
  //    .DmExceptionAddr(32'h00000000)
  // ) u_core (
  //    .clk_i                 (IO_CLK),
  //    .rst_ni                (IO_RST_N),

  //    .test_en_i             ('b0),

  //    .hart_id_i             (32'b0),
  //    // First instruction executed is at 0x0 + 0x80
  //    .boot_addr_i           (32'h00000000),

  //    .instr_req_o           (instr_req),
  //    .instr_gnt_i           (instr_gnt),
  //    .instr_rvalid_i        (instr_rvalid),
  //    .instr_addr_o          (instr_addr),
  //    .instr_rdata_i         (instr_rdata),
  //    .instr_err_i           ('b0),

  //    .data_req_o            (data_req),
  //    .data_gnt_i            (data_gnt),
  //    .data_rvalid_i         (data_rvalid),
  //    .data_we_o             (data_we),
  //    .data_be_o             (data_be),
  //    .data_addr_o           (data_addr),
  //    .data_wdata_o          (data_wdata),
  //    .data_rdata_i          (data_rdata),
  //    .data_err_i            ('b0),

  //    .irq_software_i        (1'b0),
  //    .irq_timer_i           (1'b0),
  //    .irq_external_i        (1'b0),
  //    .irq_fast_i            (15'b0),
  //    .irq_nm_i              (1'b0),

  //    .debug_req_i           ('b0),

  //    .fetch_enable_i        ('b1),
  //    .core_sleep_o          ()
  // );

  // Connect Ibex to SRAM
  // always_comb begin
  //   mem_req        = 1'b0;
  //   mem_addr       = 32'b0;
  //   mem_write      = 1'b0;
  //   mem_be         = 4'b0;
  //   mem_wdata      = 32'b0;
  //   if (instr_req) begin
  //     mem_req        = (instr_addr & ~MEM_MASK) == MEM_START;
  //     mem_addr       = instr_addr;
  //   end else if (data_req) begin
  //     mem_req        = (data_addr & ~MEM_MASK) == MEM_START;
  //     mem_write      = data_we;
  //     mem_be         = data_be;
  //     mem_addr       = data_addr;
  //     mem_wdata      = data_wdata;
  //   end
  // end

  // temp
  // always_comb begin
    // mem_req        = 1'b0;
    // mem_addr       = 32'b0;
    // mem_write      = 1'b0;
    // mem_be         = 4'b0;
    // mem_wdata      = 32'b0;
    // // if (instr_req) begin
    //   mem_req        = (instr_addr & ~MEM_MASK) == MEM_START;
    //   mem_addr       = instr_addr;
    // // end else if (data_req) begin
    // //   mem_req        = (data_addr & ~MEM_MASK) == MEM_START;
    // //   mem_write      = data_we;
    // //   mem_be         = data_be;
    //   mem_addr       = data_addr;
    //   mem_wdata      = data_wdata;
    // end
  // end


  // SRAM block for instruction and data storage
  /*ram_1p #(
    .Depth(MEM_SIZE / 4)
  ) u_ram (
    .clk_i     ( IO_CLK        ),
    .rst_ni    ( IO_RST_N      ),
    .req_i     ( mem_req        ),
    .we_i      ( mem_write      ),
    .be_i      ( mem_be         ),
    .addr_i    ( mem_addr       ),
    .wdata_i   ( mem_wdata      ),
    .rvalid_o  ( mem_rvalid     ),
    .rdata_o   ( mem_rdata      )
  );*/

  jtag_bridge jtag_inst (
    .clk_clk                       (IO_CLK),                       //             clk.clk
    .reset_reset_n                 (IO_RST_N)
  );
  
 //  on_chip_mem mem_inst (
	// 	.clk_clk           (IO_CLK),           //    clk.clk
	// 	.reset_reset_n     (IO_RST_N),     //  reset.reset_n
	// 	.mem_if_address    (mem_addr),    // mem_if.address
	// 	.mem_if_clken      ('1),      //       .clken
	// 	.mem_if_chipselect ('1), //       .chipselect
	// 	.mem_if_write      (mem_write),      //       .write
	// 	.mem_if_readdata   (mem_rdata),   //       .readdata
	// 	.mem_if_writedata  (mem_wdata),  //       .writedata
	// 	.mem_if_byteenable (mem_be)  //       .byteenable
	// );

  // SRAM to Ibex
  // assign instr_rdata    = mem_rdata;
  // assign data_rdata     = mem_rdata;
  // assign instr_rvalid   = mem_rvalid;
  // always_ff @(posedge IO_CLK or negedge IO_RST_N) begin
  //   if (!IO_RST_N) begin
  //     instr_gnt    <= 'b0;
  //     data_gnt     <= 'b0;
  //     data_rvalid  <= 'b0;
  //     mem_rvalid   <= 'b0;
  //   end else begin
  //     instr_gnt    <= instr_req && mem_req;
  //     data_gnt     <= ~instr_req && data_req && mem_req;
  //     data_rvalid  <= ~instr_req && data_req && mem_req;
  //     mem_rvalid   <= instr_req && mem_req;
  //   end
  // end

  always_ff @(posedge IO_CLK or negedge IO_RST_N) begin
    if(~IO_RST_N) begin
      mem_rvalid <= 0;
    end else begin
      mem_rvalid <= mem_read;
    end
  end

  // Connect the LED output to the lower four bits of the most significant
  // byte
  // logic [3:0] leds;
  // logic [3:0] mem_value;
  // logic read;
  // always_ff @(posedge IO_CLK or negedge IO_RST_N) begin
  //   if (!IO_RST_N) begin
  //     leds <= 4'b0;
  //     read <= 1'b0;
  //   end else begin
  //     if(mem_rvalid == 1'b1 && !read) begin
  //      // leds <= mem_rdata[8:5];
  //      read <= 1'b1;
  //     end

  //     if (mem_req && data_req && data_we) begin
  //       for (int i = 0; i < 4; i = i + 1) begin
  //         if (data_be[i] == 1'b1) begin
  //           leds <= data_wdata[i*8 +: 4];
  //         end
  //       end
  //     end
  //   end
  // end
  assign LED_N[1:0] = '1;
  assign LED_N[3:2] = '0;

  // Clock and reset
  // clkgen_xil7series
  //   clkgen(
  //     .IO_CLK,
  //     .IO_RST_N,
  //     .IO_CLK,
  //     .IO_RST_N
  //   );

endmodule
