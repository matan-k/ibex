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
  parameter logic [31:0] RAM_OFFSET = 'h0000C000;

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

  // ROM arbiter
  logic [31:0] instr_mem_addr;
  logic        instr_mem_req;
  logic        instr_mem_write;
  logic  [3:0] instr_mem_be;
  logic [31:0] instr_mem_wdata;
  logic        instr_mem_rvalid;
  logic [31:0] instr_mem_rdata;
  logic        instr_mem_read;

  // RAM arbiter
  logic [31:0] data_mem_addr;
  logic        data_mem_req;
  logic        data_mem_write;
  logic  [3:0] data_mem_be;
  logic [31:0] data_mem_wdata;
  logic        data_mem_rvalid;
  logic [31:0] data_mem_rdata;
  logic        data_mem_read;

  // LED events
  logic led_event;


  ibex_core #(
     .DmHaltAddr(32'h00000000),
     .DmExceptionAddr(32'h00000000)
  ) u_core (
     .clk_i                 (IO_CLK),
     .rst_ni                (IO_RST_N),

     .test_en_i             ('b0),

     .hart_id_i             (32'b0),
     // First instruction executed is at 0x0 + 0x80
     .boot_addr_i           (32'h00000000),

     .instr_req_o           (instr_req),
     .instr_gnt_i           (instr_gnt),
     .instr_rvalid_i        (instr_rvalid),
     .instr_addr_o          (instr_addr),
     .instr_rdata_i         (instr_rdata),
     .instr_err_i           ('b0),

     .data_req_o            (data_req),
     .data_gnt_i            (data_gnt),
     .data_rvalid_i         (data_rvalid),
     .data_we_o             (data_we),
     .data_be_o             (data_be),
     .data_addr_o           (data_addr),
     .data_wdata_o          (data_wdata),
     .data_rdata_i          (data_rdata),
     .data_err_i            ('b0),

     .irq_software_i        (1'b0),
     .irq_timer_i           (1'b0),
     .irq_external_i        (1'b0),
     .irq_fast_i            (15'b0),
     .irq_nm_i              (1'b0),

     .debug_req_i           ('b0),

     .fetch_enable_i        ('b1),
     .core_sleep_o          ()
  );

  //Connect Ibex to SRAM
  always_comb begin
    instr_mem_req        = 1'b0;
    instr_mem_addr       = 32'b0;
    instr_mem_write      = 1'b0;
    instr_mem_be         = 4'b0;
    instr_mem_wdata      = 32'b0;
    data_mem_req        = 1'b0;
    data_mem_addr       = 32'b0;
    data_mem_write      = 1'b0;
    data_mem_be         = 4'b0;
    data_mem_wdata      = 32'b0;
    led_event           = 1'b0;

    // ROM assignments
    if (instr_req) begin
      instr_mem_req        = (instr_addr & ~MEM_MASK) == MEM_START;
      instr_mem_addr       = instr_addr / 4;
    end 

    // RAM assignments
    if (data_req) begin
      data_mem_req        = (data_addr & ~MEM_MASK) == MEM_START;
      data_mem_write      = data_we;
      data_mem_be         = data_be;
      data_mem_addr       = (data_addr - RAM_OFFSET ) / 4; // translate absolute address to relative address
      data_mem_wdata      = data_wdata;
      led_event           = data_addr == RAM_OFFSET;
    end
  end

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

  jtag_bridge memories_and_jtag (
    .clk_clk           (IO_CLK),           //    clk.clk
    .reset_reset_n     (IO_RST_N),     //  reset.reset_n
    .ram_if_address    (data_mem_addr),    // ram_if.address
    .ram_if_chipselect (data_mem_req), //       .chipselect
    .ram_if_clken      ('1),      //       .clken
    .ram_if_write      (data_mem_write),      //       .write
    .ram_if_readdata   (data_mem_rdata),   //       .readdata
    .ram_if_writedata  (data_mem_wdata),  //       .writedata
    .ram_if_byteenable (data_mem_be), //       .byteenable
    .rom_if_address    (instr_mem_addr),    // rom_if.address
    .rom_if_chipselect (instr_mem_req), //       .chipselect
    .rom_if_clken      ('1),      //       .clken
    .rom_if_write      (instr_mem_write),      //       .write
    .rom_if_readdata   (instr_mem_rdata),   //       .readdata
    .rom_if_writedata  (instr_mem_wdata),  //       .writedata
    .rom_if_byteenable (instr_mem_be)  //       .byteenable
  );

  //SRAM to Ibex
  assign instr_rdata    = instr_mem_rdata;
  assign data_rdata     = data_mem_rdata;
  always_ff @(posedge IO_CLK or negedge IO_RST_N) begin
    if (!IO_RST_N) begin
      instr_gnt       <= 'b0;
      instr_rvalid    <= 'b0;
      data_gnt        <= 'b0;
      data_rvalid     <= 'b0;
    end else begin
      instr_gnt       <= instr_req && instr_mem_req;
      instr_rvalid    <= instr_req && instr_mem_req;
      data_gnt        <= data_req && data_mem_req;
      data_rvalid     <= data_req && data_mem_req;
    end
  end

  //Connect the LED output to the lower four bits of the most significant
  //byte
  logic [3:0] leds;

  always_ff @(posedge IO_CLK or negedge IO_RST_N) begin
    if (!IO_RST_N) begin
      leds <= 4'b0;
    end else begin
      if (led_event && data_req && data_we) begin
        for (int i = 0; i < 4; i = i + 1) begin
          if (data_be[i] == 1'b1) begin
            leds <= data_wdata[i*8 +: 4];
          end
        end
      end
    end
  end
  assign LED_N = leds;

  // Clock and reset
  // clkgen_xil7series
  //   clkgen(
  //     .IO_CLK,
  //     .IO_RST_N,
  //     .IO_CLK,
  //     .IO_RST_N
  //   );

endmodule
