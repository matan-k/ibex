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
  parameter logic [31:0] RAM_OFFSET = 'h0000D000;
  parameter logic [31:0] RAM_MASK  = 'h0000D000;

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
  logic [31:0] flash_addr;
  logic        instr_flash_req;
  logic        data_flash_req;
  logic        flash_write;
  logic  [3:0] flash_be;
  logic [31:0] flash_wdata;
  logic        flash_rvalid;
  logic [31:0] flash_rdata;
  logic        flash_read;
  logic        flash_wait;
  logic [31:0] flash_rdata_endianity_fix;
  logic [31:0] flash_addr_endianity_fix;

  // Sampled ROM arbiter (for ROM on external Flash)
  logic [31:0] flash_addr_s;
  logic        flash_write_s;
  logic  [3:0] flash_be_s;
  logic [31:0] flash_wdata_s;
  logic        flash_rvalid_s;
  logic [31:0] flash_rdata_s;
  logic        flash_read_s;
  logic        flash_wait_s; 
  logic        during_data_req;

  // RAM arbiter
  logic [31:0] ram_addr;
  logic        ram_req;
  logic        ram_write;
  logic  [3:0] ram_be;
  logic [31:0] ram_wdata;
  logic        ram_rvalid;
  logic [31:0] ram_rdata;
  logic        ram_read;

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

  // Sample the interface for timing closure
  always_ff @(posedge IO_CLK or negedge IO_RST_N) begin
    if(~IO_RST_N) begin
      flash_addr_s <= '0;
      flash_write_s <= '0;
      flash_be_s <= '0;
      flash_wdata_s <= '0;
      flash_rvalid_s <= '0;
      flash_rdata_s <= '0;
      flash_read_s <= '0;
    end else begin
      // if (!flash_wait) begin
        flash_addr_s <= flash_addr;
        flash_write_s <= flash_write;
        flash_be_s <= flash_be;
        flash_wdata_s <= flash_wdata;
        flash_rvalid_s <= flash_rvalid;
        flash_rdata_s <= flash_rdata_endianity_fix;
        // flash_rdata_s <= flash_rdata;
        flash_read_s <= flash_read;
      // end
    end
  end

  genvar i;
  generate
    for (i = 0; i < 4; i++) begin : endianity_fix
      assign flash_rdata_endianity_fix[(i+1)*8-1: i*8] = flash_rdata[(4-i)*8-1 : (3-i)*8];
      assign flash_addr_endianity_fix[(i+1)*8-1: i*8] = flash_addr[(4-i)*8-1 : (3-i)*8];
    end
  endgenerate

  //Connect Ibex to SRAM
  always_comb begin
    instr_flash_req  = 1'b0;
    data_flash_req   = 1'b0;
    flash_addr       = 32'b0;
    flash_write      = 1'b0;
    flash_read       = 1'b0;
    flash_be         = 4'b0;
    flash_wdata      = 32'b0;
    ram_req          = 1'b0;
    ram_addr         = 32'b0;
    ram_write        = 1'b0;
    ram_be           = 4'b0;
    ram_wdata        = 32'b0;
    led_event        = 1'b0;

    // assign flash_read = flash_req;

    // ROM assignments
    if (instr_req) begin
      // On-chip ROM ------
      // flash_req        = (instr_addr & ~MEM_MASK) == MEM_START;
      // flash_addr       = instr_addr / 4;
      // ------

      // Flash ROM -----
      instr_flash_req = (instr_addr & ~MEM_MASK) == MEM_START;
      flash_addr      = instr_addr / 4;
      flash_read      = instr_flash_req;
      // -----
    end else if (data_req) begin
      // Flash
      // data_flash_req = data_addr < RAM_OFFSET;
      // flash_addr     = data_addr / 4;
      // flash_read     = data_flash_req;

      // RAM
      ram_req        = (data_addr & ~MEM_MASK) == MEM_START;
      ram_write      = data_we;
      ram_be         = data_be;
      ram_addr       = (data_addr - RAM_OFFSET ) / 4; // translate absolute address to relative address
      ram_wdata      = data_wdata;
      led_event      = data_addr == RAM_OFFSET;
    end
  end

  // On-chip ROM -----
  // jtag_bridge memories_and_jtag (
  //   .clk_clk           (IO_CLK),           //    clk.clk
  //   .reset_reset_n     (IO_RST_N),     //  reset.reset_n

  //   //ROM
  //   .rom_if_address    (flash_addr),    // rom_if.address
  //   .rom_if_chipselect (flash_req), //       .chipselect
  //   .rom_if_clken      ('1),      //       .clken
  //   .rom_if_write      (flash_write),      //       .write
  //   .rom_if_readdata   (flash_rdata),   //       .readdata
  //   .rom_if_writedata  (flash_wdata),  //       .writedata
  //   .rom_if_byteenable (flash_be)  //       .byteenable

  //   //RAM
  //   .ram_if_address    (ram_addr),    // ram_if.address
  //   .ram_if_chipselect (ram_req), //       .chipselect
  //   .ram_if_clken      ('1),      //       .clken
  //   .ram_if_write      (ram_write),      //       .write
  //   .ram_if_readdata   (ram_rdata),   //       .readdata
  //   .ram_if_writedata  (ram_wdata),  //       .writedata
  //   .ram_if_byteenable (ram_be), //       .byteenable
  // );
  // ------


  // ROM on external flash -----
  epcq_and_ram u0 (
    .clk_clk              (IO_CLK),              //      clk.clk
    .reset_reset_n        (IO_RST_N),        //    reset.reset_n

    // ROM
    .rom_if_write         (flash_write_s),         //   rom_if.write
    .rom_if_burstcount    (1'b1),    //         .burstcount
    .rom_if_waitrequest   (flash_wait),   //         .waitrequest
    .rom_if_read          (flash_read_s),          //         .read
    .rom_if_address       (flash_addr_s),       //         .address
    .rom_if_writedata     (flash_wdata_s),     //         .writedata
    .rom_if_readdata      (flash_rdata),      //         .readdata
    .rom_if_readdatavalid (flash_rvalid), //         .readdatavalid
    .rom_if_byteenable    (flash_be_s),    //         .byteenable
    
    // RAM
    .ram_if_address       (ram_addr),       //   ram_if.address
    .ram_if_chipselect    (ram_req),    //         .chipselect
    .ram_if_clken         ('1),         //         .clken
    .ram_if_write         (ram_write),         //         .write
    .ram_if_readdata      (ram_rdata),      //         .readdata
    .ram_if_writedata     (ram_wdata),     //         .writedata
    .ram_if_byteenable    (ram_be),    //         .byteenable

    .ctrl_irq_irq         ()          // ctrl_irq.irq
  );
  // -----

  //SRAM to Ibex
  assign instr_rdata    = flash_rdata_s;
  assign data_rdata     = ram_rvalid ? ram_rdata : flash_rdata_s;

  // External flash ROM ----
  assign instr_rvalid   = (~during_data_req) && flash_rvalid_s;
  assign data_rvalid    = (flash_rvalid_s && during_data_req) | ram_rvalid;

  // -----

  always_ff @(posedge IO_CLK or negedge IO_RST_N) begin
    if (!IO_RST_N) begin
      instr_gnt       <= 'b0;
      data_gnt        <= 'b0;
      ram_rvalid      <= 'b0;
      during_data_req <= 'b0;
      // On chip ROM -----
      // instr_rvalid    <= 'b0;
      // -----
    end else begin
      // On-chip ROM -----
      // instr_gnt       <= instr_req && flash_req;
      // instr_rvalid    <= instr_req && flash_req;
      // ---

      instr_gnt       <= instr_flash_req && ~flash_wait;
      data_gnt        <= (ram_req) | (data_flash_req && ~flash_wait);
      ram_rvalid      <= ram_req;

      if(data_flash_req && ~flash_wait) begin
        during_data_req <= 1'b1;
      end else if(flash_rvalid_s) begin
        during_data_req <= 1'b0;
      end
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
  assign LED_N[3:0] = leds[3:0];

endmodule
