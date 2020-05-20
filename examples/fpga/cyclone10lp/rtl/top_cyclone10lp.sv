// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module top_cyclone10lp (
    input               IO_CLK,
    input               IO_RST_N,
    output [3:0]        LED_N
);

  parameter bit RV32E           = 1'b0;
  parameter bit RV32M           = 1'b1;
  parameter bit BranchTargetALU = 1'b0;
  parameter bit WritebackStage  = 1'b0;

  parameter int          ROM_SIZE  = 64 * 1024; // 64 kB
  parameter int          RAM_SIZE  = 8 * 1024; // 8kB
  parameter logic [31:0] MEM_START = 32'h00000000;
  parameter logic [31:0] MEM_MASK  = ~(ROM_SIZE - 1);
  parameter logic [31:0] RAM_OFFSET = 'h0000D000;
  parameter logic [31:0] RAM_MASK  = ~(RAM_SIZE - 1);

  // Instruction connection to SRAM
  logic        instr_req;
  logic        instr_gnt;
  logic        instr_rvalid;
  logic [31:0] instr_addr;
  logic [31:0] instr_rdata;

  logic        instr_rvalid_s;
  logic [31:0] instr_rdata_s;
  logic        instr_gnt_aux;
  logic        instr_gnt_aux_s; 

  // Data connection to SRAM
  logic        data_req;
  logic        data_gnt;
  logic        data_rvalid;
  logic        data_we;
  logic  [3:0] data_be;
  logic [31:0] data_addr;
  logic [31:0] data_wdata;
  logic [31:0] data_rdata;

  logic        data_rvalid_s;
  logic [31:0] data_rdata_s;

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
  logic [31:0] flash_wdata_endianity_fix;
  logic [31:0] instr_addr_s;
  logic        flash_req;
  logic        flash_data_gnt;

  // Sampled ROM arbiter (for ROM on external Flash)
  logic [31:0] flash_addr_s;
  logic        flash_write_s;
  logic  [3:0] flash_be_s;
  logic [31:0] flash_wdata_s;
  logic        flash_rvalid_s;
  logic [31:0] flash_rdata_s;
  logic        flash_read_s;
  logic        flash_wait_s; 
  logic        during_flash_data_read;
  logic        during_flash_data_write;
  logic        internal_ack;

  // RAM arbiter
  logic [31:0] ram_addr;
  logic        ram_req;
  logic        ram_write;
  logic  [3:0] ram_be;
  logic [31:0] ram_wdata;
  logic        ram_rvalid;
  logic [31:0] ram_rdata;
  logic        ram_read;
  logic [31:0] data_addr_s;
  logic [31:0] data_wdata_s;
  logic [3:0]  data_be_s;
  logic        data_we_s;

  // Instruction printer
  logic [31:0] print_addr;
  logic        print_req;
  logic        print_write;
  logic  [3:0] print_be;
  logic [31:0] print_wdata;
  logic        print_rvalid;
  logic [31:0] print_rdata;
  logic        print_read;
  logic [31:0] print_counter;

  // Regfile write data selection
  typedef enum logic [1:0]{
    IDLE = 2'b00,
    INSTR_READ = 2'b01,
    DATA_READ =2'b10,
    DATA_WRITE = 2'b11
  } flash_state;

  flash_state current_state;

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

  assign data_flash_req = data_req & (data_addr < RAM_OFFSET);
  assign ram_req = data_req & (data_addr >= RAM_OFFSET);

  always_ff @(posedge IO_CLK or negedge IO_RST_N) begin 
    if(~IO_RST_N) begin
      current_state <= IDLE;
      flash_addr <= '0;
      flash_write <= '0;
      flash_be <= '0;
      flash_wdata <= '0;
      flash_rvalid_s <= '0;
      flash_read <= '0;
      instr_rvalid <= '0;
      instr_rdata <= '0;
      flash_req <= 1'b0;
      flash_data_gnt <= 1'b0;
    end else begin
       case (current_state)

          IDLE: begin
            instr_rvalid <= 1'b0;
            instr_rdata <= '0;
            flash_rvalid_s <= 1'b0;

            if(instr_req && ~flash_wait) begin
              current_state <= INSTR_READ;
              flash_read <= 1'b1;
              flash_write <= 1'b0;
              flash_addr <= instr_addr / 4;
              flash_be <= '0;
              flash_wdata <= '0;
              flash_req <= 1'b1;
              instr_gnt <= 1'b1;
            end else if(data_flash_req && ~flash_wait) begin
              current_state <= (data_we) ? DATA_WRITE : DATA_READ;
              flash_read <= ~data_we;
              flash_write <= data_we;
              flash_addr <= data_addr / 4;
              flash_be <= data_be;
              flash_wdata <= flash_wdata_endianity_fix;
              flash_req <= 1'b1;
              flash_data_gnt <= 1'b1;
            end else begin
              flash_req <= 1'b0;
              instr_gnt <= 1'b0;
            end
          end

          INSTR_READ: begin
            flash_req <= 1'b0;
            instr_gnt <= 1'b0;
            flash_read <= 1'b0;
            flash_write <= 1'b0;
            if(flash_rvalid) begin
              current_state <= IDLE;
              instr_rvalid <= 1'b1;
              instr_rdata <= flash_rdata_endianity_fix;
            end
          end

          DATA_READ: begin
            flash_req <= 1'b0;
            flash_read <= 1'b0;
            flash_write <= 1'b0;
            flash_data_gnt <= 1'b0;
            if(flash_rvalid) begin
              current_state <= IDLE;
              flash_rvalid_s <= 1'b1;
              flash_rdata_s <= flash_rdata_endianity_fix;
            end

          end

          DATA_WRITE: begin
            flash_req <= 1'b0;
            flash_read <= 1'b0;
            flash_write <= 1'b0;
            flash_data_gnt <= 1'b0;
            if(~flash_wait && ~flash_req) begin
              flash_rvalid_s <= 1'b1;
              current_state <= IDLE;
            end
          end
       endcase
    end
  end

  assign data_rvalid = (ram_rvalid) ? ram_rvalid : flash_rvalid_s;
  assign data_rdata = (ram_rvalid) ? ram_rdata : flash_rdata_s;
  assign data_gnt = ram_req | flash_data_gnt;

`ifndef Simulation_f
  always_ff @(posedge IO_CLK or negedge IO_RST_N) begin 
    if(~IO_RST_N) begin
      ram_rvalid <= 0;
    end else begin
      ram_rvalid <= ram_req;
    end
  end
`endif
  

  // // Sample the interface for timing closure
  // always_ff @(posedge IO_CLK or negedge IO_RST_N) begin
  //   if(~IO_RST_N) begin
  //     flash_addr_s <= '0;
  //     flash_write_s <= '0;
  //     flash_be_s <= '0;
  //     flash_wdata_s <= '0;
  //     flash_rvalid_s <= '0;
  //     flash_rdata_s <= '0;
  //     flash_read_s <= '0;
  //     data_addr_s   <= '0;
  //     data_wdata_s <= '0;
  //     flash_wait_s  <='0;
  //     instr_addr_s <= '0;
  //     data_we_s <= '0;
  //     data_be_s <= '0;
  //   end else begin
  //     // if (!flash_wait) begin
  //       instr_addr_s <= instr_addr;
  //       data_addr_s <= data_addr;
  //       data_wdata_s <= data_wdata;
  //       data_we_s    <= data_we;
  //       data_be_s    <= data_be;

  //       if(instr_flash_req/* && instr_gnt*/) begin
  //         flash_addr_s       <= instr_addr_s / 4;
  //         flash_read_s       <= 1'b1;
  //         flash_write_s      <= '0;
  //         flash_wdata_s      <= '0;
  //         flash_be_s         <= '0;
  //       end else if(data_flash_req/* && data_gnt*/) begin
  //         flash_addr_s       <= data_addr_s / 4 /* + 1*/;
  //         flash_read_s       <= ~data_we /**/; // Used to partially work. Read gets a priority over write..
  //         flash_write_s      <= data_we;
  //         flash_wdata_s      <= data_wdata_s;
  //         flash_be_s         <= data_be;
  //       end else begin
  //         flash_addr_s       <= '0;
  //         flash_read_s       <= '0;
  //         flash_write_s      <= '0;
  //         flash_wdata_s      <= '0;
  //         flash_be_s         <= '0;
  //       end

  //       flash_rvalid_s <= flash_rvalid;
  //       flash_rdata_s <= flash_rdata_endianity_fix;
  //       // flash_rdata_s <= flash_rdata;

  //       flash_wait_s <= flash_wait;
  //     // end
  //   end
  // end

  genvar i;
  generate
    for (i = 0; i < 4; i++) begin: endianity_fix 
      assign flash_rdata_endianity_fix[(i+1)*8-1: i*8] = flash_rdata[(4-i)*8-1 : (3-i)*8];
      assign flash_wdata_endianity_fix[(i+1)*8-1: i*8] = data_wdata[(4-i)*8-1 : (3-i)*8];
    end
  endgenerate

  //Connect Ibex to SRAM
  // always_comb begin
  //   instr_flash_req  = 1'b0;
  //   data_flash_req   = 1'b0;
  //   // instr_addr_s       = 32'b0;
  //   flash_write      = 1'b0;
  //   flash_read       = 1'b0;
  //   flash_be         = 4'b0;
  //   flash_wdata      = 32'b0;
  //   ram_req          = 1'b0;
  //   ram_addr         = 32'b0;
  //   ram_write        = 1'b0;
  //   ram_be           = 4'b0;
  //   ram_wdata        = 32'b0;
  //   led_event        = 1'b0;
  //   print_req        = '0;
  //   print_addr       = '0;
  //   print_write      = '0;
  //   print_wdata      = '0;

  //  // ROM assignments. Data first.
    
  //  if (instr_req) begin
  //    instr_flash_req = (instr_addr & MEM_MASK) == MEM_START;

  //    // print_req       = (instr_addr & MEM_MASK) == MEM_START;
  //    // print_addr      = print_counter;
  //    // print_write     = print_req;
  //    // print_wdata     = instr_addr;
  //    // -----
  //  end else if (data_req) begin
  //     // Flash
  //     data_flash_req = data_addr < RAM_OFFSET;
      
  //     print_req       = (data_addr & MEM_MASK) == MEM_START;
  //     print_addr      = print_counter;
  //     print_write     = print_req;
  //     print_wdata     = data_addr;

  //     // RAM
  //     ram_req        = data_addr >=RAM_OFFSET;
  //     // ram_req       = (data_addr & MEM_MASK) == MEM_START;

  //     ram_write      = data_we;
  //     ram_be         = data_be;
  //     ram_addr       = (data_addr - RAM_OFFSET) / 4; // translate absolute address to relative address
  //     ram_wdata      = data_wdata;

  //     // LED
  //     led_event      = data_addr == RAM_OFFSET;
  //   end
  // end

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

  // flash_reader u0 (
  //   .clk_clk       (IO_CLK),       //   clk.clk
  //   .reset_reset_n (IO_RST_N)  // reset.reset_n
  // );

`ifdef Simulation_f 

  flash_and_ram_simulator #(
    .Depth(20000)
) u_ram (
    .clk_i(IO_CLK),
    .rst_ni(IO_RST_N),

    .flash_req_i(flash_req),
    .flash_we_i(flash_write),
    .flash_be_i(flash_be),
    .flash_addr_i(flash_addr),
    .flash_wdata_i(flash_wdata),
    .flash_rvalid_o(flash_rvalid),
    .flash_rdata_o(flash_rdata),
    .flash_wait_o(flash_wait),

    .ram_req_i(ram_req),
    .ram_we_i(data_we),
    .ram_be_i(data_be),
    .ram_addr_i(data_addr / 4),
    .ram_wdata_i(data_wdata),
    .ram_rvalid_o(ram_rvalid),
    .ram_rdata_o(ram_rdata)
);
`else 

// ROM on external flash -----
epcq_and_ram u0 (
  .clk_clk              (IO_CLK),              //      clk.clk
  .reset_reset_n        (IO_RST_N),        //    reset.reset_n

  // ROM
  .rom_if_write         (flash_write),         //   rom_if.write
  .rom_if_burstcount    (1'b1),    //         .burstcount
  .rom_if_waitrequest   (flash_wait),   //         .waitrequest
  .rom_if_read          (flash_read),          //         .read
  .rom_if_address       (flash_addr),       //         .address
  .rom_if_writedata     (flash_wdata),     //         .writedata
  .rom_if_readdata      (flash_rdata),      //         .readdata
  .rom_if_readdatavalid (flash_rvalid), //         .readdatavalid
  .rom_if_byteenable    (flash_be),    //         .byteenable
  
  // RAM
  .ram_if_address       ((data_addr - RAM_OFFSET) / 4),       //   ram_if.address
  .ram_if_chipselect    (ram_req),    //         .chipselect
  .ram_if_clken         ('1),         //         .clken
  .ram_if_write         (data_we),         //         .write
  .ram_if_readdata      (ram_rdata),      //         .readdata
  .ram_if_writedata     (data_wdata),     //         .writedata
  .ram_if_byteenable    (data_be),    //         .byteenable

  // Instruction debug interface
  .instr_print_if_address    (print_counter),    // instr_print_if.address
  .instr_print_if_chipselect (print_req), //               .chipselect
  .instr_print_if_clken      ('1),      //               .clken
  .instr_print_if_write      (print_req),      //               .write
  .instr_print_if_readdata   (),   //               .readdata
  .instr_print_if_writedata  (instr_addr),  //               .writedata
  .instr_print_if_byteenable ('1),

  .ctrl_irq_irq         ()          // ctrl_irq.irq
);
// -----


`endif

  // //SRAM to Ibex
  // assign instr_rdata    = flash_rdata_s;
  // assign data_rdata     = ram_rvalid ? ram_rdata : flash_rdata_s;

  // // External flash ROM ----
  // assign instr_rvalid   = (~during_flash_data_read) & flash_rvalid_s;
  // assign data_rvalid    = (during_flash_data_read & flash_rvalid_s) | (internal_ack) | (ram_rvalid);
  // assign instr_gnt = instr_gnt_aux/* & ~data_flash_req & ~flash_wait*/;
  // // -----

  // always_ff @(posedge IO_CLK or negedge IO_RST_N) begin
  //   if (!IO_RST_N) begin
  //     instr_gnt_aux       <= 'b0;
  //     data_gnt        <= 'b0;
  //     ram_rvalid      <= 'b0;
  //     during_flash_data_read <= 'b0;
  //     during_flash_data_write <= 'b0;
  //     print_counter   <= '0;

  //     instr_rdata_s <= '0;
  //     data_rdata_s <='0;
  //     instr_rvalid_s <='0;
  //     data_rvalid_s <='0;
  //     internal_ack  <= '0;

  //     // On chip ROM -----
  //     // instr_rvalid    <= 'b0;
  //     // -----
  //   end else begin
  //     // On-chip ROM -----
  //     // instr_gnt       <= instr_req && flash_req;
  //     // instr_rvalid    <= instr_req && flash_req;
  //     // ---
  //     instr_rdata_s <= instr_rdata;
  //     data_rdata_s <=data_rdata;
  //     instr_rvalid_s <=instr_rvalid;
  //     data_rvalid_s <=data_rvalid;

  //     // Instruction request is delayed by one clock to enable data request to be processed
  //     instr_gnt_aux   <= instr_flash_req & ~flash_wait /*& ~instr_gnt_aux*/;

  //     data_gnt        <= (ram_req) | (data_flash_req & ~flash_wait);
  //     ram_rvalid      <= ram_req;
  //     internal_ack    <= during_flash_data_write & ~flash_wait;
  //     // internal_ack    <= flash_write_s & data_gnt;

  //     if(flash_rvalid_s) begin
  //       during_flash_data_read <= 1'b0;
  //     end else if(data_flash_req && ~data_we) begin
  //       during_flash_data_read <= 1'b1;
  //     end

  //     if(flash_write_s) begin
  //       during_flash_data_write <= 1'b1;
  //     end else if(~flash_wait) begin
  //       during_flash_data_write <= 1'b0;
  //     end

  //     if(data_gnt && print_counter < 500) begin
  //       print_counter <= print_counter + 1;
  //     end
  //   end
  // end

  assign print_req = instr_req && ~flash_wait;

  always_ff @(posedge IO_CLK or negedge IO_RST_N) begin
    if(~IO_RST_N) begin
       print_counter <= 0;
    end else begin
      if(instr_gnt && print_counter < 500) begin
        print_counter <= print_counter + 1;
      end
    end
  end

  //Connect the LED output to the lower four bits of the most significant
  //byte
  logic [3:0] leds;
  logic sample_trigger;
  logic [31:0] read_counter;

  always_ff @(posedge IO_CLK or negedge IO_RST_N) begin
    if (!IO_RST_N) begin
      leds <= 4'b0;
      sample_trigger <=1'b0;
      read_counter <= '0;
    end else begin
      if (data_addr == RAM_OFFSET && data_req && data_we) begin
        for (int i = 0; i < 4; i = i + 1) begin
          if (data_be[i] == 1'b1) begin
            leds <= data_wdata[i*8 +: 4];
          end
        end
      end

      // if(instr_addr == 'he8) begin
      //   sample_trigger <= 1'b1;
      // end

      if(data_addr == 'ha330) begin
        // if(read_counter == 0) begin
          sample_trigger <= 1'b1;
        // end
        read_counter <= read_counter + 1;
      end
    end
  end
  assign LED_N[2:0] = leds[2:0];
  assign LED_N[3] = sample_trigger;

endmodule
