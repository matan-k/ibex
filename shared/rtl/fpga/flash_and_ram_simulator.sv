// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

/**
 * Dual-port RAM with 1 cycle read/write delay, 32 bit words.
 *
 * The two ports are in Read-First Mode: when reading from and writing to the same address
 * simultaneously, the old content is returned, before the new content is written. New content
 * is made available to both ports with one cycle delay.
 *
 * Simultaneous write operations by both ports to the same address are to be avoided: The data
 * written to memory is not determined.
 */
module flash_and_ram_simulator #(
    parameter int Depth = 128
) (
    input               clk_i,
    input               rst_ni,

    input               flash_req_i,
    input               flash_we_i,
    input        [ 3:0] flash_be_i,
    input        [31:0] flash_addr_i,
    input        [31:0] flash_wdata_i,
    output logic        flash_rvalid_o,
    output logic [31:0] flash_rdata_o,
    output logic        flash_wait_o,

    input               ram_req_i,
    input               ram_we_i,
    input        [ 3:0] ram_be_i,
    input        [31:0] ram_addr_i,
    input        [31:0] ram_wdata_i,
    output logic        ram_rvalid_o,
    output logic [31:0] ram_rdata_o
);

  localparam int Aw = $clog2(Depth);
  localparam int unsigned DELAY = 10; // Delay is clocks. Minimal delay is 1 clock.

  logic [31:0] flash_mem [Depth];
  logic [31:0] ram_mem [Depth];
  logic        flash_wait;
  logic        write;

  logic [31:0] flash_addr_idx;
  // assign flash_addr_idx = flash_addr_i[Aw-1+2:2];
  assign flash_addr_idx = flash_addr_i;
  logic [31-Aw:0] unused_a_addr_parts;
  assign unused_a_addr_parts = {flash_addr_i[31:Aw+2], flash_addr_i[1:0]};

  logic [31:0] ram_addr_idx;
  // assign ram_addr_idx = ram_addr_i[Aw-1+2:2];
  assign ram_addr_idx = ram_addr_i;
  logic [31-Aw:0] unused_b_addr_parts;
  assign unused_b_addr_parts = {ram_addr_i[31:Aw+2], ram_addr_i[1:0]};

  assign flash_wait= time_left_counter != '0 | flash_rvalid_o;
  assign flash_wait_o = flash_wait;

  // simulates the latency in real RAMs
  logic [31:0] time_left_counter;
  logic [31:0] buffered_value;

  always @(posedge clk_i) begin
    if (flash_req_i && ~flash_wait) begin
      if (flash_we_i == 1'b1) begin
        for (int k = 0; k < 4; k = k + 1) begin
          if (flash_be_i[k] == 1'b1) begin
            flash_mem[flash_addr_idx][k*8 +: 8] <= flash_wdata_i[k*8 +: 8];
          end
        end
      end
    end

    buffered_value <= flash_mem[flash_addr_idx];
    if(time_left_counter == 1 || DELAY == 1) begin
        flash_rdata_o <= flash_mem[flash_addr_idx];
    end
  end

  always @(posedge clk_i) begin
    if (ram_req_i && ~flash_wait) begin
      if (ram_we_i) begin
        for (int m = 0; m < 4; m = m + 1) begin
          if (ram_be_i[m] == 1'b1) begin
            ram_mem[ram_addr_idx][m*8 +: 8] <= ram_wdata_i[m*8 +: 8];
          end
        end
      end
      ram_rdata_o <= ram_mem[ram_addr_idx];
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      flash_rvalid_o <= '0;
      write <= '0;
    end else begin

      // Timer activation.
      if(flash_req_i) begin
        time_left_counter <= DELAY - 1; // Delay is DELAY cycles
        write <= flash_we_i;
      end else if(time_left_counter > 0) begin
        time_left_counter <= time_left_counter - 1;
        write <= '0;
      end

      // Valid toggling
      if(time_left_counter == 1 | (flash_req_i & DELAY ==1)) begin
        flash_rvalid_o <= 1'b1;
      end else begin
        flash_rvalid_o <= 1'b0;
      end
    end 
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ram_rvalid_o <= '0;
    end else begin
      ram_rvalid_o <= ram_req_i;
    end
  end

  `ifdef VERILATOR
    // Task for loading 'mem' with SystemVerilog system task $readmemh()
    export "DPI-C" task simutil_verilator_memload;
    // Function for setting a specific 32 bit element in |mem|
    // Returns 1 (true) for success, 0 (false) for errors.
    export "DPI-C" function simutil_verilator_set_mem;

    task simutil_verilator_memload;
      input string file;
      $readmemh(file, flash_mem);
    endtask

    // TODO: Allow 'val' to have other widths than 32 bit
    function int simutil_verilator_set_mem(input int index,
                                           input logic[31:0] val);
      if (index >= Depth) begin
        return 0;
      end

      flash_mem[index] = val;
      return 1;
    endfunction
  `endif

  `ifdef SRAM_INIT_FILE
    localparam MEM_FILE = `"`SRAM_INIT_FILE`";
    initial begin
      $display("Initializing SRAM from %s", MEM_FILE);
      $readmemh(MEM_FILE, mem);
    end
  `endif
endmodule
