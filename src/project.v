/*
 * Copyright (c) 2024 Jorge Luis Chuquimia Parra
 * SPDX-License-Identifier: Apache-2.0
 *
 * CRC_FIFO: Motor CRC-32 con FIFO de 64 bytes y 2 canales
 * FIFO reducida a 64 bytes por canal para caber en tile 1x1
 * Polynomial: 0xEDB88320 (reflejado IEEE 802.3)
 */

`default_nettype none

module tt_um_27jorge05_crc_fifo(
  input  wire [7:0] ui_in,
  output wire [7:0] uo_out,
  input  wire [7:0] uio_in,
  output wire [7:0] uio_out,
  output wire [7:0] uio_oe,
  input  wire       ena,
  input  wire       clk,
  input  wire       rst_n
);

  // VGA signals
  wire hsync;
  wire vsync;
  wire [1:0] R;
  wire [1:0] G;
  wire [1:0] B;
  wire video_active;
  wire [9:0] pix_x;
  wire [9:0] pix_y;

  assign uo_out  = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};
  assign uio_out = 8'b0;
  assign uio_oe  = 8'b0;

  // Interfaz de control
  wire       wr     = ui_in[0];
  wire       rd     = ui_in[1];
  wire [3:0] addr   = ui_in[5:2];
  wire       enable = ui_in[6];
  wire       ch_sel = ui_in[7];

  // HSync/VSync generator
  hvsync_generator hvsync_gen(
    .clk(clk),
    .reset(~rst_n),
    .hsync(hsync),
    .vsync(vsync),
    .display_on(video_active),
    .hpos(pix_x),
    .vpos(pix_y)
  );

  // =========================================================
  // FIFO 64 bytes — Canal 0
  // =========================================================
  reg [7:0] fifo0 [0:63];
  reg [5:0] fifo0_wr_ptr;
  reg [5:0] fifo0_rd_ptr;
  wire      fifo0_empty = (fifo0_wr_ptr == fifo0_rd_ptr);
  wire      fifo0_full  = ((fifo0_wr_ptr + 6'd1) == fifo0_rd_ptr);
  wire [5:0] fifo0_count = fifo0_wr_ptr - fifo0_rd_ptr;

  // FIFO 64 bytes — Canal 1
  reg [7:0] fifo1 [0:63];
  reg [5:0] fifo1_wr_ptr;
  reg [5:0] fifo1_rd_ptr;
  wire      fifo1_empty = (fifo1_wr_ptr == fifo1_rd_ptr);
  wire      fifo1_full  = ((fifo1_wr_ptr + 6'd1) == fifo1_rd_ptr);
  wire [5:0] fifo1_count = fifo1_wr_ptr - fifo1_rd_ptr;

  // =========================================================
  // CRC-32 — función combinacional
  // =========================================================
  function [31:0] crc32_byte;
    input [31:0] crc_in;
    input [7:0]  data_byte;
    reg [31:0] crc;
    integer i;
    begin
      crc = crc_in ^ {24'b0, data_byte};
      for (i = 0; i < 8; i = i + 1) begin
        if (crc[0])
          crc = (crc >> 1) ^ 32'hEDB88320;
        else
          crc = crc >> 1;
      end
      crc32_byte = crc;
    end
  endfunction

  // =========================================================
  // Registros CRC y FSM
  // =========================================================
  localparam IDLE     = 2'b00;
  localparam PROCESS  = 2'b01;
  localparam FINALIZE = 2'b10;
  localparam DONE     = 2'b11;

  reg [31:0] crc0_reg;
  reg        crc0_done;
  reg [1:0]  fsm0_state;

  reg [31:0] crc1_reg;
  reg        crc1_done;
  reg [1:0]  fsm1_state;

  // =========================================================
  // FIFO Write — Canal 0
  // =========================================================
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      fifo0_wr_ptr <= 6'b0;
    end else if (wr && enable && !ch_sel && addr == 4'd0 && !fifo0_full) begin
      fifo0[fifo0_wr_ptr] <= uio_in;
      fifo0_wr_ptr <= fifo0_wr_ptr + 6'd1;
    end
  end

  // FIFO Write — Canal 1
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      fifo1_wr_ptr <= 6'b0;
    end else if (wr && enable && ch_sel && addr == 4'd0 && !fifo1_full) begin
      fifo1[fifo1_wr_ptr] <= uio_in;
      fifo1_wr_ptr <= fifo1_wr_ptr + 6'd1;
    end
  end

  // =========================================================
  // FSM + CRC — Canal 0
  // =========================================================
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      crc0_reg     <= 32'hFFFFFFFF;
      crc0_done    <= 1'b0;
      fifo0_rd_ptr <= 6'b0;
      fsm0_state   <= IDLE;
    end else begin
      case (fsm0_state)
        IDLE: begin
          crc0_done <= 1'b0;
          if (!fifo0_empty && enable) begin
            crc0_reg   <= 32'hFFFFFFFF;
            fsm0_state <= PROCESS;
          end
        end
        PROCESS: begin
          if (!fifo0_empty) begin
            crc0_reg     <= crc32_byte(crc0_reg, fifo0[fifo0_rd_ptr]);
            fifo0_rd_ptr <= fifo0_rd_ptr + 6'd1;
          end else begin
            fsm0_state <= FINALIZE;
          end
        end
        FINALIZE: begin
          crc0_reg   <= ~crc0_reg;
          crc0_done  <= 1'b1;
          fsm0_state <= DONE;
        end
        DONE: begin
          if (wr && enable && !ch_sel && addr == 4'd0) begin
            crc0_done  <= 1'b0;
            fsm0_state <= IDLE;
          end
        end
      endcase
    end
  end

  // =========================================================
  // FSM + CRC — Canal 1
  // =========================================================
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      crc1_reg     <= 32'hFFFFFFFF;
      crc1_done    <= 1'b0;
      fifo1_rd_ptr <= 6'b0;
      fsm1_state   <= IDLE;
    end else begin
      case (fsm1_state)
        IDLE: begin
          crc1_done <= 1'b0;
          if (!fifo1_empty && enable) begin
            crc1_reg   <= 32'hFFFFFFFF;
            fsm1_state <= PROCESS;
          end
        end
        PROCESS: begin
          if (!fifo1_empty) begin
            crc1_reg     <= crc32_byte(crc1_reg, fifo1[fifo1_rd_ptr]);
            fifo1_rd_ptr <= fifo1_rd_ptr + 6'd1;
          end else begin
            fsm1_state <= FINALIZE;
          end
        end
        FINALIZE: begin
          crc1_reg   <= ~crc1_reg;
          crc1_done  <= 1'b1;
          fsm1_state <= DONE;
        end
        DONE: begin
          if (wr && enable && ch_sel && addr == 4'd0) begin
            crc1_done  <= 1'b0;
            fsm1_state <= IDLE;
          end
        end
      endcase
    end
  end

  // IRQ
  wire irq = crc0_done | crc1_done | fifo0_full | fifo1_full;

  // =========================================================
  // VGA — visualización del estado CRC
  // =========================================================
  reg [7:0] frame_counter;
  always @(posedge vsync or negedge rst_n) begin
    if (!rst_n) frame_counter <= 8'b0;
    else        frame_counter <= frame_counter + 8'd1;
  end

  // Barras FIFO — ancho proporcional (fifo0_count es 6 bits, max 63)
  // Escala: 63 → 640px, multiplicamos por 10
  wire [9:0] bar0_width = {4'b0, fifo0_count} * 10;
  wire [9:0] bar1_width = {4'b0, fifo1_count} * 10;

  wire bar0_active = (pix_y >= 10'd100 && pix_y < 10'd160) &&
                     (pix_x < bar0_width) && video_active;

  wire bar1_active = (pix_y >= 10'd220 && pix_y < 10'd280) &&
                     (pix_x < bar1_width) && video_active;

  // Grid CRC-32 bits (32 bloques de 20px en fila 340-460)
  wire [4:0] crc_bit_idx = pix_x[9:5];
  wire [4:0] crc_cell_x  = pix_x[4:0];
  wire crc_grid_active = (pix_y >= 10'd340 && pix_y < 10'd460) &&
                         (pix_x < 10'd640) &&
                         (crc_cell_x >= 5'd2 && crc_cell_x < 5'd18) &&
                         video_active;

  wire [31:0] crc_show  = ch_sel ? crc1_reg : crc0_reg;
  wire        crc_bit_on = crc_show[crc_bit_idx];

  // Scanner animado
  wire [9:0] scan_line   = {2'b0, frame_counter[5:0], 2'b0};
  wire       scanner_act = (pix_y == (scan_line & 10'd479)) && video_active;

  // Indicadores FSM (4 bloques de 160px, fila 290-330)
  wire [1:0] fsm_cell    = pix_x[9:8];
  wire       fsm_zone    = (pix_y >= 10'd290 && pix_y < 10'd330) && video_active;

  // =========================================================
  // Color logic
  // =========================================================
  reg [1:0] pix_R, pix_G, pix_B;

  always @(*) begin
    pix_R = 2'b00;
    pix_G = 2'b00;
    pix_B = 2'b00;

    if (!video_active) begin
      pix_R = 2'b00; pix_G = 2'b00; pix_B = 2'b00;

    end else if (pix_y < 10'd80) begin
      // Header — azul con degradado verde
      pix_R = 2'b00;
      pix_G = pix_x[8:7];
      pix_B = 2'b11;

    end else if (bar0_active) begin
      // Barra canal 0 — verde
      pix_R = 2'b00; pix_G = 2'b11; pix_B = 2'b01;

    end else if (pix_y >= 10'd100 && pix_y < 10'd160) begin
      // Fondo barra canal 0
      pix_R = 2'b00; pix_G = 2'b01; pix_B = 2'b00;

    end else if (bar1_active) begin
      // Barra canal 1 — cyan
      pix_R = 2'b00; pix_G = 2'b11; pix_B = 2'b11;

    end else if (pix_y >= 10'd220 && pix_y < 10'd280) begin
      // Fondo barra canal 1
      pix_R = 2'b00; pix_G = 2'b00; pix_B = 2'b01;

    end else if (fsm_zone) begin
      // Indicadores FSM
      case (fsm_cell)
        2'd0: case (fsm0_state)
          IDLE:     begin pix_R=2'b01; pix_G=2'b01; pix_B=2'b01; end
          PROCESS:  begin pix_R=2'b00; pix_G=2'b11; pix_B=2'b00; end
          FINALIZE: begin pix_R=2'b11; pix_G=2'b11; pix_B=2'b00; end
          DONE:     begin pix_R=2'b00; pix_G=2'b00; pix_B=2'b11; end
        endcase
        2'd1: case (fsm1_state)
          IDLE:     begin pix_R=2'b01; pix_G=2'b01; pix_B=2'b01; end
          PROCESS:  begin pix_R=2'b00; pix_G=2'b11; pix_B=2'b00; end
          FINALIZE: begin pix_R=2'b11; pix_G=2'b11; pix_B=2'b00; end
          DONE:     begin pix_R=2'b00; pix_G=2'b00; pix_B=2'b11; end
        endcase
        2'd2: begin
          if (irq) begin pix_R=2'b11; pix_G=2'b00; pix_B=2'b00; end
        end
        2'd3: begin
          if (enable) begin pix_R=2'b00; pix_G=2'b11; pix_B=2'b11; end
          else        begin pix_R=2'b01; pix_G=2'b00; pix_B=2'b00; end
        end
      endcase

    end else if (crc_grid_active) begin
      // Grid bits CRC
      if (crc_bit_on) begin
        pix_R = 2'b11; pix_G = 2'b10; pix_B = 2'b00; // ámbar
      end else begin
        pix_R = 2'b00; pix_G = 2'b00; pix_B = 2'b10; // azul oscuro
      end

    end else if (scanner_act) begin
      pix_R = 2'b11; pix_G = 2'b11; pix_B = 2'b11;

    end else begin
      pix_R = 2'b00;
      pix_G = (pix_y[7:6] == 2'b00) ? 2'b01 : 2'b00;
      pix_B = 2'b00;
    end
  end

  assign R = pix_R;
  assign G = pix_G;
  assign B = pix_B;

  wire _unused_ok = &{ena, rd, addr, fifo0_count, fifo1_count};

endmodule