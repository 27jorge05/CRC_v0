/*
 * Copyright (c) 2024 Jorge Luis Chuquimia Parra
 * SPDX-License-Identifier: Apache-2.0
 *
 * CRC_FIFO: Motor CRC-32 paralelo con FIFO de 2 KiB y 2 canales independientes
 * Polynomial: 0x04C11DB7 (IEEE 802.3 / Ethernet / ZIP / PNG)
 */

`default_nettype none

module tt_um_27jorge05_crc_fifo(
  input  wire [7:0] ui_in,    // Dedicated inputs
  output wire [7:0] uo_out,   // Dedicated outputs
  input  wire [7:0] uio_in,   // IOs: Input path
  output wire [7:0] uio_out,  // IOs: Output path
  output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
  input  wire       ena,      // always 1 when the design is powered
  input  wire       clk,      // clock
  input  wire       rst_n     // reset_n - low to reset
);

  // =========================================================
  // VGA signals (requerido por el playground / TinyVGA PMOD)
  // =========================================================
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

  // =========================================================
  // Interfaz de control desde ui_in
  //   ui_in[0]   = wr       (write strobe)
  //   ui_in[1]   = rd       (read strobe)
  //   ui_in[5:2] = addr[3:0]
  //   ui_in[6]   = enable
  //   ui_in[7]   = channel select (canal 0 o canal 1)
  // =========================================================
  wire        wr      = ui_in[0];
  wire        rd      = ui_in[1];
  wire [3:0]  addr    = ui_in[5:2];
  wire        enable  = ui_in[6];
  wire        ch_sel  = ui_in[7];

  // =========================================================
  // Generador HSync/VSync
  // =========================================================
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
  // FIFO 2 KiB — Canal 0
  // 2048 entradas x 8 bits
  // =========================================================
  reg [7:0]  fifo0 [0:2047];
  reg [10:0] fifo0_wr_ptr;
  reg [10:0] fifo0_rd_ptr;
  wire       fifo0_empty = (fifo0_wr_ptr == fifo0_rd_ptr);
  wire       fifo0_full  = ((fifo0_wr_ptr + 1'b1) == fifo0_rd_ptr);
  wire [10:0] fifo0_count = fifo0_wr_ptr - fifo0_rd_ptr;

  // =========================================================
  // FIFO 2 KiB — Canal 1
  // =========================================================
  reg [7:0]  fifo1 [0:2047];
  reg [10:0] fifo1_wr_ptr;
  reg [10:0] fifo1_rd_ptr;
  wire       fifo1_empty = (fifo1_wr_ptr == fifo1_rd_ptr);
  wire       fifo1_full  = ((fifo1_wr_ptr + 1'b1) == fifo1_rd_ptr);
  wire [10:0] fifo1_count = fifo1_wr_ptr - fifo1_rd_ptr;

  // =========================================================
  // CRC-32 paralelo — función combinacional (32 bits/ciclo)
  // Polynomial: 0x04C11DB7, procesamiento bit-a-bit unrolled
  // =========================================================
  function [31:0] crc32_word;
    input [31:0] crc_in;
    input [7:0]  data_byte;
    reg [31:0] crc;
    reg [31:0] poly;
    integer i;
    begin
      poly = 32'hEDB88320; // reflejado para LSB-first
      crc  = crc_in ^ {24'b0, data_byte};
      for (i = 0; i < 8; i = i + 1) begin
        if (crc[0])
          crc = (crc >> 1) ^ poly;
        else
          crc = crc >> 1;
      end
      crc32_word = crc;
    end
  endfunction

  // =========================================================
  // Registros CRC — Canal 0
  // =========================================================
  reg [31:0] crc0_reg;
  reg        crc0_done;
  reg [7:0]  crc0_byte_buf [0:3]; // buffer 4 bytes para word paralela

  // =========================================================
  // Registros CRC — Canal 1
  // =========================================================
  reg [31:0] crc1_reg;
  reg        crc1_done;

  // =========================================================
  // FSM Estados
  // =========================================================
  localparam IDLE      = 2'b00;
  localparam PROCESS   = 2'b01;
  localparam FINALIZE  = 2'b10;
  localparam DONE      = 2'b11;

  reg [1:0] fsm0_state;
  reg [1:0] fsm1_state;

  // =========================================================
  // FIFO Write — Canal 0
  // =========================================================
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      fifo0_wr_ptr <= 11'b0;
    end else if (wr && enable && !ch_sel && addr == 4'd0 && !fifo0_full) begin
      fifo0[fifo0_wr_ptr] <= uio_in; // dato por uio_in (bus bidireccional)
      fifo0_wr_ptr <= fifo0_wr_ptr + 1'b1;
    end
  end

  // =========================================================
  // FIFO Write — Canal 1
  // =========================================================
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      fifo1_wr_ptr <= 11'b0;
    end else if (wr && enable && ch_sel && addr == 4'd0 && !fifo1_full) begin
      fifo1[fifo1_wr_ptr] <= uio_in;
      fifo1_wr_ptr <= fifo1_wr_ptr + 1'b1;
    end
  end

  // =========================================================
  // FSM + CRC Engine — Canal 0
  // =========================================================
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      crc0_reg      <= 32'hFFFFFFFF;
      crc0_done     <= 1'b0;
      fifo0_rd_ptr  <= 11'b0;
      fsm0_state    <= IDLE;
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
            // Procesa 1 byte por ciclo (el árbol XOR es combinacional)
            crc0_reg     <= crc32_word(crc0_reg, fifo0[fifo0_rd_ptr]);
            fifo0_rd_ptr <= fifo0_rd_ptr + 1'b1;
          end else begin
            fsm0_state <= FINALIZE;
          end
        end
        FINALIZE: begin
          crc0_reg   <= ~crc0_reg; // inversión final IEEE 802.3
          crc0_done  <= 1'b1;
          fsm0_state <= DONE;
        end
        DONE: begin
          // Espera reset del canal para nueva operación
          if (wr && enable && !ch_sel && addr == 4'd0) begin
            crc0_done  <= 1'b0;
            fsm0_state <= IDLE;
          end
        end
      endcase
    end
  end

  // =========================================================
  // FSM + CRC Engine — Canal 1
  // =========================================================
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      crc1_reg      <= 32'hFFFFFFFF;
      crc1_done     <= 1'b0;
      fifo1_rd_ptr  <= 11'b0;
      fsm1_state    <= IDLE;
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
            crc1_reg     <= crc32_word(crc1_reg, fifo1[fifo1_rd_ptr]);
            fifo1_rd_ptr <= fifo1_rd_ptr + 1'b1;
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

  // =========================================================
  // IRQ: sube cuando cualquier canal termina o FIFO llena
  // =========================================================
  wire irq = crc0_done | crc1_done | fifo0_full | fifo1_full;

  // =========================================================
  // Visualización VGA del estado del motor CRC
  // Muestra barras de ocupación FIFO, estado FSM y bits de CRC
  // =========================================================

  // Región de la pantalla: 640x480
  // Fila 0-80:    Título / header
  // Fila 80-200:  Canal 0 — barra FIFO + bits CRC
  // Fila 200-320: Canal 1 — barra FIFO + bits CRC  
  // Fila 320-400: Estado FSM (cuadros de estado)
  // Fila 400-480: Grid de bits del CRC activo

  // Barra de ocupación Canal 0 (pix_y 100–160)
  wire [9:0] bar0_width = {fifo0_count[10:1], 1'b0}; // escala a 0-640
  wire bar0_active = (pix_y >= 10'd100 && pix_y < 10'd160) &&
                     (pix_x < bar0_width) &&
                     video_active;

  // Barra de ocupación Canal 1 (pix_y 220–280)
  wire [9:0] bar1_width = {fifo1_count[10:1], 1'b0};
  wire bar1_active = (pix_y >= 10'd220 && pix_y < 10'd280) &&
                     (pix_x < bar1_width) &&
                     video_active;

  // Grid de bits del CRC-32 (pix_y 340–460) — 32 bloques de 20x30px
  wire [4:0] crc_bit_idx = pix_x[9:5]; // qué bit del CRC (0-31)
  wire [9:0] crc_cell_x  = pix_x - {crc_bit_idx, 5'b0};
  wire crc_grid_active = (pix_y >= 10'd340 && pix_y < 10'd460) &&
                         (pix_x < 10'd640) &&
                         (crc_cell_x >= 10'd2 && crc_cell_x < 10'd18) &&
                         video_active;

  // Selecciona CRC a mostrar según ch_sel (usa el valor de ui_in)
  wire [31:0] crc_show = ch_sel ? crc1_reg : crc0_reg;
  wire        crc_bit_on = crc_show[crc_bit_idx];

  // Pulso animado usando vsync counter
  reg [7:0] frame_counter;
  always @(posedge vsync or negedge rst_n) begin
    if (!rst_n) frame_counter <= 8'b0;
    else        frame_counter <= frame_counter + 1'b1;
  end

  // Línea de "scanner" horizontal animada
  wire [9:0] scan_line = {frame_counter[5:0], 4'b0}; // 0 a 960, wraps visually
  wire scanner_active = (pix_y == (scan_line & 10'd479)) && video_active;

  // Estado FSM → color del indicador (pix_y 290-330, cuadritos 4x)
  wire [9:0] fsm_cell_w = 10'd160;
  wire [1:0] fsm_cell = pix_x[9:8]; // 4 celdas de 160px
  wire fsm_indicator = (pix_y >= 10'd290 && pix_y < 10'd330) && video_active;

  // =========================================================
  // Lógica de color final
  // =========================================================
  reg [1:0] pix_R, pix_G, pix_B;

  always @(*) begin
    pix_R = 2'b00;
    pix_G = 2'b00;
    pix_B = 2'b00;

    if (!video_active) begin
      pix_R = 2'b00; pix_G = 2'b00; pix_B = 2'b00;

    // Header gradient (pix_y < 80)
    end else if (pix_y < 10'd80) begin
      pix_R = 2'b00;
      pix_G = pix_x[8:7];   // degradado horizontal verde
      pix_B = 2'b11;

    // Barra FIFO canal 0 — verde brillante
    end else if (bar0_active) begin
      pix_R = 2'b00;
      pix_G = 2'b11;
      pix_B = 2'b01;

    // Fondo barra canal 0 (zona de la barra pero no activa)
    end else if (pix_y >= 10'd100 && pix_y < 10'd160 && video_active) begin
      pix_R = 2'b00;
      pix_G = 2'b01;
      pix_B = 2'b00;

    // Barra FIFO canal 1 — cyan
    end else if (bar1_active) begin
      pix_R = 2'b00;
      pix_G = 2'b11;
      pix_B = 2'b11;

    // Fondo barra canal 1
    end else if (pix_y >= 10'd220 && pix_y < 10'd280 && video_active) begin
      pix_R = 2'b00;
      pix_G = 2'b00;
      pix_B = 2'b01;

    // Indicador FSM — colores según estado
    end else if (fsm_indicator) begin
      case (fsm_cell)
        2'd0: begin // Canal 0 estado
          case (fsm0_state)
            IDLE:     begin pix_R=2'b01; pix_G=2'b01; pix_B=2'b01; end // gris
            PROCESS:  begin pix_R=2'b00; pix_G=2'b11; pix_B=2'b00; end // verde
            FINALIZE: begin pix_R=2'b11; pix_G=2'b11; pix_B=2'b00; end // amarillo
            DONE:     begin pix_R=2'b00; pix_G=2'b00; pix_B=2'b11; end // azul
          endcase
        end
        2'd1: begin // Canal 1 estado
          case (fsm1_state)
            IDLE:     begin pix_R=2'b01; pix_G=2'b01; pix_B=2'b01; end
            PROCESS:  begin pix_R=2'b00; pix_G=2'b11; pix_B=2'b00; end
            FINALIZE: begin pix_R=2'b11; pix_G=2'b11; pix_B=2'b00; end
            DONE:     begin pix_R=2'b00; pix_G=2'b00; pix_B=2'b11; end
          endcase
        end
        2'd2: begin // IRQ indicator
          if (irq) begin pix_R=2'b11; pix_G=2'b00; pix_B=2'b00; end // rojo = IRQ
          else     begin pix_R=2'b00; pix_G=2'b00; pix_B=2'b00; end
        end
        2'd3: begin // Enable indicator
          if (enable) begin pix_R=2'b00; pix_G=2'b11; pix_B=2'b11; end
          else        begin pix_R=2'b01; pix_G=2'b00; pix_B=2'b00; end
        end
      endcase

    // Grid de bits CRC
    end else if (crc_grid_active) begin
      if (crc_bit_on) begin
        // bit 1 → color cálido ámbar
        pix_R = 2'b11;
        pix_G = 2'b10;
        pix_B = 2'b00;
      end else begin
        // bit 0 → azul oscuro
        pix_R = 2'b00;
        pix_G = 2'b00;
        pix_B = 2'b10;
      end

    // Línea scanner animada
    end else if (scanner_active) begin
      pix_R = 2'b11;
      pix_G = 2'b11;
      pix_B = 2'b11;

    // Fondo general — negro con leve tono
    end else begin
      pix_R = 2'b00;
      pix_G = pix_y[7:6] == 2'b00 ? 2'b01 : 2'b00;
      pix_B = 2'b00;
    end
  end

  assign R = pix_R;
  assign G = pix_G;
  assign B = pix_B;

  // Suppress unused warnings
  wire _unused_ok = &{ena, uio_in, rd, addr};

endmodule