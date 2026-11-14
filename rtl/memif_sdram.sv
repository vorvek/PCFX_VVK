// Use external MiSTer SDRAM as memory backing store
//
// Copyright (c) 2025-2026 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

module memif_sdram
  (
    input         CPU_CLK,
    input         CPU_CE,
    input         CPU_RESn,
    input         CPU_BCYSTn,

    input [19:0]  ROM_A,
    output [15:0] ROM_DO,
    input         ROM_CEn,
    output        ROM_READYn,

    input [20:0]  RAM_A,
    input [31:0]  RAM_DI,
    output [31:0] RAM_DO,
    input         RAM_CEn,
    input         RAM_WEn,
    input [3:0]   RAM_BEn,
    output        RAM_READYn,

    input [14:0]  SRAM_A,
    input [7:0]   SRAM_DI,
    output [7:0]  SRAM_DO,
    input         SRAM_CEn,
    input         SRAM_WEn,
    output        SRAM_READYn,

    input [22:0]  BMP_A,
    input [7:0]   BMP_DI,
    output [7:0]  BMP_DO,
    input         BMP_CEn,
    input         BMP_WEn,
    output        BMP_READYn,

    // KRAM bank A memory interface
    input [18:1]  KRAMA_A,
    output [15:0] KRAMA_DI,
    input [15:0]  KRAMA_DO,
    input [1:0]   KRAMA_BE,
    input         KRAMA_WR,
    input         KRAMA_REQ,
    output        KRAMA_ACK,

    // KRAM bank B memory interface
    input [18:1]  KRAMB_A,
    output [15:0] KRAMB_DI,
    input [15:0]  KRAMB_DO,
    input [1:0]   KRAMB_BE,
    input         KRAMB_WR,
    input         KRAMB_REQ,
    output        KRAMB_ACK,

    // ROM / RAM loader/saver interface
	input [26:0]  LS_ADDR,      // byte address
	input [31:0]  LS_DIN,       // data input from loader
	input         LS_WE_REQ,    // loader requests write
	output        LS_WE_ACK,
	output [31:0] LS_DOUT,      // data output to saver
	input         LS_RD_REQ,    // saver requests read
	output        LS_RD_ACK,

    input         SDRAM_CLK,
    input         SDRAM_INIT,       // SDRAM controller init (PLL not locked)
    input         KRAM_CLR_REQ,     // rising edge: clear KRAM (core reset)
    output        KRAM_CLR_BUSY,    // KRAM clear in progress
    output [26:0] SDRAM_CH1_ADDR,
    input [31:0]  SDRAM_CH1_DOUT,
    output [31:0] SDRAM_CH1_DIN,
    output        SDRAM_CH1_REQ,
    output        SDRAM_CH1_RNW,
    output [3:0]  SDRAM_CH1_BE,
    input         SDRAM_CH1_READY,
    output [26:0] SDRAM_CH2_ADDR,
    input [15:0]  SDRAM_CH2_DOUT,
    output [15:0] SDRAM_CH2_DIN,
    output        SDRAM_CH2_REQ,
    output        SDRAM_CH2_RNW,
    input         SDRAM_CH2_READY,
    output [26:0] SDRAM_CH3_ADDR,
    input [15:0]  SDRAM_CH3_DOUT,
    output [15:0] SDRAM_CH3_DIN,
    output        SDRAM_CH3_REQ,
    output        SDRAM_CH3_RNW,
    input         SDRAM_CH3_READY
   );

`include "memif_sdram_part.svh"

// SDRAM_CLK is assumed to be N * CPU_CLK, where N > 1.

// With SDRAM_CLK = 100MHz and CPU_CLK/CE = 25MHz, ROM reads take 4
// CPU cycles or 2 wait states. Coincidentally, that's the correct
// timing for PC-FX.
// TODO: Verify this

logic           ch1_req, ch1_act;
logic           ch1_ready, ch1_ready_d = '1;

logic [15:0]    rom_do;
logic           rom_start_req;
logic           rom_readyn = '1;

logic [31:0]    ram_do;
logic           ram_start_req;
logic           ram_readyn = '1;

logic [7:0]     sram_do;
logic           sram_start_req;
logic           sram_readyn = '1;

logic [7:0]     bmp_do;
logic           bmp_start_req;
logic           bmp_readyn = '1;

logic           mem_start_req;
logic           mem_pend_req = '0;
logic           mem_req;
logic           mem_act = '0;
logic           mem_act_ram = '0;   // the active access is a RAM cache op
logic           ram_sel;            // SDRAM ch1 address mux selects RAM op
logic           ram_ackn = '1;      // RAM cache saw its op done (handshake)
logic           ram_op_start = '0;  // RAM cache SDRAM op requested / in flight
logic           ram_op_rd;          // op is a read (CPU waits, unless discard)
logic           ram_op_disc;        // read op of a cache hit: drop the data
logic           ram_spec;           // RAM read in T1, start SDRAM at once
logic [20:0]    ram_op_a;
logic [31:0]    ram_op_d;
logic [3:0]     ram_op_be;
logic           mem_we;
logic           mem_rdy;
logic           mem_readyn;

logic           ls_rd_req, ls_we_req, ls_req;
logic           ls_rd_ack_d = '0, ls_we_ack_d = '0;
logic           ls_act = '0;
logic           ls_done;
logic           ls_sel;
logic [2:0]     ls_dwell = '0;
logic           ls_act_dwell;

logic [26:0]    sdram_ch1_addr;
logic [31:0]    sdram_ch1_din;
logic [3:0]     sdram_ch1_be;

assign ls_rd_req = LS_RD_REQ ^ ls_rd_ack_d;
assign ls_we_req = LS_WE_REQ ^ ls_we_ack_d;
assign ls_req = ~mem_act & (ls_rd_req | ls_we_req);
assign ls_done = ls_act & SDRAM_CH1_READY;
assign ls_sel = ls_req | ls_act;
assign ls_act_dwell = ls_act | |ls_dwell;
assign LS_RD_ACK = ls_rd_ack_d ^ ((LS_RD_REQ ^ ls_rd_ack_d) & ls_done);
assign LS_WE_ACK = ls_we_ack_d ^ ((LS_WE_REQ ^ ls_we_ack_d) & ls_done);

// RAM accesses go through the RAM cache (below), which starts SDRAM
// operations with ram_op_start (a level, held until the op is done). A
// RAM read starts in T1 (ram_spec) when no RAM op is in flight, before
// the cache tag check; the cache then takes the op over at the end of T1.
assign rom_start_req = ~CPU_BCYSTn & ~ROM_CEn;
assign ram_spec = ~CPU_BCYSTn & ~RAM_CEn & RAM_WEn & ~ram_op_start;
assign ram_start_req = ram_op_start | ram_spec;
assign sram_start_req = ~CPU_BCYSTn & ~SRAM_CEn;
assign bmp_start_req = ~CPU_BCYSTn & ~BMP_CEn;
assign mem_start_req = rom_start_req | sram_start_req | bmp_start_req;

assign mem_req = ~ls_act_dwell & ~mem_act & (mem_start_req | mem_pend_req | ram_start_req) & mem_readyn;
assign mem_readyn = rom_readyn & ram_ackn & sram_readyn & bmp_readyn;
assign ram_sel = mem_act ? mem_act_ram : ram_start_req;

assign ch1_req = mem_req | ls_req;
assign ch1_act = mem_act | ls_act;
assign ch1_ready = (ch1_ready_d & ~(~ch1_act & ch1_req)) | SDRAM_CH1_READY;

always @(posedge SDRAM_CLK) begin
    mem_pend_req <= (mem_pend_req | mem_start_req) & ~(~CPU_RESn | (mem_act & ~mem_act_ram));
    ch1_ready_d <= ch1_ready;

    if (ls_done) begin
        ls_rd_ack_d <= LS_RD_ACK;
        ls_we_ack_d <= LS_WE_ACK;
    end
end

always @(posedge SDRAM_CLK) begin
    if (~ch1_act & SDRAM_CH1_REQ) begin
        if (ls_req)
            ls_act <= '1;
        else if (mem_req) begin
            mem_act <= '1;
            mem_act_ram <= ram_start_req;
        end
    end

    if (mem_act & (~CPU_RESn | (ch1_ready & ~mem_readyn)))
        mem_act <= '0;
    if (ls_act & ls_done) begin
        ls_act <= '0;
        ls_dwell <= '1;
    end

    if (|ls_dwell)
        ls_dwell <= ls_dwell - 1'd1;
end

assign mem_rdy = mem_act & ch1_ready;

always @(posedge CPU_CLK) if (CPU_CE) begin
    rom_readyn <= ROM_CEn | ~(mem_rdy & ~mem_act_ram);
    sram_readyn <= SRAM_CEn | ~(mem_rdy & ~mem_act_ram);
    bmp_readyn <= BMP_CEn | ~(mem_rdy & ~mem_act_ram);
end

//////////////////////////////////////////////////////////////////////
// RAM cache
//
// Direct-mapped, 16K x 32-bit words (64 KB), write-through. SDRAM reads
// take about 4 CPU cycles per bus cycle, but the real RAM (and Mednafen)
// has no wait states inside a DRAM page. Raster interrupt handlers that
// set the next line's RCR (Lip) missed the compare by a few cycles.
//
// - Read hit: READYn in the first T2 cycle (no wait state).
// - Read miss: SDRAM read, then the word is filled into the cache. The
//   read starts in T1 (ram_spec), so a miss is not slower than without
//   the cache; on a hit, the data of that read is dropped.
// - Write: posted. READYn at once if no SDRAM op is in flight; the SDRAM
//   write runs in the background. A hit updates the cached bytes; a
//   full-word miss allocates the word.
// The tag RAM is cleared on CPU reset. Only the CPU writes this RAM.

localparam RC_IDXW = 14;
localparam RC_TAGW = 19 - RC_IDXW;

logic [RC_IDXW-1:0] rc_idx, rc_widx, rc_clr_idx;
logic [RC_TAGW-1:0] rc_tag;
logic [RC_TAGW:0]   rc_tag_q, rc_tag_wd;   // {valid, tag}
logic [31:0]        rc_q, rc_wd;
logic [3:0]         rc_wbe;
logic               rc_twe;
logic               rc_clr_act = '1;
logic               rc_hit, rc_t1;
logic               rc_dsel = '0;       // RAM_DO from the cache
logic               acc_wait = '0;      // RAM access waits for a free op slot
logic               ram_done, ram_free;
logic               cur_v, cur_we;
logic               rc_spec;

assign rc_idx = RAM_A[RC_IDXW+1:2];
assign rc_tag = RAM_A[20:RC_IDXW+2];
assign rc_t1 = ~CPU_BCYSTn & ~RAM_CEn;
assign rc_hit = ~rc_clr_act & rc_tag_q[RC_TAGW] & (rc_tag_q[RC_TAGW-1:0] == rc_tag);

// SDRAM op done (sampled at CE, as the other readyn signals)
assign ram_done = ram_op_start & mem_rdy & mem_act_ram;
assign ram_free = ~ram_op_start | ram_done;

// Access to start now: a waiting one, or a new one that is not a read hit
assign cur_v = acc_wait | (rc_t1 & ~(RAM_WEn & rc_hit));
assign cur_we = ~RAM_WEn;

// Read in T1 with no op in flight: its SDRAM read already started
assign rc_spec = rc_t1 & RAM_WEn & ~ram_op_start;

dpram #(.addr_width(RC_IDXW), .data_width(RC_TAGW + 1)) rc_tagram
   (
    .clock(CPU_CLK),
    .address_a(rc_idx),
    .data_a('0),
    .enable_a(1'b1),
    .wren_a(1'b0),
    .q_a(rc_tag_q),
    .cs_a(1'b1),

    .address_b(rc_clr_act ? rc_clr_idx : rc_widx),
    .data_b(rc_clr_act ? '0 : rc_tag_wd),
    .enable_b(1'b1),
    .wren_b(rc_clr_act | (CPU_CE & rc_twe)),
    .q_b(),
    .cs_b(1'b1)
    );

genvar rc_i;
generate
    for (rc_i = 0; rc_i < 4; rc_i++) begin :rc_lane
        dpram #(.addr_width(RC_IDXW), .data_width(8)) rc_dataram
           (
            .clock(CPU_CLK),
            .address_a(rc_idx),
            .data_a('0),
            .enable_a(1'b1),
            .wren_a(1'b0),
            .q_a(rc_q[rc_i*8+:8]),
            .cs_a(1'b1),

            .address_b(rc_widx),
            .data_b(rc_wd[rc_i*8+:8]),
            .enable_b(1'b1),
            .wren_b(~rc_clr_act & CPU_CE & rc_wbe[rc_i]),
            .q_b(),
            .cs_b(1'b1)
            );
    end
endgenerate

// Tag clear on CPU reset (every CPU_CLK)
always @(posedge CPU_CLK) begin
    if (~CPU_RESn) begin
        rc_clr_act <= '1;
        rc_clr_idx <= '0;
    end
    else if (rc_clr_act) begin
        rc_clr_idx <= rc_clr_idx + 1'd1;
        if (&rc_clr_idx)
            rc_clr_act <= '0;
    end
end

// Cache write port (data and tag), used at the next CE edge
always @* begin
    rc_widx = ram_op_a[RC_IDXW+1:2];
    rc_wd = SDRAM_CH1_DOUT;
    rc_wbe = '0;
    rc_twe = '0;
    rc_tag_wd = {1'b1, ram_op_a[20:RC_IDXW+2]};

    if (ram_done & ram_op_rd & ~ram_op_disc) begin
        // Fill after a read miss
        rc_wbe = '1;
        rc_twe = '1;
    end
    else if (cur_v & cur_we & ram_free & CPU_RESn) begin
        // Posted write (CPU holds A, D, BEn): update or allocate
        rc_widx = rc_idx;
        rc_wd = RAM_DI;
        rc_tag_wd = {1'b1, rc_tag};
        if (rc_hit)
            rc_wbe = ~RAM_BEn;
        else if (RAM_BEn == 4'b0000) begin
            rc_wbe = '1;
            rc_twe = '1;
        end
    end
end

always @(posedge CPU_CLK) if (CPU_CE) begin
    ram_readyn <= '1;
    ram_ackn <= '1;

    if (~CPU_RESn) begin
        ram_op_start <= '0;
        acc_wait <= '0;
        rc_dsel <= '0;
    end
    else begin
        if (ram_done) begin
            ram_op_start <= '0;
            ram_ackn <= '0;
            if (ram_op_rd & ~ram_op_disc) begin
                ram_readyn <= '0;
                rc_dsel <= '0;
            end
        end

        if (rc_t1 & RAM_WEn & rc_hit) begin
            ram_readyn <= '0;
            rc_dsel <= '1;
        end

        if (rc_spec) begin
            // Take over the read that started in T1 (no op was in flight)
            ram_op_start <= '1;
            ram_op_rd <= '1;
            ram_op_disc <= rc_hit;
            ram_op_a <= RAM_A;
            acc_wait <= '0;
        end
        else if (cur_v & ram_free) begin
            // Start the SDRAM op. The CPU holds A, D and BEn until READYn.
            ram_op_start <= '1;
            ram_op_rd <= ~cur_we;
            ram_op_disc <= '0;
            ram_op_a <= RAM_A;
            ram_op_d <= RAM_DI;
            ram_op_be <= ~RAM_BEn;
            acc_wait <= '0;
            if (cur_we)
                ram_readyn <= '0;   // posted write
        end
        else if (rc_t1 & ~(RAM_WEn & rc_hit))
            acc_wait <= '1;
    end
end

// SDRAM_CH1_DOUT is in the SDRAM_CLK domain. Latching into the CPU_CLK
// domain helps close timing.
always @(posedge CPU_CLK) if (CPU_CE) begin
    rom_do <= SDRAM_CH1_DOUT[15:0];
    ram_do <= SDRAM_CH1_DOUT[31:0];
    sram_do <= SDRAM_CH1_DOUT[(8 * SRAM_A[0])+:8];
    bmp_do <= SDRAM_CH1_DOUT[(8 * BMP_A[0])+:8];
end

assign LS_DOUT = SDRAM_CH1_DOUT;

assign ROM_DO = rom_do;
assign ROM_READYn = rom_readyn;

assign RAM_DO = rc_dsel ? rc_q : ram_do;
assign RAM_READYn = ram_readyn;

assign SRAM_DO = sram_do;
assign SRAM_READYn = sram_readyn;

assign BMP_DO = bmp_do;
assign BMP_READYn = bmp_readyn;

always @* begin
    mem_we = '0;
    sdram_ch1_addr = 'X;
    sdram_ch1_din = 'X;
    sdram_ch1_be = 'X;
    if (ls_sel) begin
        mem_we = ls_we_req;
        sdram_ch1_din = LS_DIN;
        sdram_ch1_be = '1;
        sdram_ch1_addr = LS_ADDR;
    end
    else if (ram_sel) begin
        // Before the end of T1, a started read uses the CPU address.
        mem_we = ram_op_start & ~ram_op_rd;
        sdram_ch1_din = ram_op_d;
        sdram_ch1_be = mem_we ? ram_op_be : 4'b1111;
        sdram_ch1_addr = RAM_BASE_A + 27'(ram_op_start ? ram_op_a : RAM_A);
    end
    else if (~ROM_CEn) begin
        sdram_ch1_addr = ROM_BASE_A + 27'(ROM_A);
        sdram_ch1_be = '1;
    end
    else if (~SRAM_CEn) begin
        mem_we = ~SRAM_WEn;
        sdram_ch1_din = {4{SRAM_DI}};
        sdram_ch1_be = {2'b00, SRAM_A[0], ~SRAM_A[0]};
        sdram_ch1_addr = SRAM_BASE_A + 27'(SRAM_A);
    end
    else if (~BMP_CEn) begin
        mem_we = ~BMP_WEn;
        sdram_ch1_din = {4{BMP_DI}};
        sdram_ch1_be = {2'b00, BMP_A[0], ~BMP_A[0]};
        sdram_ch1_addr = BMP_BASE_A + 27'(BMP_A);
    end
end

assign SDRAM_CH1_ADDR = sdram_ch1_addr;
assign SDRAM_CH1_DIN = sdram_ch1_din;
assign SDRAM_CH1_BE = sdram_ch1_be;
assign SDRAM_CH1_RNW = ~mem_we;
assign SDRAM_CH1_REQ = ch1_ready_d & ch1_req;

//////////////////////////////////////////////////////////////////////
// KRAM clear after core load and on each reset
//
// The SDRAM keeps its contents when the core is loaded again or reset,
// so KRAM would still hold the data of the last session (for example a
// RAINBOW movie frame, which then shows at the next game start). After
// the FPGA is configured, and on each rising edge of KRAM_CLR_REQ (the
// core reset), write zeros to all of KRAM A and B. The core is held in
// reset until the clear is done. (Real hardware keeps DRAM contents on
// a reset; a clean start is preferred here.)

logic           kclr_act = '1;      // set by FPGA configuration and reset
logic [2:0]     kclr_req_s = '0;    // KRAM_CLR_REQ to SDRAM_CLK, and edge
logic           kclr_busy = '0;     // a write is in flight
logic           kclr_issue = '0;    // request pulse
logic [1:0]     kclr_pend = '0;     // bank A/B write not yet ready
logic [17:0]    kclr_a = '0;        // word address
logic [7:0]     kclr_start = '0;    // delay after SDRAM init

always @(posedge SDRAM_CLK) begin
    logic [1:0] pend;

    kclr_issue <= '0;
    pend = kclr_pend & ~{SDRAM_CH3_READY, SDRAM_CH2_READY};
    kclr_req_s <= {kclr_req_s[1:0], KRAM_CLR_REQ};

    if (SDRAM_INIT | (kclr_req_s[1] & ~kclr_req_s[2])) begin
        // Start over. (init also clears pending requests in the
        // controller.) A KING write in flight finishes during the start
        // delay; its READY is not counted, because kclr_pend is clear.
        kclr_act <= '1;
        kclr_start <= '0;
        kclr_busy <= '0;
        kclr_pend <= '0;
        kclr_a <= '0;
    end
    else if (kclr_act) begin
        if (~&kclr_start)
            kclr_start <= kclr_start + 1'd1;
        else if (~kclr_busy) begin
            kclr_issue <= '1;
            kclr_pend <= 2'b11;
            kclr_busy <= '1;
        end
        else begin
            kclr_pend <= pend;
            if (pend == '0) begin
                kclr_busy <= '0;
                if (&kclr_a)
                    kclr_act <= '0;
                else
                    kclr_a <= kclr_a + 1'd1;
            end
        end
    end
end

assign KRAM_CLR_BUSY = kclr_act;

//////////////////////////////////////////////////////////////////////

logic           ch2_req;
logic           krama_req_d;
logic           krama_ack;
logic           krama_ack_d = '0;

assign ch2_req = ~krama_req_d & KRAMA_REQ;
assign krama_ack = SDRAM_CH2_READY;

always @(posedge SDRAM_CLK) begin
    krama_req_d <= KRAMA_REQ;
    krama_ack_d <= krama_ack;
end

assign SDRAM_CH2_ADDR = KRAMA_BASE_A + 27'({kclr_act ? kclr_a : KRAMA_A, 1'b0});
assign SDRAM_CH2_DIN = kclr_act ? '0 : KRAMA_DO;
assign SDRAM_CH2_REQ = kclr_act ? kclr_issue : ch2_req;
assign SDRAM_CH2_RNW = kclr_act ? 1'b0 : ~KRAMA_WR;
assign KRAMA_DI = SDRAM_CH2_DOUT;
assign KRAMA_ACK = (krama_ack | krama_ack_d) & ~kclr_act;  // double for CPU_CLK

//////////////////////////////////////////////////////////////////////

logic           ch3_req;
logic           kramb_req_d;
logic           kramb_ack;
logic           kramb_ack_d = '0;

assign ch3_req = ~kramb_req_d & KRAMB_REQ;
assign kramb_ack = SDRAM_CH3_READY;

always @(posedge SDRAM_CLK) begin
    kramb_req_d <= KRAMB_REQ;
    kramb_ack_d <= kramb_ack;
end

assign SDRAM_CH3_ADDR = KRAMB_BASE_A + 27'({kclr_act ? kclr_a : KRAMB_A, 1'b0});
assign SDRAM_CH3_DIN = kclr_act ? '0 : KRAMB_DO;
assign SDRAM_CH3_REQ = kclr_act ? kclr_issue : ch3_req;
assign SDRAM_CH3_RNW = kclr_act ? 1'b0 : ~KRAMB_WR;
assign KRAMB_DI = SDRAM_CH3_DOUT;
assign KRAMB_ACK = (kramb_ack | kramb_ack_d) & ~kclr_act;  // double for CPU_CLK

endmodule
