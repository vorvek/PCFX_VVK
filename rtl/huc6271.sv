// HuC6271 (RAINBOW)
//
// Copyright (c) 2026 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

// References:
// - https://github.com/libretro-mirrors/mednafen-git/blob/master/src/pcfx/rainbow.cpp
// - https://github.com/libretro-mirrors/mednafen-git/blob/master/src/pcfx/idct.cpp
// - PC-FXGA Authoring Software / GMAKER Starter Kit Plus (Ver. 1.0) / Device Description: HuC6271

// TODO: Rewrite to optimize for synthesis (e.g., use RAMs)

// RAINBOW_DEBUG: show a strip that timed out (decode started but did not
// reach the end of the strip data within 16 lines) as a solid band.
//`define RAINBOW_DEBUG 1

module huc6271
   (
    input             CLK,
    input             CE,
    input             RESn,

    // CPU memory / I/O bus interface
    input [4:1]       A,         // A[1]: high half of a 32-bit register
    input [15:0]      DI,
    output [15:0]     DO,
    input             CSn,
    input             WRn,
    input             RDn,

    // K-BUS interface
    input [7:0]       KBUS_DI,
    output            KBUS_REQ,
    input             KBUS_ACK,

    // R-RAM bank A memory interface
    output reg [12:0] RA_A,
    input [7:0]       RA_DI,
    output reg [7:0]  RA_DO,
    output reg        RA_OEn,
    output reg        RA_WEn,

    // R-RAM bank B memory interface
    output reg [12:0] RB_A,
    input [7:0]       RB_DI,
    output reg [7:0]  RB_DO,
    output reg        RB_OEn,
    output reg        RB_WEn,

    // Video interface
    input             DCK, // pixel clock enable
    input             HSYNC_NEGEDGE,
    output reg        VDMODE, // 0=palette, 1=YUV
    output reg [23:0] VD, // [7:0] = palette data / [23:0] = {Y,U,V}

    // Debug: {window count, decoded strip count, header count,
    //         {SRT, DCT state, input state}}
    output [31:0]     DBG
    );

logic           si_hdr_det, si_block, si_end;
logic           si_hdr_dct;
wor             si_dec_blk_done;

//////////////////////////////////////////////////////////////////////
// CPU memory / I/O bus interface
//
// 04h control: bit 0 SRT (0 = reset, 1 = run), taken at the next HSync.
//     In reset, the decoder, the input FIFO and both strip buffers are
//     cleared, nothing is displayed, and the null color registers are
//     cleared and ignore writes (MAME huc6271.cpp).
// 08h Y, 0Ch U, 10h V: null macroblock color (signed; the decoder uses
//     value + 80h).
// TODO: 00h horizontal scroll, 04h bits 1-2 (endless scroll, block noise
//     filter), 14h horizontal sync adjust

logic [7:0]     back_y, back_u, back_v;
logic [2:0]     ctrl_reg;
logic           rb_run;

wire dec_resn = RESn & rb_run;

// The registers are 32 bits wide on a 16-bit bus. Only the low half
// (A[1] = 0) holds bits; a write to the high half (for example the second
// bus cycle of a 32-bit store) does not change them.
wire [2:0] reg_a = A[4:2];
wire reg_wr = ~CSn & ~WRn & ~A[1];

always @(posedge CLK) if (CE) begin
    if (~RESn)
        ctrl_reg <= '0;
    else if (reg_wr & (reg_a == 3'd1))
        ctrl_reg <= DI[2:0];
end

always @(posedge CLK) begin
    if (~RESn)
        rb_run <= '0;
    else if (HSYNC_NEGEDGE)
        rb_run <= ctrl_reg[0];
end

always @(posedge CLK) if (CE) begin
    if (~dec_resn) begin
        back_y <= 8'h80;
        back_u <= 8'h80;
        back_v <= 8'h80;
    end
    else if (reg_wr) begin
        case (reg_a)
            3'd2: back_y <= DI[7:0] + 8'h80;
            3'd3: back_u <= DI[7:0] + 8'h80;
            3'd4: back_v <= DI[7:0] + 8'h80;
            default: ;
        endcase
    end
end

//////////////////////////////////////////////////////////////////////
// Main FSM

logic [9:0]     h_cnt;
logic [3:0]     v_cnt;
logic           rbsel; // 1: Decode to B, output from A
logic           dec_act;
logic [1:0]     dec_valid;
logic [1:0]     dec_vdmode;
logic           dec_vend;
logic [1:0]     dbg_to;

always @(posedge CLK) begin
    if (~dec_resn) begin
        h_cnt <= '0;
        v_cnt <= '0;
        rbsel <= '0;
        dec_act <= '0;
        dec_vend <= '0;
        dec_valid <= '0;
        dec_vdmode <= '0;
        dbg_to <= '0;
    end
    else begin
        if (CE)
            dec_vend <= '0;

        if (DCK) begin
            h_cnt <= h_cnt + 1'd1;
        end
        if (HSYNC_NEGEDGE) begin
            if (v_cnt == 4'd15) begin
`ifdef RAINBOW_DEBUG
                dbg_to[rbsel] <= dec_act & ~si_end;
`endif
                dec_valid[rbsel] <= (dec_act & si_end);
                dec_vdmode[rbsel] <= si_hdr_dct;
                dec_act <= '0;
                dec_vend <= '1;
                rbsel <= ~rbsel;
            end

            h_cnt <= '0;
            if (|v_cnt | dec_act | dec_valid[~rbsel])
                v_cnt <= v_cnt + 1'd1;
        end
        if (CE & si_hdr_det)
            dec_act <= '1;
    end
end

//////////////////////////////////////////////////////////////////////
// Compressed data stream injest

typedef enum bit [2:0] {
    SIS_INIT,
    SIS_HDR_TYPE,
    SIS_HDR_LEN1,
    SIS_HDR_LEN2,
    SIS_BLOCK,
    SIS_BLOCK_DONE,
    SIS_END
} sis_t;

logic           kbus_req;
sis_t           sist;
logic           si_din_ones, si_din_zeros;
logic [3:0]     si_hdr;
logic [15:0]    si_len;
logic           si_ready, si_block_req;

wire si_din = kbus_req & KBUS_ACK;

// The 2-byte block length field is included in the length of the
// block.  RLE blocks ends precisely at the end of the block.  DCT
// blocks, however, sometimes extend one byte (maybe two) past the
// end.  Additionally, sending those trailing bytes to the DCT
// bitstream decoder buffer simplifies its design.
wire si_len_end = si_len <= 16'd2; // include 2 length bytes
wire si_len_end_tail = si_len == '0;

always @(posedge CLK) if (CE) begin
    if (~dec_resn) begin
        sist <= SIS_INIT;
        si_hdr <= '0;
        si_len <= '1;
        si_din_ones <= '0;
        si_din_zeros <= '1;
    end
    else begin
        if (si_din) begin
            si_din_ones <= &KBUS_DI;
            si_din_zeros <= ~|KBUS_DI;
        end

        case (sist)
            // Block search as Mednafen (rainbow.cpp): FFh, then a valid
            // block type (F0h-F3h RLE, F8h/FFh DCT). The byte before FFh
            // does not matter; at the change from one stream to the next
            // it is often not zero.
            SIS_INIT:
                if (si_din) begin
                    if (&KBUS_DI)
                        sist <= SIS_HDR_TYPE;
                end
            SIS_HDR_TYPE:
                if (si_din) begin
                    if ((KBUS_DI[7:2] == 6'b111100) | (KBUS_DI == 8'hF8) |
                        (KBUS_DI == 8'hFF)) begin
                        si_hdr <= KBUS_DI[3:0];
                        sist <= SIS_HDR_LEN1;
                    end
                    else
                        sist <= SIS_INIT;
                end
            SIS_HDR_LEN1:
                if (si_din) begin
                    si_len[15:8] <= KBUS_DI;
                    sist <= SIS_HDR_LEN2;
                end
            SIS_HDR_LEN2:
                if (si_din) begin
                    si_len[7:0] <= KBUS_DI;
                    sist <= SIS_BLOCK;
                end
            SIS_BLOCK:
                // Read trailing zeros after block end to set
                // si_din_zeros, so the next block header is
                // recognized.
                if (si_din) begin
                    si_len <= si_len - 1'd1;
                    if (si_len_end_tail)
                        sist <= SIS_BLOCK_DONE;
                end
            SIS_BLOCK_DONE:
                if (si_dec_blk_done)
                    sist <= SIS_END;
            default: ;
        endcase

        if (dec_vend)
            sist <= SIS_INIT;
    end
end

always @* begin
    kbus_req = dec_resn;
    si_ready = si_din;
    case (sist)
        SIS_BLOCK: begin
            kbus_req = si_block_req;
            si_ready &= ~si_din_ones;
            // Cut off the block ending based on block type.
            si_ready &= (si_hdr_dct | ~si_len_end);
        end
        SIS_BLOCK_DONE,
        SIS_END:
            kbus_req = '0;
        default: ;
    endcase
end

assign KBUS_REQ = kbus_req;

assign si_hdr_det = (sist == SIS_HDR_LEN1);
assign si_block = ((sist == SIS_BLOCK) | (sist == SIS_BLOCK_DONE));
assign si_end = (sist == SIS_END);
assign si_hdr_dct = si_hdr[3];

//////////////////////////////////////////////////////////////////////
// DCT decoder clock enable
//
// The DCT path (decoder, IDCT, R-RAM store) runs at the full CLK rate,
// twice the chip's 21.48 MHz (CE). At CE rate it can miss the 16-line
// deadline on detailed images; the real chip has a deeper pipeline. The
// path meets timing at CLK, and the R-RAM writes on every CLK. The
// K-BUS input, the RLE decoder and the video output stay at CE.

wire            DCE = 1'b1;

//////////////////////////////////////////////////////////////////////
// Block data buffer
//
// This is a solution to a problem seen when KING video fetch is fully
// utilizing all KRAM bandwidth during active render, leaving only the
// H-Blank region for us to read KRAM over K-BUS.  In that situation,
// read stalls (si_busy) rapidly eat up our real-time bandwidth budget
// and can ultimately lead to decoder timeout.  This small buffer
// ensures that what little usable K-BUS time we have is fully
// utilized.

logic           sbuf_din, sbuf_en, sbuf_req, sbuf_ready, sbuf_rinc, sbuf_end_zero;
wor             sbuf_busy_rle, sbuf_busy_dct;
logic           sbuf_full, sbuf_empty;
logic [7:0]     sbuf_rdata, sbuf_dout;

fifo1 #(.DSIZE(8), .ASIZE(7)) sbuf_fifo
   (
    .WRST_N(dec_resn),
    .WCLK(CLK),
    .WDATA(KBUS_DI),
    .WINC(sbuf_din & CE),
    .WFULL(sbuf_full),
    .RRST_N(dec_resn),
    .RCLK(CLK),
    .RINC(sbuf_rinc & (si_hdr_dct ? DCE : CE)),
    .REMPTY(sbuf_empty),
    .RDATA(sbuf_rdata)
    );

assign si_block_req = ~sbuf_full;

wire sbuf_busy = si_hdr_dct ? sbuf_busy_dct : sbuf_busy_rle;

assign sbuf_din = si_block & si_block_req & si_ready;
// The second term empties the FIFO between blocks
assign sbuf_req = si_block ? ~sbuf_busy : ~sbuf_empty;
assign sbuf_rinc = sbuf_req & ~sbuf_empty;
assign sbuf_ready = sbuf_rinc;
assign sbuf_dout = sbuf_rdata;

//////////////////////////////////////////////////////////////////////
// RLE block decoder

typedef enum bit [1:0] {
    RLES_INIT,
    RLES_IN1,
    RLES_IN2,
    RLES_FILL
} rles_t;

rles_t          rles;
logic [7:0]     rle_in1_run, rle_run;
logic [7:0]     rle_in1_cnt, rle_cnt;
logic [12:0]    rle_raddr;
logic [7:0]     rle_rdata;
logic           rle_rwe;

always @* begin
    rle_in1_run = '0;
    rle_in1_cnt = '0;
    case (si_hdr[1:0])
        2'd0: {rle_in1_run[3:0], rle_in1_cnt[3:0]} = sbuf_dout;
        2'd1: {rle_in1_run[4:0], rle_in1_cnt[2:0]} = sbuf_dout;
        2'd2: {rle_in1_run[5:0], rle_in1_cnt[1:0]} = sbuf_dout;
        2'd3: {rle_in1_run[6:0], rle_in1_cnt[0:0]} = sbuf_dout;
        default: ;
    endcase
end

always @(posedge CLK) if (CE) begin
    if (~si_block | ~dec_act) begin
        rles <= RLES_INIT;
    end
    else begin
        case (rles)
            RLES_INIT: begin
                rle_run <= '0;
                rle_cnt <= '1;
                rle_raddr <= '0;
                if (si_block & ~si_hdr_dct)
                    rles <= RLES_IN1;
            end
            RLES_IN1:
                if (sbuf_ready) begin
                    rle_run <= rle_in1_run;
                    rle_cnt <= rle_in1_cnt - 1'd1;
                    rles <= RLES_FILL;
                    if (rle_in1_cnt == '0)
                        rles <= RLES_IN2;
                end
            RLES_IN2:
                if (sbuf_ready) begin
                    rle_cnt <= sbuf_dout;
                    rles <= RLES_FILL;
                end
            RLES_FILL: begin
                if (rle_raddr[0]) begin
                    if (rle_cnt == '0)
                        rles <= RLES_IN1;
                    rle_cnt <= rle_cnt - 1'd1;
                end
                rle_raddr <= rle_raddr + 1'd1;
            end
            default: ;
        endcase
    end
end

always @* begin
    rle_rdata = '0;
    rle_rwe = (rles == RLES_FILL);
    if (rle_raddr[0])
        rle_rdata = rle_run;
end

assign sbuf_busy_rle = (rles == RLES_FILL);
assign si_dec_blk_done = &rle_raddr;

//////////////////////////////////////////////////////////////////////
// DCT block decoder

// Main FSM state
typedef enum bit [3:0] {
    DCTS_INIT,
    DCTS_IQTBL,
    DCTS_Y00,
    DCTS_Y01,
    DCTS_Y10,
    DCTS_Y11,
    DCTS_U,
    DCTS_V,
    DCTS_COL_DONE,
    DCTS_SYNC_STORE,
    DCTS_BLK_DONE,
    DCTS_NULL       // fill a run of null macroblocks
} dcts_t;

// Plane (Ynn,U,V) decode state
typedef enum bit [2:0] {
    DCTPS_INIT,
    DCTPS_DECODE,
    DCTPS_SYNC_STORE,
    DCTPS_IDCT,
    DCTPS_DONE
} dctps_t;

// Decoder (iDCT input) state
typedef enum bit [2:0]
{
    DCTDS_INIT,
    DCTDS_DC_CODE,
    DCTDS_DC_K,
    DCTDS_AC_CODE,
    DCTDS_AC_K,
    DCTDS_AC_STORE,
    DCTDS_AC_END,
    DCTDS_DONE
} dctds_t;

dcts_t              dcts;
dctps_t             dctps;
dctds_t             dctds;
logic [3:0]         dct_col;
logic               dct_plane_y, dct_plane_u, dct_plane_v;
logic [7:0]         dct_iqtbl [128];
logic [6:0]         dct_iqtbl_widx;
logic [7:0]         dct_iq;
logic [22:0]        dct_bits_buf;
logic [5:0]         dct_bits_cnt;
logic [5:0]         dct_bits_pop_cnt, dct_bits_peek_cnt;
logic [11:0]        dct_bits_pop;
logic signed [11:0] dct_bits_pop_mse;
logic               dct_bits_ready;
logic [7:0]         dct_bits_code, dct_bits_code_d;
logic [3:0]         dct_qc;
logic [5:0]         dct_ic_cnt;
logic signed [8:0]  dct_dc_y, dct_dc_u, dct_dc_v;
logic [3:0]         dct_ac_zeros;
logic signed [8:0]  dct_ac_val;
logic               dct_ac_zero;
logic signed [8:0]  dct_acdc;
logic signed [15:0] dct_ictbl [8][8];
logic [5:0]         dct_ictbl_widx;
logic signed [15:0] dct_ictbl_wd;
logic               dct_ictbl_we;
logic               idct_act;
logic               idct_input_done;
logic               idct_done;
logic [7:0]         dct_idtbl [8][8];
logic [2:0]         dct_idx, dct_idy;
logic               dct_store_act;
dcts_t              dcts_store;
logic [3:0]         dct_store_col;
logic               dct_store_plane_y, dct_store_plane_v;
logic [1:0]         dct_store_plane_ynn;
logic               dct_sync_store;
logic [12:0]        dct_raddr;
logic [7:0]         dct_rdata;
logic               dct_rwe;
// Null macroblock run: DC code 0Fh, then an AC code whose zero run + 1
// is the number of macroblocks to fill with the null color (back_*).
logic               dct_null_code;  // DC code 0Fh read, run code follows
logic [4:0]         dct_null_run;
logic [2:0]         dct_null_plane;
logic [1:0]         dct_null_ph;
logic               dct_null_trig;
logic               dct_store_null;

always @(posedge CLK) if (DCE) begin
    dct_ictbl_we <= '0;
    dct_ac_zero <= '0;

    // Main FSM
    case (dcts)
        DCTS_INIT: begin
            dct_col <= '0;
            dctds <= DCTDS_INIT;
            dcts_store <= DCTS_INIT;
            dct_store_col <= '0;
            dct_null_code <= '0;
            dct_null_ph <= '0;
            dct_null_plane <= '0;
            dct_null_trig <= '0;
            dct_store_null <= '0;
            if (si_block & si_hdr_dct) begin
                if (si_hdr[2]) begin
                    dcts <= DCTS_IQTBL;
                    dct_iqtbl_widx <= '0;
                end
                else
                    dcts <= DCTS_Y00;
            end
        end
        DCTS_IQTBL:
            if (sbuf_ready) begin
                dct_iqtbl[dct_iqtbl_widx] <= sbuf_dout;
                dct_iqtbl_widx <= dct_iqtbl_widx + 1'd1;
                if (dct_iqtbl_widx == 7'd127)
                    dcts <= DCTS_Y00;
            end
        DCTS_Y00,
            DCTS_Y01,
            DCTS_Y10,
            DCTS_Y11,
            DCTS_U,
            DCTS_V:
                if (dctps == DCTPS_DONE) begin
                    dcts <= dcts_t'(dcts + 1'd1);
                    if (dcts != DCTS_V) begin
                        dctps <= DCTPS_INIT;
                    end
                end
        DCTS_COL_DONE: begin
            dct_col <= dct_col + 1'd1;
            dctps <= DCTPS_INIT;
            if (dct_col == 4'd15)
                dcts <= DCTS_SYNC_STORE;
            else
                dcts <= DCTS_Y00;
        end
        DCTS_SYNC_STORE:
            if (dct_sync_store)
                dcts <= DCTS_BLK_DONE;
        DCTS_BLK_DONE:
            dctds <= DCTDS_INIT;
        DCTS_NULL:
            // Store each of the 6 blocks of the macroblock with the null
            // color, through the IDCT output store path.
            case (dct_null_ph)
                2'd0:
                    if (dct_sync_store) begin
                        dcts_store <= dcts_t'(DCTS_Y00 + dct_null_plane);
                        dct_store_col <= dct_col;
                        dct_store_null <= '1;
                        dct_null_trig <= '1;
                        dct_null_ph <= 2'd1;
                    end
                2'd1: begin
                    dct_null_trig <= '0;
                    if (dct_store_act)
                        dct_null_ph <= 2'd2;
                end
                default:
                    if (~dct_store_act) begin
                        dct_null_ph <= 2'd0;
                        dct_null_plane <= dct_null_plane + 1'd1;
                        if (dct_null_plane == 3'd5) begin
                            dct_null_plane <= '0;
                            dct_null_run <= dct_null_run - 1'd1;
                            if (dct_col == 4'd15) begin
                                dct_store_null <= '0;
                                dcts <= DCTS_SYNC_STORE;
                            end
                            else begin
                                dct_col <= dct_col + 1'd1;
                                if (dct_null_run == 5'd1) begin
                                    dct_store_null <= '0;
                                    dctps <= DCTPS_INIT;
                                    dcts <= DCTS_Y00;
                                end
                            end
                        end
                    end
            endcase
        default: ;
    endcase

    // Start each strip from a clean state. A strip that did not finish
    // in its 16-line window has timed out; without this reset the FSM can
    // stay stuck (for example waiting for bits after the block end) and
    // every later strip times out too (black RAINBOW layer).
    if (~dec_resn | ~dec_act)
        dcts <= DCTS_INIT;

    // Plane FSM
    case (dctps)
        DCTPS_INIT:
            if (dct_plane_y | dct_plane_u | dct_plane_v) begin
                dctps <= DCTPS_DECODE;
                dctds <= DCTDS_DC_CODE;
            end
        DCTPS_DECODE: begin
            if (dctds == DCTDS_DONE) begin
`ifdef TB_VPU
                dump_ictbl();
`endif
                dctps <= DCTPS_SYNC_STORE;
            end
        end
        DCTPS_SYNC_STORE:
            if (dct_sync_store) begin
                dctps <= DCTPS_IDCT;
                dct_store_col <= dct_col;
                dcts_store <= dcts;
            end
        DCTPS_IDCT: begin
            if (idct_input_done)
                dctps <= DCTPS_DONE;
        end
        default: ;
    endcase

    // Decode FSM
    case (dctds)
        DCTDS_DC_CODE: begin
            dct_ic_cnt <= '0;
            if (dct_bits_ready) begin
                if (dct_bits_code < 8'h0f)
                    dctds <= DCTDS_DC_K;
                else if (dct_bits_code == 8'h0f) begin
                    dct_null_code <= '1;
                    dctds <= DCTDS_AC_CODE;
                end
                else
                    dctds <= DCTDS_DC_CODE;
            end
        end
        DCTDS_DC_K:
            if (dct_bits_ready) begin
                dct_ictbl_we <= '1;
                dctds <= DCTDS_AC_CODE;
            end
        DCTDS_AC_CODE:
            if (dct_bits_ready)
                dctds <= DCTDS_AC_K;
        DCTDS_AC_K:
            if (dct_bits_ready) begin
                if (dct_null_code) begin
                    // The run code and its value bits are consumed.
                    dct_null_code <= '0;
                    if (dcts == DCTS_Y00) begin
                        // First block of a macroblock: null run
                        dct_null_run <= 5'(dct_bits_code_d[7:4]) + 1'd1;
                        dct_null_plane <= '0;
                        dct_null_ph <= '0;
                        dctds <= DCTDS_DONE;
                        dctps <= DCTPS_INIT;
                        dcts <= DCTS_NULL;
                    end
                    else begin
                        // Other blocks (as MAME): DC difference 0, then
                        // the AC codes of the block
                        dct_ictbl_we <= '1;
                        dctds <= DCTDS_AC_CODE;
                    end
                end
                else begin
                    dctds <= DCTDS_AC_STORE;
                    if (dct_bits_pop_mse == '0 && dct_ac_zeros == '0)
                        dctds <= DCTDS_AC_END;
                end
            end
        DCTDS_AC_STORE: begin
            dct_ictbl_we <= '1;

            if (dct_ac_zeros != '0)
                dct_ac_zero <= '1;
            else
                dctds <= DCTDS_AC_CODE;

            dct_ic_cnt <= dct_ic_cnt + 1'd1;
            if (dct_ic_cnt + 1'd1 == 6'd63)
                dctds <= DCTDS_DONE;
        end
        DCTDS_AC_END: begin
            dct_ictbl_we <= '1;
            dct_ac_zero <= '1;

            dct_ic_cnt <= dct_ic_cnt + 1'd1;
            if (dct_ic_cnt + 1'd1 == 6'd63)
                dctds <= DCTDS_DONE;
        end
        default: ;
    endcase

    if (~dec_act) begin
        dcts <= DCTS_INIT;
        dctps <= DCTPS_INIT;
        dctds <= DCTDS_INIT;
    end
end

assign sbuf_busy_dct = (dcts == DCTS_INIT) | 
                     ((dcts >= DCTS_Y00) & (dcts < DCTS_BLK_DONE) &
                      (dctds == DCTDS_INIT));
assign si_dec_blk_done = (dcts == DCTS_BLK_DONE);

always @* begin
    dct_plane_y = '0;
    dct_plane_u = '0;
    dct_plane_v = '0;
    case (dcts)
        DCTS_Y00, DCTS_Y01, DCTS_Y10, DCTS_Y11: begin
            dct_plane_y = '1;
        end
        DCTS_U:
            dct_plane_u = '1;
        DCTS_V:
            dct_plane_v = '1;
        default: ;
    endcase
end

// Decoder input buffer
//
// The bitstream is apparently read starting at the most significant
// bit, so it is shifted into the buffer from the right.
always @(posedge CLK) if (DCE) begin
    if (dctds == DCTDS_INIT) begin
        dct_bits_buf <= '0;
        dct_bits_cnt <= '0;
    end
    else begin
    logic [5:0] cnt;
        cnt = dct_bits_cnt;
        if (dct_bits_ready) begin
            cnt -= dct_bits_pop_cnt;
        end
        if (sbuf_ready) begin
            dct_bits_buf <= {dct_bits_buf[$left(dct_bits_buf)-8:0], sbuf_dout};
            cnt += 6'd8;
        end
        dct_bits_cnt <= cnt;
    end
end

wire dct_bits_full = dct_bits_cnt > 6'($size(dct_bits_buf) - 8);
assign sbuf_busy_dct = (dctds != DCTDS_INIT) & dct_bits_full;

task dct_bits_get(input [5:0] cnt, output [11:0] pbuf);
    pbuf = $size(pbuf)'(dct_bits_buf >> (dct_bits_cnt - cnt));
endtask

always @* begin
    dct_bits_ready = '0;
    dct_bits_code = '0;
    dct_bits_peek_cnt = '0;
    dct_bits_pop_cnt = '0;
    dct_bits_pop = '0;
    case (dctds)
        DCTDS_DC_CODE: begin
            if (dct_plane_y) begin
                dct_bits_peek_cnt = 6'd9;
                dct_bits_get(dct_bits_peek_cnt, dct_bits_pop);
                huffdec_dcy(dct_bits_pop[8:0], dct_bits_code, dct_bits_pop_cnt);
            end
            else begin
                dct_bits_peek_cnt = 6'd8;
                dct_bits_get(dct_bits_peek_cnt, dct_bits_pop);
                huffdec_dcuv(dct_bits_pop[7:0], dct_bits_code, dct_bits_pop_cnt);
            end
            dct_bits_ready = dct_bits_cnt >= dct_bits_peek_cnt;
        end
        DCTDS_DC_K,
        DCTDS_AC_K: begin
            dct_bits_pop_cnt = dct_bits_code_d[5:0];
            if (dctds == DCTDS_AC_K)
                dct_bits_pop_cnt[5:4] = '0;
            dct_bits_get(dct_bits_pop_cnt, dct_bits_pop);
            dct_bits_ready = dct_bits_cnt >= dct_bits_pop_cnt;
        end
        DCTDS_AC_CODE: begin
            dct_bits_peek_cnt = 6'd12;
            dct_bits_get(dct_bits_peek_cnt, dct_bits_pop);
            if (dct_plane_y)
                huffdec_acy(dct_bits_pop[11:0], dct_bits_code, dct_bits_pop_cnt);
            else
                huffdec_acuv(dct_bits_pop[11:0], dct_bits_code, dct_bits_pop_cnt);
            dct_bits_ready = dct_bits_cnt >= dct_bits_peek_cnt;
        end
        default: ;
    endcase
end

// Huffman decoders
task huffdec_dcy(input [8:0] bbuf, output [7:0] code, output [5:0] pop);
    casez (bbuf)
        9'b000zzzzzz:   {code, pop} = {8'h00, 6'd03};
        9'b001zzzzzz:   {code, pop} = {8'h02, 6'd03};
        9'b010zzzzzz:   {code, pop} = {8'h03, 6'd03};
        9'b011zzzzzz:   {code, pop} = {8'h04, 6'd03};
        9'b100zzzzzz:   {code, pop} = {8'h05, 6'd03};
        9'b101zzzzzz:   {code, pop} = {8'h06, 6'd03};
        9'b110zzzzzz:   {code, pop} = {8'h07, 6'd03};
        9'b1110zzzzz:   {code, pop} = {8'h01, 6'd04};
        9'b111100zzz:   {code, pop} = {8'h08, 6'd06};
        9'b1111010zz:   {code, pop} = {8'h09, 6'd07};
        9'b1111011zz:   {code, pop} = {8'h0f, 6'd07};
        9'b11111zzzz:   {code, pop} = {8'(bbuf[4:0]), 6'd09};
        default:        {code, pop} = {8'h00, 6'd00};
    endcase
endtask

logic [7:0] huffdec_acy_fzz [0:'h7f] = '{
    8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,
    8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,
    8'h08, 8'h14, 8'h15, 8'h16, 8'h17, 8'h18, 8'h23, 8'h24,
    8'h25, 8'h26, 8'h27, 8'h28, 8'h32, 8'h33, 8'h34, 8'h35,
    8'h36, 8'h37, 8'h38, 8'h42, 8'h43, 8'h44, 8'h45, 8'h46,
    8'h47, 8'h48, 8'h52, 8'h53, 8'h54, 8'h55, 8'h56, 8'h57,
    8'h58, 8'h62, 8'h63, 8'h64, 8'h65, 8'h66, 8'h67, 8'h68,
    8'h72, 8'h73, 8'h74, 8'h75, 8'h76, 8'h77, 8'h78, 8'h81,
    8'h82, 8'h83, 8'h84, 8'h85, 8'h86, 8'h87, 8'h88, 8'h91,
    8'h92, 8'h93, 8'h94, 8'h95, 8'h96, 8'h97, 8'h98, 8'ha1,
    8'ha2, 8'ha3, 8'ha4, 8'ha5, 8'ha6, 8'ha7, 8'ha8, 8'hb1,
    8'hb2, 8'hb3, 8'hb4, 8'hb5, 8'hb6, 8'hb7, 8'hb8, 8'hc1,
    8'hc2, 8'hc3, 8'hc4, 8'hc5, 8'hc6, 8'hc7, 8'hc8, 8'hd1,
    8'hd2, 8'hd3, 8'hd4, 8'hd5, 8'hd6, 8'hd7, 8'hd8, 8'he1,
    8'he2, 8'he3, 8'he4, 8'he5, 8'he6, 8'he7, 8'he8, 8'hf1,
    8'hf2, 8'hf3, 8'hf4, 8'hf5, 8'hf6, 8'hf7, 8'hf8, 8'h10
};

task huffdec_acy(input [11:0] bbuf, output [7:0] code, output [5:0] pop);
    casez (bbuf)
        12'b00zzzzzzzzzz:   {code, pop} = {8'h01, 6'd02};
        12'b01zzzzzzzzzz:   {code, pop} = {8'h02, 6'd02};
        12'b100zzzzzzzzz:   {code, pop} = {8'h03, 6'd03};
        12'b1010zzzzzzzz:   {code, pop} = {8'h04, 6'd04};
        12'b1011zzzzzzzz:   {code, pop} = {8'h11, 6'd04};
        12'b11000zzzzzzz:   {code, pop} = {8'h05, 6'd05};
        12'b11001zzzzzzz:   {code, pop} = {8'h12, 6'd05};
        12'b11010zzzzzzz:   {code, pop} = {8'h21, 6'd05};
        12'b110110zzzzzz:   {code, pop} = {8'h06, 6'd06};
        12'b110111zzzzzz:   {code, pop} = {8'h31, 6'd06};
        12'b111000zzzzzz:   {code, pop} = {8'h41, 6'd06};
        12'b111001zzzzzz:   {code, pop} = {8'h51, 6'd06};
        12'b1110100zzzzz:   {code, pop} = {8'h13, 6'd07};
        12'b1110101zzzzz:   {code, pop} = {8'h22, 6'd07};
        12'b1110110zzzzz:   {code, pop} = {8'h61, 6'd07};
        12'b111011100zzz:   {code, pop} = {8'h07, 6'd09};
        12'b111011101zzz:   {code, pop} = {8'h71, 6'd09};
        12'b11101111zzzz,
        12'b11110000zzzz:   {code, pop} = {{bbuf[8], bbuf[3:1], 4'h9}, 6'd11};
        12'b11110001zzzz,
        12'b1111001zzzzz,
        12'b111101zzzzzz:   {code, pop} = {huffdec_acy_fzz[bbuf[6:0]], 6'd12};
        12'b11111zzzzzzz:   {code, pop} = {8'h00, 6'd05};
        default:            {code, pop} = {8'h00, 6'd00};
    endcase
endtask

task huffdec_dcuv(input [7:0] bbuf, output [7:0] code, output [5:0] pop);
    casez (bbuf)
        8'b00zzzzzz:    {code, pop} = {8'h00, 6'd02};
        8'b01zzzzzz:    {code, pop} = {8'h01, 6'd02};
        8'b10zzzzzz:    {code, pop} = {8'h02, 6'd02};
        8'b110zzzzz:    {code, pop} = {8'h03, 6'd03};
        8'b1110zzzz:    {code, pop} = {8'h04, 6'd04};
        8'b11110zzz:    {code, pop} = {8'h05, 6'd05};
        8'b111110zz:    {code, pop} = {8'h06, 6'd06};
        8'b1111110z:    {code, pop} = {8'h07, 6'd07};
        8'b11111110:    {code, pop} = {8'h08, 6'd08};
        8'b11111111:    {code, pop} = {8'h09, 6'd08};
        default:        {code, pop} = {8'h00, 6'd00};
    endcase
endtask

logic [7:0] huffdec_acuv_fzz [0:'h7f] = '{
    8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,
    8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,
    8'h06, 8'h07, 8'h08, 8'h14, 8'h15, 8'h16, 8'h17, 8'h18,
    8'h23, 8'h24, 8'h25, 8'h26, 8'h27, 8'h28, 8'h33, 8'h34,
    8'h35, 8'h36, 8'h37, 8'h38, 8'h42, 8'h43, 8'h44, 8'h45,
    8'h46, 8'h47, 8'h48, 8'h52, 8'h53, 8'h54, 8'h55, 8'h56,
    8'h57, 8'h58, 8'h62, 8'h63, 8'h64, 8'h65, 8'h66, 8'h67,
    8'h68, 8'h72, 8'h73, 8'h74, 8'h75, 8'h76, 8'h77, 8'h78,
    8'h82, 8'h83, 8'h84, 8'h85, 8'h86, 8'h87, 8'h88, 8'h91,
    8'h92, 8'h93, 8'h94, 8'h95, 8'h96, 8'h97, 8'h98, 8'ha1,
    8'ha2, 8'ha3, 8'ha4, 8'ha5, 8'ha6, 8'ha7, 8'ha8, 8'hb1,
    8'hb2, 8'hb3, 8'hb4, 8'hb5, 8'hb6, 8'hb7, 8'hb8, 8'hc1,
    8'hc2, 8'hc3, 8'hc4, 8'hc5, 8'hc6, 8'hc7, 8'hc8, 8'hd1,
    8'hd2, 8'hd3, 8'hd4, 8'hd5, 8'hd6, 8'hd7, 8'hd8, 8'he1,
    8'he2, 8'he3, 8'he4, 8'he5, 8'he6, 8'he7, 8'he8, 8'hf1,
    8'hf2, 8'hf3, 8'hf4, 8'hf5, 8'hf6, 8'hf7, 8'hf8, 8'h10
};

task huffdec_acuv(input [11:0] bbuf, output [7:0] code, output [5:0] pop);
    casez (bbuf)
        12'b00zzzzzzzzzz:   {code, pop} = {8'h01, 6'd02};
        12'b01zzzzzzzzzz:   {code, pop} = {8'h02, 6'd02};
        12'b100zzzzzzzzz:   {code, pop} = {8'h11, 6'd03};
        12'b1010zzzzzzzz:   {code, pop} = {8'h03, 6'd04};
        12'b1011zzzzzzzz:   {code, pop} = {8'h21, 6'd04};
        12'b11000zzzzzzz:   {code, pop} = {8'h04, 6'd05};
        12'b11001zzzzzzz:   {code, pop} = {8'h12, 6'd05};
        12'b11010zzzzzzz:   {code, pop} = {8'h31, 6'd05};
        12'b11011zzzzzzz:   {code, pop} = {8'h41, 6'd05};
        12'b111000zzzzzz:   {code, pop} = {8'h51, 6'd06};
        12'b1110010zzzzz:   {code, pop} = {8'h05, 6'd07};
        12'b1110011zzzzz:   {code, pop} = {8'h13, 6'd07};
        12'b1110100zzzzz:   {code, pop} = {8'h22, 6'd07};
        12'b1110101zzzzz:   {code, pop} = {8'h61, 6'd07};
        12'b1110110zzzzz:   {code, pop} = {8'h71, 6'd07};
        12'b111011100zzz:   {code, pop} = {8'h32, 6'd09};
        12'b111011101zzz:   {code, pop} = {8'h81, 6'd09};
        12'b11101111zzzz,
        12'b11110000zzzz:   {code, pop} = {{bbuf[8], bbuf[3:1], 4'h9}, 6'd11};
        12'b11110001zzzz,
        12'b1111001zzzzz,
        12'b111101zzzzzz:   {code, pop} = {huffdec_acuv_fzz[bbuf[6:0]], 6'd12};
        12'b11111zzzzzzz:   {code, pop} = {8'h00, 6'd05};
        default:            {code, pop} = {8'h00, 6'd00};
    endcase
endtask

always @* begin
    if (~dct_bits_pop[dct_bits_pop_cnt - 1])
        // "Funny" sign extension
        dct_bits_pop_mse = (dct_bits_pop | ~((1 << dct_bits_pop_cnt) - 1)) + 1'd1;
    else
        dct_bits_pop_mse = dct_bits_pop & ((1 << dct_bits_pop_cnt) - 1);
end

// Code -> coefficients
always @(posedge CLK) if (DCE) begin
    if (~RESn) begin
        dct_qc <= 4'd4;
    end
    if (dctds == DCTDS_INIT) begin
        dct_bits_code_d <= '0;
        dct_dc_y <= '0;
        dct_dc_u <= '0;
        dct_dc_v <= '0;
        dct_ac_zeros <= '0;
        dct_ac_val <= '0;
    end
    else if (dctds == DCTDS_AC_STORE) begin
        if (dct_ac_zeros != '0) begin
            dct_ac_zeros <= dct_ac_zeros - 1'd1;
        end
    end
    else if (dct_bits_ready) begin
        case (dctds)
            DCTDS_DC_CODE: begin
                dct_bits_code_d <= dct_bits_code;
                if (dct_bits_code > 8'h0f)
                    dct_qc <= dct_bits_code[3:0];
                // 0Fh (null run) is handled in the decode FSM.
            end
            DCTDS_AC_CODE: begin
                dct_bits_code_d <= dct_bits_code;
                if (~dct_null_code)
                    dct_ac_zeros <= dct_ac_zeros + dct_bits_code[7:4];
            end
            DCTDS_DC_K: begin
                if (dct_plane_y)
                    dct_dc_y <= dct_dc_y + $size(dct_dc_y)'(dct_bits_pop_mse);
                else if (dct_plane_u)
                    dct_dc_u <= dct_dc_u + $size(dct_dc_y)'(dct_bits_pop_mse);
                else if (dct_plane_v)
                    dct_dc_v <= dct_dc_v + $size(dct_dc_y)'(dct_bits_pop_mse);
            end
            DCTDS_AC_K: begin
                if (dct_null_code) begin
                    // A null run resets the DC predictors.
                    if (dcts == DCTS_Y00) begin
                        dct_dc_y <= '0;
                        dct_dc_u <= '0;
                        dct_dc_v <= '0;
                    end
                end
                else begin
                    dct_ac_val <= $size(dct_ac_val)'(dct_bits_pop_mse);
                    if (dct_bits_pop_mse == '0 && dct_ac_zeros == 4'd1)
                        dct_ac_zeros <= 4'd15;
                end
            end
            default: ;
        endcase
    end
end

// IQ table * QC * coefficients -> image data table
always @* begin
    dct_acdc = dct_ac_val;
    if (dct_ac_zero)
        dct_acdc = '0;

    if (dct_ic_cnt == '0) begin
        if (dct_plane_y)
            dct_acdc = dct_dc_y;
        else if (dct_plane_u)
            dct_acdc = dct_dc_u;
        else if (dct_plane_v)
            dct_acdc = dct_dc_v;
    end
end

always @* begin
logic [11:0] x;
logic [3:0]  qc;
logic [6:0]  iqtbl_ridx;

    iqtbl_ridx[5:0] = dct_ictbl_widx;
    iqtbl_ridx[6] = ~dct_plane_y;

    qc = dct_qc;
    if (iqtbl_ridx == 7'h40) // for dct_dc_u/v
        qc = 4'd1;
    x = (dct_iqtbl[iqtbl_ridx] * qc) >> 2;

    if (x < 12'd1)
        x = 12'd1;
    else if (x > 12'hFE)
        x = 12'hFE;

    dct_iq = x[7:0];
end

logic [5:0] dct_zigzag_tbl [64] = '{
    6'h00, 6'h01, 6'h08, 6'h10, 6'h09, 6'h02, 6'h03, 6'h0A,
    6'h11, 6'h18, 6'h20, 6'h19, 6'h12, 6'h0B, 6'h04, 6'h05,
    6'h0C, 6'h13, 6'h1A, 6'h21, 6'h28, 6'h30, 6'h29, 6'h22,
    6'h1B, 6'h14, 6'h0D, 6'h06, 6'h07, 6'h0E, 6'h15, 6'h1C,
    6'h23, 6'h2A, 6'h31, 6'h38, 6'h39, 6'h32, 6'h2B, 6'h24,
    6'h1D, 6'h16, 6'h0F, 6'h17, 6'h1E, 6'h25, 6'h2C, 6'h33,
    6'h3A, 6'h3B, 6'h34, 6'h2D, 6'h26, 6'h1F, 6'h27, 6'h2E,
    6'h35, 6'h3C, 6'h3D, 6'h36, 6'h2F, 6'h37, 6'h3E, 6'h3F
};

assign dct_ictbl_widx = dct_zigzag_tbl[dct_ic_cnt];
assign dct_ictbl_wd = $size(dct_ictbl_wd)'($signed(9'(dct_iq)) * dct_acdc);

always @(posedge CLK) if (DCE) begin
    if (dct_ictbl_we)
        dct_ictbl[dct_ictbl_widx[5:3]][dct_ictbl_widx[2:0]] <= dct_ictbl_wd;
end

// Store IDCT output to R-RAM
always @* begin
    dct_store_plane_y = '0;
    dct_store_plane_v = '0;
    dct_store_plane_ynn = '0;
    case (dcts_store)
        DCTS_Y00, DCTS_Y01, DCTS_Y10, DCTS_Y11: begin
            dct_store_plane_y = '1;
            dct_store_plane_ynn = 2'(dcts_store - DCTS_Y00);
        end
        DCTS_V:
            dct_store_plane_v = '1;
        default: ;
    endcase
end

always @(posedge CLK) if (DCE) begin
    dct_rwe <= dct_store_act;
    if (dct_store_plane_y)
        dct_raddr <= {dct_store_plane_ynn[0], dct_idy, dct_store_col, 
                      dct_store_plane_ynn[1], dct_idx[2:1], 1'b0, dct_idx[0]};
    else
        dct_raddr <= {dct_idy, 1'b0, dct_store_col, 
                      dct_idx, 1'b1, dct_store_plane_v};
    if (dct_store_null)
        dct_rdata <= dct_store_plane_y ? back_y : dct_store_plane_v ? back_v : back_u;
    else
        dct_rdata <= dct_idtbl[dct_idy][dct_idx];
end

always @(posedge CLK) if (DCE) begin
    if (~dct_store_act) begin
        dct_idx <= '0;
        dct_idy <= '0;
        dct_store_act <= idct_done | dct_null_trig;
    end
    else begin
        dct_idx <= dct_idx + 1'd1;
        if (&dct_idx)
            dct_idy <= dct_idy + 1'd1;
        if (&dct_idx & &dct_idy)
            dct_store_act <= '0;
    end

    if (dcts == DCTS_INIT)
        dct_store_act <= '0;
end

assign dct_sync_store = ~idct_act & ~dct_store_act;

`ifdef TB_VPU
task dump_ictbl;
    $display("IDCT col=%1d plane=%1d", dct_col, 
             3'(dcts - DCTS_Y00 + 4'd1));
/* -----\/----- EXCLUDED -----\/-----
    for (int i = 0; i < 8; i ++) begin
        $display("%02x: %05x %05x %05x %05x %05x %05x %05x %05x", i[5:0],
                 dct_ictbl[i][0], dct_ictbl[i][1], dct_ictbl[i][2], dct_ictbl[i][3],
                 dct_ictbl[i][4], dct_ictbl[i][5], dct_ictbl[i][6], dct_ictbl[i][7]);
    end
 -----/\----- EXCLUDED -----/\----- */
endtask
`endif

//////////////////////////////////////////////////////////////////////
// 2-D 8x8 Inverse Discrete Cosine Transform

localparam IDW = 24;

logic [7:0] [IDW-1:0] idct_bufr1 [5];
logic [7:0] [IDW-1:0] idct_bufr2 [5];
logic [7:0] [IDW-1:0] idct_bufrt [8];
logic [5:0]           idct_step;

always @(posedge CLK) if (DCE) begin
    idct_input_done <= '0;

    if (~dec_act)
        idct_act <= '0;

    if (~idct_act) begin
        idct_step <= '0;
        idct_done <= '0;
        idct_act <= (dctps == DCTPS_IDCT);
    end
    else if (~idct_done) begin
        idct_run(int'(idct_step));
        idct_step <= idct_step + 1'd1;
        if (idct_step == 6'd7)
            idct_input_done <= '1;
        if (idct_step == 6'd26)
            idct_done <= '1;
    end
    else
        idct_act <= '0;
end

`define IDCT_PRESHIFT 9
`define EFF_RSHIFT_1D_COEFF 2
`define EFF_RSHIFT_1D_POST  6
`define C_COEFF(m) (IDW'((m) >>> (30 - (IDW - `EFF_RSHIFT_1D_COEFF))))

`ifdef TB_VPU
integer      fout = $fopen("huc6271_yuvblk.hex", "w");
`endif

function [7:0] idct_clamp(input signed [IDW-1:0] din);
    idct_clamp = din[7:0];
    if (din < IDW'(0))
        idct_clamp = 8'd0;
    else if (din > IDW'(255))
        idct_clamp = 8'd255;
endfunction

task idct_run(input int stage);
`ifdef TB_VPU
static logic [7:0] [IDW-1:0] bufro [8];
`endif
int i;

    // Input to first IDCT as rows
    if (stage >= 0 && stage <= 7) begin
        i = stage - 0;
        for (int j = 0; j < 8; j++)
            idct_bufr1[0][j] <= IDW'(dct_ictbl[i][j]);
    end

    // First 1-D IDCT processes each row
    if (stage >= 1 && stage <= 11) begin
        for (int s = 0; s < 4; s++) begin
        logic [7:0] [IDW-1:0] c_out;
            IDCT_1D(s, idct_bufr1[s], c_out, 0);
            idct_bufr1[s+1] <= c_out;
        end
    end

    if (stage >= 1 && stage <= 12) begin
/* -----\/----- EXCLUDED -----\/-----
        for (int j = 0; j < 8; j++)
            $display("%d %08x %08x %08x %08x %08x", stage, 
                     idct_bufr1[0][j], idct_bufr1[1][j], 
                     idct_bufr1[2][j], idct_bufr1[3][j], idct_bufr1[4][j]);
 -----/\----- EXCLUDED -----/\----- */
    end

    // Translate first IDCT output from rows to columns
    if (stage >= 5 && stage <= 12) begin
        i = stage - 5;
        for (int j = 0; j < 8; j++)
            idct_bufrt[j][i] <= idct_bufr1[4][j];
    end

    // Input to second IDCT as columns
    if (stage >= 13 && stage <= 20) begin
        i = stage - 13;
/* -----\/----- EXCLUDED -----\/-----
        $display("%d %08x %08x %08x %08x %08x %08x %08x %08x", stage, 
                 idct_bufrt[i][0], idct_bufrt[i][1], idct_bufrt[i][2], 
                 idct_bufrt[i][3], idct_bufrt[i][4], idct_bufrt[i][5], 
                 idct_bufrt[i][6], idct_bufrt[i][7]);
 -----/\----- EXCLUDED -----/\----- */
        for (int j = 0; j < 8; j++)
            idct_bufr2[0][j] <= idct_bufrt[i][j];
    end

    // Second 1-D IDCT processes each column
    if (stage >= 14 && stage <= 24) begin
        for (int s = 0; s < 4; s++) begin
        logic [7:0] [IDW-1:0] c_out;
            IDCT_1D(s, idct_bufr2[s], c_out, `EFF_RSHIFT_1D_POST);
            idct_bufr2[s+1] <= c_out;
        end
    end

    if (stage >= 13 && stage <= 24) begin
/* -----\/----- EXCLUDED -----\/-----
        for (int j = 0; j < 8; j++)
            $display("%d %08x %08x %08x %08x %08x", stage,
                     idct_bufr2[0][j], idct_bufr2[1][j], 
                     idct_bufr2[2][j], idct_bufr2[3][j], idct_bufr2[4][j]);
 -----/\----- EXCLUDED -----/\----- */
    end

    // Output from second IDCT as columns
    if (stage >= 18 && stage <= 25) begin
        i = stage - 18;
        for (int j = 0; j < 8; j++) begin
`ifdef TB_VPU
            bufro[j][i] <= idct_bufr2[4][j];
`endif
            dct_idtbl[j][i] <= idct_clamp(idct_bufr2[4][j] + IDW'(128));
        end
    end

`ifdef TB_VPU
    if (stage == 26) begin
        for (int i = 0; i < 8; i++)
            for (int j = 0; j < 8; j++)
                $fdisplay(fout, "%08x", bufro[i][j]);
    end
`endif
endtask

`ifdef TB_VPU
final
    $fclose(fout);
`endif

task IDCT_1D(input int                 step,
             input [7:0] [IDW-1:0]     c_in,
             output [7:0] [IDW-1:0]    c_out,
             input int                 psh);

const static logic signed [IDW-1:0] coeffs [9] = '{
    `C_COEFF(  581104888), //  0.5411961001461970 * 2^30 + 0.5
    `C_COEFF(-1984016189), // -1.8477590650225736 * 2^30 + 0.5
    `C_COEFF(  821806413), //  0.7653668647301796 * 2^30 + 0.5

    `C_COEFF( -596538995), // -0.5555702330196022 * 2^30 + 0.5
    `C_COEFF( 1489322693), //  1.3870398453221474 * 2^30 + 0.5
    `C_COEFF(  296244703), //  0.2758993792829430 * 2^30 + 0.5

    `C_COEFF(  209476638), //  0.1950903220161282 * 2^30 + 0.5
    `C_COEFF(  843633538), //  0.7856949583871022 * 2^30 + 0.5
    `C_COEFF(-1262586814)  // -1.1758756024193586 * 2^30 + 0.5
};
logic signed [IDW-1:0] c [8];

    for (int i = 0; i < 8; i++)
        c[i] = c_in[i];

    if (step == 0) begin
    logic signed [IDW-1:0] m;
        if (psh == 0) begin
            c_out[0] = c[0] << (`IDCT_PRESHIFT - `EFF_RSHIFT_1D_COEFF);
            c_out[4] = c[4] << (`IDCT_PRESHIFT - `EFF_RSHIFT_1D_COEFF);

            c_out[7] = (c[7] + c[1]) << `IDCT_PRESHIFT;
            c_out[1] = (c[7] - c[1]) << `IDCT_PRESHIFT;

            c_out[3] = IDW'((33'(46341) * c[5]) >>> (15 - `IDCT_PRESHIFT));
            c_out[5] = IDW'((33'(46341) * c[3]) >>> (15 - `IDCT_PRESHIFT));

            m = IDW'(33'(35468) * IDW'(c[2] + c[6]));
            c_out[2] = IDW'((34'(-121095) * c[6] + 34'(m))
                            >>> (16 - `IDCT_PRESHIFT + `EFF_RSHIFT_1D_COEFF));
            c_out[6] = IDW'((34'(  50159) * c[2] + 34'(m))
                            >>> (16 - `IDCT_PRESHIFT + `EFF_RSHIFT_1D_COEFF));
        end
        else begin
            c_out[0] = (c[0] >>> `EFF_RSHIFT_1D_COEFF) + IDW'((1 << psh) >>> 1);
            c_out[4] = c[4] >>> `EFF_RSHIFT_1D_COEFF;

            c_out[7] = c[7] + c[1];
            c_out[1] = c[7] - c[1];

            c_out[3] = IDW'((c[5] * (IDW+8)'(181)) >>> 7);
            c_out[5] = IDW'((c[3] * (IDW+8)'(181)) >>> 7);

            m = IDW'(((IDW*2)'(coeffs[0]) * IDW'(c[2] + c[6])) >>> IDW);
            c_out[2] = IDW'(((IDW*2)'(coeffs[1]) * c[6]) >>> IDW) + m;
            c_out[6] = IDW'(((IDW*2)'(coeffs[2]) * c[2]) >>> IDW) + m;
        end
    end

    if (step == 1) begin
        {c_out[0], c_out[4]} = {c[0] + c[4], c[0] - c[4]};
        {c_out[7], c_out[5]} = {c[7] + c[5], c[7] - c[5]};
        {c_out[3], c_out[1]} = {c[3] + c[1], c[3] - c[1]};
        {c_out[2], c_out[6]} = {c[2], c[6]};
    end
    
    if (step == 2) begin
    logic signed [IDW-1:0] m1, r1;
    logic signed [IDW-1:0] m2, r2;
        m1 = IDW'(((IDW*2)'(coeffs[3]) * IDW'(c[7] + c[1])) >>> IDW);
        r1 = IDW'(((IDW*2)'(coeffs[4]) * c[1]) >>> IDW) + m1;
        c_out[1] = IDW'(((IDW*2)'(coeffs[5]) * c[7]) >>> IDW) - m1;
        c_out[7] = r1;

        m2 = IDW'(((IDW*2)'(coeffs[6]) * IDW'(c[3] + c[5])) >>> IDW);
        r2 = IDW'(((IDW*2)'(coeffs[7]) * c[5]) >>> IDW) + m2;
        c_out[5] = IDW'(((IDW*2)'(coeffs[8]) * c[3]) >>> IDW) + m2;
        c_out[3] = r2;

        {c_out[0], c_out[6]} = {c[0] + c[6], c[0] - c[6]};
        {c_out[4], c_out[2]} = {c[4] + c[2], c[4] - c[2]};
    end

    if (step == 3) begin
        c_out[0] = (c[0] + c[1]) >>> psh;
        c_out[1] = (c[4] + c[5]) >>> psh;
        c_out[2] = (c[2] + c[3]) >>> psh;
        c_out[3] = (c[6] + c[7]) >>> psh;
        c_out[4] = (c[6] - c[7]) >>> psh;
        c_out[5] = (c[2] - c[3]) >>> psh;
        c_out[6] = (c[4] - c[5]) >>> psh;
        c_out[7] = (c[0] - c[1]) >>> psh;
    end
endtask

//////////////////////////////////////////////////////////////////////
// Video output

localparam [9:0] VO_HACT_START = 10'd62;
localparam [9:0] VO_HACT_END   = VO_HACT_START + 10'd256;
localparam [9:0] VO_RACT_START = VO_HACT_START - 10'd1;

logic [12:0]    vo_rcnt, vo_raddr;
logic [7:0]     vo_rdata;
logic [31:0]    vo_rbuf1, vo_rbuf2, vo_rbuf2_d;
logic           vo_ract, vo_rtrg;
logic           vo_hact;
logic           vo_rre;
logic [23:0]    vo_vd_p, vo_vd;
logic           vo_roff_en;
logic [2:0]     vo_roff;

wire vo_valid = dec_valid[~rbsel];
wire vo_vdmode = dec_vdmode[~rbsel];
wire vo_ract_pos = (h_cnt == VO_RACT_START);
wire vo_ract_neg = vo_rtrg & &vo_rcnt[8:0];
wire vo_hact_p = (h_cnt >= VO_HACT_START) & (h_cnt < VO_HACT_END);

always @(posedge CLK) begin
    if (~RESn | ~vo_valid) begin
        vo_rcnt <= '0;
        vo_hact <= '0;
        vo_ract <= '0;
        vo_rtrg <= '0;
        vo_roff_en <= '0;
        vo_roff <= '0;
    end
    else begin
        if (DCK) begin
            vo_hact <= vo_hact_p;
        end
        if (CE) begin
            vo_ract <= (vo_ract & ~vo_ract_neg) | vo_ract_pos;
            if (vo_ract) begin
                vo_rtrg <= ~vo_rtrg;
                if (vo_rtrg)
                    vo_rcnt <= vo_rcnt + 1'd1;
            end
            else
                vo_rtrg <= '0;

            if (~vo_roff_en)
                vo_roff_en <= vo_hact & vo_rtrg;
            else if (vo_roff_en)
                vo_roff_en <= vo_hact;

            if (vo_roff_en)
                vo_roff <= vo_roff + 1'd1;
            else
                vo_roff <= '0;
        end
    end
end

assign vo_rre = vo_rtrg;

always @* begin
    vo_raddr = vo_rcnt;
    if (vo_vdmode)
        // U/V are only stored in even rows.
        if (vo_raddr[1])
            vo_raddr[9] = 1'b0; // matches 1'b0 in dct_raddr above
end

always @(posedge CLK) if (CE) begin
    vo_rbuf2_d <= vo_rbuf2;

    if (~vo_valid) begin
        vo_rbuf1 <= '0;
    end
    else if (vo_rtrg) begin
        vo_rbuf1 <= {vo_rbuf1[23:0], vo_rdata};
    end
end

always @* begin
    vo_rbuf2 = vo_rbuf2_d;
    if (~|vo_rcnt[1:0])
        vo_rbuf2 = vo_rbuf1;
end

always @* begin
    vo_vd_p = '0;
    if (~vo_vdmode) begin // palette
        case (vo_roff[2])
            1'b0: vo_vd_p[15:0] = vo_rbuf2[16+:16];
            1'b1: vo_vd_p[15:0] = vo_rbuf2[00+:16];
        endcase
    end
    else begin // YUV
        case (vo_roff[2])
            1'b0: vo_vd_p[16+:8] = vo_rbuf2[24+:8];
            1'b1: vo_vd_p[16+:8] = vo_rbuf2[16+:8];
        endcase
        vo_vd_p[08+:8] = vo_rbuf2[08+:8]; // U
        vo_vd_p[00+:8] = vo_rbuf2[00+:8]; // V
    end
end

wire dbg_to_disp = dbg_to[~rbsel];

always @(posedge CLK) if (DCK) begin
    vo_vd <= '0;
    if (vo_hact)
        vo_vd <= vo_vd_p;
`ifdef RAINBOW_DEBUG
    if (dbg_to_disp & vo_hact_p)
        vo_vd <= {8'd150, 8'd255, 8'd255};
`endif
end

`ifdef RAINBOW_DEBUG
assign VDMODE = (vo_vdmode & vo_valid) | dbg_to_disp;
`else
assign VDMODE = vo_vdmode & vo_valid;
`endif
assign VD = vo_vd;

//////////////////////////////////////////////////////////////////////
// R-RAM memory interface MUX

logic [12:0]    dec_raddr;
logic [7:0]     dec_rdata;
logic           dec_rwe;

assign dec_raddr = si_hdr_dct ? dct_raddr : rle_raddr;
assign dec_rdata = si_hdr_dct ? dct_rdata : rle_rdata;
assign dec_rwe   = si_hdr_dct ? dct_rwe   : rle_rwe;

always @* begin
    if (~rbsel) begin
        // Decode to A, output from B
        RA_A = dec_raddr;
        RA_DO = dec_rdata;
        RA_OEn = '1;
        RA_WEn = ~dec_rwe;

        RB_A = vo_raddr;
        RB_DO = '0;
        RB_OEn = ~vo_rre;
        RB_WEn = '1;
        vo_rdata = RB_DI;
    end
    else begin
        // Decode to B, output from A
        RB_A = dec_raddr;
        RB_DO = dec_rdata;
        RB_OEn = '1;
        RB_WEn = ~dec_rwe;

        RA_A = vo_raddr;
        RA_DO = '0;
        RA_OEn = ~vo_rre;
        RA_WEn = '1;
        vo_rdata = RA_DI;
    end
end

`ifdef VERILATOR
// Simulation log: RAINBOW strip state at each 16-line window end, printed
// only when it changes.
logic [6:0] dbg_rb_last = '1;
always @(posedge CLK) begin
    if (dec_resn & HSYNC_NEGEDGE & (v_cnt == 4'd15) &
        ({rb_run, dec_act, si_end, si_hdr} != dbg_rb_last)) begin
        dbg_rb_last <= {rb_run, dec_act, si_end, si_hdr};
        $display("%t: RB act=%b end=%b hdr=%x", $time, dec_act, si_end, si_hdr);
    end
    if (~dec_resn & (dbg_rb_last[6] | (dbg_rb_last == '1))) begin
        dbg_rb_last <= '0;
        $display("%t: RB reset", $time);
    end
end
`endif

//////////////////////////////////////////////////////////////////////
// Debug counters (free running)

logic [7:0]     dbg_win_cnt, dbg_valid_cnt, dbg_hdr_cnt;
logic           dbg_hdr_d;

always @(posedge CLK) begin
    if (~RESn) begin
        dbg_win_cnt <= '0;
        dbg_valid_cnt <= '0;
        dbg_hdr_cnt <= '0;
        dbg_hdr_d <= '0;
    end
    else begin
        dbg_hdr_d <= si_hdr_det;
        if (si_hdr_det & ~dbg_hdr_d)
            dbg_hdr_cnt <= dbg_hdr_cnt + 1'd1;
        if (dec_resn & HSYNC_NEGEDGE & (v_cnt == 4'd15)) begin
            dbg_win_cnt <= dbg_win_cnt + 1'd1;
            if (dec_act & si_end)
                dbg_valid_cnt <= dbg_valid_cnt + 1'd1;
        end
    end
end

assign DBG = {dbg_win_cnt, dbg_valid_cnt, dbg_hdr_cnt, rb_run, 4'(dcts), 3'(sist)};

endmodule
