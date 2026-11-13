// HuC6230 (SOUNDBOX)
//
// Copyright (c) 2026 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

// References:
// - https://github.com/MiSTer-devel/TurboGrafx16_MiSTer/blob/master/rtl/HUC6280/psg.vhd
// - PC-FXGA Authoring Software / GMAKER Starter Kit Plus (Ver. 1.0) / Device Description: HuC6230
// - https://github.com/mamedev/mame/blob/master/src/devices/sound/huc6230.cpp
//   (line input level, VCA step behavior)

module huc6230
   (
    input         CLK,
    input         CE,
    input         RESn,

    // CPU memory / I/O bus interface
    input [5:1]   A,
    input [7:0]   DI,
    input         CSn,
    input         WRn,

    // K-BUS interface
    input [7:0]   KBUS_DI,
    input         KBUS_RHnL,
    input [1:0]   KBUS_CSn,

    // Video timing sync. interface
    input         DCK, // pixel clock enable
    input         HSYNC_NEGEDGE,

    // Line input (CD-DA)
    input signed [15:0] LINE_L,
    input signed [15:0] LINE_R,

    // Audio interface (signed)
    output signed [15:0] SLOUT, // left channel
    output signed [15:0] SROUT // right channel
    );

typedef struct packed {
    logic [2:1]    sres;
    logic [2:1]    interp;
    logic [1:0]    div;
} adp_cr_t;

logic               hsync_negedge, hsync_negedge_d;

adp_cr_t            adp_cr, adp_cr_next;

//////////////////////////////////////////////////////////////////////
// I/O bus / Register interface

// R11-R14: ADPCM #1 L, #1 R, #2 L, #2 R volume. R15, R16: PCM (line) L, R
// volume. All are cleared at reset (fully attenuated).
logic [5:0]         vol [1:6];

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        adp_cr_next <= '0;
        for (int i = 1; i <= 6; i++)
            vol[i] <= '0;
    end
    else begin
        if (~CSn & ~WRn) begin
            case (A)
                5'h10: adp_cr_next <= DI[5:0];
                5'h11, 5'h12, 5'h13, 5'h14, 5'h15, 5'h16:
                    vol[A[3:1]] <= DI[5:0];
                default: ;
            endcase
        end
    end
end

always @(posedge CLK) begin
    if (~RESn) begin
        adp_cr <= '0;
    end
    else if (hsync_negedge) begin
        // This register becomes effective on -HSYNC.
        adp_cr <= adp_cr_next;
    end
end

//////////////////////////////////////////////////////////////////////
// ADPCM sample clock generator

// Assumes CLK = 2 * 21.48 MHz
localparam [11:0] PCM_CLOCKS = 12'(2730 / 2); // 31.75 kHz

logic           res_hsync;
logic [11:0]    p_cnt;
logic           pck;
logic [3:0]     adp_cnt;
logic           adpck, adpck_div2;

assign hsync_negedge = DCK & HSYNC_NEGEDGE;

wire p_wrap = p_cnt == (PCM_CLOCKS - 1'd1);

always @(posedge CLK) begin
    hsync_negedge_d <= hsync_negedge;
    res_hsync <= (res_hsync & ~hsync_negedge_d) | ~RESn;
    p_cnt <= (res_hsync | pck) ? '0 : p_cnt + 1'd1;
    pck <= hsync_negedge | p_wrap;
end

always @(posedge CLK) begin
    if (res_hsync)
        adp_cnt <= '0;
    else if (pck)
        adp_cnt <= adp_cnt + 1'd1;
end

// adpck = pck / 2^DIV
wire [3:0] adp_mask = 4'((2 << adp_cr.div) - 1);
assign adpck = pck & ({adp_cnt[2:0], 1'b1} & adp_mask) == adp_mask;

// adpck_div2 = adpck / 2
assign adpck_div2 = pck & (adp_cnt & adp_mask) == adp_mask;

//////////////////////////////////////////////////////////////////////
// ADPCM decoders

logic [15:0]    adpcm1_sout, adpcm2_sout;

wire adpcm1_csn = KBUS_CSn[0] | ~KBUS_CSn[1];
wire adpcm2_csn = KBUS_CSn[1] | ~KBUS_CSn[0];

huc6230_adpcm adpcm1
   (
    .CLK(CLK),
    .RESn(RESn),

    .SRES(adp_cr.sres[1]),
    .INTERP(adp_cr.interp[1]),
    .DIV(adp_cr.div),

    .KBUS_DI(KBUS_DI),
    .KBUS_RHnL(KBUS_RHnL),
    .KBUS_CSn(adpcm1_csn),

    .SCK_ADPCM_DIV2(adpck_div2),
    .SCK_ADPCM(adpck),
    .SCK_PCM(pck),
    .SOUT(adpcm1_sout)
    );

huc6230_adpcm adpcm2
   (
    .CLK(CLK),
    .RESn(RESn),

    .SRES(adp_cr.sres[2]),
    .INTERP(adp_cr.interp[2]),
    .DIV(adp_cr.div),

    .KBUS_DI(KBUS_DI),
    .KBUS_RHnL(KBUS_RHnL),
    .KBUS_CSn(adpcm2_csn),

    .SCK_ADPCM_DIV2(adpck_div2),
    .SCK_ADPCM(adpck),
    .SCK_PCM(pck),
    .SOUT(adpcm2_sout)
    );

//////////////////////////////////////////////////////////////////////
// Line input volume (VCA)
//
// The level (0-1023) moves one step at a time towards 16n+15 when rising
// or 16n when falling (n = register value, 3Fh = 1023). The step time
// grows as the level falls: 1.49 us per step at 512 and up, doubling for
// each lower octave; the step to or from 0 takes 2956.16 us.

function automatic [16:0] vca_step_time(input [9:0] level);
    // in CLK cycles (42.95 MHz)
    if (level >= 10'd512) return 17'd64;
    if (level >= 10'd256) return 17'd128;
    if (level >= 10'd128) return 17'd256;
    if (level >= 10'd64)  return 17'd512;
    if (level >= 10'd32)  return 17'd1024;
    if (level >= 10'd16)  return 17'd2048;
    return 17'd126980;
endfunction

logic [9:0]         vca_level [2];

genvar ch;
generate
    for (ch = 0; ch < 2; ch++) begin : g_vca
        logic [5:0]  v;
        logic [9:0]  up, down, target, next;
        logic [16:0] wait_cnt;
        logic        moving;

        assign v = vol[5 + ch];
        assign up = (v == 6'h3f) ? 10'd1023 : (v != 0) ? {v, 4'hf} : 10'd0;
        assign down = (v == 6'h3f) ? 10'd1023 : {v, 4'h0};
        assign target = (up > vca_level[ch]) ? up :
                        (down < vca_level[ch]) ? down : vca_level[ch];

        always @* begin
            if (target > vca_level[ch])
                next = (vca_level[ch] < 10'd16) ? ((target < 10'd16) ? target : 10'd16)
                                                : vca_level[ch] + 1'd1;
            else
                next = (vca_level[ch] < 10'd16) ? target : vca_level[ch] - 1'd1;
        end

        always @(posedge CLK) begin
            if (~RESn) begin
                vca_level[ch] <= '0;
                moving <= '0;
            end
            else if (target == vca_level[ch])
                moving <= '0;
            else if (~moving) begin
                moving <= '1;
                wait_cnt <= vca_step_time((target > vca_level[ch])
                                          ? vca_level[ch] + 1'd1 : vca_level[ch]);
            end
            else if (wait_cnt > 17'd1)
                wait_cnt <= wait_cnt - 1'd1;
            else begin
                vca_level[ch] <= next;
                wait_cnt <= vca_step_time((target > next) ? next + 1'd1 : next);
            end
        end
    end
endgenerate

//////////////////////////////////////////////////////////////////////
// Output mixer
//
// Mixing ratio ADPCM#1 : ADPCM#2 : LINE : PSG = 1 : 1 : 1 : 2 (HuC6230
// manual, chapter 6). Each input's full scale is half of the output's.
// ADPCM volume: 3Fh = 0 dB, -1.5 dB per step, 1Bh and below mute.
// TODO: PSG

// Gain, 1.0 = 65536
function automatic [16:0] adpcm_gain(input [5:0] v);
    case (v)
        6'h1c: return 17'd155;
        6'h1d: return 17'd185;
        6'h1e: return 17'd220;
        6'h1f: return 17'd261;
        6'h20: return 17'd310;
        6'h21: return 17'd369;
        6'h22: return 17'd438;
        6'h23: return 17'd521;
        6'h24: return 17'd619;
        6'h25: return 17'd735;
        6'h26: return 17'd874;
        6'h27: return 17'd1039;
        6'h28: return 17'd1234;
        6'h29: return 17'd1467;
        6'h2a: return 17'd1744;
        6'h2b: return 17'd2072;
        6'h2c: return 17'd2463;
        6'h2d: return 17'd2927;
        6'h2e: return 17'd3479;
        6'h2f: return 17'd4135;
        6'h30: return 17'd4915;
        6'h31: return 17'd5841;
        6'h32: return 17'd6942;
        6'h33: return 17'd8250;
        6'h34: return 17'd9806;
        6'h35: return 17'd11654;
        6'h36: return 17'd13851;
        6'h37: return 17'd16462;
        6'h38: return 17'd19565;
        6'h39: return 17'd23253;
        6'h3a: return 17'd27636;
        6'h3b: return 17'd32846;
        6'h3c: return 17'd39037;
        6'h3d: return 17'd46396;
        6'h3e: return 17'd55142;
        6'h3f: return 17'd65536;
        default: return 17'd0;
    endcase
endfunction

logic signed [15:0] a1, a2;
logic signed [34:0] m_a1l, m_a1r, m_a2l, m_a2r;
logic signed [27:0] m_ll, m_lr;
logic signed [18:0] mix_l, mix_r;
logic signed [15:0] out_l, out_r;

// ADPCM decoder output is offset binary.
assign a1 = {~adpcm1_sout[15], adpcm1_sout[14:0]};
assign a2 = {~adpcm2_sout[15], adpcm2_sout[14:0]};

function automatic signed [15:0] clamp16(input signed [18:0] x);
    if (x > 19'sd32767)
        return 16'sh7fff;
    if (x < -19'sd32768)
        return 16'sh8000;
    return 16'(x);
endfunction

always @(posedge CLK) begin
    m_a1l <= a1 * $signed({1'b0, adpcm_gain(vol[1])});
    m_a1r <= a1 * $signed({1'b0, adpcm_gain(vol[2])});
    m_a2l <= a2 * $signed({1'b0, adpcm_gain(vol[3])});
    m_a2r <= a2 * $signed({1'b0, adpcm_gain(vol[4])});
    m_ll <= LINE_L * $signed({1'b0, vca_level[0]});
    m_lr <= LINE_R * $signed({1'b0, vca_level[1]});

    // Each product at full scale is 2^15 * 2^16 (ADPCM) or 2^15 * 2^10
    // (line). Scale both to 2^14, half of the output's full scale.
    mix_l <= 19'(m_a1l >>> 17) + 19'(m_a2l >>> 17) + 19'(m_ll >>> 11);
    mix_r <= 19'(m_a1r >>> 17) + 19'(m_a2r >>> 17) + 19'(m_lr >>> 11);

    out_l <= clamp16(mix_l);
    out_r <= clamp16(mix_r);
end

assign SLOUT = out_l;
assign SROUT = out_r;

endmodule

`include "huc6230_adpcm.sv"
