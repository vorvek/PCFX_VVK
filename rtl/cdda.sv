// CD-DA player for the emulated CD drive
//
// Copyright (c) 2026 vorvek
//
// This program is GPL licensed. See COPYING for the full license.

// Plays audio sectors that the HPS reads from the CD image, and keeps
// the track table that the audio commands need.
//
// - Sector port arbiter: fake_cd (data commands) has priority. The
//   player and the TOC loader use the port only while fake_cd is idle,
//   because fake_cd reads the shared sector buffer after its own read.
// - TOC: loaded after each mount from the HPS TOC sector (LBA -65536,
//   READ TOC format, binary M:S:F). The HPS gives no lead-out entry; it
//   mounts the image with size = lead-out LBA * 2352.
// - Player: fetches 2352-byte sectors into a 2048-frame FIFO and outputs
//   one stereo frame per 44.1 kHz tick.
//
// References:
// - MAME src/devices/bus/nscsi/pcfxcd.cpp (NEC CD-ROM DRIVE:FX)
// - Main_MiSTer pcfx branch, support/pcfx/pcfx.cpp (TOC and sector format)

module cdda
  #(parameter int CLK_HZ = 42954545)
   (
    input                CLK,
    input                RESn,

    // Disc
    input                CD_EN,       // a disc is mounted
    input                MOUNT,       // pulse: an image was (un)mounted
    input [31:0]         IMG_SIZE,    // valid with MOUNT: lead-out * 2352

    // Control
    input                PLAY,        // pulse: play PLAY_START..PLAY_END-1
    input                PLAY_PAUSE,  // with PLAY: seek, then pause (status 2)
    input [18:0]         PLAY_START,
    input [18:0]         PLAY_END,
    input                PLAY_REPEAT,
    input                PLAY_MUTE,
    input                PAUSE,       // pulse: stop output, keep position
    input [1:0]          PAUSE_STATUS,// status code for PAUSE (1 or 2)
    input                RESUME,      // pulse
    input                STOP,        // pulse

    output reg [1:0]     STATUS,      // 0 playing, 1 still, 2 paused, 3 stopped
    output reg [18:0]    POS_LBA,     // play position
    output reg [18:0]    START_LBA,   // start of the last play

    // Track table
    output reg           TOC_VALID,
    output reg [6:0]     FIRST_TRACK,
    output reg [6:0]     LAST_TRACK,
    output reg [18:0]    LEADOUT,
    input [6:0]          TT_TRACK,    // lookup: 1-clock latency
    output [18:0]        TT_START,
    output [3:0]         TT_CONTROL,
    output reg [6:0]     Q_TRACK,     // track at the Q position
    output reg [18:0]    Q_TRACK_START,
    output reg [3:0]     Q_CONTROL,

    // Sector port from fake_cd
    input [31:0]         A_SD_LBA,
    input                A_SD_RD,
    output               A_SD_ACK,
    input [12:0]         A_SDBUF_ADDR,
    input                A_BUSY,      // fake_cd command in progress

    // Sector port to the HPS
    output [31:0]        SD_LBA,
    output               SD_RD,
    input                SD_ACK,
    output [12:0]        SDBUF_ADDR,
    input [15:0]         SDBUF_DOUT,

    // Audio
    output signed [15:0] OUT_L,
    output signed [15:0] OUT_R
    );

localparam [1:0] ST_PLAYING = 2'd0;
localparam [1:0] ST_STOPPED = 2'd3;

localparam int SECTOR_FRAMES = 588;     // 2352 bytes / 4
localparam int SECTOR_WORDS = 1176;
localparam int FIFO_AW = 11;            // 2048 frames

//////////////////////////////////////////////////////////////////////
// 44.1 kHz tick

logic [25:0] tick_acc;
logic        tick;

always @(posedge CLK) begin
    tick <= '0;
    if (tick_acc >= 26'(CLK_HZ - 44100)) begin
        tick_acc <= tick_acc - 26'(CLK_HZ - 44100);
        tick <= '1;
    end
    else
        tick_acc <= tick_acc + 26'd44100;
end

//////////////////////////////////////////////////////////////////////
// Lead-out: IMG_SIZE / 2352, by restoring division

logic [31:0] div_n;
logic [12:0] div_r;
logic [18:0] div_q;
logic [5:0]  div_i;
logic        div_run;
logic [13:0] div_r2;

assign div_r2 = {div_r, div_n[31]};

always @(posedge CLK) begin
    if (MOUNT) begin
        div_n <= IMG_SIZE;
        div_r <= '0;
        div_q <= '0;
        div_i <= 6'd32;
        div_run <= '1;
    end
    else if (div_run) begin
        if (div_i == 0) begin
            LEADOUT <= div_q;
            div_run <= '0;
        end
        else begin
            div_n <= div_n << 1;
            div_r <= (div_r2 >= 14'd2352) ? 13'(div_r2 - 14'd2352) : div_r2[12:0];
            div_q <= {div_q[17:0], (div_r2 >= 14'd2352)};
            div_i <= div_i - 1'd1;
        end
    end
end

//////////////////////////////////////////////////////////////////////
// Track table: start LBA and control for tracks 0..127

logic [22:0] tt_mem_a [128];  // lookup copy
logic [22:0] tt_mem_b [128];  // scanner copy
logic        tt_we;
logic [6:0]  tt_wa;
logic [22:0] tt_wd;
logic [22:0] tt_qa, tt_qb;
logic [6:0]  tt_rb;

always @(posedge CLK) begin
    if (tt_we) begin
        tt_mem_a[tt_wa] <= tt_wd;
        tt_mem_b[tt_wa] <= tt_wd;
    end
    tt_qa <= tt_mem_a[TT_TRACK];
    tt_qb <= tt_mem_b[tt_rb];
end

assign TT_START = (TT_TRACK == LAST_TRACK + 1'd1) ? LEADOUT : tt_qa[18:0];
assign TT_CONTROL = tt_qa[22:19];

//////////////////////////////////////////////////////////////////////
// Sector port arbiter and readers

typedef enum bit [2:0] {
    PS_IDLE,
    PS_A,           // fake_cd owns the port
    PS_REQ,         // our read: wait for ACK
    PS_XFER,        // HPS writes the sector buffer
    PS_COPY,        // copy audio frames into the FIFO
    PS_TOC          // parse the TOC sector
} ps_t;

ps_t         ps;
logic        p_toc;         // the current read is the TOC
logic [31:0] p_lba;
logic        p_rd;
logic [10:0] cp_addr;       // sector buffer word address
logic [12:0] p_bufaddr;
logic [15:0] cp_lo;
logic        sd_ack_d;

wire a_own = (ps == PS_A) | ((ps == PS_IDLE) & A_SD_RD);

assign SD_LBA = a_own ? A_SD_LBA : p_lba;
assign SD_RD = a_own ? A_SD_RD : p_rd;
assign A_SD_ACK = a_own & SD_ACK;
assign SDBUF_ADDR = ((ps == PS_COPY) | (ps == PS_TOC)) ? p_bufaddr : A_SDBUF_ADDR;

// FIFO
logic [31:0]        fifo_mem [1 << FIFO_AW];
logic [FIFO_AW-1:0] fifo_wa, fifo_ra;
logic [FIFO_AW:0]   fifo_cnt;
logic               fifo_we, fifo_re, fifo_flush;
logic [31:0]        fifo_wd, fifo_q;

always @(posedge CLK) begin
    if (fifo_we)
        fifo_mem[fifo_wa] <= fifo_wd;
    fifo_q <= fifo_mem[fifo_ra];
end

always @(posedge CLK) begin
    if (~RESn | fifo_flush) begin
        fifo_wa <= '0;
        fifo_ra <= '0;
        fifo_cnt <= '0;
    end
    else begin
        if (fifo_we)
            fifo_wa <= fifo_wa + 1'd1;
        if (fifo_re)
            fifo_ra <= fifo_ra + 1'd1;
        fifo_cnt <= fifo_cnt + (FIFO_AW+1)'(fifo_we) - (FIFO_AW+1)'(fifo_re);
    end
end

wire fifo_room = fifo_cnt <= (FIFO_AW+1)'((1 << FIFO_AW) - SECTOR_FRAMES);

// Player state
logic [18:0] fetch_lba, end_lba;
logic        repeat_en, mute;
logic [9:0]  frame_cnt;
logic        fetch_seq;     // fetches belong to the current play
logic        toc_load;      // TOC load pending
logic [6:0]  toc_n;         // entries parsed
logic [2:0]  toc_w;         // word within an 8-byte entry
logic [7:0]  toc_ctl, toc_trk, toc_m, toc_s;
logic [31:0] out_frame;

wire want_audio = (STATUS != ST_STOPPED) & (fetch_lba < end_lba) & fifo_room;

always @(posedge CLK) begin
    sd_ack_d <= SD_ACK;
    fifo_we <= '0;
    fifo_flush <= '0;
    tt_we <= '0;

    if (~RESn) begin
        ps <= PS_IDLE;
        p_rd <= '0;
        STATUS <= ST_STOPPED;
        POS_LBA <= '0;
        START_LBA <= '0;
        fetch_lba <= '0;
        end_lba <= '0;
        repeat_en <= '0;
        mute <= '0;
        frame_cnt <= '0;
        TOC_VALID <= '0;
        toc_load <= '0;
    end
    else begin
        if (MOUNT | ~CD_EN) begin
            TOC_VALID <= '0;
            toc_load <= CD_EN | MOUNT;
            STATUS <= ST_STOPPED;
        end

        case (ps)
            PS_IDLE:
                if (A_SD_RD)
                    ps <= PS_A;
                else if (~A_BUSY & toc_load & CD_EN & ~MOUNT & ~div_run) begin
                    p_toc <= '1;
                    p_lba <= 32'hffff0000;  // -65536: TOC from track 0
                    p_rd <= '1;
                    toc_load <= '0;
                    ps <= PS_REQ;
                end
                else if (~A_BUSY & want_audio) begin
                    p_toc <= '0;
                    p_lba <= 32'(fetch_lba);
                    p_rd <= '1;
                    fetch_seq <= '1;
                    ps <= PS_REQ;
                end
            PS_A:
                if (sd_ack_d & ~SD_ACK)
                    ps <= PS_IDLE;
            PS_REQ:
                if (SD_ACK) begin
                    p_rd <= '0;
                    ps <= PS_XFER;
                end
            PS_XFER:
                if (~SD_ACK) begin
                    cp_addr <= '0;
                    p_bufaddr <= '0;
                    toc_n <= '0;
                    toc_w <= '0;
                    ps <= p_toc ? PS_TOC : PS_COPY;
                end
            PS_COPY: begin
                // Buffer read has 1 clock latency: address n, data n-1.
                p_bufaddr <= p_bufaddr + 1'd1;
                cp_addr <= p_bufaddr[10:0];
                if (p_bufaddr != 0) begin
                    if (cp_addr[0]) begin
                        fifo_wd <= {SDBUF_DOUT, cp_lo};  // {R, L}
                        fifo_we <= fetch_seq;
                    end
                    else
                        cp_lo <= SDBUF_DOUT;
                end
                if (cp_addr == 11'(SECTOR_WORDS - 1)) begin
                    if (fetch_seq)
                        fetch_lba <= fetch_lba + 1'd1;
                    ps <= PS_IDLE;
                end
            end
            PS_TOC: begin
                // Words: 1 = {last, first}; entry i at word 2+4i:
                // +0 {adr_ctl, res}, +1 {res, track}, +2 {min, zero},
                // +3 {frame, sec}
                p_bufaddr <= p_bufaddr + 1'd1;
                cp_addr <= p_bufaddr[10:0];
                if (p_bufaddr == 13'd2) begin
                    FIRST_TRACK <= SDBUF_DOUT[6:0];
                    LAST_TRACK <= SDBUF_DOUT[14:8];
                end
                else if (p_bufaddr >= 13'd3) begin
                    toc_w <= toc_w + 1'd1;
                    case (toc_w[1:0])
                        2'd0: toc_ctl <= SDBUF_DOUT[15:8];
                        2'd1: toc_trk <= SDBUF_DOUT[7:0];
                        2'd2: toc_m <= SDBUF_DOUT[15:8];
                        2'd3: begin
                            tt_we <= (toc_trk != 0) & (toc_trk < 8'd100);
                            tt_wa <= toc_trk[6:0];
                            tt_wd <= {toc_ctl[3:0],
                                      19'(toc_m) * 19'd4500
                                      + 19'(SDBUF_DOUT[7:0]) * 19'd75
                                      + 19'(SDBUF_DOUT[15:8]) - 19'd150};
                            toc_n <= toc_n + 1'd1;
                            if (toc_n == 7'(LAST_TRACK - FIRST_TRACK)) begin
                                TOC_VALID <= '1;
                                ps <= PS_IDLE;
                            end
                        end
                    endcase
                end
                if (p_bufaddr == 13'd500) begin   // 99 entries end at word 397
                    TOC_VALID <= '1;
                    ps <= PS_IDLE;
                end
            end
            default: ps <= PS_IDLE;
        endcase

        // Output
        fifo_re <= '0;
        if (tick) begin
            if (STATUS == ST_PLAYING) begin
                if (POS_LBA >= end_lba) begin
                    if (repeat_en) begin
                        fifo_flush <= '1;
                        fetch_lba <= START_LBA;
                        POS_LBA <= START_LBA;
                        frame_cnt <= '0;
                        fetch_seq <= '0;
                    end
                    else
                        STATUS <= ST_STOPPED;
                end
                else if (fifo_cnt != 0) begin
                    fifo_re <= '1;
                    if (frame_cnt == 10'(SECTOR_FRAMES - 1)) begin
                        frame_cnt <= '0;
                        POS_LBA <= POS_LBA + 1'd1;
                    end
                    else
                        frame_cnt <= frame_cnt + 1'd1;
                end
            end
        end

        // Control
        if (PLAY) begin
            fifo_flush <= '1;
            fetch_seq <= '0;
            fetch_lba <= PLAY_START;
            end_lba <= PLAY_END;
            POS_LBA <= PLAY_START;
            START_LBA <= PLAY_START;
            frame_cnt <= '0;
            repeat_en <= PLAY_REPEAT;
            mute <= PLAY_MUTE;
            STATUS <= PLAY_PAUSE ? 2'd2 : ST_PLAYING;
        end
        else if (PAUSE)
            STATUS <= PAUSE_STATUS;
        else if (RESUME)
            STATUS <= ST_PLAYING;
        else if (STOP)
            STATUS <= ST_STOPPED;
    end
end

// fifo_re is set on a tick; the FIFO output is valid one clock later.
logic fifo_re_d;
always @(posedge CLK) begin
    fifo_re_d <= fifo_re;
    if (~RESn | (STATUS != ST_PLAYING))
        out_frame <= '0;
    else if (fifo_re_d)
        out_frame <= mute ? '0 : fifo_q;
end

assign OUT_L = out_frame[15:0];
assign OUT_R = out_frame[31:16];

//////////////////////////////////////////////////////////////////////
// Q position to track: scan the table, latch at the end of each pass

// The track at an LBA is the last track whose start is <= the LBA (the
// first track if the LBA is before it). Two clocks per entry: address,
// then compare.

logic [18:0] sc_lba;
logic        sc_cmp;     // 0: address cycle, 1: compare cycle
logic [6:0]  sc_cand;
logic [18:0] sc_cand_start;
logic [3:0]  sc_cand_ctl;

always @(posedge CLK) begin
    if (~RESn | ~TOC_VALID) begin
        tt_rb <= '0;
        sc_cmp <= '0;
        Q_TRACK <= 7'd1;
        Q_TRACK_START <= '0;
        Q_CONTROL <= '0;
    end
    else if (~sc_cmp) begin
        if (tt_rb == FIRST_TRACK)
            sc_lba <= (STATUS == ST_STOPPED) ? START_LBA : POS_LBA;
        sc_cmp <= '1;
    end
    else begin
        logic hit;
        hit = (tt_rb == FIRST_TRACK) | (tt_qb[18:0] <= sc_lba);
        sc_cmp <= '0;
        if (hit) begin
            sc_cand <= tt_rb;
            sc_cand_start <= tt_qb[18:0];
            sc_cand_ctl <= tt_qb[22:19];
        end
        if (tt_rb >= LAST_TRACK) begin
            Q_TRACK <= hit ? tt_rb : sc_cand;
            Q_TRACK_START <= hit ? tt_qb[18:0] : sc_cand_start;
            Q_CONTROL <= hit ? tt_qb[22:19] : sc_cand_ctl;
            tt_rb <= FIRST_TRACK;
        end
        else
            tt_rb <= tt_rb + 1'd1;
    end
end

endmodule
