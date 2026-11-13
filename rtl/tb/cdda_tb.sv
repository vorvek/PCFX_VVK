// CD-DA unit testbench: fake_cd + cdda + sector buffer, with a model of
// the SCSI bridge (command / data-in / status) and of the HPS sector
// reads (DPI, cd.chd through the MiSTer main code).
//
// Output: cdda.wav (44.1 kHz stereo) and a command log.

`timescale 1ns / 1ps

import "DPI-C" function bit pcfx_mount_cd();
import "DPI-C" function int pcfx_cd_img_size();
import "DPI-C" task pcfx_read_cd(bit [7:0] buffer [], input int lba,
                                 input int cnt);

module cdda_tb;

logic clk = 1;
logic resn = 0;

always #11.640 clk = ~clk;  // 42.954545 MHz

//////////////////////////////////////////////////////////////////////
// DUT

logic [95:0] command;
logic        comm_send = 0;
wire         stat_get;
wire [7:0]   status;
wire [7:0]   cd_data;
wire         cd_wr;
logic        cd_en = 0;
logic        mount = 0;
logic [31:0] img_size = 0;

wire [31:0] fcd_sd_lba, sd_lba;
wire        fcd_sd_rd, fcd_sd_ack, fcd_busy, sd_rd;
logic       sd_ack = 0;
wire [12:0] fcd_sdbuf_addr, sdbuf_addr;
wire [15:0] sdbuf_dout;

wire        au_play, au_play_pause, au_play_repeat, au_play_mute;
wire [18:0] au_play_start, au_play_end;
wire        au_pause, au_resume, au_stop;
wire [1:0]  au_pause_status, au_status;
wire [18:0] au_pos, au_start;
wire        toc_valid;
wire [6:0]  first_track, last_track, tt_track, q_track;
wire [18:0] leadout, tt_start, q_track_start;
wire [3:0]  tt_control, q_control;
wire signed [15:0] out_l, out_r;

fake_cd fake_cd
    (
     .CLK(clk),
     .RESn(resn),
     .STAT_GET(stat_get),
     .COMMAND(command),
     .COMM_SEND(comm_send),
     .DOUT_REQ(),
     .DOUT('0),
     .DOUT_SEND(1'b0),
     .STATUS(status),
     .CD_DATA(cd_data),
     .CD_WR(cd_wr),
     .CD_READY(1'b1),
     .CD_DATA_END(1'b1),
     .MSGOUT_PEND(1'b0),
     .MSGOUT('0),
     .MSGOUT_SEND(1'b0),
     .MEDIUM_EMPTY(~cd_en),
     .SD_LBA(fcd_sd_lba),
     .SD_RD(fcd_sd_rd),
     .SD_ACK(fcd_sd_ack),
     .SDBUF_ADDR(fcd_sdbuf_addr),
     .SDBUF_DOUT(sdbuf_dout),
     .BUSY(fcd_busy),
     .AU_PLAY(au_play),
     .AU_PLAY_PAUSE(au_play_pause),
     .AU_PLAY_START(au_play_start),
     .AU_PLAY_END(au_play_end),
     .AU_PLAY_REPEAT(au_play_repeat),
     .AU_PLAY_MUTE(au_play_mute),
     .AU_PAUSE(au_pause),
     .AU_PAUSE_STATUS(au_pause_status),
     .AU_RESUME(au_resume),
     .AU_STOP(au_stop),
     .AU_STATUS(au_status),
     .AU_POS(au_pos),
     .AU_START(au_start),
     .TOC_VALID(toc_valid),
     .FIRST_TRACK(first_track),
     .LAST_TRACK(last_track),
     .LEADOUT(leadout),
     .TT_TRACK(tt_track),
     .TT_START(tt_start),
     .TT_CONTROL(tt_control),
     .Q_TRACK(q_track),
     .Q_TRACK_START(q_track_start),
     .Q_CONTROL(q_control)
     );

cdda cdda
    (
     .CLK(clk),
     .RESn(resn),
     .CD_EN(cd_en),
     .MOUNT(mount),
     .IMG_SIZE(img_size),
     .PLAY(au_play),
     .PLAY_PAUSE(au_play_pause),
     .PLAY_START(au_play_start),
     .PLAY_END(au_play_end),
     .PLAY_REPEAT(au_play_repeat),
     .PLAY_MUTE(au_play_mute),
     .PAUSE(au_pause),
     .PAUSE_STATUS(au_pause_status),
     .RESUME(au_resume),
     .STOP(au_stop),
     .STATUS(au_status),
     .POS_LBA(au_pos),
     .START_LBA(au_start),
     .TOC_VALID(toc_valid),
     .FIRST_TRACK(first_track),
     .LAST_TRACK(last_track),
     .LEADOUT(leadout),
     .TT_TRACK(tt_track),
     .TT_START(tt_start),
     .TT_CONTROL(tt_control),
     .Q_TRACK(q_track),
     .Q_TRACK_START(q_track_start),
     .Q_CONTROL(q_control),
     .A_SD_LBA(fcd_sd_lba),
     .A_SD_RD(fcd_sd_rd),
     .A_SD_ACK(fcd_sd_ack),
     .A_SDBUF_ADDR(fcd_sdbuf_addr),
     .A_BUSY(fcd_busy),
     .SD_LBA(sd_lba),
     .SD_RD(sd_rd),
     .SD_ACK(sd_ack),
     .SDBUF_ADDR(sdbuf_addr),
     .SDBUF_DOUT(sdbuf_dout),
     .OUT_L(out_l),
     .OUT_R(out_r)
     );

logic [12:0] hps_addr = 0;
logic [15:0] hps_data = 0;
logic        hps_wr = 0;

dpram #(.addr_width(13), .data_width(16)) cdbuf
   (
    .clock(clk),
    .address_a(hps_addr), .data_a(hps_data), .enable_a(1'b1),
    .wren_a(hps_wr), .q_a(), .cs_a(1'b1),
    .address_b(sdbuf_addr), .data_b('0), .enable_b(1'b1),
    .wren_b(1'b0), .q_b(sdbuf_dout), .cs_b(1'b1)
    );

//////////////////////////////////////////////////////////////////////
// HPS model: answer sector reads after a delay

// A clocked state machine, not a timed process: in a timed process, the
// simulator dropped the non-blocking assignments after the DPI task call.
bit [7:0] secbuf [2352];
int       sd_reads = 0;
int       hps_st = 0;
int       hps_cnt = 0;

always @(posedge clk) begin
    case (hps_st)
        0: if (sd_rd) begin
            hps_cnt <= 2000;            // HPS latency (about 47 us)
            hps_st <= 1;
        end
        1: if (hps_cnt == 0) begin
            sd_ack <= 1;
            pcfx_read_cd(secbuf, sd_lba, 1);
            sd_reads <= sd_reads + 1;
            hps_cnt <= 0;
            hps_st <= 2;
        end
        else
            hps_cnt <= hps_cnt - 1;
        2: begin
            hps_addr <= 13'(hps_cnt);
            hps_data <= {secbuf[2*hps_cnt+1], secbuf[2*hps_cnt]};
            hps_wr <= 1;
            hps_cnt <= hps_cnt + 1;
            if (hps_cnt == 1175)
                hps_st <= 3;
        end
        3: begin
            hps_wr <= 0;
            sd_ack <= 0;
            hps_st <= 4;
        end
        default:
            hps_st <= 0;
    endcase
end

//////////////////////////////////////////////////////////////////////
// SCSI bridge model

bit [7:0] din [256];
int       din_n;

always @(posedge clk)
    if (cd_wr) begin
        din[din_n[7:0]] <= cd_data;
        din_n <= din_n + 1;
    end

task scsi(input bit [7:0] cdb [12]);
    for (int i = 0; i < 12; i++)
        command[i*8+:8] = cdb[i];
    din_n = 0;
    $display("[%0t] send %02x", $time, cdb[0]);
    @(posedge clk);
    comm_send <= 1;
    @(posedge clk);
    comm_send <= 0;
    @(posedge clk iff stat_get);
    @(posedge clk);
    $write("cmd %02x: status %02x data", cdb[0], status);
    for (int i = 0; i < din_n && i < 40; i++)
        $write(" %02x", din[i]);
    $write("\n");
endtask

function automatic bit [7:0] bcd(input int v);
    return 8'(((v / 10) << 4) | (v % 10));
endfunction

task play_tracks_nec(input int t1, input int t2);
    // D8: start at BCD track t1, paused; D9: end at BCD track t2, stereo
    scsi('{8'hd8, 8'h00, bcd(t1), 0, 0, 0, 0, 0, 0, 8'h80, 0, 0});
    scsi('{8'hd9, 8'h03, bcd(t2), 0, 0, 0, 0, 0, 0, 8'h80, 0, 0});
endtask

task wait_ms(input int ms);
    repeat (ms) #1000000;
endtask

//////////////////////////////////////////////////////////////////////
// WAV writer: one frame per 44.1 kHz tick

int  wav = 0;
int  wav_frames = 0;
bit  wav_on = 0;

task wav_open(input string fn);
    wav = $fopen(fn, "wb");
    // RIFF header; sizes patched at close
    $fwrite(wav, "RIFF%c%c%c%cWAVEfmt ", 0, 0, 0, 0);
    $fwrite(wav, "%c%c%c%c%c%c%c%c", 16, 0, 0, 0, 1, 0, 2, 0);
    $fwrite(wav, "%c%c%c%c%c%c%c%c", 8'h44, 8'hac, 0, 0, 8'h10, 8'hb1, 8'h02, 0);
    $fwrite(wav, "%c%c%c%cdata%c%c%c%c", 4, 0, 16, 0, 0, 0, 0, 0);
endtask

task wav_close;
    int n;
    n = wav_frames * 4;
    $fseek(wav, 4, 0);
    $fwrite(wav, "%c%c%c%c", n[7:0] + 8'd36, n[15:8], n[23:16], n[31:24]);
    $fseek(wav, 40, 0);
    $fwrite(wav, "%c%c%c%c", n[7:0], n[15:8], n[23:16], n[31:24]);
    $fclose(wav);
endtask

logic [3:0] tick_d;
always @(posedge clk) begin
    tick_d <= {tick_d[2:0], cdda.tick};
    if (tick_d[3] & wav_on) begin
        $fwrite(wav, "%c%c%c%c", out_l[7:0], out_l[15:8], out_r[7:0], out_r[15:8]);
        wav_frames <= wav_frames + 1;
    end
end

//////////////////////////////////////////////////////////////////////
// Test sequence

int ms_play;

// Watchdog: report state and stop if the test does not finish
initial begin
    #(64'd30_000_000_000);  // 30 s of simulated time
    $display("[%0t] TIMEOUT: fake_cd tst=%0d au_run=%0d astep=%0d cv_busy=%0d; cdda ps=%0d toc_valid=%0d sd_rd=%0d sd_ack=%0d",
             $time, fake_cd.tst, fake_cd.au_run, fake_cd.astep, fake_cd.cv_busy,
             cdda.ps, toc_valid, sd_rd, sd_ack);
    $finish;
end

// Progress: one line per simulated 100 ms
longint cyc = 0;
always @(posedge clk) cyc <= cyc + 1;

int     ps_prev = -1;
bit     ack_prev = 0;
int     n_changes = 0;
always @(posedge clk) begin
    if ((int'(cdda.ps) != ps_prev || sd_ack != ack_prev) && n_changes < 40) begin
        $display("[%0t] ps=%0d sd_ack=%0d sd_rd=%0d p_toc=%0d toc_load=%0d toc_valid=%0d",
                 $time, cdda.ps, sd_ack, sd_rd, cdda.p_toc, cdda.toc_load, toc_valid);
        n_changes <= n_changes + 1;
    end
    ps_prev <= int'(cdda.ps);
    ack_prev <= sd_ack;
end

initial forever begin
    #(64'd100_000_000);
    $display("[%0t] tick: cyc=%0d fake_cd tst=%0d cdda ps=%0d status=%0d pos=%0d fifo=%0d reads=%0d sd_rd=%0d sd_ack=%0d",
             $time, cyc, fake_cd.tst, cdda.ps, au_status, au_pos, cdda.fifo_cnt, sd_reads, sd_rd, sd_ack);
end

initial begin
    if (!$value$plusargs("play_ms=%d", ms_play))
        ms_play = 2000;

    repeat (10) @(posedge clk);
    resn <= 1;
    repeat (10) @(posedge clk);

    $display("[%0t] mount", $time);
    void'(pcfx_mount_cd());
    img_size <= pcfx_cd_img_size();
    cd_en <= 1;
    mount <= 1;
    @(posedge clk);
    mount <= 0;
    $display("[%0t] img_size %0d, wait for TOC", $time, img_size);
    @(posedge clk iff toc_valid);
    $display("TOC: tracks %0d-%0d, lead-out %0d", first_track, last_track, leadout);

    scsi('{8'h12, 0, 0, 0, 36, 0, 0, 0, 0, 0, 0, 0});           // INQUIRY
    scsi('{8'hde, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});            // first/last
    scsi('{8'hde, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});            // lead-out
    for (int t = 1; t <= 4; t++)
        scsi('{8'hde, 2, bcd(t), 0, 0, 0, 0, 0, 0, 0, 0, 0});   // track start
    scsi('{8'hdd, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});           // subq, stopped
    scsi('{8'hd9, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});            // SAPEP while stopped: CHECK
    scsi('{8'h03, 0, 0, 0, 18, 0, 0, 0, 0, 0, 0, 0});           // REQUEST SENSE

    // Play track 3 (audio) to the start of track 4
    wav_open("cdda.wav");
    wav_on = 1;
    play_tracks_nec(3, 4);
    wait_ms(ms_play / 2);
    scsi('{8'hdd, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});           // subq, playing
    scsi('{8'hda, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});            // still
    wait_ms(100);
    scsi('{8'hdd, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});           // subq, still
    scsi('{8'h4b, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0});            // resume
    wait_ms(ms_play / 2);
    scsi('{8'h42, 2, 8'h40, 1, 0, 0, 0, 0, 16, 0, 0, 0});       // READ SUB-CHANNEL, MSF
    scsi('{8'h1b, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});            // stop
    scsi('{8'hdd, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});           // subq, stopped
    wav_on = 0;
    repeat (8) @(posedge clk);
    wav_close();

    $display("sector reads: %0d, wav frames: %0d", sd_reads, wav_frames);
    $finish;
end

endmodule
