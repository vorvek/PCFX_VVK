// Emulate a CD drive on the SCSI-CD bridge
//
// Copyright (c) 2025-2026 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

// References:
// - https://www.staff.uni-mainz.de/tacke/scsi/SCSI2.html
// - https://github.com/libretro-mirrors/mednafen-git/blob/master/src/cdrom/scsicd.cpp
// - https://github.com/mamedev/mame/blob/master/src/devices/bus/nscsi/pcfxcd.cpp
//   (NEC audio commands D8h-DEh and their responses)

module fake_cd
   (
    input             CLK,
    input             RESn,

    output reg        STAT_GET,
    input [95:0]      COMMAND,
    input             COMM_SEND,
    output reg        DOUT_REQ,
    input [79:0]      DOUT,
    input             DOUT_SEND,
    output reg [7:0]  STATUS,
    output reg [7:0]  CD_DATA,
    output reg        CD_WR,
    input             CD_READY,
    input             CD_DATA_END,
    input             MSGOUT_PEND,
    input [7:0]       MSGOUT,
    input             MSGOUT_SEND,

    input             MEDIUM_EMPTY,
    output reg [31:0] SD_LBA,
    output reg        SD_RD,
    input             SD_ACK,
    output [12:0]     SDBUF_ADDR,
    input [15:0]      SDBUF_DOUT,
    output            BUSY,

    // CD-DA player (cdda)
    output reg        AU_PLAY,
    output reg        AU_PLAY_PAUSE,
    output reg [18:0] AU_PLAY_START,
    output reg [18:0] AU_PLAY_END,
    output reg        AU_PLAY_REPEAT,
    output reg        AU_PLAY_MUTE,
    output reg        AU_PAUSE,
    output reg [1:0]  AU_PAUSE_STATUS,
    output reg        AU_RESUME,
    output            AU_STOP,
    input [1:0]       AU_STATUS,
    input [18:0]      AU_POS,
    input [18:0]      AU_START,
    input             TOC_VALID,
    input [6:0]       FIRST_TRACK,
    input [6:0]       LAST_TRACK,
    input [18:0]      LEADOUT,
    output reg [6:0]  TT_TRACK,
    input [18:0]      TT_START,
    input [3:0]       TT_CONTROL,
    input [6:0]       Q_TRACK,
    input [18:0]      Q_TRACK_START,
    input [3:0]       Q_CONTROL
    );

typedef enum bit [7:0]
{
    CMDOP_TEST_UNIT_READY = 8'h00,
    CMDOP_REQUEST_SENSE = 8'h03,
    CMDOP_NO_OPERATION = 8'h0d,
    CMDOP_INQUIRY = 8'h12,
    CMDOP_MODE_SELECT = 8'h15,
    CMDOP_START_STOP_UNIT = 8'h1b,
    CMDOP_PREVENT_ALLOW_MEDIUM_REMOVAL = 8'h1e,
    CMDOP_READ_10 = 8'h28,
    CMDOP_READ_SUB_CHANNEL = 8'h42,
    CMDOP_READ_TOC = 8'h43,
    CMDOP_READ_HEADER = 8'h44,
    CMDOP_PLAY_AUDIO_10 = 8'h45,
    CMDOP_PLAY_AUDIO_MSF = 8'h47,
    CMDOP_PLAY_AUDIO_TRACK_INDEX = 8'h48,
    CMDOP_PAUSE_RESUME = 8'h4b,
    CMDOP_NEC_SAPSP = 8'hd8,        // set audio playback start position
    CMDOP_NEC_SAPEP = 8'hd9,        // set audio playback end position
    CMDOP_NEC_PAUSE = 8'hda,        // still
    CMDOP_NEC_SET_STOP_TIME = 8'hdb,
    CMDOP_NEC_EJECT = 8'hdc,
    CMDOP_NEC_READ_SUBQ = 8'hdd,
    CMDOP_NEC_GET_DIR_INFO = 8'hde
} _cmdop_t;

localparam [1:0] AU_ST_PLAYING = 2'd0;
localparam [1:0] AU_ST_STILL = 2'd1;
localparam [1:0] AU_ST_PAUSED = 2'd2;
localparam [1:0] AU_ST_STOPPED = 2'd3;

typedef enum bit [7:0]
{
    STATUS_GOOD = 8'h00,
    STATUS_CHECK_CONDITION = 8'h02
} _status_t;

localparam DATA_SENSE_LEN = 18;

typedef enum bit [7:0]
{
    SENSE_KEY_NO_SENSE = 8'h00,
    SENSE_KEY_NOT_READY = 8'h02,
    SENSE_KEY_ILLEGAL_REQUEST = 8'h05
} _sense_key_t;

// NEC sense sub-error codes (sense byte 12), GMAKER SCSI_CMD.H
typedef enum bit [15:0]
{
    ASC_NO_ADDITIONAL_SENSE_CODE = 16'h0000,
    ASC_NO_DISC = 16'h000b, // NEC-specific
    ASC_INVALID_COMMAND_OPERATION_CODE = 16'h0020,
    ASC_INVALID_PARAMETER = 16'h0022,
    ASC_INVALID_REQUEST_IN_CDB = 16'h0027,  // eject on a top-loading drive
    ASC_AUDIO_NOT_PLAYING = 16'h002c        // MAME pcfxcd.cpp
} _asc_t;

typedef enum bit [2:0]
{
    TS_IDLE,
    TS_CMD,
    TS_DATA_OUT, // initiator to target
    TS_DATA_IN, // target to initiator
    TS_DATA_IN_END,
    TS_STAT,
    TS_MSG_OUT // initiator to target
} tst_t;

tst_t           tst;
logic           cmd_start, cmd_cont, cmd_done;
logic           medium_req;
logic [7:0]     sense_key;
logic [15:0]    asc; // additional sense code and code qualifier
logic [7:0]     sense_do;
logic           data_out;
logic [11:0]    datalen, datapos, sdbuf_off;
logic [15:0]    blkcnt;
logic           dataloop;
logic [11:0]    sdbuf_baddr;
logic [12:0]    sdbuf_addr;
logic [7:0]     sdbuf_dbout;
logic           sd_ack_d;
logic           cd_wr_d;

wire            dataend = ((datapos + 1'd1) == datalen);
wire            blkend = (blkcnt == '0);

always @* begin
    sense_do = '0;
    case (datapos)
        'd0:  sense_do = 8'h70;
        'd2:  sense_do = sense_key;
        'd7:  sense_do = 8'h0A;     // Additional sense length
        'd12: sense_do = asc[7:0];  // Additional sense code
        'd13: sense_do = asc[15:8]; // Additional sense code qualifier
        default: ;
    endcase
end

assign sdbuf_baddr = datapos + sdbuf_off;
assign sdbuf_addr = 13'(sdbuf_baddr >> 1);

always @* begin
    sdbuf_off = '0;
    case (COMMAND[0+:8])
        CMDOP_READ_10: begin
            sdbuf_off = 12'd16; // skip header
        end
        CMDOP_READ_TOC: ;
        CMDOP_READ_HEADER: begin
            if (datapos == 12'd0) // Data mode
                sdbuf_off = 12'd15;
            else if (datapos >= 12'd5) // address
                sdbuf_off = 12'(12 - 5);
        end
        default: ;
    endcase
end

assign SDBUF_ADDR = sdbuf_addr;

always @* begin
    if (sdbuf_baddr[0])
        sdbuf_dbout = SDBUF_DOUT[8+:8];
    else
        sdbuf_dbout = SDBUF_DOUT[0+:8];
end

always @* begin
    CD_DATA = '0;
    if (tst == TS_DATA_IN) begin
        CD_DATA = sdbuf_dbout;
        case (COMMAND[0+:8])
            CMDOP_REQUEST_SENSE: begin
                CD_DATA = sense_do;
            end
            CMDOP_READ_HEADER: begin
                if (datapos >= 12'd1 && datapos <= 12'd4) // reserved
                    CD_DATA = '0;
            end
            CMDOP_INQUIRY,
            CMDOP_READ_SUB_CHANNEL,
            CMDOP_NEC_READ_SUBQ,
            CMDOP_NEC_GET_DIR_INFO:
                CD_DATA = resp[datapos[5:0]];
            default: ;
        endcase
    end
end

always @* begin
    medium_req = '0;
    case (COMMAND[0+:8])
        CMDOP_TEST_UNIT_READY:
            medium_req = '1;
        default: ;
    endcase
end

assign BUSY = (tst != TS_IDLE);

//////////////////////////////////////////////////////////////////////
// Audio and information commands
//
// These run over several clocks: address decode, track table lookup,
// LBA to M:S:F conversion. The result goes to au_ok / au_asc /
// au_datalen and the response bytes to resp[].

function automatic [6:0] bcd2bin(input [7:0] b);
    return 7'(b[7:4] * 4'd10 + b[3:0]);
endfunction

function automatic [7:0] bin2bcd(input [6:0] v);
    logic [3:0] t;
    t = (v >= 7'd90) ? 4'd9 : (v >= 7'd80) ? 4'd8 : (v >= 7'd70) ? 4'd7 :
        (v >= 7'd60) ? 4'd6 : (v >= 7'd50) ? 4'd5 : (v >= 7'd40) ? 4'd4 :
        (v >= 7'd30) ? 4'd3 : (v >= 7'd20) ? 4'd2 : (v >= 7'd10) ? 4'd1 : 4'd0;
    return {t, 4'(v - t * 4'd10)};
endfunction

// M:S:F (binary) to frames, without the 150-frame offset
function automatic [19:0] msf2fr(input [6:0] m, input [6:0] s, input [6:0] f);
    return 20'(m) * 20'd4500 + 20'(s) * 20'd75 + 20'(f);
endfunction

wire [7:0] cb [12];
genvar gi;
generate
    for (gi = 0; gi < 12; gi++) begin : g_cb
        assign cb[gi] = COMMAND[gi*8+:8];
    end
endgenerate

// Frames to M:S:F (binary), by repeated subtraction
logic        cv_start, cv_busy, cv_ph;
logic [19:0] cv_in, cv_v;
logic [6:0]  cv_m, cv_s;

always @(posedge CLK) begin
    if (cv_start) begin
        cv_v <= cv_in;
        cv_m <= '0;
        cv_s <= '0;
        cv_ph <= '0;
        cv_busy <= '1;
    end
    else if (cv_busy) begin
        if (~cv_ph) begin
            if (cv_v >= 20'd4500) begin
                cv_v <= cv_v - 20'd4500;
                cv_m <= cv_m + 1'd1;
            end
            else
                cv_ph <= '1;
        end
        else begin
            if (cv_v >= 20'd75) begin
                cv_v <= cv_v - 20'd75;
                cv_s <= cv_s + 1'd1;
            end
            else
                cv_busy <= '0;
        end
    end
end

wire [6:0] cv_f = cv_v[6:0];

logic        au_go, au_run, au_done, au_ok;
logic [15:0] au_asc;
logic [11:0] au_datalen;
logic [7:0]  resp [36];
logic [3:0]  astep;
logic        au_stop, rd_stop;
logic [19:0] a_addr;        // decoded NEC address (LBA)
logic [18:0] a_start;
logic [18:0] q_lba;
logic [6:0]  a_m, a_s, a_f;

assign AU_STOP = au_stop | rd_stop;

// INQUIRY data of the PC-FX drive (MAME pcfxcd.cpp)
function automatic [7:0] inquiry(input [5:0] i);
    case (i)
        6'd0: return 8'h05;     // CD-ROM device
        6'd1: return 8'h80;     // removable
        6'd2: return 8'h02;     // SCSI-2
        6'd3: return 8'h00;
        6'd4: return 8'h1f;     // additional length
        6'd5, 6'd6, 6'd7: return 8'h00;
        default: return 8'("NEC     CD-ROM DRIVE:FX 1.0 " >> (8 * (35 - i)));
    endcase
endfunction

wire au_stopped = (AU_STATUS == AU_ST_STOPPED);

always @(posedge CLK) begin
    au_done <= '0;
    cv_start <= '0;
    AU_PLAY <= '0;
    AU_PAUSE <= '0;
    AU_RESUME <= '0;
    au_stop <= '0;

    if (~RESn) begin
        au_run <= '0;
    end
    else if (au_go) begin
        au_run <= '1;
        astep <= '0;
        au_ok <= '1;
        au_asc <= ASC_NO_ADDITIONAL_SENSE_CODE;
        au_datalen <= '0;
        AU_PLAY_PAUSE <= '0;
        AU_PLAY_REPEAT <= '0;
        AU_PLAY_MUTE <= '0;
    end
    else if (au_run & ~cv_busy & ~cv_start) begin
        astep <= astep + 1'd1;

        if (astep != 4'd15) case (cb[0])
            CMDOP_NEC_SAPSP,
            CMDOP_NEC_SAPEP: begin
                // Address: byte 9 bits 7-6 select LBA, BCD M:S:F, BCD track
                case (astep)
                    4'd0: begin
                        if ((cb[0] == CMDOP_NEC_SAPEP) & au_stopped) begin
                            au_ok <= '0;
                            au_asc <= ASC_AUDIO_NOT_PLAYING;
                            astep <= 4'd15;
                        end
                        TT_TRACK <= bcd2bin(cb[2]);
                        case (cb[9][7:6])
                            2'b00: a_addr <= 20'({cb[3], cb[4], cb[5]});
                            2'b01: a_addr <= msf2fr(bcd2bin(cb[2]), bcd2bin(cb[3]),
                                                    bcd2bin(cb[4])) - 20'd150;
                            default: ;
                        endcase
                    end
                    4'd1: ; // table lookup latency
                    4'd2: begin
                        if (cb[9][7:6] == 2'b10) begin
                            if ((TT_TRACK == 0) | (TT_TRACK > LAST_TRACK + 1'd1)
                                | ~TOC_VALID) begin
                                au_ok <= '0;
                                au_asc <= ASC_INVALID_PARAMETER;
                                astep <= 4'd15;
                            end
                            a_addr <= 20'(TT_START);
                        end
                        else if (cb[9][7:6] == 2'b11) begin
                            au_ok <= '0;
                            au_asc <= ASC_INVALID_PARAMETER;
                            astep <= 4'd15;
                        end
                        a_start <= (AU_STATUS != AU_ST_PLAYING) ? AU_POS : AU_START;
                    end
                    4'd3: begin
                        if (cb[0] == CMDOP_NEC_SAPSP) begin
                            if (a_addr > 20'(LEADOUT)) begin
                                au_ok <= '0;
                                au_asc <= ASC_INVALID_PARAMETER;
                            end
                            else begin
                                AU_PLAY <= '1;
                                AU_PLAY_START <= a_addr[18:0];
                                AU_PLAY_END <= LEADOUT;
                                AU_PLAY_PAUSE <= ~cb[1][0];
                            end
                        end
                        else begin
                            // Mode: 0 mute, 1-3 play, 4 repeat
                            if (a_addr[18:0] > a_start) begin
                                AU_PLAY <= '1;
                                AU_PLAY_START <= a_start;
                                AU_PLAY_END <= a_addr[18:0];
                                AU_PLAY_REPEAT <= (cb[1][2:0] == 3'd4);
                                AU_PLAY_MUTE <= (cb[1][2:0] == 3'd0);
                            end
                            else
                                au_stop <= '1;
                        end
                        astep <= 4'd15;
                    end
                    default: ;
                endcase
            end

            CMDOP_NEC_PAUSE: begin
                if (au_stopped) begin
                    au_ok <= '0;
                    au_asc <= ASC_AUDIO_NOT_PLAYING;
                end
                else begin
                    AU_PAUSE <= '1;
                    AU_PAUSE_STATUS <= AU_ST_STILL;
                end
                astep <= 4'd15;
            end

            CMDOP_PAUSE_RESUME: begin
                if (au_stopped) begin
                    au_ok <= '0;
                    au_asc <= ASC_AUDIO_NOT_PLAYING;
                end
                else if (cb[8][0])
                    AU_RESUME <= '1;
                else begin
                    AU_PAUSE <= '1;
                    AU_PAUSE_STATUS <= AU_ST_PAUSED;
                end
                astep <= 4'd15;
            end

            CMDOP_START_STOP_UNIT: begin
                if (cb[4][1]) begin             // LoEj: top-loading drive
                    au_ok <= '0;
                    au_asc <= ASC_INVALID_REQUEST_IN_CDB;
                end
                else if (~cb[4][0]) begin       // stop
                    if (au_stopped) begin
                        au_ok <= '0;
                        au_asc <= ASC_AUDIO_NOT_PLAYING;
                    end
                    else
                        au_stop <= '1;
                end
                astep <= 4'd15;
            end

            CMDOP_NEC_EJECT: begin
                au_ok <= '0;
                au_asc <= ASC_INVALID_REQUEST_IN_CDB;
                astep <= 4'd15;
            end

            CMDOP_PLAY_AUDIO_10: begin
                if ({cb[7], cb[8]} != 0) begin
                    AU_PLAY <= '1;
                    AU_PLAY_START <= {cb[3][2:0], cb[4], cb[5]};
                    AU_PLAY_END <= 19'({cb[3][2:0], cb[4], cb[5]} + {cb[7], cb[8]});
                end
                astep <= 4'd15;
            end

            CMDOP_PLAY_AUDIO_MSF: begin
                // Binary M:S:F, start in bytes 3-5, end in bytes 6-8
                case (astep)
                    4'd0: begin
                        a_addr <= msf2fr(cb[3][6:0], cb[4][6:0], cb[5][6:0]);
                        a_start <= 19'(msf2fr(cb[6][6:0], cb[7][6:0], cb[8][6:0]) - 20'd150);
                    end
                    default: begin
                        if ((a_addr < 20'd150) | (a_start < 19'(a_addr - 20'd150))
                            | (19'(a_addr - 20'd150) >= LEADOUT)) begin
                            au_ok <= '0;
                            au_asc <= ASC_INVALID_PARAMETER;
                        end
                        else if (a_start != 19'(a_addr - 20'd150)) begin
                            AU_PLAY <= '1;
                            AU_PLAY_START <= 19'(a_addr - 20'd150);
                            AU_PLAY_END <= a_start;
                        end
                        astep <= 4'd15;
                    end
                endcase
            end

            CMDOP_PLAY_AUDIO_TRACK_INDEX: begin
                // Start track byte 4, end track byte 7. Plays up to the start
                // of the end track (MAME). Index fields are ignored.
                case (astep)
                    4'd0: begin
                        if ((cb[4] == 0) | (cb[4] > 8'(LAST_TRACK)) | (cb[7] < cb[4])
                            | ~TOC_VALID) begin
                            au_ok <= '0;
                            au_asc <= ASC_INVALID_PARAMETER;
                            astep <= 4'd15;
                        end
                        TT_TRACK <= cb[4][6:0];
                    end
                    4'd1: ;
                    4'd2: begin
                        a_start <= TT_START;
                        TT_TRACK <= (cb[7] > 8'(LAST_TRACK)) ? LAST_TRACK + 1'd1 : cb[7][6:0];
                    end
                    4'd3: ;
                    4'd4: begin
                        AU_PLAY <= '1;
                        AU_PLAY_START <= a_start;
                        AU_PLAY_END <= TT_START;
                        astep <= 4'd15;
                    end
                    default: ;
                endcase
            end

            CMDOP_INQUIRY: begin
                for (int i = 0; i < 36; i++)
                    resp[i] <= inquiry(6'(i));
                au_datalen <= (cb[4] > 8'd36) ? 12'd36 : 12'(cb[4]);
                astep <= 4'd15;
            end

            CMDOP_NEC_READ_SUBQ,
            CMDOP_READ_SUB_CHANNEL: begin
                case (astep)
                    4'd0: begin
                        q_lba <= au_stopped ? AU_START : AU_POS;
                    end
                    4'd1: begin                 // absolute M:S:F
                        cv_in <= 20'(q_lba) + 20'd150;
                        cv_start <= '1;
                    end
                    4'd2: begin
                        {a_m, a_s, a_f} <= {cv_m, cv_s, cv_f};
                        cv_in <= 20'(q_lba - Q_TRACK_START);
                        cv_start <= '1;
                    end
                    4'd3: begin                 // cv_* = relative M:S:F
                        if (cb[0] == CMDOP_NEC_READ_SUBQ) begin
                            resp[0] <= {6'd0, AU_STATUS};
                            resp[1] <= {Q_CONTROL, 4'd1};
                            resp[2] <= bin2bcd(Q_TRACK);
                            resp[3] <= 8'h01;
                            resp[4] <= bin2bcd(cv_m);
                            resp[5] <= bin2bcd(cv_s);
                            resp[6] <= bin2bcd(cv_f);
                            resp[7] <= bin2bcd(a_m);
                            resp[8] <= bin2bcd(a_s);
                            resp[9] <= bin2bcd(a_f);
                            au_datalen <= (cb[1] > 8'd10) ? 12'd10 : 12'(cb[1]);
                        end
                        else begin
                            // SCSI-2 READ SUB-CHANNEL, format 01h (current position)
                            for (int i = 0; i < 16; i++)
                                resp[i] <= '0;
                            case (AU_STATUS)
                                AU_ST_PLAYING: resp[1] <= 8'h11;
                                AU_ST_STOPPED: resp[1] <= 8'h15;
                                default:       resp[1] <= 8'h12;
                            endcase
                            if (cb[2][6]) begin     // SubQ
                                resp[3] <= 8'd12;
                                resp[4] <= 8'h01;
                                resp[5] <= {4'd1, Q_CONTROL};
                                resp[6] <= {1'b0, Q_TRACK};
                                resp[7] <= 8'h01;
                                if (cb[1][1]) begin // MSF
                                    resp[9] <= {1'b0, a_m};
                                    resp[10] <= {1'b0, a_s};
                                    resp[11] <= {1'b0, a_f};
                                    resp[13] <= {1'b0, cv_m};
                                    resp[14] <= {1'b0, cv_s};
                                    resp[15] <= {1'b0, cv_f};
                                end
                                else begin
                                    {resp[9], resp[10], resp[11]} <= 24'(q_lba);
                                    {resp[13], resp[14], resp[15]} <= 24'(q_lba - Q_TRACK_START);
                                end
                            end
                            if ({cb[7], cb[8]} > (cb[2][6] ? 16'd16 : 16'd4))
                                au_datalen <= cb[2][6] ? 12'd16 : 12'd4;
                            else
                                au_datalen <= 12'({cb[7], cb[8]});
                        end
                        astep <= 4'd15;
                    end
                    default: ;
                endcase
            end

            CMDOP_NEC_GET_DIR_INFO: begin
                case (astep)
                    4'd0: begin
                        TT_TRACK <= bcd2bin(cb[2]);
                        resp[3] <= '0;
                        if (~TOC_VALID | (cb[1][1:0] == 2'd3)) begin
                            au_ok <= '0;
                            au_asc <= ASC_INVALID_PARAMETER;
                            astep <= 4'd15;
                        end
                    end
                    4'd1: ;
                    4'd2: begin
                        au_datalen <= 12'd4;
                        case (cb[1][1:0])
                            2'd0: begin         // first and last track
                                resp[0] <= bin2bcd(FIRST_TRACK);
                                resp[1] <= bin2bcd(LAST_TRACK);
                                resp[2] <= '0;
                                astep <= 4'd15;
                            end
                            2'd1: begin         // lead-out
                                cv_in <= 20'(LEADOUT) + 20'd150;
                                cv_start <= '1;
                            end
                            default: begin      // track start and control
                                if ((TT_TRACK == 0) | (TT_TRACK > LAST_TRACK)) begin
                                    au_ok <= '0;
                                    au_asc <= ASC_INVALID_PARAMETER;
                                    astep <= 4'd15;
                                end
                                cv_in <= 20'(TT_START) + 20'd150;
                                cv_start <= '1;
                                resp[3] <= {4'd0, TT_CONTROL};
                            end
                        endcase
                    end
                    4'd3: begin
                        resp[0] <= bin2bcd(cv_m);
                        resp[1] <= bin2bcd(cv_s);
                        resp[2] <= bin2bcd(cv_f);
                        astep <= 4'd15;
                    end
                    default: ;
                endcase
            end

            default:                            // 0Dh, DBh: no operation
                astep <= 4'd15;
        endcase

        if (astep == 4'd15) begin
            au_run <= '0;
            au_done <= '1;
            if (~au_ok)
                au_datalen <= '0;
        end
    end
end

always @(posedge CLK) begin
    cmd_start <= '0;
    cmd_cont <= '0;
    cmd_done <= '0;
    STAT_GET <= '0;
    DOUT_REQ <= '0;
    au_go <= '0;
    rd_stop <= '0;
    sd_ack_d <= SD_ACK;
    cd_wr_d <= CD_WR;

    if (~RESn) begin
        STATUS <= '0;
        CD_WR <= '0;
        SD_LBA <= '0;
        SD_RD <= '0;

        tst <= TS_IDLE;
        datalen <= '0;
        sense_key <= SENSE_KEY_NO_SENSE;
        asc <= ASC_NO_ADDITIONAL_SENSE_CODE;
    end
    else begin
        if (~sd_ack_d & SD_ACK)
            SD_RD <= '0;

        case (tst)
            TS_IDLE:
                if (COMM_SEND) begin
                    $display("fake_cd: COMMAND=%x", COMMAND);
                    blkcnt <= '0;
                    datapos <= '0;
                    data_out <= '0;
                    cmd_start <= '1;
                    tst <= TS_CMD;
                end
            TS_CMD:
                if (cmd_done) begin
                    if (datalen != 0) begin
                        if (data_out) begin
                            DOUT_REQ <= '1;
                            tst <= TS_DATA_OUT;
                        end
                        else
                            tst <= TS_DATA_IN;
                    end
                    else
                        tst <= TS_STAT;
                end
            TS_DATA_OUT: begin // initiator to target
                if (DOUT_SEND)
                    tst <= TS_STAT;
            end
            TS_DATA_IN: begin // target to initiator
                if (MSGOUT_PEND) // early termination by ATN
                    tst <= TS_MSG_OUT;
                else if (CD_WR) begin
                    datapos <= datapos + 1'd1;
                    if (dataend) begin
                        if (~blkend) begin
                            blkcnt <= blkcnt - 1'd1;
                            datapos <= '0;
                            cmd_cont <= '1;
                            tst <= TS_CMD;
                        end
                        else begin
                            tst <= TS_DATA_IN_END;
                        end
                    end
                end
            end
            TS_DATA_IN_END: begin
                // To close a race between us finishing sending data
                // to the SCSI bridge and the initiator asserting ATN,
                // we wait here for the bridge to finish sending data
                // to the initiator.
                if (CD_DATA_END) begin
                    if (MSGOUT_PEND) // early termination by ATN
                        tst <= TS_MSG_OUT;
                    else // normal completion
                        tst <= TS_STAT;
                end
            end
            TS_STAT: begin
                STAT_GET <= '1;
                tst <= TS_IDLE;
            end
            TS_MSG_OUT: begin
                if (MSGOUT_SEND) begin
                    // Error handling (ie retry) would go here.
                    $display("fake_cd: MSGOUT=%x", MSGOUT);
                    tst <= TS_IDLE;
                end
            end
            default: ;
        endcase

        if ((tst == TS_CMD) & cmd_start) begin
            STATUS <= STATUS_GOOD;
            datalen <= '0;

            if (medium_req & MEDIUM_EMPTY) begin
                STATUS <= STATUS_CHECK_CONDITION;
                sense_key <= SENSE_KEY_NOT_READY;
                asc <= ASC_NO_DISC;
                cmd_done <= '1;
            end
            else begin
                case (COMMAND[0+:8])
                    CMDOP_TEST_UNIT_READY: begin
                        cmd_done <= '1;
                    end
                    CMDOP_REQUEST_SENSE: begin
                        datalen <= $size(datalen)'(COMMAND[32+:8]);
                        cmd_done <= '1;
                    end
                    CMDOP_MODE_SELECT: begin
                        data_out <= '1;
                        datalen <= $size(datalen)'(COMMAND[4*8+:8]);
                        cmd_done <= '1;
                    end
                    CMDOP_PREVENT_ALLOW_MEDIUM_REMOVAL: begin
                        cmd_done <= '1;
                    end
                    CMDOP_READ_10: begin
                        rd_stop <= '1;  // a data read stops audio playback
                        // TODO: 2336 for Mode 2 blocks
                        datalen <= $size(datalen)'(2048);
                        if (COMMAND[(7*8)+:16] != '0) begin
                            blkcnt <= $size(blkcnt)'
                                      ({COMMAND[(7*8)+:8], COMMAND[(8*8)+:8]}
                                       - 1'd1);
                            SD_LBA <= {COMMAND[(2*8)+:8], COMMAND[(3*8)+:8],
                                       COMMAND[(4*8)+:8], COMMAND[(5*8)+:8]};
                            SD_RD <= '1;
                        end
                        else
                            cmd_done <= '1;
                    end
                    CMDOP_READ_TOC: begin
                        datalen <= $size(datalen)'
                                   ({COMMAND[(7*8)+:8], COMMAND[(8*8)+:8]});
                        SD_LBA <= 32'(-65536 + COMMAND[(6*8)+:8]);
                        SD_RD <= '1;
                    end
                    CMDOP_READ_HEADER: begin
                        datalen <= $size(datalen)'
                                   ({COMMAND[(7*8)+:8], COMMAND[(8*8)+:8]});
                        SD_LBA <= {COMMAND[(2*8)+:8], COMMAND[(3*8)+:8],
                                   COMMAND[(4*8)+:8], COMMAND[(5*8)+:8]};
                        SD_RD <= '1;
                    end
                    CMDOP_NO_OPERATION,
                    CMDOP_INQUIRY,
                    CMDOP_START_STOP_UNIT,
                    CMDOP_READ_SUB_CHANNEL,
                    CMDOP_PLAY_AUDIO_10,
                    CMDOP_PLAY_AUDIO_MSF,
                    CMDOP_PLAY_AUDIO_TRACK_INDEX,
                    CMDOP_PAUSE_RESUME,
                    CMDOP_NEC_SAPSP,
                    CMDOP_NEC_SAPEP,
                    CMDOP_NEC_PAUSE,
                    CMDOP_NEC_SET_STOP_TIME,
                    CMDOP_NEC_EJECT,
                    CMDOP_NEC_READ_SUBQ,
                    CMDOP_NEC_GET_DIR_INFO:
                        au_go <= '1;
                    default: begin
                        STATUS <= STATUS_CHECK_CONDITION;
                        sense_key <= SENSE_KEY_ILLEGAL_REQUEST;
                        asc <= ASC_INVALID_COMMAND_OPERATION_CODE;
                        cmd_done <= '1;
                    end
                endcase
            end
        end

        if ((tst == TS_CMD) & au_done) begin
            if (~au_ok) begin
                STATUS <= STATUS_CHECK_CONDITION;
                sense_key <= SENSE_KEY_ILLEGAL_REQUEST;
                asc <= au_asc;
            end
            datalen <= au_datalen;
            cmd_done <= '1;
        end

        if ((tst == TS_CMD) & cmd_cont) begin
            case (COMMAND[0+:8])
                CMDOP_READ_10: begin
                    SD_LBA <= SD_LBA + 1'd1;
                    SD_RD <= '1;
                end
                default: ;
            endcase
        end

        if ((tst == TS_CMD) & ~(cmd_start | cmd_cont)) begin
            case (COMMAND[0+:8])
                CMDOP_READ_10,
                CMDOP_READ_TOC,
                CMDOP_READ_HEADER: begin
                    cmd_done <= sd_ack_d & ~SD_ACK;
                end
                default: ;
            endcase
        end

        if ((tst == TS_DATA_OUT) & DOUT_SEND) begin
            $display("fake_cd: DOUT=%x", DOUT);
            case (COMMAND[0+:8])
                CMDOP_MODE_SELECT: begin
                    // TODO
                end
                default: ;
            endcase
        end

        if (tst == TS_DATA_IN) begin
            // Due to FIFO pipeline delays in the SCSI module, CD_WR
            // must stay low for at least 2x clocks after falling.
            CD_WR <= ~(CD_WR | cd_wr_d) & CD_READY;

            case (COMMAND[0+:8])
                CMDOP_REQUEST_SENSE: begin
                    if (CD_WR & dataend) begin
                        sense_key <= SENSE_KEY_NO_SENSE;
                        asc <= ASC_NO_ADDITIONAL_SENSE_CODE;
                    end
                end
                default: ;
            endcase

            if (CD_WR & (COMMAND[0+:8] != CMDOP_READ_10))
                $display("fake_cd: TS_DATA_IN: [%x] = %x", datapos, CD_DATA);
        end
        else
            CD_WR <= '0;
    end
end

endmodule
