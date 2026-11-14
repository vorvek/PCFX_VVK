// Debug serial output (development aid)
//
// Sends text lines on TXD (115200 baud, 8N1) to the HPS UART, which Linux
// reads as /dev/ttyS1:
//   "C <24 hex>"  each SCSI command (12 bytes, byte 0 first)
//   "B <4 x addr.flags.data>"  the last 4 CPU bus cycles, about 10x per s
//                 flags = {ST[1:0], RW, ERROR}
//   "S ss nnnn"   SCSI status byte ss at the end of a command, and the
//                 number of CD sector reads (SD_RD) since the command
//   "R wwvvhhss"  RAINBOW debug after each B line: window, decoded strip
//                 and header counters, {SRT, DCT state, input state}
//   "P m vvvvvv nnnnnn"  RAINBOW pixels after each R line: mode and value of
//                 the last nonzero pixel, count of nonzero pixels (DCK)
//   "V rr vvvv"   HuC6261 register write (palette data port 02h omitted)
//   "Q a vvvv"    HuC6271 register write (a = A[4:1])
//   "I xx"        KING interrupt status, when bit 6, 5 or 4 changes
//                 (SCSI bus, SCSI DMA, subcode)
//
// Input: each byte received on RXD presses one button of controller 1 for
// about 100 ms: r RUN, s SELECT, 1-6 buttons I-VI, u d l e = up, down,
// left, right.
//
// This program is GPL licensed. See COPYING for the full license.

module dbg_uart
   #(parameter CLK_HZ = 42954545,
     parameter BAUD = 115200)
   (
    input         CLK,
    input         RESn,

    input [31:0]  CPU_A,
    input [31:0]  CPU_DI,
    input [31:0]  CPU_DO,
    input [1:0]   CPU_ST,
    input         CPU_RW,
    input         CPU_BCYSTn,
    input         CPU_READYn,
    input         CE,
    input         ERROR,

    input [95:0]  SCSI_CMD,
    input         SCSI_CMD_SEND,
    input [7:0]   SCSI_STAT,
    input         SCSI_STAT_GET,
    input         SD_RD,
    input [7:0]   KING_ISR,
    input [31:0]  RB_DBG,
    input         VCE_CSn,
    input         VPU_CSn,
    input         IO_WRn,
    input [4:1]   IO_A,
    input [15:0]  IO_DI,
    input         VPU_DCK,
    input         VPU_VDMODE,
    input [23:0]  VPU_VD,

    output reg    TXD,
    input         RXD,
    output core_pkg::joypad_t JP
    );

localparam int DIV = (CLK_HZ + BAUD / 2) / BAUD;
localparam int TICK = CLK_HZ / 10;

function [7:0] hexc(input [3:0] n);
    hexc = (n < 4'd10) ? (8'h30 + 8'(n)) : (8'h41 + 8'(n) - 8'd10);
endfunction

//////////////////////////////////////////////////////////////////////
// Receiver and button press timer

logic [2:0]     rx_s;
logic           rx_act;
logic [15:0]    rx_cnt;
logic [3:0]     rx_bit;
logic [7:0]     rx_sh;
logic [7:0]     rx_byte;
logic           rx_stb;
logic [22:0]    jp_hold;
core_pkg::joypad_t jp;

always @(posedge CLK) begin
    rx_s <= {rx_s[1:0], RXD};
    rx_stb <= '0;
    if (~RESn) begin
        rx_act <= '0;
    end
    else if (~rx_act) begin
        if (rx_s[2:1] == 2'b10) begin   // start bit edge
            rx_act <= '1;
            rx_cnt <= 16'(DIV + DIV / 2 - 1);   // middle of data bit 0
            rx_bit <= '0;
        end
    end
    else if (rx_cnt != '0)
        rx_cnt <= rx_cnt - 1'd1;
    else begin
        rx_cnt <= 16'(DIV - 1);
        if (rx_bit == 4'd8) begin       // stop bit
            rx_act <= '0;
            rx_byte <= rx_sh;
            rx_stb <= rx_s[1];
        end
        else begin
            rx_sh <= {rx_s[1], rx_sh[7:1]};
            rx_bit <= rx_bit + 1'd1;
        end
    end
end

always @(posedge CLK) begin
    if (~RESn) begin
        jp <= '0;
        jp_hold <= '0;
    end
    else begin
        if (jp_hold != '0)
            jp_hold <= jp_hold - 1'd1;
        else
            jp <= '0;
        if (rx_stb) begin
            jp <= '0;
            jp_hold <= 23'(CLK_HZ / 10);
            case (rx_byte)
                "r": jp.run <= '1;
                "s": jp.select <= '1;
                "1": jp.b[1] <= '1;
                "2": jp.b[2] <= '1;
                "3": jp.b[3] <= '1;
                "4": jp.b[4] <= '1;
                "5": jp.b[5] <= '1;
                "6": jp.b[6] <= '1;
                "u": jp.u <= '1;
                "d": jp.d <= '1;
                "l": jp.l <= '1;
                "e": jp.r <= '1;
                default: ;
            endcase
        end
    end
end

assign JP = jp;

//////////////////////////////////////////////////////////////////////
// Bus cycle capture: address at the cycle start, data at the cycle end

logic [31:0]    ring_a [4];
logic [3:0]     ring_f [4];
logic [31:0]    ring_d [4];
logic [1:0]     ring_w;
logic           cyc_act;
logic [31:0]    cyc_a;
logic [3:0]     cyc_f;

always @(posedge CLK) begin
    if (~RESn) begin
        ring_w <= '0;
        cyc_act <= '0;
    end
    else if (CE) begin
        if (~CPU_BCYSTn) begin
            cyc_act <= '1;
            cyc_a <= CPU_A;
            cyc_f <= {CPU_ST, CPU_RW, ERROR};
        end
        else if (cyc_act & ~CPU_READYn) begin
            cyc_act <= '0;
            ring_a[ring_w] <= cyc_a;
            ring_f[ring_w] <= cyc_f;
            ring_d[ring_w] <= cyc_f[1] ? CPU_DI : CPU_DO;
            ring_w <= ring_w + 1'd1;
        end
    end
end

//////////////////////////////////////////////////////////////////////
// Line buffer

localparam int LMAX = 96;

logic [7:0]     line [LMAX];
logic [6:0]     llen, lpos;
logic           lbusy;
logic [31:0]    tick_cnt;
logic           tick_pend;
logic [95:0]    cmd;
logic           cmd_pend;
logic           cmd_send_d;
logic [7:0]     stat;
logic           stat_pend, stat_get_d;
logic [15:0]    rd_cnt, rd_cnt_l;
logic           sd_rd_d;
logic [7:0]     isr_d, isr_l;
logic           isr_pend;
logic           rb_pend;
logic           px_pend;
logic           wr_d;
logic [4:0]     vce_ar;
logic           vw_pend, qw_pend;
logic [4:0]     vw_reg;
logic [3:0]     qw_reg;
logic [15:0]    vw_val, qw_val;
logic [24:0]    px_last, px_last_l;
logic [23:0]    px_cnt, px_cnt_l;
logic [31:0]    rb_l;

// UART transmitter
logic [15:0]    baud_cnt;
logic [3:0]     bit_cnt;
logic [9:0]     shreg;
logic           tx_busy;

always @(posedge CLK) begin
    if (~RESn) begin
        lbusy <= '0;
        tick_cnt <= '0;
        tick_pend <= '0;
        cmd_pend <= '0;
        cmd_send_d <= '0;
        stat_pend <= '0;
        stat_get_d <= '0;
        rd_cnt <= '0;
        sd_rd_d <= '0;
        isr_d <= '0;
        isr_pend <= '0;
        rb_pend <= '0;
        px_pend <= '0;
        px_cnt <= '0;
        wr_d <= '0;
        vw_pend <= '0;
        qw_pend <= '0;
        tx_busy <= '0;
        TXD <= 1'b1;
    end
    else begin
        cmd_send_d <= SCSI_CMD_SEND;
        if (SCSI_CMD_SEND & ~cmd_send_d & ~cmd_pend) begin
            cmd <= SCSI_CMD;
            cmd_pend <= '1;
        end
        if (SCSI_CMD_SEND & ~cmd_send_d)
            rd_cnt <= '0;

        sd_rd_d <= SD_RD;
        if (SD_RD & ~sd_rd_d)
            rd_cnt <= rd_cnt + 1'd1;

        stat_get_d <= SCSI_STAT_GET;
        if (SCSI_STAT_GET & ~stat_get_d & ~stat_pend) begin
            stat <= SCSI_STAT;
            rd_cnt_l <= rd_cnt;
            stat_pend <= '1;
        end

        isr_d <= KING_ISR;
        if (((KING_ISR ^ isr_d) & 8'h70) != '0 & ~isr_pend) begin
            isr_l <= KING_ISR;
            isr_pend <= '1;
        end

        tick_cnt <= tick_cnt + 1'd1;
        if (tick_cnt == TICK - 1) begin
            tick_cnt <= '0;
            tick_pend <= '1;
        end

        // Fill the line buffer
        if (~lbusy) begin
            if (cmd_pend) begin
                cmd_pend <= '0;
                line[0] <= "C";
                line[1] <= " ";
                for (int i = 0; i < 12; i++) begin
                    line[2 + 2*i] <= hexc(cmd[i*8+4+:4]);
                    line[3 + 2*i] <= hexc(cmd[i*8+:4]);
                end
                line[26] <= 8'h0d;
                line[27] <= 8'h0a;
                llen <= 7'd28;
                lpos <= '0;
                lbusy <= '1;
            end
            else if (stat_pend) begin
                stat_pend <= '0;
                line[0] <= "S";
                line[1] <= " ";
                line[2] <= hexc(stat[7:4]);
                line[3] <= hexc(stat[3:0]);
                line[4] <= " ";
                for (int i = 0; i < 4; i++)
                    line[5 + i] <= hexc(rd_cnt_l[(3-i)*4+:4]);
                line[9] <= 8'h0d;
                line[10] <= 8'h0a;
                llen <= 7'd11;
                lpos <= '0;
                lbusy <= '1;
            end
            else if (isr_pend) begin
                isr_pend <= '0;
                line[0] <= "I";
                line[1] <= " ";
                line[2] <= hexc(isr_l[7:4]);
                line[3] <= hexc(isr_l[3:0]);
                line[4] <= 8'h0d;
                line[5] <= 8'h0a;
                llen <= 7'd6;
                lpos <= '0;
                lbusy <= '1;
            end
            else if (tick_pend) begin
                tick_pend <= '0;
                rb_pend <= '1;
                rb_l <= RB_DBG;
                px_pend <= '1;
                px_last_l <= px_last;
                px_cnt_l <= px_cnt;
                px_cnt <= '0;
                line[0] <= "B";
                for (int e = 0; e < 4; e++) begin
                    // oldest first
                    line[1 + e*20] <= " ";
                    for (int i = 0; i < 8; i++)
                        line[2 + e*20 + i] <= hexc(ring_a[2'(ring_w + e)][(7-i)*4+:4]);
                    line[10 + e*20] <= ".";
                    line[11 + e*20] <= hexc(ring_f[2'(ring_w + e)]);
                    line[12 + e*20] <= ".";
                    for (int i = 0; i < 8; i++)
                        line[13 + e*20 + i] <= hexc(ring_d[2'(ring_w + e)][(7-i)*4+:4]);
                end
                line[81] <= 8'h0d;
                line[82] <= 8'h0a;
                llen <= 7'd83;
                lpos <= '0;
                lbusy <= '1;
            end
        end

        if (VPU_DCK & (VPU_VD != '0)) begin
            px_last <= {VPU_VDMODE, VPU_VD};
            if (~(tick_pend & ~lbusy))
                px_cnt <= px_cnt + 1'd1;
        end

        // Register write snoop (first cycle of each write)
        wr_d <= ~IO_WRn & ~(VCE_CSn & VPU_CSn);
        if (~IO_WRn & ~(VCE_CSn & VPU_CSn) & ~wr_d) begin
            if (~VCE_CSn) begin
                if (~IO_A[2])
                    vce_ar <= IO_DI[4:0];
                else if ((vce_ar != 5'd2) & ~vw_pend) begin
                    vw_reg <= vce_ar;
                    vw_val <= IO_DI;
                    vw_pend <= '1;
                end
            end
            else if (~qw_pend) begin
                qw_reg <= IO_A;
                qw_val <= IO_DI;
                qw_pend <= '1;
            end
        end

        if (~lbusy & ~cmd_pend & ~stat_pend & ~isr_pend & ~tick_pend & vw_pend) begin
            vw_pend <= '0;
            line[0] <= "V";
            line[1] <= " ";
            line[2] <= hexc({3'b0, vw_reg[4]});
            line[3] <= hexc(vw_reg[3:0]);
            line[4] <= " ";
            for (int i = 0; i < 4; i++)
                line[5 + i] <= hexc(vw_val[(3-i)*4+:4]);
            line[9] <= 8'h0d;
            line[10] <= 8'h0a;
            llen <= 7'd11;
            lpos <= '0;
            lbusy <= '1;
        end
        else if (~lbusy & ~cmd_pend & ~stat_pend & ~isr_pend & ~tick_pend & qw_pend) begin
            qw_pend <= '0;
            line[0] <= "Q";
            line[1] <= " ";
            line[2] <= hexc(qw_reg);
            line[3] <= " ";
            for (int i = 0; i < 4; i++)
                line[4 + i] <= hexc(qw_val[(3-i)*4+:4]);
            line[8] <= 8'h0d;
            line[9] <= 8'h0a;
            llen <= 7'd10;
            lpos <= '0;
            lbusy <= '1;
        end

        // RAINBOW pixel line after the R line
        if (~lbusy & ~cmd_pend & ~stat_pend & ~isr_pend & ~tick_pend & ~vw_pend & ~qw_pend & ~rb_pend & px_pend) begin
            px_pend <= '0;
            line[0] <= "P";
            line[1] <= " ";
            line[2] <= hexc({3'b0, px_last_l[24]});
            line[3] <= " ";
            for (int i = 0; i < 6; i++)
                line[4 + i] <= hexc(px_last_l[(5-i)*4+:4]);
            line[10] <= " ";
            for (int i = 0; i < 6; i++)
                line[11 + i] <= hexc(px_cnt_l[(5-i)*4+:4]);
            line[17] <= 8'h0d;
            line[18] <= 8'h0a;
            llen <= 7'd19;
            lpos <= '0;
            lbusy <= '1;
        end

        // RAINBOW line after the B line
        if (~lbusy & ~cmd_pend & ~stat_pend & ~isr_pend & ~tick_pend & ~vw_pend & ~qw_pend & rb_pend) begin
            rb_pend <= '0;
            line[0] <= "R";
            line[1] <= " ";
            for (int i = 0; i < 8; i++)
                line[2 + i] <= hexc(rb_l[(7-i)*4+:4]);
            line[10] <= 8'h0d;
            line[11] <= 8'h0a;
            llen <= 7'd12;
            lpos <= '0;
            lbusy <= '1;
        end

        // Send the line buffer
        if (tx_busy) begin
            if (baud_cnt == '0) begin
                baud_cnt <= 16'(DIV - 1);
                TXD <= shreg[0];
                shreg <= {1'b1, shreg[9:1]};
                bit_cnt <= bit_cnt - 1'd1;
                if (bit_cnt == 4'd0) begin
                    tx_busy <= '0;
                    TXD <= 1'b1;
                end
            end
            else
                baud_cnt <= baud_cnt - 1'd1;
        end
        else if (lbusy) begin
            if (lpos == llen) begin
                lbusy <= '0;
            end
            else begin
                shreg <= {1'b1, line[lpos], 1'b0};
                lpos <= lpos + 1'd1;
                bit_cnt <= 4'd10;
                baud_cnt <= '0;
                tx_busy <= '1;
            end
        end
    end
end

endmodule
