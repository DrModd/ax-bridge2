`timescale 1ns/1ps
// ax-bridge2: AX5689 input bridge with two audio master clocks
// (Tang Primer 25K, GW5A-LV25MG121NC1/I0).
//
// Clocks
//   sys_clk  50 MHz (oscillator on the module): service domain - I2C and
//            registers, S/PDIF receiver, frequency meter, input switching,
//            clock manager, timers. Never stops.
//   mclk_in  audio master clock from the oscillator multiplexer on the
//            amplifier board: 45.1584 MHz (44.1 kHz family) or 49.152 MHz
//            (48 kHz family), chosen by mclk_sel. The same MCLK goes to the
//            Fox (external-clock mode) and to the AX5689. Audio core only.
//   input BCLKs: front end (bclk_rx + FIFO write side), selected by in_mux.
//
// Clock manager (auto mode)
//   input 0 (Fox): mclk_sel follows the Fox's own MCLK_SEL pin (fox_fsel),
//                  so the Fox plays from our oscillator: synchronous, the
//                  ASRC is bypassed (core.v);
//   input 4 (phono ADC, SYNC_IN4 = 1): the AK5572 gets its MCLK from the
//                  same oscillator mux, so it is synchronous too: no ASRC;
//                  the meter sees our own family, so the clock never moves;
//   other inputs:  the frequency meter (sys_clk) measures the source and
//                  mclk_sel picks its family; FIFO + ASRC with a ratio ~1
//                  only absorbs the difference between the crystals.
//   Switching: core silent and stopped -> (optional STM32 acknowledge, so
//   it can mute and later re-initialise the AX5689) -> front end and core
//   in reset -> mclk_sel -> 20 ms for the oscillators -> release.
//
//   5 inputs: 0 = Fox (PCM/DoP/DSD), 1 = S/PDIF (4 lines), 2 = I2S PCM
//   (Bluetooth), 3 = HDMI (PCM/DoP/DSD), 4 = phono / line ADC (AK5572).
//   Output: I2S, 32-bit slots, MCLK/256 = 176.4 or 192 kHz.
module top #(
    parameter WBITS = 18,                 // core rate window 2^WBITS MCLK
    parameter GSH   = 0,                  // DPLL gear length shift (sim only)
    parameter DSD_SDATA_IS_RIGHT = 1,
    parameter USE_DCS = 0,
    parameter SYNC_IN4 = 1,               // phono ADC runs from our MCLK (AK5572 MCLK from the oscillator mux)
    parameter [6:0] I2C_ADDR = 7'h5A,
    parameter [3:0] ATT_DEFAULT = 4'd0,
    parameter integer SYS_HZ = 50000000,
    parameter integer CLK_SETTLE_MS = 20, // oscillator start-up after a switch
    parameter integer ACK_WAIT_MS = 300,  // STM32 acknowledge time-out
    parameter MWBITS = 18,                // frequency meter window 2^MWBITS sys clocks
    parameter [3:0] CLK_CTRL_RST = 4'b1101,   // CLK_CTRL after reset (0x14); 4'b0000 = fixed 49.152 MHz, no sync path
    parameter COEF_DIR = "coef/"
) (
    input  wire sys_clk,        // 50 MHz service clock
    input  wire mclk_in,        // audio MCLK (45.1584 / 49.152 MHz)
    output reg  mclk_sel,       // 1: 45.1584 MHz, 0: 49.152 MHz (same polarity as the Fox's MCLK_SEL)
    input  wire fox_fsel,       // Fox MCLK_SEL (GPIO1_D1): 1 = 44.1 kHz family
    input  wire i2s_bclk_in,    // input 0 (Fox): BCLK / DSD bit clock
    input  wire i2s_lrck_in,
    input  wire i2s_sdata_in,
    input  wire dsd_on,
    input  wire [3:0] spdif_in, // input 1: 0 coax, 1 optical, 2 AES/EBU, 3 HDMI ARC
    input  wire in2_bclk, in2_lrck, in2_sdata,
    input  wire in3_bclk, in3_lrck, in3_sdata,
    input  wire in3_dsd,
    input  wire in4_bclk, in4_lrck, in4_sdata,
    input  wire i2c_scl,
    inout  wire i2c_sda,
    output wire i2s_bclk_out,
    output wire i2s_lrck_out,
    output wire i2s_sdata_out,
    output wire audio_on
);
    localparam integer MS = SYS_HZ / 1000;

    // ================= service domain (sys_clk) =================
    reg [7:0] por = 8'd0;
    always @(posedge sys_clk) if (~&por) por <= por + 8'd1;
    wire rst_n = &por;

    // ---------------- registers ----------------
    reg  [2:0] ctl_sel;
    reg  [1:0] ctl_sline;
    reg        ctl_mute;
    reg  [3:0] ctl_att;
    reg  [7:0] ctl_dmode;
    reg  [3:0] ctl_dinv, ctl_dswap;
    reg  [3:0] ctl_clk;          // [0] auto, [1] manual family (1 = 44.1), [2] sync Fox, [3] wait for ack

    // ---------------- input selection state ----------------
    reg  [2:0] sel_cur;
    reg  [1:0] sline_cur;
    reg        fe_run;           // front end out of reset
    reg        core_run;         // core out of reset (MCLK valid)
    reg        stop;             // core: silent, do not start

    // ---------------- front end (input clocks) ----------------
    wire m_bclk, m_lrck, m_sdata, m_dsd, m_dauto, m_drs;
    wire [4:0] mux_active;
    in_mux #(.USE_DCS(USE_DCS)) u_mux (
        .clk(sys_clk), .rst_n(rst_n), .sel(sel_cur),
        .bclk_in({in4_bclk, in3_bclk, in2_bclk, 1'b0, i2s_bclk_in}),   // input 1: S/PDIF
        .lrck_in({in4_lrck, in3_lrck, in2_lrck, 1'b0, i2s_lrck_in}),
        .sdata_in({in4_sdata, in3_sdata, in2_sdata, 1'b0, i2s_sdata_in}),
        .dsd_on0(dsd_on), .dsd_on3(in3_dsd),
        .dsd_mode(ctl_dmode), .dsd_inv(ctl_dinv), .dsd_swap(ctl_dswap),
        .bclk(m_bclk), .lrck(m_lrck), .sdata(m_sdata), .dsd_on(m_dsd),
        .dsd_auto(m_dauto), .dsd_rs(m_drs),
        .active(mux_active)
    );

    wire fe_arst_n = rst_n & fe_run;
    reg [1:0] rst_b;
    always @(posedge m_bclk or negedge fe_arst_n)
        if (!fe_arst_n) rst_b <= 2'b00; else rst_b <= {rst_b[0], 1'b1};

    wire        rx_we;
    wire [64:0] rx_wd;
    bclk_rx u_rx (
        .bclk(m_bclk), .rst_n(rst_b[1]),
        .lrck(m_lrck), .sdata(m_sdata), .dsd_on(m_dsd), .auto(m_dauto), .rs(m_drs),
        .wr_en(rx_we), .wr_data(rx_wd)
    );
    // item toggle + DSD tag for the frequency meter
    reg rx_tgl, rx_dsd;
    always @(posedge m_bclk or negedge rst_b[1])
        if (!rst_b[1]) begin rx_tgl <= 1'b0; rx_dsd <= 1'b0; end
        else if (rx_we) begin rx_tgl <= ~rx_tgl; rx_dsd <= rx_wd[64]; end

    // read sides of both FIFOs: MCLK domain, released synchronously
    wire mc_arst_n = rst_n & fe_run & core_run;
    reg [1:0] rst_m;
    always @(posedge mclk_in or negedge mc_arst_n)
        if (!mc_arst_n) rst_m <= 2'b00; else rst_m <= {rst_m[0], 1'b1};

    wire        f_empty, f_full, f_re;
    wire [64:0] f_rd;
    afifo #(.W(65), .AW(4)) u_fifo (
        .wclk(m_bclk), .wrst_n(rst_b[1]), .we(rx_we), .wdata(rx_wd), .full(f_full),
        .rclk(mclk_in), .rrst_n(rst_m[1]), .re(f_re), .rdata(f_rd), .empty(f_empty)
    );

    // ---------------- input 1: S/PDIF receiver (sys_clk) ----------------
    wire        sp_valid, sp_locked, sp_nonaudio, sp_err;
    wire [31:0] sp_l, sp_r;
    wire [7:0]  sp_cs0, sp_cs3;
    spdif_rx u_spdif (
        .clk(sys_clk), .rst_n(fe_arst_n), .line(spdif_in), .sel(sline_cur),
        .valid(sp_valid), .l(sp_l), .r(sp_r), .locked(sp_locked),
        .nonaudio(sp_nonaudio), .err(sp_err), .cs0(sp_cs0), .cs3(sp_cs3)
    );
    wire        s_empty, s_full, s_re;
    wire [63:0] s_rd;
    afifo #(.W(64), .AW(3)) u_sfifo (
        .wclk(sys_clk), .wrst_n(fe_arst_n), .we(sp_valid && sel_cur == 3'd1), .wdata({sp_l, sp_r}),
        .full(s_full),
        .rclk(mclk_in), .rrst_n(rst_m[1]), .re(s_re), .rdata(s_rd), .empty(s_empty)
    );
    wire [4:0] in_active = {mux_active[4:2], sp_locked, mux_active[0]};

    // ---------------- frequency meter (sys_clk) ----------------
    // items of the selected input per 2^MWBITS sys clocks -> rate class and family
    reg [2:0] mt_s, md_s;
    always @(posedge sys_clk) begin mt_s <= {mt_s[1:0], rx_tgl}; md_s <= {md_s[1:0], rx_dsd}; end
    wire m_item = (sel_cur == 3'd1) ? sp_valid : (mt_s[2] != mt_s[1]);
    wire mt_isdsd = (sel_cur == 3'd1) ? 1'b0 : md_s[2];
    localparam integer MN44 = ((1 << MWBITS) * 882 + SYS_HZ / 100) / (SYS_HZ / 50);   // rounded
    localparam integer MN48 = ((1 << MWBITS) * 960 + SYS_HZ / 100) / (SYS_HZ / 50);
    wire       mt_stable, mt_fam48, mt_dsd, mt_dop, mt_seq;
    wire [2:0] mt_k;
    rate_mon #(.WBITS(MWBITS), .N44_0(MN44), .N48_0(MN48), .N44_1(MN44), .N48_1(MN48)) u_meter (
        .clk(sys_clk), .rst_n(fe_arst_n), .item(m_item), .item_dsd(mt_isdsd), .dop(1'b0), .alt(1'b0),
        .stable(mt_stable), .k(mt_k), .fam48(mt_fam48), .dsd(mt_dsd), .is_dop(mt_dop), .seq(mt_seq)
    );

    // ---------------- status from the core ----------------
    wire [31:0] cst_a;
    reg  [31:0] st1, st2, st3, cst_q;
    reg  [1:0]  st_n;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            st1 <= 32'd0; st2 <= 32'd0; st3 <= 32'd0; cst_q <= 32'd0; st_n <= 2'd0;
        end else begin
            st1 <= core_run ? cst_a : 32'd0; st2 <= st1; st3 <= st2;
            if (st3 != st2) st_n <= 2'd0;
            else if (st_n != 2'd3) st_n <= st_n + 2'd1;
            else cst_q <= st3;
        end
    end
    wire       c_running = cst_q[6];
    wire       c_run     = cst_q[7];
    wire       c_idle    = cst_q[12] || !core_run;
    reg        c_sw;                           // core freshly released: its control levels settle
    wire [2:0] cfg_k     = cst_q[2:0];
    wire       cfg_fam48 = cst_q[3];
    wire       cfg_dsd   = cst_q[4];
    wire       cfg_dop   = cst_q[5];

    // ---------------- clock manager / input sequencer ----------------
    reg [2:0] fs_s;
    always @(posedge sys_clk) fs_s <= {fs_s[1:0], fox_fsel};
    wire fox44 = fs_s[2];

    localparam Q_IDLE = 3'd0, Q_STOP = 3'd1, Q_ACK = 3'd2, Q_SWITCH = 3'd3,
               Q_SETTLE = 3'd4, Q_MEASURE = 3'd5;
    reg  [2:0]  q;
    reg  [24:0] t;                 // timer, sys clocks
    reg         want44;            // family wanted for the selected input
    reg         clk_chg;           // the running sequence changes the clock
    reg         ack;               // STM32 acknowledge (write 0x15 bit 0)
    reg         clk_req;           // waiting for the acknowledge
    wire        in_chg = (ctl_sel != sel_cur) || (ctl_sline != sline_cur);
    // family wanted for an input: Fox -> its pin; others -> meter (else keep)
    wire        meas44 = mt_stable ? ~mt_fam48 : mclk_sel;
    // auto = 0 and sync = 0: the all-asynchronous mode - every input through
    // the full ASRC, one fixed output rate, no oscillator switching. It is
    // always 49.152 MHz / 192 kHz: the ASRC converts 1:1 and 44.1 -> 48 but
    // not 48 -> 44.1, so a fixed 45.1584 MHz would reject 48 kHz sources.
    wire        all_async = !ctl_clk[0] && !ctl_clk[2];
    wire        want_now = !ctl_clk[0] ? (ctl_clk[1] && !all_async) : (sel_cur == 3'd0 ? fox44 : meas44);

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            q <= Q_SETTLE; t <= 25'd0; mclk_sel <= 1'b0; want44 <= 1'b0; clk_chg <= 1'b1;
            sel_cur <= 3'd0; sline_cur <= 2'd0; fe_run <= 1'b0; core_run <= 1'b0; stop <= 1'b1;
            clk_req <= 1'b0;
        end else begin
            if (~&t) t <= t + 25'd1;
            case (q)
            Q_IDLE: begin
                stop <= 1'b0;
                if (in_chg || want_now != mclk_sel) begin
                    stop <= 1'b1;                     // core fades out and stops
                    want44 <= want_now;
                    t <= 25'd0;
                    q <= Q_STOP;
                end
            end
            Q_STOP:                                   // wait for silence (fade: ~5 ms)
                if ((c_idle && t > 25'd1000) || t >= 30 * MS) begin
                    clk_chg <= !in_chg && (want44 != mclk_sel);
                    t <= 25'd0;
                    if (!in_chg && want44 != mclk_sel && ctl_clk[3]) begin
                        clk_req <= 1'b1;              // let the STM32 mute / prepare the AX5689
                        q <= Q_ACK;
                    end else
                        q <= Q_SWITCH;
                end
            Q_ACK:
                if (ack || t >= ACK_WAIT_MS * MS) begin
                    clk_req <= 1'b0; t <= 25'd0; q <= Q_SWITCH;
                end
            Q_SWITCH: begin
                fe_run <= 1'b0;                       // front end in reset
                if (t == 25'd100) begin               // reset has settled
                    if (clk_chg) begin
                        core_run <= 1'b0;
                        mclk_sel <= want44;
                    end else begin
                        sel_cur <= ctl_sel;
                        sline_cur <= ctl_sline;
                        // a new Fox input brings its family with it
                        if (ctl_clk[0] && ctl_sel == 3'd0 && fox44 != mclk_sel) begin
                            core_run <= 1'b0;
                            mclk_sel <= fox44;
                            clk_chg <= 1'b1;
                            end
                    end
                    t <= 25'd0;
                    q <= Q_SETTLE;
                end
            end
            Q_SETTLE:
                if (t >= (clk_chg ? CLK_SETTLE_MS * MS : 2048)) begin
                    fe_run <= 1'b1;
                    core_run <= 1'b1;
                    t <= 25'd0;
                    q <= Q_MEASURE;
                end
            Q_MEASURE:
                // a non-Fox input: let the meter find the family before the
                // core may start (two 5 ms windows), so it does not start on
                // the wrong clock only to be switched a moment later
                if (sel_cur == 3'd0 || !ctl_clk[0] || mt_stable || t >= 40 * MS) begin
                    q <= Q_IDLE;
                    clk_chg <= 1'b0;
                end
            default: q <= Q_IDLE;
            endcase
        end
    end

    // core-ready window: ~10 us after release (reset synchroniser, control levels)
    reg [9:0] c_sw_n;
    always @(posedge sys_clk or negedge rst_n)
        if (!rst_n) begin c_sw <= 1'b1; c_sw_n <= 10'd0; end
        else if (!core_run) begin c_sw <= 1'b1; c_sw_n <= 10'd0; end
        else if (c_sw) begin c_sw_n <= c_sw_n + 10'd1; if (&c_sw_n) c_sw <= 1'b0; end

    // ---------------- audio core (MCLK) ----------------
    reg         eq_tgl, eq_apl_tgl;
    reg  [6:0]  eq_idx, eq_widx;
    reg  [31:0] eq_wdat;
    reg  [23:0] eq_sh;
    reg  [1:0]  eq_bc;
    reg  [3:0]  eq_nsec_new;
    core #(.WBITS(WBITS), .GSH(GSH), .SYNC_IN4(SYNC_IN4), .COEF_DIR(COEF_DIR)) u_core (
        .clk(mclk_in), .arst_n(rst_n & core_run),
        .f_empty(f_empty), .f_rd(f_rd), .f_re(f_re),
        .s_empty(s_empty), .s_rd(s_rd), .s_re(s_re),
        .sel_a(sel_cur), .att_a(ctl_att), .mute_a(ctl_mute), .stop_a(stop),
        .mfam44_a(mclk_sel), .sync_en_a(ctl_clk[2]),
        .eq_tgl_a(eq_tgl), .eq_widx_a(eq_widx), .eq_wdat_a(eq_wdat),
        .eq_apl_tgl_a(eq_apl_tgl), .eq_nsec_new_a(eq_nsec_new),
        .stat(cst_a),
        .i2s_bclk_out(i2s_bclk_out), .i2s_lrck_out(i2s_lrck_out), .i2s_sdata_out(i2s_sdata_out),
        .audio_on(audio_on)
    );

    // ---------------- I2C (sys_clk) ----------------
    // 0x00 CTRL    rw  [2:0] input 0..4 (5..7 -> 4), [7] mute
    // 0x01 STATUS  r   [0] audio on, [1] locking/locked, [3:2] input [1:0], [7:4] inputs 0..3 live
    // 0x02 FORMAT  r   [2:0] k, [3] 48k family, [4] native DSD, [5] DoP, [7] valid
    // 0x03 RATE    r   0 none; PCM 1..10 = 44.1..768; 0x11..0x14 DSD64..512; 0x21..0x23 DoP (+0x08 48k)
    // 0x04 PLL     r   DPLL gear 0..4 (4 = synchronous path, no ASRC)
    // 0x05 ID      r   0xA5
    // 0x06 VERSION r   0x20 (ax-bridge2)
    // 0x07 ATT     rw  PCM headroom 0..12 x 0.5 dB
    // 0x08 CLIP    r   [0] within ~85 ms, [7] sticky (w: clear)
    // 0x09..0x0C   EQ (as ax-bridge; coefficients for the CURRENT output rate,
    //              176.4 or 192 kHz - reload after a clock switch: the core
    //              is reset and comes back with the EQ off)
    // 0x0D DSD_MODE, 0x0E DSD_CFG (as ax-bridge)
    // 0x0F INPUTS  r   [4:0] live inputs, [7:5] current input
    // 0x10..0x13   S/PDIF (as ax-bridge 0x07)
    // 0x14 CLK_CTRL rw [0] auto (1), [1] manual family when auto = 0 (1 = 44.1 kHz,
    //                  45.1584 MHz; only with [2] = 1), [2] synchronous path for
    //                  the Fox and the phono ADC (1), [3] wait for the STM32
    //                  acknowledge before switching the clock (1); reset 0x0D.
    //                  Modes: 0x0D two oscillators, sync Fox/ADC, async inputs via
    //                  FIFO+ASRC 1:1 (default); 0x00 ALL-ASYNC: 49.152 MHz fixed,
    //                  192 kHz out, every input (Fox and ADC too) through the full
    //                  ASRC (44.1 -> 48 included), the oscillator never switches.
    // 0x15 CLK_STAT r  [0] MCLK 45.1584 MHz, [1] switch request waiting for the
    //                  acknowledge, [2] switching / settling, [3] synchronous path
    //                  active, [4] meter valid, [5] meter: 44.1 family, [6] audio core
    //                  running (EQ writes are taken only then), [7] the
    //                  clock was switched since the last clear
    //              w   bit 0 = 1: acknowledge (switch now); bit 7 = 1: clear [7]
    // 0x16 METER   r   meter: [2:0] k, [3] 48k family, [4] DSD, [7] valid
    wire       sda_oe;
    wire [7:0] i2c_ptr, i2c_waddr, i2c_wdata;
    wire       i2c_wr;
    reg  [7:0] i2c_rd;
    reg  [7:0] rate_code;
    assign i2c_sda = sda_oe ? 1'b0 : 1'bz;
    i2c_slave #(.ADDR(I2C_ADDR), .PORT(8'h0C)) u_i2c (
        .clk(sys_clk), .rst_n(rst_n), .scl_in(i2c_scl), .sda_in(i2c_sda),
        .sda_oe(sda_oe), .ptr(i2c_ptr), .rdata(i2c_rd),
        .wr(i2c_wr), .waddr(i2c_waddr), .wdata(i2c_wdata));

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            ctl_sel <= 3'd0; ctl_sline <= 2'd0; ctl_mute <= 1'b0; ctl_att <= ATT_DEFAULT;
            ctl_dmode <= 8'hC1; ctl_dinv <= 4'd0;
            ctl_dswap <= {3'b111, (DSD_SDATA_IS_RIGHT != 0)};
            ctl_clk <= CLK_CTRL_RST;
            ack <= 1'b0;
        end else begin
            ack <= 1'b0;
            if (i2c_wr) case (i2c_waddr)
                8'h00: begin ctl_sel <= (i2c_wdata[2:0] > 3'd4) ? 3'd4 : i2c_wdata[2:0]; ctl_mute <= i2c_wdata[7]; end
                8'h07: ctl_att <= (i2c_wdata[3:0] > 4'd12) ? 4'd12 : i2c_wdata[3:0];
                8'h0D: ctl_dmode <= i2c_wdata;
                8'h0E: begin ctl_dswap <= i2c_wdata[3:0]; ctl_dinv <= i2c_wdata[7:4]; end
                8'h10: ctl_sline <= i2c_wdata[1:0];
                8'h14: ctl_clk <= i2c_wdata[3:0];
                8'h15: ack <= i2c_wdata[0];
                default: ;
            endcase
        end
    end
    reg clk_sw_seen, mclk_sel_q;
    always @(posedge sys_clk or negedge rst_n)
        if (!rst_n) begin clk_sw_seen <= 1'b0; mclk_sel_q <= 1'b0; end
        else begin
            mclk_sel_q <= mclk_sel;
            if (mclk_sel != mclk_sel_q) clk_sw_seen <= 1'b1;
            else if (i2c_wr && i2c_waddr == 8'h15 && i2c_wdata[7]) clk_sw_seen <= 1'b0;
        end

    // EQ coefficient port -> toggles for the core
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            eq_tgl <= 1'b0; eq_apl_tgl <= 1'b0; eq_idx <= 7'd0; eq_widx <= 7'd0;
            eq_wdat <= 32'd0; eq_sh <= 24'd0; eq_bc <= 2'd0; eq_nsec_new <= 4'd0;
        end else if (i2c_wr) begin
            case (i2c_waddr)
                8'h09: if (i2c_wdata[0]) eq_apl_tgl <= ~eq_apl_tgl;
                8'h0A: eq_nsec_new <= (i2c_wdata[3:0] > 4'd10) ? 4'd10 : i2c_wdata[3:0];
                8'h0B: begin eq_idx <= i2c_wdata[6:0]; eq_bc <= 2'd0; end
                8'h0C: begin
                    if (eq_bc == 2'd3) begin
                        eq_widx <= eq_idx;
                        eq_wdat <= {eq_sh, i2c_wdata};
                        eq_tgl <= ~eq_tgl;
                        eq_idx <= eq_idx + 7'd1;
                    end else
                        eq_sh <= {eq_sh[15:0], i2c_wdata};
                    eq_bc <= eq_bc + 2'd1;
                end
                default: ;
            endcase
        end
    end

    // sticky flags
    reg clip_sticky, sp_err_sticky;
    reg [21:0] sp_err_tmr;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            clip_sticky <= 1'b0; sp_err_sticky <= 1'b0; sp_err_tmr <= 22'd0;
        end else begin
            if (cst_q[11]) clip_sticky <= 1'b1;
            else if (i2c_wr && i2c_waddr == 8'h08) clip_sticky <= 1'b0;
            if (sp_err && sp_locked) sp_err_tmr <= {22{1'b1}};
            else if (sp_err_tmr != 22'd0) sp_err_tmr <= sp_err_tmr - 22'd1;
            if (sp_err && sp_locked) sp_err_sticky <= 1'b1;
            else if (i2c_wr && i2c_waddr == 8'h11) sp_err_sticky <= 1'b0;
        end
    end

    always @(*) begin
        if (!c_running)
            rate_code = 8'h00;
        else if (cfg_dsd)
            rate_code = {4'h1, cfg_fam48, cfg_k - 3'd1};
        else if (cfg_dop)
            rate_code = {4'h2, cfg_fam48, cfg_k - 3'd1};
        else
            rate_code = 8'd1 + {4'd0, cfg_k, 1'b0} + {7'd0, cfg_fam48};
        case (i2c_ptr)
            8'h00: i2c_rd = {ctl_mute, 4'd0, ctl_sel};
            8'h01: i2c_rd = {in_active[3:0], sel_cur[1:0], c_running, c_run};
            8'h02: i2c_rd = {c_running, 1'b0, cfg_dop, cfg_dsd, cfg_fam48, cfg_k};
            8'h03: i2c_rd = rate_code;
            8'h04: i2c_rd = {5'd0, cst_q[10:8]};
            8'h05: i2c_rd = 8'hA5;
            8'h06: i2c_rd = 8'h20;
            8'h07: i2c_rd = {4'd0, ctl_att};
            8'h08: i2c_rd = {clip_sticky, 6'd0, cst_q[11]};
            8'h09: i2c_rd = {cst_q[17:14], 2'd0, cst_q[18], cst_q[19]};
            8'h0A: i2c_rd = {4'd0, eq_nsec_new};
            8'h0B: i2c_rd = {1'b0, eq_idx};
            8'h0D: i2c_rd = ctl_dmode;
            8'h0E: i2c_rd = {ctl_dinv, ctl_dswap};
            8'h0F: i2c_rd = {sel_cur, in_active};
            8'h10: i2c_rd = {6'd0, ctl_sline};
            8'h11: i2c_rd = {sp_err_sticky, 4'd0, sp_err_tmr != 22'd0, sp_nonaudio, sp_locked};
            8'h12: i2c_rd = sp_cs0;
            8'h13: i2c_rd = sp_cs3;
            8'h14: i2c_rd = {4'd0, ctl_clk};
            8'h15: i2c_rd = {clk_sw_seen, core_run & ~c_sw, ~mt_fam48 & mt_stable, mt_stable, cst_q[13],
                             q == Q_SWITCH || q == Q_SETTLE, clk_req, mclk_sel};
            8'h16: i2c_rd = {mt_stable, 2'd0, mt_dsd, mt_fam48, mt_k};
            default: i2c_rd = 8'h00;
        endcase
    end
endmodule
