`timescale 1ns/1ps
// Audio core of ax-bridge2: everything that runs on the audio master clock
// MCLK (45.1584 MHz for the 44.1 kHz family, 49.152 MHz for 48 kHz; the
// clock manager in top.v picks the oscillator). Output I2S = MCLK/256:
// 176.4 or 192 kHz, 32-bit slots.
//
//   FIFO items (input front end or the S/PDIF receiver)
//     -> PCM headroom -> half-band x2/x4 / /2 /4, or CIC5 + FIR for DSD
//        (all five FIRs multiplier-free: shift-and-add, rtl/fir_csd.v)
//     -> 176.4 / 192 kHz intermediate stream
//     -> synchronous source (Fox on our MCLK): small elastic buffer, no ASRC
//        any other source: ASRC with a nominal ratio of 1 (it only absorbs
//        the difference between the two crystals)
//     -> fade -> room-correction EQ -> I2S transmitter
//
// Control comes from the 50 MHz service domain (top.v) as quasi-static
// levels; every level is synchronised and taken only after it has been
// stable for 4 clocks, so a multi-bit value is never seen half-changed.
// EQ coefficient words / apply arrive with toggle handshakes. The whole
// core is held in reset while the clock manager switches MCLK.
module core #(
    parameter WBITS = 18,
    parameter GSH   = 0,
    parameter COEF_DIR = "coef/"
) (
    input  wire        clk,            // MCLK
    input  wire        arst_n,         // async reset from the service domain
    // FIFO from the I2S/DSD front end (show-ahead)
    input  wire        f_empty,
    input  wire [64:0] f_rd,
    output reg         f_re,
    // FIFO from the S/PDIF receiver (show-ahead)
    input  wire        s_empty,
    input  wire [63:0] s_rd,
    output reg         s_re,
    // control (service domain levels)
    input  wire [2:0]  sel_a,          // current input 0..4
    input  wire [3:0]  att_a,          // PCM headroom
    input  wire        mute_a,
    input  wire        stop_a,         // input / clock switch: go silent, do not start
    input  wire        mfam44_a,       // 1: MCLK is 45.1584 MHz
    input  wire        sync_en_a,      // allow the synchronous (no-ASRC) path for input 0
    input  wire        eq_tgl_a,       // EQ word ready (toggle)
    input  wire [6:0]  eq_widx_a,
    input  wire [31:0] eq_wdat_a,
    input  wire        eq_apl_tgl_a,   // EQ apply (toggle)
    input  wire [3:0]  eq_nsec_new_a,
    // status (to the service domain, quasi-static)
    output wire [31:0] stat,
    // I2S out
    output wire        i2s_bclk_out,
    output wire        i2s_lrck_out,
    output wire        i2s_sdata_out,
    output wire        audio_on
);
    // ---------------- reset ----------------
    reg [1:0] rs;
    always @(posedge clk or negedge arst_n)
        if (!arst_n) rs <= 2'b00; else rs <= {rs[0], 1'b1};
    wire rst_n = rs[1];

    // ---------------- control from the service domain ----------------
    localparam CW = 3 + 4 + 1 + 1 + 1 + 1 + 4;
    wire [CW-1:0] c_a = {sel_a, att_a, mute_a, stop_a, mfam44_a, sync_en_a, eq_nsec_new_a};
    reg  [CW-1:0] c_s1, c_s2, c_s3, c_q;
    reg  [1:0]    c_st;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_s1 <= {CW{1'b0}}; c_s2 <= {CW{1'b0}}; c_s3 <= {CW{1'b0}}; c_st <= 2'd0;
            c_q <= {CW{1'b0}} | (1'b1 << 6);            // stop until the levels are in
        end else begin
            c_s1 <= c_a; c_s2 <= c_s1; c_s3 <= c_s2;
            if (c_s3 != c_s2) c_st <= 2'd0;
            else if (c_st != 2'd3) c_st <= c_st + 2'd1;
            else c_q <= c_s3;
        end
    end
    wire [2:0] sel        = c_q[14:12];
    wire [3:0] ctl_att    = c_q[11:8];
    wire       mute       = c_q[7] | c_q[6];          // mute or stop: no audio
    wire       stop_req   = c_q[6];
    wire       mfam44     = c_q[5];
    wire       sync_en    = c_q[4];
    wire [3:0] eq_nsec_new = c_q[3:0];

    // EQ words / apply: toggles; the data has been stable for a long time
    // when the toggle is seen (one word per 4 I2C bytes)
    reg [2:0] et, at;
    reg       e_init;
    reg       eq_we, eq_apply;
    reg [6:0] eq_widx;
    reg [31:0] eq_wdat;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            et <= 3'd0; at <= 3'd0; e_init <= 1'b0; eq_we <= 1'b0; eq_apply <= 1'b0;
            eq_widx <= 7'd0; eq_wdat <= 32'd0;
        end else begin
            et <= {et[1:0], eq_tgl_a};
            at <= {at[1:0], eq_apl_tgl_a};
            e_init <= 1'b1;                           // ignore the first comparison after reset
            eq_we <= e_init && (et[2] != et[1]);
            eq_apply <= e_init && (at[2] != at[1]);
            if (et[2] != et[1]) begin eq_widx <= eq_widx_a; eq_wdat <= eq_wdat_a; end
        end
    end

    // ---------------- item intake (MCLK) ----------------
    // At most one item per 8 MCLK (the densest input, DSD512, delivers
    // one item per ~35 MCLK) so a FIFO backlog never floods a stage queue.
    reg [2:0]  thr;
    reg        item;
    reg        item_dsd;
    reg [31:0] item_l, item_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            thr <= 3'd0; f_re <= 1'b0; s_re <= 1'b0; item <= 1'b0; item_dsd <= 1'b0;
            item_l <= 32'd0; item_r <= 32'd0;
        end else begin
            f_re <= 1'b0;
            item <= 1'b0;
            if (thr != 3'd0) thr <= thr - 3'd1;
            s_re <= 1'b0;
            if (sel == 3'd1) begin
                // S/PDIF frames: from the receiver on the 50 MHz clock (<= 192 kHz)
                if (!s_empty && !s_re && thr == 3'd0) begin
                    s_re <= 1'b1;
                    item <= 1'b1; item_dsd <= 1'b0;
                    item_l <= s_rd[63:32]; item_r <= s_rd[31:0];
                    thr <= 3'd7;
                end
            end else if (!f_empty && !f_re && thr == 3'd0) begin
                f_re <= 1'b1;
                item <= 1'b1;
                item_dsd <= f_rd[64];
                item_l <= f_rd[63:32];
                item_r <= f_rd[31:0];
                thr <= 3'd7;
            end
        end
    end
    wire item_pcm = item & ~item_dsd;

    wire dop;
    dop_det u_dop (
        .clk(clk), .rst_n(rst_n), .frame(item_pcm), .l(item_l), .r(item_r), .dop(dop)
    );

    wire       rm_stable, rm_fam48, rm_dsd, rm_dop, rm_seq;
    wire [2:0] rm_k;
    rate_mon #(.WBITS(WBITS)) u_rm (
        .clk(clk), .rst_n(rst_n), .item(item), .item_dsd(item_dsd), .dop(dop), .alt(mfam44),
        .stable(rm_stable), .k(rm_k), .fam48(rm_fam48), .dsd(rm_dsd),
        .is_dop(rm_dop), .seq(rm_seq)
    );

    // ---------------- configuration / control ----------------
    localparam P_X4 = 3'd0, P_X2 = 3'd1, P_X1 = 3'd2, P_D2 = 3'd3,
               P_D4 = 3'd4, P_DSD = 3'd5;
    localparam C_IDLE = 3'd0, C_FILL = 3'd1, C_LOCK = 3'd2, C_RUN = 3'd3,
               C_FADE = 3'd4;

    // class -> path (combinational, from rate_mon)
    reg [2:0] cls_path;
    reg       cls_ok;
    always @(*) begin
        // 48 kHz family on the 45.1584 MHz clock is not played: the
        // controller switches the clock to 49.152 MHz first
        cls_ok = rm_stable && !(rm_fam48 && mfam44);
        cls_path = P_X1;
        if (rm_dsd) begin
            cls_path = P_DSD;
            if (rm_k < 3'd2) cls_ok = 1'b0;                   // DSD64..512
        end else if (rm_dop) begin
            cls_path = P_DSD;
            if (rm_k < 3'd2 || rm_k > 3'd4) cls_ok = 1'b0;    // DoP 64..256
        end else begin
            case (rm_k)
                3'd0: cls_path = P_X4;
                3'd1: cls_path = P_X2;
                3'd2: cls_path = P_X1;
                3'd3: cls_path = P_D2;
                3'd4: cls_path = P_D4;
                default: cls_ok = 1'b0;
            endcase
        end
    end

    reg [2:0] cst;
    reg [2:0] path;
    reg [2:0] cfg_k;
    reg       cfg_fam48, cfg_dsd, cfg_dop;
    reg       cfg_sync;           // synchronous source: elastic buffer instead of the ASRC
    reg       seq_seen;
    reg [1:0] fresh;          // window ends seen since entering IDLE
    reg [24:0] tmo;           // FILL/LOCK timeout (0.68 s)
    wire      running = (cst == C_FILL) || (cst == C_LOCK) || (cst == C_RUN);

    // ---- events: one per 2^k items (44.1 / 48 kHz) ----
    reg [4:0] ecnt;
    wire [4:0] emask = (5'd1 << cfg_k) - 5'd1;
    wire ev = running && item && ((ecnt & emask) == emask);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ecnt <= 5'd0;
        else if (!running) ecnt <= 5'd0;
        else if (item) ecnt <= ecnt + 5'd1;
    end

    // ---- event interval watchdog (+-3 %): 1024 MCLK when the source family
    //      matches the clock (always, with the clock manager), 1114.6 for
    //      44.1 kHz on 49.152 MHz (fixed-clock boards) ----
    reg [11:0] evint;
    reg        evseen;
    reg        ev_bad;
    wire        fam_same = (cfg_fam48 != mfam44);
    wire [11:0] ev_lo = fam_same ? 12'd992  : 12'd1081;
    wire [11:0] ev_hi = fam_same ? 12'd1056 : 12'd1148;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            evint <= 12'd0; evseen <= 1'b0; ev_bad <= 1'b0;
        end else if (!running) begin
            evint <= 12'd0; evseen <= 1'b0; ev_bad <= 1'b0;
        end else begin
            if (ev) begin
                evint <= 12'd1;
                evseen <= 1'b1;
                if (evseen && (evint < ev_lo || evint > ev_hi))
                    ev_bad <= 1'b1;
            end else begin
                if (~&evint) evint <= evint + 12'd1;
                if (evseen && evint > ev_hi)
                    ev_bad <= 1'b1;
            end
        end
    end

    // ---- live mode mismatch (immediate) ----
    wire mode_bad = running && item &&
                    ((item_dsd != cfg_dsd) || (!item_dsd && (dop != cfg_dop)));

    // ---------------- DSP stages ----------------
    wire stg_clr = (cst == C_IDLE);
    wire feed = running || (cst == C_FADE);
    // PCM headroom: 0..6 dB in 0.5 dB steps ahead of every interpolating /
    // resampling filter, so inter-sample peaks of hot masters do not hit
    // the S24 rails. 0 dB is bit-transparent (gain exactly 65536).
    reg  [16:0] att_g;
    always @(*) begin
        case (ctl_att)
            4'd0:  att_g = 17'd65536;  4'd1:  att_g = 17'd61870;
            4'd2:  att_g = 17'd58409;  4'd3:  att_g = 17'd55142;
            4'd4:  att_g = 17'd52057;  4'd5:  att_g = 17'd49145;
            4'd6:  att_g = 17'd46396;  4'd7:  att_g = 17'd43801;
            4'd8:  att_g = 17'd41350;  4'd9:  att_g = 17'd39037;
            4'd10: att_g = 17'd36854;  4'd11: att_g = 17'd34792;
            default: att_g = 17'd32846;
        endcase
    end
    wire signed [41:0] pl_g = $signed(item_l[31:8]) * $signed({1'b0, att_g});
    wire signed [41:0] pr_g = $signed(item_r[31:8]) * $signed({1'b0, att_g});
    wire signed [41:0] pl_r = pl_g + 42'sd32768;
    wire signed [41:0] pr_r = pr_g + 42'sd32768;
    wire signed [23:0] pl = pl_r[39:16];
    wire signed [23:0] pr = pr_r[39:16];
    wire pcm_feed = feed & item_pcm & ~cfg_dop & ~cfg_dsd;

    wire hba_v, hbb_v, hda_v, hdb_v, cic_v, dsf_v;
    wire signed [23:0] hba_l, hba_r, hbb_l, hbb_r, hda_l, hda_r, hdb_l, hdb_r;
    wire signed [23:0] cic_l, cic_r, dsf_l, dsf_r;
    wire ovf_a, ovf_b, ovf_c, ovf_d, ovf_e, ovf_f;
    wire clp_a, clp_b, clp_c, clp_d, clp_f, clp_s;

    fir_csd #(.N(175), .INTERP(1), .AW(7), .PW(9), .PLEN(322), .P1(319), .FILE({COEF_DIR, "hb_a.prog"})) u_hba (
        .clk(clk), .rst_n(rst_n), .clr(stg_clr),
        .in_valid(pcm_feed && path == P_X4), .in_l(pl), .in_r(pr),
        .out_valid(hba_v), .out_l(hba_l), .out_r(hba_r), .ovf(ovf_a), .clip(clp_a));

    wire hbb_in = (path == P_X4) ? hba_v : (pcm_feed && path == P_X2);
    fir_csd #(.N(91), .INTERP(1), .AW(6), .PW(8), .PLEN(184), .P1(181), .FILE({COEF_DIR, "hb_b.prog"})) u_hbb (
        .clk(clk), .rst_n(rst_n), .clr(stg_clr),
        .in_valid(hbb_in),
        .in_l(path == P_X4 ? hba_l : pl), .in_r(path == P_X4 ? hba_r : pr),
        .out_valid(hbb_v), .out_l(hbb_l), .out_r(hbb_r), .ovf(ovf_b), .clip(clp_b));

    fir_csd #(.N(23), .INTERP(0), .AW(5), .PW(6), .PLEN(50), .FILE({COEF_DIR, "hd_a.prog"})) u_hda (
        .clk(clk), .rst_n(rst_n), .clr(stg_clr),
        .in_valid(pcm_feed && path == P_D4), .in_l(pl), .in_r(pr),
        .out_valid(hda_v), .out_l(hda_l), .out_r(hda_r), .ovf(ovf_c), .clip(clp_c));

    wire hdb_in = (path == P_D4) ? hda_v : (pcm_feed && path == P_D2);
    fir_csd #(.N(31), .INTERP(0), .AW(5), .PW(7), .PLEN(66), .FILE({COEF_DIR, "hd_b.prog"})) u_hdb (
        .clk(clk), .rst_n(rst_n), .clr(stg_clr),
        .in_valid(hdb_in),
        .in_l(path == P_D4 ? hda_l : pl), .in_r(path == P_D4 ? hda_r : pr),
        .out_valid(hdb_v), .out_l(hdb_l), .out_r(hdb_r), .ovf(ovf_d), .clip(clp_d));

    // DSD chunks: native items carry {L16,R16} in item_r; DoP frames carry
    // 16 bits per channel under the marker byte.
    wire cic_in = feed && (path == P_DSD) && item &&
                  (cfg_dsd ? item_dsd : (!item_dsd && cfg_dop));
    wire [15:0] ch_l = cfg_dsd ? item_r[31:16] : item_l[23:8];
    wire [15:0] ch_r = cfg_dsd ? item_r[15:0]  : item_r[23:8];
    cic5_dsd u_cic (
        .clk(clk), .rst_n(rst_n), .clr(stg_clr), .r_log2(cfg_k + 3'd1),
        .in_valid(cic_in), .in_l(ch_l), .in_r(ch_r),
        .out_valid(cic_v), .out_l(cic_l), .out_r(cic_r), .ovf(ovf_e));

    fir_csd #(.N(95), .INTERP(0), .AW(7), .PW(8), .PLEN(169), .FILE({COEF_DIR, "dsd.prog"}),
              .LANES(2), .PLEN2(166), .FILE2({COEF_DIR, "dsd.prog2"})) u_dsf (
        .clk(clk), .rst_n(rst_n), .clr(stg_clr),
        .in_valid(cic_v), .in_l(cic_l), .in_r(cic_r),
        .out_valid(dsf_v), .out_l(dsf_l), .out_r(dsf_r), .ovf(ovf_f), .clip(clp_f));

    wire stage_ovf = ovf_a | ovf_b | ovf_c | ovf_d | ovf_e | ovf_f;

    // intermediate stream -> ASRC
    reg               iv;
    reg signed [23:0] il, ir;
    always @(*) begin
        case (path)
            P_X4, P_X2: begin iv = hbb_v; il = hbb_l; ir = hbb_r; end
            P_D2, P_D4: begin iv = hdb_v; il = hdb_l; ir = hdb_r; end
            P_DSD:      begin iv = dsf_v; il = dsf_l; ir = dsf_r; end
            default:    begin iv = pcm_feed; il = pl; ir = pr; end
        endcase
    end

    wire [7:0] fpos;
    wire       a_yv, a_started, a_fault;
    wire [2:0] a_gear;
    wire signed [31:0] a_yl, a_yr;
    wire signed [47:0] a_e;
    // fam48 = 1: nominal ratio 1 (source family = clock family)
    asrc #(.GSH(GSH), .FILE({COEF_DIR, "asrc.bin"})) u_asrc (
        .clk(clk), .rst_n(rst_n), .clr(stg_clr | cfg_sync), .fam48(fam_same),
        .arm(running), .in_valid(iv), .in_l(il), .in_r(ir),
        .ev(ev), .fpos(fpos),
        .y_valid(a_yv), .y_l(a_yl), .y_r(a_yr),
        .started(a_started), .gear(a_gear), .fault(a_fault), .dbg_e(a_e),
        .clip(clp_s));

    // ---------------- synchronous source: no ASRC ----------------
    // The Fox runs on this very MCLK (it gets the same oscillator), so the
    // intermediate stream comes at exactly MCLK/256 on average and only
    // needs a small elastic buffer (bursts of the filters). Its level never
    // drifts; under- or overflow means the source is not synchronous after
    // all -> fault, relock.
    reg  signed [23:0] sq_l [0:15];
    reg  signed [23:0] sq_r [0:15];
    reg  [3:0] sq_w, sq_rp;
    reg  [4:0] sq_n;
    reg        sq_err;
    wire       sq_rd = cfg_sync && (cst == C_RUN || cst == C_FADE) && (fpos == 8'd128);
    wire       sq_wr = cfg_sync && running && iv;
    always @(posedge clk) if (sq_wr) begin sq_l[sq_w] <= il; sq_r[sq_w] <= ir; end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sq_w <= 4'd0; sq_rp <= 4'd0; sq_n <= 5'd0; sq_err <= 1'b0;
        end else if (!cfg_sync || cst == C_IDLE) begin
            sq_w <= 4'd0; sq_rp <= 4'd0; sq_n <= 5'd0; sq_err <= 1'b0;
        end else begin
            if (sq_wr) sq_w <= sq_w + 4'd1;
            if (sq_rd) sq_rp <= sq_rp + 4'd1;
            sq_n <= sq_n + {4'd0, sq_wr} - {4'd0, sq_rd};
            if ((sq_rd && !sq_wr && sq_n == 5'd0) || (sq_wr && !sq_rd && sq_n == 5'd16))
                sq_err <= 1'b1;
        end
    end

    // ---------------- control FSM ----------------
    reg [10:0] gain;          // 0..1024
    wire fault = (cfg_sync ? sq_err : a_fault) | ev_bad | mode_bad | stage_ovf | stop_req | mute |
                 (rm_seq != seq_seen && (!cls_ok || cls_path != path ||
                  rm_k != cfg_k || rm_fam48 != cfg_fam48));
    wire tick = (fpos == 8'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cst <= C_IDLE; path <= P_X1; cfg_k <= 3'd0; cfg_fam48 <= 1'b0;
            cfg_dsd <= 1'b0; cfg_dop <= 1'b0; seq_seen <= 1'b0; fresh <= 2'd0;
            tmo <= 25'd0; gain <= 11'd0; cfg_sync <= 1'b0;
        end else begin
            case (cst)
                C_IDLE: begin
                    gain <= 11'd0;
                    if (rm_seq != seq_seen) begin
                        seq_seen <= rm_seq;
                        if (fresh != 2'd2) fresh <= fresh + 2'd1;
                    end
                    if (stop_req) begin
                        fresh <= 2'd0;              // input / clock switch in progress
                    end else if (fresh == 2'd2 && cls_ok && !mute) begin
                        path <= cls_path;
                        cfg_k <= rm_k;
                        cfg_fam48 <= rm_fam48;
                        cfg_dsd <= rm_dsd;
                        cfg_dop <= rm_dop;
                        seq_seen <= rm_seq;
                        tmo <= 25'd0;
                        cfg_sync <= sync_en && (sel == 3'd0) && (rm_fam48 != mfam44);
                        cst <= C_FILL;
                    end
                end
                C_FILL, C_LOCK, C_RUN: begin
                    if (rm_seq != seq_seen) seq_seen <= rm_seq;
                    if (cst != C_RUN) tmo <= tmo + 25'd1;
                    if (fault || (&tmo)) begin
                        cst <= C_FADE;
                    end else begin
                        if (cfg_sync) begin
                            if (cst != C_RUN && sq_n >= 5'd8) cst <= C_RUN;   // half full
                        end else begin
                            if (cst == C_FILL && a_started) cst <= C_LOCK;
                            if (cst == C_LOCK && a_gear >= 3'd2) cst <= C_RUN;
                        end
                        if (cst == C_RUN && tick && gain != 11'd1024)
                            gain <= gain + 11'd1;
                    end
                end
                C_FADE: begin
                    if (tick) begin
                        if (gain <= 11'd8) begin
                            gain <= 11'd0;
                            fresh <= 2'd0;
                            seq_seen <= rm_seq;
                            cst <= C_IDLE;
                        end else begin
                            gain <= gain - 11'd8;
                        end
                    end
                end
                default: cst <= C_IDLE;
            endcase
        end
    end

    // ---------------- output: hold, fade, TX ----------------
    reg signed [31:0] hl, hr;     // last ASRC output (coasts in FADE)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hl <= 32'sd0; hr <= 32'sd0;
        end else if (cst == C_IDLE) begin
            hl <= 32'sd0; hr <= 32'sd0;
        end else if (cfg_sync) begin
            if (sq_rd && cst != C_FADE) begin
                hl <= {sq_l[sq_rp], 8'd0}; hr <= {sq_r[sq_rp], 8'd0};
            end
        end else if (a_yv && cst != C_FADE) begin
            hl <= a_yl; hr <= a_yr;
        end
    end
    // ---------------- room-correction EQ (192 kHz, after the ASRC) ----------------
    // Runs once per output frame on the faded ASRC sample (fade BEFORE the
    // EQ: start / stop of a stream enters the filters smoothly, so a
    // subsonic high-pass or a deep LF section cannot ring on the start
    // transient). One frame of latency; nsec = 0 (reset default) is a
    // bit-exact bypass.
    wire signed [43:0] gl = hl * $signed({1'b0, gain});
    wire signed [43:0] gr = hr * $signed({1'b0, gain});
    reg  signed [31:0] fdl, fdr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fdl <= 32'sd0; fdr <= 32'sd0;
        end else if (fpos == 8'd255) begin
            fdl <= gl[41:10];
            fdr <= gr[41:10];
        end
    end
    wire signed [31:0] eq_l, eq_r;
    wire        eq_done, eq_clip, eq_pend, eq_bank;
    wire [3:0]  eq_nsec;
    eq_biquad #(.NS(10)) u_eq (
        .clk(clk), .rst_n(rst_n), .clr(cst == C_IDLE), .start(fpos == 8'd0),
        .in_l(fdl), .in_r(fdr), .out_l(eq_l), .out_r(eq_r),
        .done(eq_done), .clip(eq_clip),
        .cw_we(eq_we), .cw_idx(eq_widx), .cw_data(eq_wdat),
        .apply_req(eq_apply), .nsec_new(eq_nsec_new),
        .apply_pend(eq_pend), .bank(eq_bank), .nsec(eq_nsec));

    reg  signed [31:0] ol, orr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ol <= 32'sd0; orr <= 32'sd0;
        end else if (fpos == 8'd255) begin
            ol <= eq_l;
            orr <= eq_r;
        end
    end

    i2s_tx32 u_tx (
        .clk(clk), .rst_n(rst_n), .in_l(ol), .in_r(orr), .fpos(fpos),
        .bclk_out(i2s_bclk_out), .lrck_out(i2s_lrck_out), .sdata_out(i2s_sdata_out)
    );


    assign audio_on = (cst == C_RUN);

    // saturation anywhere in the chain (only while audio is passed), held ~85 ms
    wire       clip_now = (cst == C_RUN) &&
                          (clp_a | clp_b | clp_c | clp_d | clp_f | (clp_s & ~cfg_sync) | eq_clip);
    reg [21:0] clip_tmr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) clip_tmr <= 22'd0;
        else if (clip_now) clip_tmr <= {22{1'b1}};
        else if (clip_tmr != 22'd0) clip_tmr <= clip_tmr - 22'd1;
    end

    // status word: [2:0] k, [3] fam48, [4] dsd, [5] dop, [6] running, [7] run,
    // [10:8] gear, [11] clip, [12] idle, [13] sync path, [17:14] eq nsec,
    // [18] eq bank, [19] eq apply pending, [31:20] 0
    assign stat = {12'd0, eq_pend, eq_bank, eq_nsec, cfg_sync && running, cst == C_IDLE,
                   clip_tmr != 22'd0, cfg_sync ? 3'd4 : a_gear, cst == C_RUN, running,
                   cfg_dop, cfg_dsd, cfg_fam48, cfg_k};
endmodule
