`timescale 1ns/1fs
// Fox native DSD with the left channel on its own line (tb/run_sys.py --dsdl).
// top is built with the D2 board port set (no AXB_SIM_PORTS): one 49.152 MHz
// oscillator, no mclk_sel / fox_fsel / in3_dsd, CLK_CTRL = 0x00 after reset.
// FD = FOX_DSDL:
//   1: D2 layout - DSD left on i2s_dsdl_in (Fox pin 27), the LRCK pin carries
//      noise while DSD plays;
//   0: old three-wire layout - DSD left on the LRCK pin, i2s_dsdl_in carries
//      noise all the time.
// The unused line always carries an unrelated random bit stream: taken by
// mistake it turns a channel into noise, which the THD+N check catches.
//   0. DSD64 (own clock, +31 ppm), L 1000 Hz / R 3000 Hz, swap default
//      (RIGHT on SDATA): output L 1000, R 3000
//   1. DSD_CFG (0x0E) bit 0 = 0 (LEFT on SDATA): output L 3000, R 1000
//      (for the D2 layout "RIGHT" then comes from the DSD_L line)
//   2. back to bit 0 = 1, Fox PCM 44.1k (-17 ppm), 997 / 3001 Hz: the PCM
//      path ignores i2s_dsdl_in
// Also: VERSION 0x21, CLK_CTRL 0x00, CLK_CTRL bits [1:0] locked to 0 (one
// oscillator), mclk_sel never leaves 0.
module tb_dsdl;
    parameter integer FD   = 1;
    parameter integer DSDN = 1;
    reg sclk = 0;  always #10.0 sclk = ~sclk;              // 50 MHz
    reg mclk = 0;  always #10.172526 mclk = ~mclk;         // 49.152 MHz, the only oscillator

    // ---- the Fox: native DSD64 or PCM 44.1k, each on its own clock ----
    reg  dsd_en = 0, pcm_en = 0;
    wire db, dl, dr, dx, pb, pl, pd, px;
    tb_src #(.MODE(1), .FS(2822400.0), .PPM(31.0), .DSDN(DSDN),
             .T0(91.3)) fox_dsd (.en(dsd_en), .bclk(db), .lrck(dl), .sdata(dr), .dsd_on(dx));
    tb_src #(.MODE(0), .FS(44100.0), .PPM(-17.0), .FL(997.0), .FR(3001.0),
             .T0(47.9)) fox_pcm (.en(pcm_en), .bclk(pb), .lrck(pl), .sdata(pd), .dsd_on(px));

    // unrelated random bit stream (~2.9 Mbit/s) for the line that must be ignored
    reg [15:0] lf = 16'hACE1;
    always #171.3 lf = {lf[14:0], lf[15] ^ lf[13] ^ lf[12] ^ lf[10]};
    wire noise = lf[0];

    wire fox_bclk = (db & dsd_en) | (pb & pcm_en);
    wire fox_sdat = (dr & dsd_en) | (pd & pcm_en);
    wire fox_flag = dx;
    // LRCK pin (25): PCM LRCK; in DSD - the left channel (old layout) or noise (D2)
    wire fox_lrck = pcm_en ? pl : (dsd_en ? (FD ? noise : dl) : 1'b0);
    // pin 27 (I2S_SDO1 / DSD_L): the left channel in DSD on D2, noise otherwise
    wire fox_dsdl = (FD && dsd_en) ? dl : noise;

    reg  scl_m = 1'b1, sda_m = 1'b1;
    tri1 i2c_sda;
    assign i2c_sda = sda_m ? 1'bz : 1'b0;

    wire obclk, olrck, osdata, audio_on;
    top #(.WBITS(16), .GSH(4), .MWBITS(16), .CLK_SETTLE_MS(2), .ACK_WAIT_MS(20),
          .FOX_DSDL(FD)) dut (
        .sys_clk(sclk), .mclk_in(mclk),
        .i2s_bclk_in(fox_bclk), .i2s_lrck_in(fox_lrck), .i2s_sdata_in(fox_sdat),
        .i2s_dsdl_in(fox_dsdl), .dsd_on(fox_flag),
        .spdif_in(4'b0000),
        .in2_bclk(1'b0), .in2_lrck(1'b0), .in2_sdata(1'b0),
        .in3_bclk(1'b0), .in3_lrck(1'b0), .in3_sdata(1'b0),
        .in4_bclk(1'b0), .in4_lrck(1'b0), .in4_sdata(1'b0),
        .i2c_scl(scl_m), .i2c_sda(i2c_sda),
        .i2s_bclk_out(obclk), .i2s_lrck_out(olrck), .i2s_sdata_out(osdata),
        .audio_on(audio_on));

    integer n_sel = 0;
    // count real switches only: the reset that takes the register from X to 0 is not one
    reg sel_q = 1'bx;
    always @(dut.mclk_sel) begin
        if (sel_q === 1'b0 || sel_q === 1'b1) n_sel = n_sel + 1;
        sel_q = dut.mclk_sel;
    end

    `include "tb_v2_tasks.vh"

    localparam [8*4-1:0] PFX = FD ? "dl1" : "dl0";

    initial begin
        #(3e6);
        $display("FOX_DSDL = %0d (%0s)", FD, FD ? "D2: DSD left on its own line" : "old: DSD left on LRCK");
        rd_regs(8'h05, r0, r1, r2, r3);
        check(r0, 8'hA5, "ID"); check(r1, 8'h21, "VERSION");
        rd_regs(8'h14, r0, r1, r2, r3);
        check(r0, 8'h00, "CLK_CTRL");
        // one oscillator: auto / manual-44.1 bits cannot be set
        wr_reg(8'h14, 8'h0F);
        rd_regs(8'h14, r0, r1, r2, r3);
        check(r0, 8'h0C, "CLK_CTRL lck");
        wr_reg(8'h14, 8'h00);
        // 0. native DSD64, default order (RIGHT on SDATA)
        dsd_en = 1;
        wait_on(250.0);
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h92, "FORMAT"); check(r1, 8'h11, "RATE");    // running, DSD, 44.1 family, k 2; DSD64
        #(5e6);
        capture(0, 4096, PFX);
        // 1. swap: LEFT on SDATA (applied live, no re-capture)
        wr_reg(8'h0E, 8'h0E);
        #(10e6);
        wait_on(250.0);      // returns at once if the swap did not re-capture
        #(5e6);
        capture(1, 4096, PFX);
        wr_reg(8'h0E, 8'h0F);
        // 2. Fox PCM 44.1k
        dsd_en = 0; pcm_en = 1;
        wait_off(20.0);
        wait_on(250.0);
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h80, "FORMAT"); check(r1, 8'h01, "RATE");
        #(5e6);
        capture(2, 4096, PFX);
        if (n_sel != 0 || dut.mclk_sel !== 1'b0) begin
            $display("ERROR: mclk_sel moved %0d times (one oscillator)", n_sel); errors = errors + 1;
        end
        if (errors == 0) $display("PASS tb_dsdl");
        else             $display("FAIL tb_dsdl (%0d errors)", errors);
        $finish;
    end
    initial begin
        #(1500e6);
        $display("FAIL tb_dsdl: timeout");
        $finish;
    end
endmodule
