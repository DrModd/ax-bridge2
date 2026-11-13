`define AXB_SIM_PORTS   // top: bench ports (mclk_sel, fox_fsel, in3_dsd)
`timescale 1ns/1fs
// All-asynchronous mode, CLK_CTRL = 0x00 (tb/run_sys.py --async), with the
// SYNC_IN4 = 1 and CLK_CTRL = 0x0D after reset (set below), so the mode is
// entered and left at run time like the STM32 does it.
//   0. 0x0D: Fox 44.1k on our MCLK -> 45.1584 MHz, synchronous (setup)
//   1. CLK_CTRL = 0x00, Fox on its own clock (44.1k +37 ppm), its MCLK_SEL
//      pin still saying 44.1: the clock goes to 49.152 MHz WITHOUT an STM32
//      request (bit 3 = 0), output 192 kHz, 44.1 -> 48 through the ASRC
//   2. Fox 96k own clock (-23 ppm): no clock move, ASRC 1:1
//   3. the Fox pin to 44.1 and CLK_CTRL = 0x02 (manual 44.1, sync off) must
//      both be ignored: audio is not interrupted, the clock does not move
//   4. S/PDIF 44.1k +60 ppm: meter says 44.1, the clock stays, 44.1 -> 48
//   5. phono ADC on our MCLK (synchronous hardware, SYNC_IN4 = 1): in this
//      mode it must still go through the ASRC (1:1), not the sync path
//   6. Fox own clock 44.1k, then CLK_CTRL = 0x0D with the Fox back on our
//      MCLK: STM32 request + acknowledge, 45.1584 MHz, synchronous again
module tb_async;
    reg sclk = 0;  always #10.0 sclk = ~sclk;              // 50 MHz
    reg o45 = 0;   always #11.072164 o45 = ~o45;           // 45.1584 MHz
    reg o49 = 0;   always #10.172526 o49 = ~o49;           // 49.152 MHz
    wire mclk_sel;
    wire mclk = mclk_sel ? o45 : o49;

    // ---- input 0: the Fox, on our MCLK or on its own clock ----
    reg  fox_sync = 0, fox_o44 = 0, fox_o96 = 0, fox_fsel = 0;
    wire sb, sl, sd;
    tb_sync_src #(.FL(997.0), .FR(3001.0)) fox_s (
        .mclk(mclk), .en(fox_sync), .fam44(mclk_sel), .div(8'd16),
        .bclk(sb), .lrck(sl), .sdata(sd));
    wire ab, al, ad, ax, bb, bl, bd, bx;
    tb_src #(.MODE(0), .FS(44100.0), .PPM(37.0), .FL(997.0), .FR(3001.0),
             .T0(77.7)) fox_a (.en(fox_o44), .bclk(ab), .lrck(al), .sdata(ad), .dsd_on(ax));
    tb_src #(.MODE(0), .FS(96000.0), .PPM(-23.0), .FL(1499.0), .FR(7001.0),
             .T0(333.3)) fox_b (.en(fox_o96), .bclk(bb), .lrck(bl), .sdata(bd), .dsd_on(bx));
    // tb_src stops BCLK when disabled but LRCK / SDATA keep their last level:
    // gate every line with its own enable
    wire fb = (sb & fox_sync) | (ab & fox_o44) | (bb & fox_o96);
    wire fl = (sl & fox_sync) | (al & fox_o44) | (bl & fox_o96);
    wire fd = (sd & fox_sync) | (ad & fox_o44) | (bd & fox_o96);

    // ---- S/PDIF line 0 ----
    wire sp0;
    tb_spdif_src #(.FS(44100.0), .PPM(60.0), .FL(1499.0), .FR(5003.0), .JIT(2.0),
                   .SEED(3), .T0(101.1)) s1 (.en(1'b1), .line(sp0));
    // ---- AK5572 on our MCLK (BCLK = MCLK/4) ----
    wire b4, l4, d4;
    tb_sync_src #(.FL(1201.0), .FR(4001.0)) adc (
        .mclk(mclk), .en(1'b1), .fam44(mclk_sel), .div(8'd4),
        .bclk(b4), .lrck(l4), .sdata(d4));

    reg  scl_m = 1'b1, sda_m = 1'b1;
    tri1 i2c_sda;
    assign i2c_sda = sda_m ? 1'bz : 1'b0;

    wire obclk, olrck, osdata, audio_on;
    top #(.WBITS(16), .GSH(4), .MWBITS(16), .CLK_SETTLE_MS(2), .ACK_WAIT_MS(20), .TWO_OSC(1), .CLK_CTRL_RST(4'b1101), .SYNC_IN4(1)) dut (
        .sys_clk(sclk), .mclk_in(mclk), .mclk_sel(mclk_sel), .fox_fsel(fox_fsel),
        .i2s_bclk_in(fb), .i2s_lrck_in(fl), .i2s_dsdl_in(fl), .i2s_sdata_in(fd), .dsd_on(1'b0),
        .spdif_in({3'b000, sp0}),
        .in2_bclk(1'b0), .in2_lrck(1'b0), .in2_sdata(1'b0),
        .in3_bclk(1'b0), .in3_lrck(1'b0), .in3_sdata(1'b0), .in3_dsd(1'b0),
        .in4_bclk(b4), .in4_lrck(l4), .in4_sdata(d4),
        .i2c_scl(scl_m), .i2c_sda(i2c_sda),
        .i2s_bclk_out(obclk), .i2s_lrck_out(olrck), .i2s_sdata_out(osdata),
        .audio_on(audio_on));

    integer n_sel = 0, n_req = 0;
    always @(mclk_sel) if ($realtime > 0) n_sel = n_sel + 1;
    always @(posedge dut.clk_req) n_req = n_req + 1;
    // in all-async mode no input may ever run on the synchronous path
    reg     aa = 0;
    integer n_sync_run = 0;
    always @(posedge mclk) if (aa && dut.u_core.cst == dut.u_core.C_RUN && dut.u_core.cfg_sync)
        n_sync_run = n_sync_run + 1;

    `include "tb_v2_tasks.vh"

    integer k_sel, k_req;
    task snap; begin k_sel = n_sel; k_req = n_req; end endtask
    task no_switch(input [8*24-1:0] what); begin
        if (n_sel != k_sel || n_req != k_req || mclk_sel !== 1'b0) begin
            $display("ERROR: %0s: clock moved %0d times, %0d requests, mclk_sel = %b (expected 49.152, none)",
                     what, n_sel - k_sel, n_req - k_req, mclk_sel);
            errors = errors + 1;
        end else
            $display("          %0s: 49.152 MHz, clock did not move ok", what);
    end endtask
    task not_sync; begin
        rd_regs(8'h04, r0, r1, r2, r3);
        $display("          PLL = %02h (ASRC gear)", r0);
        if (dut.u_core.cfg_sync !== 1'b0) begin
            $display("ERROR: synchronous path in all-async mode"); errors = errors + 1;
        end
    end endtask

    initial begin
        #(3e6);
        rd_regs(8'h05, r0, r1, r2, r3);
        check(r0, 8'hA5, "ID"); check(r1, 8'h21, "VERSION");
        // 0. normal mode: Fox 44.1k synchronous on 45.1584 MHz
        fox_fsel = 1; fox_sync = 1;
        ack_switch(40.0);
        wait_on(60.0);
        rd_regs(8'h15, r0, r1, r2, r3);
        check(r0 & 8'h7F, 8'h79, "CLK_STAT");
        // 1. all-async: the Fox goes to its own clock, the pin keeps 44.1
        wr_reg(8'h15, 8'h80);
        snap;
        fox_sync = 0; fox_o44 = 1;
        wr_reg(8'h14, 8'h00);
        aa = 1;
        wait_off(20.0);
        wait_on(250.0);
        if (n_sel - k_sel != 1 || n_req != k_req || mclk_sel !== 1'b0) begin
            $display("ERROR: into 0x00: clock moved %0d times, %0d requests, mclk_sel = %b",
                     n_sel - k_sel, n_req - k_req, mclk_sel);
            errors = errors + 1;
        end else
            $display("          into 0x00: 45.1584 -> 49.152 MHz, no STM32 request ok");
        rd_regs(8'h14, r0, r1, r2, r3);
        check(r0, 8'h00, "CLK_CTRL");
        rd_regs(8'h15, r0, r1, r2, r3);
        check(r0, 8'hF0, "CLK_STAT");          // switched, 49.152, not sync, meter 44.1, running
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h80, "FORMAT"); check(r1, 8'h01, "RATE");
        not_sync;
        capture(0, 4096, "aa");
        // 2. Fox 96k on its own clock
        wr_reg(8'h15, 8'h80);
        snap;
        fox_o44 = 0; fox_o96 = 1; fox_fsel = 0;
        wait_off(20.0);
        wait_on(250.0);
        no_switch("Fox 96k");
        rd_regs(8'h15, r0, r1, r2, r3);
        check(r0, 8'h50, "CLK_STAT");          // 49.152, not sync, meter 48, running
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h89, "FORMAT"); check(r1, 8'h04, "RATE");
        not_sync;
        capture(1, 4096, "aa");
        // 3. ignored: the Fox pin and the manual family bit
        snap;
        fox_fsel = 1;
        wr_reg(8'h14, 8'h02);
        stays_on(30.0);
        no_switch("pin 44.1 + CLK_CTRL 0x02");
        wr_reg(8'h14, 8'h00);
        fox_fsel = 0;
        stays_on(5.0);
        // 4. S/PDIF 44.1k
        snap;
        wr_reg(8'h00, 8'h01);
        wait_off(20.0);
        wait_on(250.0);
        no_switch("S/PDIF 44.1k");
        rd_regs(8'h15, r0, r1, r2, r3);
        check(r0, 8'h70, "CLK_STAT");          // 49.152, not sync, meter 44.1, running
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h80, "FORMAT"); check(r1, 8'h01, "RATE");
        not_sync;
        capture(2, 4096, "aa");
        // 5. phono ADC on our MCLK: through the ASRC all the same
        snap;
        wr_reg(8'h00, 8'h04);
        wait_off(20.0);
        wait_on(250.0);
        no_switch("phono");
        rd_regs(8'h15, r0, r1, r2, r3);
        check(r0, 8'h50, "CLK_STAT");          // bit 3 = 0: NOT the sync path
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h8A, "FORMAT"); check(r1, 8'h06, "RATE");
        not_sync;
        capture(3, 4096, "aa");
        if (n_sync_run != 0) begin
            $display("ERROR: sync path ran for %0d MCLK in all-async mode", n_sync_run);
            errors = errors + 1;
        end
        // 6. back to 0x0D on the Fox (A/B listening): Fox own clock first ...
        fox_o96 = 0; fox_fsel = 1; fox_o44 = 1;
        wr_reg(8'h00, 8'h00);
        wait_off(20.0);
        wait_on(250.0);
        no_switch("Fox 44.1k own clock");
        aa = 0;
        // ... then the STM32 writes 0x0D and puts the Fox on our MCLK
        wr_reg(8'h15, 8'h80);
        snap;
        fox_o44 = 0; fox_sync = 1;
        wr_reg(8'h14, 8'h0D);
        ack_switch(60.0);
        wait_on(80.0);
        if (n_sel - k_sel != 1 || n_req - k_req != 1 || mclk_sel !== 1'b1) begin
            $display("ERROR: back to 0x0D: clock moved %0d times, %0d requests, mclk_sel = %b",
                     n_sel - k_sel, n_req - k_req, mclk_sel);
            errors = errors + 1;
        end
        rd_regs(8'h15, r0, r1, r2, r3);
        check(r0, 8'hF9, "CLK_STAT");          // switched, 45.1584, sync, meter 44.1, running
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h80, "FORMAT"); check(r1, 8'h01, "RATE"); check(r2, 8'h04, "PLL");
        capture(4, 4096, "aa");
        if (errors == 0) $display("PASS tb_async");
        else             $display("FAIL tb_async (%0d errors)", errors);
        $finish;
    end
    initial begin
        #(2500e6);
        $display("FAIL tb_async: timeout");
        $finish;
    end
endmodule
