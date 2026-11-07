`timescale 1ns/1fs
// Synchronous phono ADC, SYNC_IN4 = 1 (tb/run_sys.py --sync4).
// The AK5572 is an I2S master but gets its MCLK from the bridge's own
// oscillator mux (BCLK = MCLK/4, 256fs): 192 kHz on 49.152 MHz, 176.4 kHz
// on 45.1584 MHz. Input 4 must take the synchronous path (no ASRC) on
// either oscillator, the meter must see our own family so the clock never
// moves because of the ADC, and a forced clock change must take the ADC
// along.
//   1. phono on 49.152 MHz (reset clock): no switch, sync path, 192 kHz
//   2. Fox 44.1k moves the clock to 45.1584 MHz; then phono: no switch,
//      ADC at 176.4 kHz, sync path, output 176.4 kHz
//   3. CLK_CTRL = 0x04 (manual 49.152, sync on) while on phono: the clock
//      switches, the ADC follows to 192 kHz, sync path again
//   4. CLK_CTRL back to 0x0D (auto) on phono: the meter sees 48 -> no
//      switch, audio is not interrupted
module tb_sync4;
    reg sclk = 0;  always #10.0 sclk = ~sclk;              // 50 MHz
    reg o45 = 0;   always #11.072164 o45 = ~o45;           // 45.1584 MHz
    reg o49 = 0;   always #10.172526 o49 = ~o49;           // 49.152 MHz
    wire mclk_sel;
    wire mclk = mclk_sel ? o45 : o49;

    // Fox on our MCLK (PCM 44.1k: BCLK = MCLK/16 on 45.1584 MHz)
    reg  fox_en = 0, fox_fsel = 0;
    wire fb, fl, fd;
    tb_sync_src #(.FL(997.0), .FR(3001.0)) fox (
        .mclk(mclk), .en(fox_en), .fam44(mclk_sel), .div(8'd16),
        .bclk(fb), .lrck(fl), .sdata(fd));
    // AK5572 on our MCLK: BCLK = MCLK/4 (64fs at 256fs)
    reg  adc_en = 0;
    wire b4, l4, d4;
    tb_sync_src #(.FL(1201.0), .FR(4001.0)) adc (
        .mclk(mclk), .en(adc_en), .fam44(mclk_sel), .div(8'd4),
        .bclk(b4), .lrck(l4), .sdata(d4));

    reg  scl_m = 1'b1, sda_m = 1'b1;
    tri1 i2c_sda;
    assign i2c_sda = sda_m ? 1'bz : 1'b0;

    wire obclk, olrck, osdata, audio_on;
    top #(.WBITS(16), .GSH(4), .MWBITS(16), .CLK_SETTLE_MS(2), .ACK_WAIT_MS(20), .SYNC_IN4(1)) dut (
        .sys_clk(sclk), .mclk_in(mclk), .mclk_sel(mclk_sel), .fox_fsel(fox_fsel),
        .i2s_bclk_in(fb), .i2s_lrck_in(fl), .i2s_sdata_in(fd), .dsd_on(1'b0),
        .spdif_in(4'b0000),
        .in2_bclk(1'b0), .in2_lrck(1'b0), .in2_sdata(1'b0),
        .in3_bclk(1'b0), .in3_lrck(1'b0), .in3_sdata(1'b0), .in3_dsd(1'b0),
        .in4_bclk(b4), .in4_lrck(l4), .in4_sdata(d4),
        .i2c_scl(scl_m), .i2c_sda(i2c_sda),
        .i2s_bclk_out(obclk), .i2s_lrck_out(olrck), .i2s_sdata_out(osdata),
        .audio_on(audio_on));

    // clock moves and STM32 requests, counted independently of the registers
    integer n_sel = 0, n_req = 0;
    always @(mclk_sel) if ($realtime > 0) n_sel = n_sel + 1;
    always @(posedge dut.clk_req) n_req = n_req + 1;
    // the synchronous path must carry the ADC: never an ASRC sample in RUN
    integer n_asrc_run = 0;
    always @(posedge mclk) if (dut.u_core.cst == dut.u_core.C_RUN && !dut.u_core.cfg_sync && dut.sel_cur == 3'd4)
        n_asrc_run = n_asrc_run + 1;

    `include "tb_v2_tasks.vh"

    integer k_sel, k_req;
    task snap; begin k_sel = n_sel; k_req = n_req; end endtask
    task no_switch(input [8*24-1:0] what); begin
        if (n_sel != k_sel || n_req != k_req) begin
            $display("ERROR: %0s: clock moved %0d times, %0d requests (expected none)",
                     what, n_sel - k_sel, n_req - k_req);
            errors = errors + 1;
        end else
            $display("          %0s: clock did not move ok", what);
    end endtask

    initial begin
        #(3e6);
        rd_regs(8'h05, r0, r1, r2, r3);
        check(r0, 8'hA5, "ID"); check(r1, 8'h20, "VERSION");
        rd_regs(8'h14, r0, r1, r2, r3);
        check(r0, 8'h0D, "CLK_CTRL");
        // 1. phono on the reset oscillator (49.152 MHz)
        adc_en = 1;
        wr_reg(8'h15, 8'h80);
        snap;
        wr_reg(8'h00, 8'h04);
        wait_on(60.0);
        no_switch("phono 192k");
        rd_regs(8'h15, r0, r1, r2, r3);
        check(r0, 8'h58, "CLK_STAT");          // 49.152, sync, meter 48, running
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h8A, "FORMAT"); check(r1, 8'h06, "RATE"); check(r2, 8'h04, "PLL");
        capture(0, 4096, "s4");
        // 2. the Fox brings 45.1584 MHz; phono then stays on it at 176.4 kHz
        fox_fsel = 1; fox_en = 1;
        wr_reg(8'h00, 8'h00);
        wait_off(20.0);
        wait_on(80.0);
        rd_regs(8'h15, r0, r1, r2, r3);
        check(r0 & 8'h7F, 8'h79, "CLK_STAT");  // 45.1584, sync, meter 44.1, running
        $display("          (Fox input change: %0d STM32 requests before the clock moved)", n_req - k_req);
        wr_reg(8'h15, 8'h80);
        snap;
        wr_reg(8'h00, 8'h04);
        wait_off(20.0);
        wait_on(60.0);
        no_switch("phono 176.4k");
        rd_regs(8'h15, r0, r1, r2, r3);
        check(r0, 8'h79, "CLK_STAT");          // 45.1584, sync, meter 44.1, running, no switch
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h82, "FORMAT"); check(r1, 8'h05, "RATE"); check(r2, 8'h04, "PLL");
        capture(1, 4096, "s4");
        // 3. manual 49.152 MHz with the sync path on: the ADC follows the clock
        fox_en = 0; fox_fsel = 0;
        snap;
        wr_reg(8'h14, 8'h04);
        wait_off(20.0);
        wait_on(80.0);
        if (n_sel - k_sel != 1 || mclk_sel !== 1'b0) begin
            $display("ERROR: manual 49.152: clock moved %0d times, mclk_sel = %b", n_sel - k_sel, mclk_sel);
            errors = errors + 1;
        end
        rd_regs(8'h15, r0, r1, r2, r3);
        check(r0, 8'hD8, "CLK_STAT");          // switched, 49.152, sync, meter 48, running
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h8A, "FORMAT"); check(r1, 8'h06, "RATE"); check(r2, 8'h04, "PLL");
        capture(2, 4096, "s4");
        // 4. back to auto: the meter sees our 48 family -> nothing moves
        wr_reg(8'h15, 8'h80);
        snap;
        wr_reg(8'h14, 8'h0D);
        stays_on(30.0);
        no_switch("auto on phono");
        rd_regs(8'h15, r0, r1, r2, r3);
        check(r0, 8'h58, "CLK_STAT");
        if (n_asrc_run != 0) begin
            $display("ERROR: phono ran through the ASRC for %0d MCLK", n_asrc_run);
            errors = errors + 1;
        end
        if (errors == 0) $display("PASS tb_sync4");
        else             $display("FAIL tb_sync4 (%0d errors)", errors);
        $finish;
    end
    initial begin
        #(1500e6);
        $display("FAIL tb_sync4: timeout");
        $finish;
    end
endmodule
