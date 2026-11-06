`timescale 1ns/1fs
// ax-bridge2 clock architecture (tb/run_sys.py --v2):
//   two oscillators 45.1584 / 49.152 MHz behind a multiplexer driven by
//   mclk_sel, a 50 MHz service clock, and a Fox model that runs from the
//   multiplexed MCLK (BCLK = MCLK / div) and drives MCLK_SEL itself.
//   1. Fox PCM 44.1k (fsel = 1): clock -> 45.1584 MHz after the STM32
//      acknowledge, synchronous path (no ASRC), output 176.4 kHz
//   2. Fox PCM 96k (fsel = 0): clock -> 49.152 MHz, synchronous, 192 kHz
//   3. S/PDIF 44.1k +60 ppm: meter -> 45.1584 MHz, ASRC ratio ~1, 176.4 kHz
//   4. phono ADC 192k +17 ppm: meter -> 49.152 MHz, ASRC, 192 kHz
//   5. Fox DSD64 (fsel = 1): 45.1584 MHz, synchronous, 176.4 kHz
module tb_v2;
    parameter integer DSDN = 1;
    reg sclk = 0;  always #10.0 sclk = ~sclk;              // 50 MHz
    reg o45 = 0;   always #11.072164 o45 = ~o45;           // 45.1584 MHz
    reg o49 = 0;   always #10.172526 o49 = ~o49;           // 49.152 MHz
    wire mclk_sel;
    wire mclk = mclk_sel ? o45 : o49;

    // ---- Fox model: synchronous to MCLK ----
    reg        fox_en = 0, fox_fsel = 0, fox_dsd = 0;
    integer    fox_div = 16;          // MCLK / BCLK
    real       fox_fs = 44100.0, fox_fl = 997.0, fox_fr = 3001.0;
    reg        fb = 0, fl = 0, fd = 0;
    integer    fdiv_c = 0, fslot = 0, fn = 0;
    reg [31:0] fwl = 0, fwr = 0, fsh = 0;
    reg [31:0] dsdw [0:DSDN-1];
    integer    di = 0, dbit = 0;
    initial $readmemh("build/dsd.hex", dsdw);
    always @(posedge mclk) begin
        if (!fox_en) begin fb <= 0; fdiv_c <= 0; end
        else if (fdiv_c == fox_div / 2 - 1 || fox_div == 1) begin
            fdiv_c <= 0;
            fb <= ~fb;
        end else fdiv_c <= fdiv_c + 1;
    end
    // data changes on the falling BCLK edge
    always @(negedge fb) begin
        if (fox_dsd) begin
            // Amanero DSD: LRCK pin = left, SDATA = right, MSB first, 16 bits per word
            fl <= dsdw[di][31 - dbit];
            fd <= dsdw[di][15 - dbit];
            if (dbit == 15) begin dbit <= 0; di <= (di == DSDN - 1) ? 0 : di + 1; end
            else dbit <= dbit + 1;
        end else begin
            if (fslot == 0) begin
                fwl = $rtoi($sin(6.283185307179586 * fox_fl * fn / fox_fs) * 0.891 * 8388607.0) << 8;
                fwr = $rtoi($sin(6.283185307179586 * fox_fr * fn / fox_fs) * 0.891 * 8388607.0) << 8;
                fn = fn + 1;
            end
            // same framing as tb_src: LRCK = slot[5], data one bit late
            fl <= (fslot >= 32);
            if (fslot == 0)       fd <= fsh[0];
            else if (fslot <= 32) fd <= fwl[32 - fslot];
            else                  fd <= fwr[64 - fslot];
            if (fslot == 63) fsh = fwr;
            fslot = (fslot + 1) % 64;
        end
    end

    // ---- other sources ----
    wire sp0;
    tb_spdif_src #(.FS(44100.0), .PPM(60.0), .FL(1499.0), .FR(5003.0), .JIT(2.0),
                   .SEED(3), .T0(101.1)) s1 (.en(1'b1), .line(sp0));
    wire b4, l4, d4, x4;
    tb_src #(.MODE(0), .FS(192000.0), .PPM(17.0), .FL(1201.0), .FR(4001.0),
             .T0(555.5)) s4 (.en(1'b1), .bclk(b4), .lrck(l4), .sdata(d4), .dsd_on(x4));

    reg  scl_m = 1'b1, sda_m = 1'b1;
    tri1 i2c_sda;
    assign i2c_sda = sda_m ? 1'bz : 1'b0;

    wire obclk, olrck, osdata, audio_on;
    top #(.WBITS(16), .GSH(4), .MWBITS(16), .CLK_SETTLE_MS(2), .ACK_WAIT_MS(20)) dut (
        .sys_clk(sclk), .mclk_in(mclk), .mclk_sel(mclk_sel), .fox_fsel(fox_fsel),
        .i2s_bclk_in(fb), .i2s_lrck_in(fl), .i2s_sdata_in(fd), .dsd_on(fox_dsd),
        .spdif_in({3'b000, sp0}),
        .in2_bclk(1'b0), .in2_lrck(1'b0), .in2_sdata(1'b0),
        .in3_bclk(1'b0), .in3_lrck(1'b0), .in3_sdata(1'b0), .in3_dsd(1'b0),
        .in4_bclk(b4), .in4_lrck(l4), .in4_sdata(d4),
        .i2c_scl(scl_m), .i2c_sda(i2c_sda),
        .i2s_bclk_out(obclk), .i2s_lrck_out(olrck), .i2s_sdata_out(osdata),
        .audio_on(audio_on));

    // ---------------- I2C master, 400 kHz ----------------
    localparam real TQ = 625.0;   // quarter bit (ns)
    integer errors = 0;
    task i2c_start; begin
        sda_m = 1; scl_m = 1; #(TQ); sda_m = 0; #(TQ); scl_m = 0; #(TQ);
    end endtask
    task i2c_stop; begin
        sda_m = 0; #(TQ); scl_m = 1; #(TQ); sda_m = 1; #(2 * TQ);
    end endtask
    task i2c_bit_w(input b); begin
        sda_m = b; #(TQ); scl_m = 1; #(2 * TQ); scl_m = 0; #(TQ);
    end endtask
    task i2c_bit_r(output b); begin
        sda_m = 1; #(TQ); scl_m = 1; #(TQ); b = i2c_sda; #(TQ); scl_m = 0; #(TQ);
    end endtask
    task i2c_byte_w(input [7:0] v, output ack);
        integer i; reg a;
        begin
            for (i = 7; i >= 0; i = i - 1) i2c_bit_w(v[i]);
            i2c_bit_r(a);
            ack = ~a;
        end
    endtask
    task i2c_byte_r(output [7:0] v, input ack);
        integer i; reg b;
        begin
            for (i = 7; i >= 0; i = i - 1) begin i2c_bit_r(b); v[i] = b; end
            i2c_bit_w(~ack);
        end
    endtask
    task wr_reg(input [7:0] r, input [7:0] v);
        reg a0, a1, a2;
        begin
            i2c_start; i2c_byte_w({7'h5A, 1'b0}, a0); i2c_byte_w(r, a1); i2c_byte_w(v, a2); i2c_stop;
            if (!(a0 && a1 && a2)) begin
                $display("ERROR: no ACK writing reg %0h", r); errors = errors + 1;
            end
        end
    endtask
    task rd_regs(input [7:0] r, output [7:0] v0, output [7:0] v1, output [7:0] v2, output [7:0] v3);
        reg a0, a1, a2;
        begin
            i2c_start; i2c_byte_w({7'h5A, 1'b0}, a0); i2c_byte_w(r, a1);
            i2c_start; i2c_byte_w({7'h5A, 1'b1}, a2);
            i2c_byte_r(v0, 1); i2c_byte_r(v1, 1); i2c_byte_r(v2, 1); i2c_byte_r(v3, 0);
            i2c_stop;
            if (!(a0 && a1 && a2)) begin
                $display("ERROR: no ACK reading reg %0h", r); errors = errors + 1;
            end
        end
    endtask

    // ---------------- output capture ----------------
    reg        olr_q = 0;
    reg [31:0] osr = 0, ow_l = 0;
    integer    fo = 0, ncap = 0;
    reg        cap = 0;
    real       peak = 0.0, v;
    always @(posedge obclk) begin
        olr_q <= olrck;
        if (olrck != olr_q) begin
            v = $itor($signed({osr[30:0], osdata})) / 2147483648.0;
            if (v < 0) v = -v;
            if (v > peak) peak = v;
            if (olr_q == 1'b0) ow_l <= {osr[30:0], osdata};
            else if (cap && audio_on) begin
                $fdisplay(fo, "%0d %0d", $signed(ow_l), $signed({osr[30:0], osdata}));
                ncap = ncap + 1;
            end
            osr <= 0;
        end else
            osr <= {osr[30:0], osdata};
    end

    task capture(input integer seg, input integer n);
        reg [8*24-1:0] name;
        begin
            $sformat(name, "build/v2_seg%0d.txt", seg);
            fo = $fopen(name, "w");
            ncap = 0; cap = 1;
            wait (ncap >= n);
            cap = 0;
            $fclose(fo);
        end
    endtask

    task wait_on(input real ms_max);
        real t0;
        begin
            t0 = $realtime;
            while (!audio_on && ($realtime - t0) < ms_max * 1e6) #(10000);
            if (!audio_on) begin
                $display("ERROR: no audio within %0.1f ms", ms_max); errors = errors + 1;
            end else
                $display("%8.2f ms  audio_on (input %0d)", $realtime / 1e6, dut.sel_cur);
        end
    endtask

    task check(input [7:0] got, input [7:0] exp, input [8*12-1:0] what);
        begin
            if (got !== exp) begin
                $display("ERROR: %0s = %02h, expected %02h", what, got, exp);
                errors = errors + 1;
            end else
                $display("          %0s = %02h ok", what, got);
        end
    endtask

    task stays_off(input real ms);
        real t0;
        begin
            t0 = $realtime;
            while (($realtime - t0) < ms * 1e6) begin
                if (audio_on) begin
                    $display("ERROR: audio on at %0.2f ms (must stay silent)", $realtime / 1e6);
                    errors = errors + 1;
                    t0 = -1e12;
                end
                #(10000);
            end
            $display("%8.2f ms  silent as expected", $realtime / 1e6);
        end
    endtask

    reg [7:0] r0, r1, r2, r3;
    // acknowledge a pending clock switch like the STM32 does
    task ack_switch(input real ms_max);
        real t0;
        begin
            t0 = $realtime; r0 = 0;
            while (!(r0 & 8'h02) && ($realtime - t0) < ms_max * 1e6) begin
                rd_regs(8'h15, r0, r1, r2, r3);
            end
            if (r0 & 8'h02) begin
                $display("%8.2f ms  clock switch request, acknowledged", $realtime / 1e6);
                wr_reg(8'h15, 8'h01);
            end else begin
                $display("ERROR: no clock switch request"); errors = errors + 1;
            end
        end
    endtask

    initial begin
        #(3e6);
        rd_regs(8'h05, r0, r1, r2, r3);
        check(r0, 8'hA5, "ID"); check(r1, 8'h20, "VERSION");
        // 1. Fox PCM 44.1k
        fox_fs = 44100.0; fox_div = 16; fox_fsel = 1; fox_en = 1;
        ack_switch(40.0);
        wait_on(60.0);
        rd_regs(8'h15, r0, r1, r2, r3);
        check(r0 & 8'h7F, 8'h39, "CLK_STAT");     // 45.1584 MHz, synchronous, meter 44.1
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h80, "FORMAT"); check(r1, 8'h01, "RATE"); check(r2, 8'h04, "PLL");
        capture(0, 4096);
        // 2. Fox PCM 96k: the Fox switches its MCLK_SEL first
        fox_fsel = 0; fox_fs = 96000.0; fox_div = 8; fox_fl = 1499.0; fox_fr = 7001.0;
        ack_switch(40.0);
        wait_on(60.0);
        rd_regs(8'h15, r0, r1, r2, r3);
        check(r0 & 8'h7F, 8'h18, "CLK_STAT");     // 49.152 MHz, synchronous, meter 48
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h89, "FORMAT"); check(r1, 8'h04, "RATE");
        capture(1, 4096);
        // 3. S/PDIF 44.1k: meter -> 45.1584 MHz, ASRC
        wr_reg(8'h00, 8'h01);
        ack_switch(60.0);
        wait_on(120.0);
        rd_regs(8'h15, r0, r1, r2, r3);
        check(r0 & 8'h7F, 8'h31, "CLK_STAT");     // 45.1584, meter valid 44.1, not sync
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h80, "FORMAT"); check(r1, 8'h01, "RATE");
        capture(2, 4096);
        // 4. phono 192k: meter -> 49.152 MHz
        wr_reg(8'h00, 8'h04);
        ack_switch(60.0);
        wait_on(120.0);
        rd_regs(8'h15, r0, r1, r2, r3);
        check(r0 & 8'h7F, 8'h10, "CLK_STAT");     // 49.152, meter valid 48k family
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h8A, "FORMAT"); check(r1, 8'h06, "RATE");
        capture(3, 4096);
        // 5. back to the Fox, DSD64 (44.1 family); no acknowledge this time:
        //    the bridge must switch on its own after the time-out
        fox_dsd = 1; fox_div = 16; fox_fsel = 1; fox_fl = 997.0; fox_fr = 3001.0;
        wr_reg(8'h00, 8'h00);
        wait_on(150.0);
        rd_regs(8'h15, r0, r1, r2, r3);
        check(r0 & 8'h7F, 8'h39, "CLK_STAT");
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h92, "FORMAT"); check(r1, 8'h11, "RATE");
        capture(4, 4096);
        if (errors == 0) $display("PASS tb_v2");
        else             $display("FAIL tb_v2 (%0d errors)", errors);
        $finish;
    end
    initial begin
        #(1500e6);
        $display("FAIL tb_v2: timeout");
        $finish;
    end
endmodule
