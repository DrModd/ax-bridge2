// Shared by tb_sync4 / tb_async (included inside the bench module):
// I2C master at 400 kHz on scl_m / sda_m / i2c_sda, register access,
// output capture from obclk / olrck / osdata into build/<pfx>_seg<N>.txt,
// audio_on waits and the STM32 clock-switch acknowledge.
// Needs: sclk-independent nets scl_m, sda_m, i2c_sda, obclk, olrck,
// osdata, audio_on and the instance name dut.

    localparam real TQ = 625.0;   // quarter bit (ns)
    integer errors = 0;
    reg [7:0] r0, r1, r2, r3;
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
    always @(posedge obclk) begin
        olr_q <= olrck;
        if (olrck != olr_q) begin
            if (olr_q == 1'b0) ow_l <= {osr[30:0], osdata};
            else if (cap && audio_on) begin
                $fdisplay(fo, "%0d %0d", $signed(ow_l), $signed({osr[30:0], osdata}));
                ncap = ncap + 1;
            end
            osr <= 0;
        end else
            osr <= {osr[30:0], osdata};
    end

    // audio must stay on for the whole capture (a dropout = error)
    integer ndrop = 0;
    task capture(input integer seg, input integer n, input [8*4-1:0] pfx);
        reg [8*32-1:0] name;
        begin
            $sformat(name, "build/%0s_seg%0d.txt", pfx, seg);
            fo = $fopen(name, "w");
            ncap = 0; cap = 1;
            while (ncap < n && audio_on) #(1000);
            if (!audio_on) begin
                $display("ERROR: audio dropped during capture %0d (%0d frames)", seg, ncap);
                errors = errors + 1;
            end
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
                $display("%8.2f ms  audio_on (input %0d, %0s MHz)", $realtime / 1e6, dut.sel_cur,
                         dut.mclk_sel ? "45.1584" : "49.152");
        end
    endtask

    task wait_off(input real ms_max);
        real t0;
        begin
            t0 = $realtime;
            while (audio_on && ($realtime - t0) < ms_max * 1e6) #(10000);
            if (audio_on) begin
                $display("ERROR: audio did not stop within %0.1f ms", ms_max); errors = errors + 1;
            end
        end
    endtask

    task stays_on(input real ms);
        real t0;
        begin
            t0 = $realtime;
            while (($realtime - t0) < ms * 1e6) begin
                if (!audio_on) begin
                    $display("ERROR: audio off at %0.2f ms (must stay on)", $realtime / 1e6);
                    errors = errors + 1;
                    t0 = -1e12;
                end
                #(10000);
            end
            $display("%8.2f ms  audio stayed on", $realtime / 1e6);
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
