`timescale 1ns/1ps
// Bit-exactness of fir_csd (shift-and-add) against fir_stage (multiplier):
// the same random stream (with full-scale bursts for saturation) through
// both, at the stage's highest input rate (one input per PER clocks).
module tb_csd;
    parameter integer N = 31, INTERP = 0, AW = 5, CAW = 5, PW = 7, PLEN = 66, P1 = 0, PER = 128;
    parameter FILE = "coef/hd_b.bin";
    parameter PFILE = "coef/hd_b.prog";
    parameter integer LANES = 1, PLEN2 = 1;
    parameter PFILE2 = "coef/hd_b.prog";
    parameter integer NIN = 3000;
    reg clk = 0; always #10 clk = ~clk;
    reg rst_n = 0;
    reg iv = 0; reg signed [23:0] il = 0, ir = 0;
    wire va, vb, oa, ob, ca, cb; wire signed [23:0] la, ra, lb, rb;
    fir_stage #(.N(N), .INTERP(INTERP), .AW(AW), .CAW(CAW), .FILE(FILE)) ref_ (
        .clk(clk), .rst_n(rst_n), .clr(1'b0), .in_valid(iv), .in_l(il), .in_r(ir),
        .out_valid(va), .out_l(la), .out_r(ra), .ovf(oa), .clip(ca));
    fir_csd #(.N(N), .INTERP(INTERP), .AW(AW), .PW(PW), .PLEN(PLEN), .P1(P1), .FILE(PFILE),
              .LANES(LANES), .PLEN2(PLEN2), .FILE2(PFILE2)) dut (
        .clk(clk), .rst_n(rst_n), .clr(1'b0), .in_valid(iv), .in_l(il), .in_r(ir),
        .out_valid(vb), .out_l(lb), .out_r(rb), .ovf(ob), .clip(cb));
    reg [47:0] qa [0:4095]; reg [47:0] qb [0:4095];
    integer na = 0, nb = 0, i, k, err = 0, nca = 0, ncb = 0;
    always @(posedge clk) begin
        if (va) begin qa[na] = {la, ra}; na = na + 1; end
        if (vb) begin qb[nb] = {lb, rb}; nb = nb + 1; end
        if (ca) nca = nca + 1;
        if (cb) ncb = ncb + 1;
    end
    initial begin
        #100 rst_n = 1;
        for (i = 0; i < NIN; i = i + 1) begin
            @(posedge clk);
            iv <= 1;
            if ((i / 200) % 4 == 3) begin           // full-scale square bursts
                il <= (i % 2) ? 24'sh7FFFFF : 24'sh800000; ir <= (i % 3) ? 24'sh800000 : 24'sh7FFFFF;
            end else begin
                il <= $random; ir <= $random;
            end
            @(posedge clk); iv <= 0;
            repeat (PER - 2) @(posedge clk);
        end
        repeat (2000) @(posedge clk);
        for (k = 0; k < na && k < nb; k = k + 1)
            if (qa[k % 4096] !== qb[k % 4096]) begin
                if (err < 5) $display("mismatch at %0d: ref %h csd %h", k, qa[k % 4096], qb[k % 4096]);
                err = err + 1;
            end
        if (na != nb || err || oa || ob || (nca > 0) != (ncb > 0))
            $display("FAIL %s: outputs %0d / %0d, mismatches %0d, ovf %0d/%0d, clips %0d/%0d", PFILE, na, nb, err, oa, ob, nca, ncb);
        else
            $display("PASS %s: %0d outputs bit-exact, %0d clips", PFILE, nb, ncb);
        $finish;
    end
endmodule
