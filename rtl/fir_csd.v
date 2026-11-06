`timescale 1ns/1ps
// Multiplier-free stereo FIR stage (shift-and-add, no DSP blocks).
// Same interface and bit-identical results as fir_stage.v: the constant
// coefficients are written in canonical signed digits, and a small engine
// runs a per-output program from a ROM (tools/gen_coefs.py, coef/*.prog):
//   00 LOAD off   x  = h[newest - off]          (L and R together)
//   01 ADD  off   x += h[newest - off]          (symmetric partner tap)
//   10 TERM s sh  acc += (s ? -x : x) << sh     (one signed digit)
//   11 END        output: round half-up, saturate to S24
// Symmetric taps are added before the digits, zero taps (every other tap
// of a half-band) are simply absent. One op per clock; both channels at
// once. Cycles per output = ops of that phase + ~6. LANES = 2 splits the
// program over two engines (each with its own copy of the history) for a
// long filter at a high rate (the DSD FIR); their sums are added.
//   INTERP=1: x2 interpolator, phase 0 program at 0, phase 1 at P1
//             (phase 0 = even coefficient indices, as fir_stage)
//   INTERP=0: /2 decimator, one program per two inputs
module fir_csd #(
    parameter N      = 31,         // taps (documentation)
    parameter INTERP = 0,
    parameter AW     = 6,          // history depth 2^AW
    parameter PW     = 9,          // program address width
    parameter PLEN   = 66,         // program length (lane 0)
    parameter P1     = 0,          // start of the phase-1 program (INTERP)
    parameter FILE   = "coef/hd_b.prog",
    parameter LANES  = 1,
    parameter PLEN2  = 1,          // lane 1 program length
    parameter FILE2  = "coef/hd_b.prog"
) (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              clr,
    input  wire              in_valid,
    input  wire signed [23:0] in_l,
    input  wire signed [23:0] in_r,
    output reg               out_valid,
    output reg  signed [23:0] out_l,
    output reg  signed [23:0] out_r,
    output reg               ovf,
    output reg               clip
);
    // ---- input queue (as fir_stage) ----
    reg [47:0] q0, q1, q2, q3;
    reg [2:0]  qn;

    // ---- history write side ----
    reg  [AW-1:0] wp, newest, waddr;
    reg           we;
    reg  [47:0]   wdat;

    // ---- lanes ----
    reg           start;
    reg  [PW-1:0] spc;
    wire          d0, d1;
    wire signed [55:0] a0l, a0r, a1l, a1r;
    csd_lane #(.AW(AW), .PW(PW), .PLEN(PLEN), .FILE(FILE)) u_l0 (
        .clk(clk), .rst_n(rst_n), .clr(clr), .we(we), .waddr(waddr), .wdat(wdat),
        .newest(newest), .start(start), .spc(spc), .done(d0), .acc_l(a0l), .acc_r(a0r));
    generate if (LANES > 1) begin : g_l1
        csd_lane #(.AW(AW), .PW(PW), .PLEN(PLEN2), .FILE(FILE2)) u_l1 (
            .clk(clk), .rst_n(rst_n), .clr(clr), .we(we), .waddr(waddr), .wdat(wdat),
            .newest(newest), .start(start), .spc({PW{1'b0}}), .done(d1), .acc_l(a1l), .acc_r(a1r));
    end else begin : g_n1
        assign d1 = 1'b0; assign a1l = 56'sd0; assign a1r = 56'sd0;
    end endgenerate

    localparam S_IDLE = 2'd0, S_WAIT = 2'd1, S_RUN = 2'd2, S_OUT = 2'd3;
    reg [1:0] st;
    reg       ph;
    reg       g0, g1;                // lane results in
    reg signed [55:0] rl, rr;
    reg        fin;

    function [24:0] rnd_sat;        // {ovf, S24}
        input signed [55:0] a;
        reg signed [55:0] ar_;
        reg signed [32:0] y;
        begin
            ar_ = a + 56'sd4194304;
            y = ar_[55:23];
            if (y > 33'sd8388607)       rnd_sat = {1'b1, 24'sh7FFFFF};
            else if (y < -33'sd8388608) rnd_sat = {1'b1, 24'sh800000};
            else                        rnd_sat = {1'b0, y[23:0]};
        end
    endfunction
    wire [24:0] yl = rnd_sat(rl);
    wire [24:0] yr = rnd_sat(rr);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q0 <= 48'd0; q1 <= 48'd0; q2 <= 48'd0; q3 <= 48'd0; qn <= 3'd0;
            wp <= {AW{1'b0}}; newest <= {AW{1'b0}}; waddr <= {AW{1'b0}}; we <= 1'b0; wdat <= 48'd0;
            start <= 1'b0; spc <= {PW{1'b0}}; st <= S_IDLE; ph <= 1'b0; g0 <= 1'b0; g1 <= 1'b0;
            rl <= 56'sd0; rr <= 56'sd0; fin <= 1'b0;
            out_valid <= 1'b0; out_l <= 24'sd0; out_r <= 24'sd0; ovf <= 1'b0; clip <= 1'b0;
        end else if (clr) begin
            ovf <= 1'b0;
            qn <= 3'd0; st <= S_IDLE; ph <= 1'b0; we <= 1'b0; start <= 1'b0;
            g0 <= 1'b0; g1 <= 1'b0; fin <= 1'b0; out_valid <= 1'b0;
        end else begin
            out_valid <= 1'b0;
            we <= 1'b0;
            clip <= 1'b0;
            start <= 1'b0;
            fin <= 1'b0;
            // ---- queue ----
            case ({in_valid, (st == S_IDLE) && (qn != 3'd0)})
                2'b10: begin
                    case (qn)
                        3'd0: q0 <= {in_l, in_r};
                        3'd1: q1 <= {in_l, in_r};
                        3'd2: q2 <= {in_l, in_r};
                        3'd3: q3 <= {in_l, in_r};
                        default: ovf <= 1'b1;
                    endcase
                    if (qn != 3'd4) qn <= qn + 3'd1;
                end
                2'b01: begin q0 <= q1; q1 <= q2; q2 <= q3; qn <= qn - 3'd1; end
                2'b11: begin
                    q0 <= q1; q1 <= q2; q2 <= q3;
                    case (qn)
                        3'd1: q0 <= {in_l, in_r};
                        3'd2: q1 <= {in_l, in_r};
                        3'd3: q2 <= {in_l, in_r};
                        default: q3 <= {in_l, in_r};
                    endcase
                end
                default: ;
            endcase

            // ---- control ----
            case (st)
                S_IDLE: if (qn != 3'd0) begin
                    we <= 1'b1; waddr <= wp; wdat <= q0; newest <= wp; wp <= wp + 1'b1;
                    if (INTERP) begin
                        ph <= 1'b0; st <= S_WAIT;
                    end else begin
                        ph <= ~ph;
                        if (ph) st <= S_WAIT;
                    end
                end
                S_WAIT: begin                      // RAM write lands
                    start <= 1'b1; spc <= (INTERP && ph) ? P1[PW-1:0] : {PW{1'b0}};
                    g0 <= 1'b0; g1 <= (LANES < 2);
                    st <= S_RUN;
                end
                S_RUN: begin
                    // lane results stay in acc_* until the lane's next END
                    if (d0) g0 <= 1'b1;
                    if (d1) g1 <= 1'b1;
                    if (g0 && g1) begin
                        rl <= a0l + a1l; rr <= a0r + a1r;
                        fin <= 1'b1;
                        st <= S_OUT;
                    end
                end
                S_OUT: begin
                    if (INTERP && !ph) begin
                        ph <= 1'b1; st <= S_WAIT;
                    end else
                        st <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase

            // ---- output (fin: rl/rr complete) ----
            if (fin) begin
                out_l <= yl[23:0]; out_r <= yr[23:0]; out_valid <= 1'b1;
                clip <= yl[24] | yr[24];
            end
        end
    end
endmodule

// One shift-and-add engine: program ROM, own copy of the history, x / acc.
module csd_lane #(
    parameter AW   = 6,
    parameter PW   = 9,
    parameter PLEN = 66,
    parameter FILE = "coef/hd_b.prog"
) (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              clr,
    input  wire              we,
    input  wire [AW-1:0]     waddr,
    input  wire [47:0]       wdat,
    input  wire [AW-1:0]     newest,
    input  wire              start,      // pulse: run the program from spc
    input  wire [PW-1:0]     spc,
    output reg               done,       // pulse: acc_l / acc_r hold the result
    output reg signed [55:0] acc_l,
    output reg signed [55:0] acc_r
);
    reg  [AW-1:0] raddr;
    wire [47:0]   rdata;
    sdp_ram #(.W(48), .AW(AW)) u_hist (
        .clk(clk), .we(we), .waddr(waddr), .wdata(wdat), .raddr(raddr), .rdata(rdata));
    reg  [PW-1:0] pc;
    wire [15:0]   w1;
    rom #(.W(16), .AW(PW), .DEPTH(PLEN), .FILE(FILE)) u_prog (.clk(clk), .addr(pc), .data(w1));

    reg        iss, v1, v2, v3;
    reg [15:0] w2, w3;
    reg signed [24:0] xl, xr;
    reg signed [55:0] al, ar;
    wire [1:0] op1 = w1[15:14];
    wire [1:0] op3 = w3[15:14];
    wire signed [24:0] hl = {rdata[47], rdata[47:24]};
    wire signed [24:0] hr = {rdata[23], rdata[23:0]};
    wire [4:0]         sh3 = w3[12:8];
    wire signed [55:0] tl = {{31{xl[24]}}, xl} <<< sh3;
    wire signed [55:0] tr = {{31{xr[24]}}, xr} <<< sh3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            raddr <= {AW{1'b0}}; pc <= {PW{1'b0}}; iss <= 1'b0; v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0;
            w2 <= 16'd0; w3 <= 16'd0; xl <= 25'sd0; xr <= 25'sd0; al <= 56'sd0; ar <= 56'sd0;
            done <= 1'b0; acc_l <= 56'sd0; acc_r <= 56'sd0;
        end else if (clr) begin
            iss <= 1'b0; v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0; done <= 1'b0; al <= 56'sd0; ar <= 56'sd0;
        end else begin
            done <= 1'b0;
            if (start) begin
                pc <= spc; iss <= 1'b1;
            end else if (iss) begin
                pc <= pc + 1'b1;
                if (v1 && op1 == 2'd3) iss <= 1'b0;   // END decoded: stop fetching
            end
            v1 <= iss && !start && !(v1 && op1 == 2'd3);
            raddr <= newest - w1[AW-1:0];
            v2 <= v1; w2 <= w1;
            v3 <= v2; w3 <= w2;
            if (v3) case (op3)
                2'd0: begin xl <= hl; xr <= hr; end
                2'd1: begin xl <= xl + hl; xr <= xr + hr; end
                2'd2: begin
                    al <= w3[13] ? al - tl : al + tl;
                    ar <= w3[13] ? ar - tr : ar + tr;
                end
                default: begin acc_l <= al; acc_r <= ar; al <= 56'sd0; ar <= 56'sd0; done <= 1'b1; end
            endcase
        end
    end
endmodule
