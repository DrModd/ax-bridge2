`timescale 1ns/1ps
// Input-rate classifier. Counts items in a window of 2^WBITS clocks of
// its own clock and matches the count to
// 44.1k*2^k or 48k*2^k items per second (+-1/32 tolerance), k = 0..5.
//   PCM / DoP : one item per stereo frame  -> k = 0 (44.1/48) .. 4 (705.6/768)
//   native DSD: one item per 16 bit-clocks -> k = 2 (DSD64) .. 5 (DSD512)
// A class is reported stable after two equal windows; any other window
// result clears 'stable' at once. 'seq' toggles at every window end.
// Two sets of nominal counts (alt = 0 / 1) for a clock that switches
// between two frequencies (MCLK 49.152 / 45.1584 MHz); for a fixed clock
// both sets are equal. Defaults: 49.152 MHz (set 0), 45.1584 MHz (set 1).
module rate_mon #(
    parameter WBITS = 18,     // 2^18 clocks: 5.33 ms at 49.152 MHz
    parameter integer N44_0 = ((1 << WBITS) * 441 + 245760) / 491520,   // 44.1k items / window
    parameter integer N48_0 = (1 << WBITS) / 1024,
    parameter integer N44_1 = (1 << WBITS) / 1024,
    parameter integer N48_1 = ((1 << WBITS) * 480 + 225792) / 451584
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       item,       // one strobe per FIFO item
    input  wire       item_dsd,   // tag of that item
    input  wire       dop,        // DoP detector state
    input  wire       alt,        // 1: second set of nominal counts (static while counting)
    output reg        stable,
    output reg  [2:0] k,
    output reg        fam48,
    output reg        dsd,        // native DSD
    output reg        is_dop,
    output reg        seq
);
    localparam integer CW = WBITS - 4;     // counter width (768k max)

    reg [WBITS-1:0] wcnt;
    reg [CW-1:0]    icnt;
    reg             saw_pcm, saw_dsd, dop_all;

    function [4:0] classify;   // {valid, fam48, k[2:0]}
        input [CW-1:0] c;
        input          a;
        integer j;
        integer n44, n48;
        reg [4:0] r;
        begin
            r = 5'd0;
            for (j = 0; j < 6; j = j + 1) begin
                n44 = (a ? N44_1 : N44_0) << j;
                n48 = (a ? N48_1 : N48_0) << j;
                if ((c >= n44 - (n44 >> 5)) && (c <= n44 + (n44 >> 5)))
                    r = {1'b1, 1'b0, j[2:0]};
                if ((c >= n48 - (n48 >> 5)) && (c <= n48 + (n48 >> 5)))
                    r = {1'b1, 1'b1, j[2:0]};
            end
            classify = r;
        end
    endfunction

    wire [4:0] cl = classify(icnt, alt);
    // DSD items with PCM items in the same window -> mixed (invalid)
    wire        ok = cl[4] && !(saw_pcm && saw_dsd);
    wire [5:0]  key = {cl[3:0], saw_dsd, dop_all & ~saw_dsd};
    reg  [5:0]  last_key;
    reg         last_ok;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wcnt <= {WBITS{1'b0}};
            icnt <= {CW{1'b0}};
            saw_pcm <= 1'b0; saw_dsd <= 1'b0; dop_all <= 1'b1;
            stable <= 1'b0; k <= 3'd0; fam48 <= 1'b0; dsd <= 1'b0;
            is_dop <= 1'b0; seq <= 1'b0;
            last_key <= 6'd0; last_ok <= 1'b0;
        end else begin
            wcnt <= wcnt + 1'b1;
            if (&wcnt) begin
                seq <= ~seq;
                last_key <= key;
                last_ok <= ok;
                if (ok && last_ok && (key == last_key)) begin
                    stable <= 1'b1;
                    {fam48, k} <= cl[3:0];
                    dsd <= saw_dsd;
                    is_dop <= dop_all & ~saw_dsd;
                end else begin
                    stable <= 1'b0;
                end
                icnt <= {{(CW-1){1'b0}}, item};
                saw_pcm <= item & ~item_dsd;
                saw_dsd <= item & item_dsd;
                dop_all <= dop;
            end else begin
                if (item) begin
                    if (~&icnt)
                        icnt <= icnt + 1'b1;
                    if (item_dsd) saw_dsd <= 1'b1;
                    else          saw_pcm <= 1'b1;
                end
                if (!dop)
                    dop_all <= 1'b0;
            end
        end
    end
endmodule
