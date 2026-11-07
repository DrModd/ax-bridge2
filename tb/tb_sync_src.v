`timescale 1ns/1fs
// Behavioural I2S master clocked from the bridge's own MCLK (the Fox in
// external-clock mode, or the AK5572 phono ADC with its MCLK taken from the
// oscillator mux): BCLK = MCLK / div, 32-bit slots, same framing as tb_src
// (LRCK = slot[5], data one bit late, MSB first, changes on the falling
// BCLK edge). The sample rate follows the oscillator: fs = MCLK / (64 div),
// MCLK = 45.1584 MHz when fam44 = 1, else 49.152 MHz, so a clock switch
// moves the source with it like the real hardware. EN = 0: lines low.
module tb_sync_src #(
    parameter real FL  = 1201.0,
    parameter real FR  = 4001.0,
    parameter real AMP = 0.891
) (
    input  wire       mclk,
    input  wire       en,
    input  wire       fam44,
    input  wire [7:0] div,          // MCLK / BCLK (even, >= 2)
    output reg        bclk = 1'b0,
    output reg        lrck = 1'b0,
    output reg        sdata = 1'b0
);
    reg [7:0] dc = 8'd0;
    always @(posedge mclk) begin
        if (!en) begin bclk <= 1'b0; dc <= 8'd0; end
        else if (dc == div / 2 - 1) begin dc <= 8'd0; bclk <= ~bclk; end
        else dc <= dc + 8'd1;
    end

    function [31:0] pcm32;
        input real v;
        integer iv;
        begin
            iv = $rtoi(v * 8388607.0);
            if (iv > 8388607)  iv = 8388607;
            if (iv < -8388608) iv = -8388608;
            pcm32 = {iv[23:0], 8'h00};
        end
    endfunction

    reg [5:0]  slot = 6'd0;
    reg [31:0] wl = 0, wr = 0;
    reg        wr_lsb = 1'b0;
    real       tt = 0.0, fs;
    always @(negedge bclk) begin
        if (slot == 6'd0) begin
            fs = (fam44 ? 45158400.0 : 49152000.0) / (64.0 * div);
            wl = pcm32(AMP * $sin(2.0 * 3.14159265358979 * FL * tt));
            wr = pcm32(AMP * $sin(2.0 * 3.14159265358979 * FR * tt));
            tt = tt + 1.0 / fs;
        end
        lrck <= slot[5];
        if (slot == 6'd0)       sdata <= wr_lsb;
        else if (slot <= 6'd32) sdata <= wl[32 - slot];
        else                    sdata <= wr[64 - slot];
        if (slot == 6'd63) wr_lsb <= wr[0];
        slot <= slot + 6'd1;
    end
endmodule
