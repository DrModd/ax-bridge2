SHELL := /bin/bash
.SHELLFLAGS := -eu -o pipefail -c

RTL := rtl/mem.v rtl/afifo.v rtl/in_mux.v rtl/bclk_rx.v rtl/spdif_rx.v rtl/rate_mon.v rtl/dop_det.v \
       rtl/cic5_dsd.v rtl/fir_csd.v rtl/asrc.v rtl/i2s_tx32.v rtl/i2c_slave.v rtl/eq_biquad.v rtl/core.v rtl/top.v

.PHONY: all verify coef units csd v2 dsdl eq eqsys hdmi spdif spdif_stress corner sys switch mux clip recover lint synth jf jf_audio clean

all: verify

# Everything that must be green before flashing.
verify: coef lint units csd eq corner synth sys switch mux clip recover eqsys hdmi spdif v2 sync4 async dsdl spdif_stress

coef:
	python3 tools/gen_coefs.py --check

lint:
	mkdir -p build
	iverilog -g2005 -Wall -s top -o build/lint.vvp $(RTL)

# Bit-exact unit benches: every FIR configuration and the CIC at all R.
units:
	python3 tb/run_units.py

# Corner cases: clr in the middle of an ASRC convolution must not publish
# a stale sample; the DSD auto-detector must count the 64th edge of its
# window (all 64 alignments), never take PCM for DSD, and return to PCM.
corner:
	mkdir -p build
	iverilog -g2005 -Wall -s tb_asrc_clr -o build/asrc_clr.vvp tb/tb_asrc_clr.v rtl/mem.v rtl/asrc.v
	vvp -n build/asrc_clr.vvp | tee build/asrc_clr.log
	grep -q "PASS tb_asrc_clr" build/asrc_clr.log
	iverilog -g2005 -Wall -s tb_dsd_det -o build/dsd_det.vvp tb/tb_dsd_det.v rtl/bclk_rx.v
	vvp -n build/dsd_det.vvp | tee build/dsd_det.log
	grep -q "PASS tb_dsd_det" build/dsd_det.log

# EQ block: bit-exact vs model (live bank swap, saturation), fixed-point
# precision vs float64, coefficient quantisation.
eq:
	python3 tb/run_eq.py

# EQ through the whole bridge: coefficients loaded over I2C, levels at
# the test tones must follow the designed response.
eqsys:
	python3 tb/run_sys.py --eq

# HDMI input: native DSD64 auto-detected (no flag), L/R order, flag-only
# mode and active-low flag; a PCM input with auto-detect must stay PCM.
hdmi:
	python3 tb/run_sys.py --hdmi

# S/PDIF receiver: four lines (192k with jitter and duty distortion, 96k,
# 48k, non-audio ARC that must stay silent), line switching, status.
# Multiplier-free FIRs: fir_csd (shift-and-add) bit-exact against
# fir_stage (multiplier) for all five filters at their highest input rate.
csd:
	python3 tb/run_csd.py

spdif:
	python3 tb/run_sys.py --spdif

# S/PDIF receiver alone: line switches, on-the-fly rate changes, IEC 61937 bursts
spdif_stress:
	mkdir -p build
	iverilog -g2005 -DNSW=60 -s tb_spdif_stress -o build/spdif_stress.vvp tb/tb_spdif_stress.v rtl/spdif_rx.v
	vvp -n build/spdif_stress.vvp | grep -v 'switch to\|rate change' | tee build/spdif_stress.log
	grep -q '^PASS' build/spdif_stress.log
	iverilog -g2005 -DNSW=1 -DDUTY=10.0 -s tb_spdif_near -o build/spdif_near.vvp tb/tb_spdif_near.v tb/tb_spdif_stress.v rtl/spdif_rx.v
	vvp -n build/spdif_near.vvp | tee build/spdif_near.log
	grep -q '^PASS' build/spdif_near.log

# ax-bridge2 clocks: synchronous Fox, meter-chosen oscillator, switching
v2:
	python3 tb/run_sys.py --v2

# Synchronous phono ADC (SYNC_IN4 = 1, AK5572 on our MCLK): sync path on
# both oscillators (192 / 176.4 kHz), the ADC never moves the clock, it
# follows a forced switch (CLK_CTRL 0x04) and survives the return to 0x0D.
sync4:
	python3 tb/run_sys.py --sync4

# Fox native DSD with the left channel on its own line (D2, FOX_DSDL = 1)
# and the old three-wire layout (FOX_DSDL = 0); top with the D2 board ports,
# the ignored line carries noise; L/R tones, the DSD_CFG swap, PCM 44.1k.
dsdl:
	python3 tb/run_sys.py --dsdl

# All-async mode entered and left at run time (CLK_CTRL 0x0D -> 0x00 -> 0x0D):
# 49.152 MHz fixed, Fox 44.1/96 on its own clock, S/PDIF 44.1 and the
# synchronous phono ADC all through the ASRC; the Fox pin and the manual
# family bit ignored; back to 0x0D with the STM32 handshake.
async:
	python3 tb/run_sys.py --async

# System benches: 10 PCM rates, DSD64..512, DoP64..256 with clock offsets,
# output decoded from the I2S pins, THD+N / level / frequency checked.
# ~25-40 min with Icarus; single case: python3 tb/run_sys.py pcm44
sys:
	python3 tb/run_sys.py

# Live format switching (PCM 44.1 -> 96 -> DSD64 -> 768k -> DoP64 -> 192k,
# with and without clock stops): relock on the right path, no clicks.
switch:
	python3 tb/run_sys.py --switch

# Input selector + I2C: four transports on unrelated clocks (DSD64, PCM
# 44.1, PCM 96, dead input), selection / mute / status over I2C.
mux:
	python3 tb/run_sys.py --mux

# Headroom / clip flag: +3 dBFS inter-sample peaks at fs/4 must clip
# without headroom and pass cleanly with 3.5 dB.
clip:
	python3 tb/run_sys.py --clip

# Recovery and switch ordering: a single forced stage overflow must not
# block audio; the input mux may only move under an established reset.
recover:
	mkdir -p build
	iverilog -g2005 -Wall -s tb_recover -o build/recover.vvp tb/tb_recover.v tb/tb_src.v tb/tb_spdif_src.v $(RTL)
	vvp -n build/recover.vvp | tee build/recover.log
	grep -q "PASS tb_recover" build/recover.log

# Elaboration gate (the real synthesis is Gowin IDE):
# 8 multipliers (ASRC 3, EQ 1, headroom 2, fade 2; the five FIRs are
# shift-and-add since fir_csd), 20 inferred memories (6 FIR programs +
# 6 FIR histories, ASRC ring + kernel, EQ coefficients, input FIFO,
# S/PDIF FIFO, synchronous-path buffer L/R, headroom gain ROM), no errors.
synth:
	mkdir -p build
	yosys -p "read_verilog $(RTL); hierarchy -check -top top; proc; opt -fast; memory -nomap; opt -fast; flatten; stat" > build/synth.log
	! grep -iE "error" build/synth.log
	grep -E -q '([$$]mul +8$$|^ +8 +[$$]mul$$)' build/synth.log   # old / new yosys stat layout
	grep -E -q '([$$]mem_v2 +20$$|^ +20 +[$$]mem_v2$$)' build/synth.log   # old / new yosys stat layout

# ---- jitter_fifo (standalone module, not yet wired into top) ----
JF = jitter_fifo/rtl/jitter_fifo.v jitter_fifo/tb/tb_jitter_fifo.v
JF_LS64 = -Ptb_jitter_fifo.AW=8 -Ptb_jitter_fifo.NT=64 -Ptb_jitter_fifo.MPH_SH=7 \
          -Ptb_jitter_fifo.DIV=128 -Ptb_jitter_fifo.TOL=40 \
          -Ptb_jitter_fifo.COEF_FILE='"jitter_fifo/rtl/fir_ls64_m128.hex"'

# Self-checking ramp / pause bench with a bit-exact reference model,
# 4 runs: cubic (CR4) and 64-tap FIR, writer faster and slower.
jf:
	mkdir -p build
	iverilog -g2005 -Wall -o build/tb_jf_cr4_fast $(JF)
	iverilog -g2005 -Wall -Ptb_jitter_fifo.RD_PER=20.325 -o build/tb_jf_cr4_slow $(JF)
	iverilog -g2005 -Wall $(JF_LS64) -Ptb_jitter_fifo.RD_PER=20.406 -o build/tb_jf_ls64_fast $(JF)
	iverilog -g2005 -Wall $(JF_LS64) -Ptb_jitter_fifo.RD_PER=20.284 -o build/tb_jf_ls64_slow $(JF)
	for t in cr4_fast cr4_slow ls64_fast ls64_slow; do \
	    vvp -n build/tb_jf_$$t | tee build/tb_jf_$$t.log | grep -E "^(PASS|FAIL)"; \
	    grep -q "^PASS" build/tb_jf_$$t.log; done

# Sine through the buffer with +-2500 ppm: THD+N in bit-exact and
# interpolated stretches (~15 min).
jf_audio:
	python3 jitter_fifo/tb/run_jf.py

clean:
	rm -rf build
