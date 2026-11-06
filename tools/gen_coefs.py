#!/usr/bin/env python3
"""Generate every coefficient ROM of the AX5689 bridge (coef/*.hex).

All FIR coefficients are signed 25-bit, scale 2^23 (Q1.23 with one guard
bit so the interpolator centre tap 1.0 fits). Data samples are S24.

  hb_a   x2 interpolator 44.1/48 -> 88.2/96  (halfband, pass 20k, stop 24.1k)
  hb_b   x2 interpolator 88.2/96 -> 176.4/192 (halfband, pass 36k, stop 52.2k)
  hd_a   /2 decimator 705.6/768 -> 352.8/384 (halfband, pass 40k)
  hd_b   /2 decimator 352.8/384 -> 176.4/192 (halfband, pass 40k)
  dsd    /2 decimator for the DSD path after CIC5 (352.8 -> 176.4),
         pass 0..24k with CIC5 droop compensation, stop >= 60k
  asrc   ASRC kernel, 40 input-sample span, 256 phases, Kaiser-windowed
         sinc, cutoff 64 kHz @176.4 (pass 0..40k, stop >= 88.2k)

Run with --check to print the measured responses.
"""
import os
import sys
import numpy as np
from scipy import signal

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "coef")
CW = 25            # coefficient width
CS = 23            # coefficient scale (2^23 == 1.0)
CMAX = (1 << (CW - 1)) - 1
CMIN = -(1 << (CW - 1))

def q(h):
    v = np.round(np.asarray(h) * (1 << CS)).astype(np.int64)
    if v.max() > CMAX or v.min() < CMIN:
        raise SystemExit("coefficient overflow")
    return v

def hexw(v, width):
    digits = (width + 3) // 4
    return format(int(v) & ((1 << width) - 1), "0%dx" % digits)

def write_bin(name, vals, width):
    # exact-width binary for $readmemb in the RTL (no width mismatch)
    with open(os.path.join(OUT, name), "w") as f:
        for v in vals:
            f.write(format(int(v) & ((1 << width) - 1), "0%db" % width) + "\n")

def write_hex(name, vals, width=CW):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name), "w") as f:
        for v in vals:
            f.write(hexw(v, width) + "\n")

def kaiser_hb(N, beta):
    n = np.arange(N) - (N - 1) / 2
    h = 0.5 * np.sinc(n / 2) * np.kaiser(N, beta)
    h[(N - 1) // 2] = 0.5
    return h

def cic_resp(f_over_fout, R=8, order=5):
    x = np.asarray(f_over_fout, dtype=float) / R
    num = np.sin(np.pi * x * R)
    den = R * np.sin(np.pi * x)
    with np.errstate(invalid="ignore", divide="ignore"):
        H = np.where(np.abs(den) < 1e-15, 1.0, num / den)
    return np.abs(H) ** order

def dsd_fir(N=95, fp=24.0, fst=60.0, fs=352.8):
    grid = np.linspace(0, fp, 96)
    f = np.concatenate([grid, np.linspace(fst, fs / 2, 400)])
    d = np.concatenate([1.0 / cic_resp(grid / fs), np.zeros(400)])
    w = np.concatenate([np.full(96, 1.0), np.full(400, 3000.0)])
    M = (N - 1) // 2
    k = np.arange(M + 1)
    A = np.cos(2 * np.pi * np.outer(f / fs, k))
    A[:, 1:] *= 2
    W = np.sqrt(w)
    a, *_ = np.linalg.lstsq(A * W[:, None], d * W, rcond=None)
    return np.concatenate([a[:0:-1], a])

ASRC_L = 40
ASRC_P = 256

def asrc_kernel(L=ASRC_L, P=ASRC_P, beta=13.0, fc=64.0 / 176.4):
    n = np.arange(-L // 2 * P, L // 2 * P + 1) / P
    h = 2 * fc * np.sinc(2 * fc * n) * np.kaiser(len(n), beta)
    s = np.array([h[p::P].sum() for p in range(P)]).mean()
    return n, h / s

def build():
    stages = {
        "hb_a": (kaiser_hb(175, 12.5) * 2.0, 88.2, 20.0, 24.1, 2.0),
        "hb_b": (kaiser_hb(91, 12.25) * 2.0, 176.4, 36.0, 52.2, 2.0),
        "hd_a": (kaiser_hb(23, 12.5), 705.6, 40.0, 312.8, 1.0),
        "hd_b": (kaiser_hb(31, 12.5), 352.8, 40.0, 136.4, 1.0),
        "dsd":  (dsd_fir(), 352.8, 24.0, 60.0, 1.0),
    }
    res = {}
    for name, (h, fs, fp, fst, gain) in stages.items():
        hq = q(h)
        write_hex(f"{name}.hex", hq)
        write_bin(f"{name}.bin", hq, CW)
        res[name] = (hq, fs, fp, fst, gain)
    n, h = asrc_kernel()
    half = h[len(h) // 2:]                 # h(u/P), u = 0 .. L/2*P
    hq = q(half)
    hq = np.concatenate([hq, [0]])         # guard entry (a+1 at the edge)
    pairs = [(int(hq[a]) & ((1 << CW) - 1)) << CW | (int(hq[a + 1]) & ((1 << CW) - 1))
             for a in range(len(hq) - 1)]
    write_hex("asrc.hex", pairs, 2 * CW)
    write_bin("asrc.bin", pairs, 2 * CW)
    res["asrc"] = (hq,)
    return res

# ---------------------------------------------------------------- CSD programs
# Multiplier-free FIR (rtl/fir_csd.v): each output is a short program for a
# shift-and-add engine. Ops (16 bit):
#   00 LOAD off   x  = h[newest - off]          (both channels)
#   01 ADD  off   x += h[newest - off]          (symmetric partner)
#   10 TERM s sh  acc += (s ? -x : x) << sh     (one CSD digit)
#   11 END        output (rounded, saturated)
# Coefficients are the same integers as the multiplier ROMs, written in
# canonical signed digits, so the result is bit-identical.
def csd_digits(v):
    """canonical signed-digit form: list of (sign, shift), sign 1 = negative"""
    out = []
    x = abs(int(v))
    neg = v < 0
    sh = 0
    while x:
        if x & 1:
            if (x & 3) == 3:
                out.append((not neg, sh)); x += 1
            else:
                out.append((neg, sh)); x -= 1
        x >>= 1
        sh += 1
    return out

def csd_program(c, interp):
    N = len(c)
    phases = [0, 1] if interp else [0]
    prog, starts = [], []
    for p in phases:
        starts.append(len(prog))
        if interp:
            taps = {t: int(c[2 * t + p]) for t in range((N - p + 1) // 2) if 2 * t + p < N}
            tap_k = {t: 2 * t + p for t in taps}
        else:
            taps = {t: int(c[t]) for t in range(N)}
            tap_k = {t: t for t in taps}
        k_tap = {k: t for t, k in tap_k.items()}
        done = set()
        for t in sorted(taps):
            if t in done or taps[t] == 0:
                continue
            k = tap_k[t]
            km = N - 1 - k
            tm = k_tap.get(km)
            prog.append((0 << 14) | t)
            done.add(t)
            if tm is not None and tm != t and taps[tm] == taps[t]:
                prog.append((1 << 14) | tm)
                done.add(tm)
            for sg, sh in csd_digits(taps[t]):
                prog.append((2 << 14) | (int(sg) << 13) | (sh << 8))
        prog.append(3 << 14)
    return prog, starts

def split_lanes(prog):
    """split a one-phase program into two balanced lanes (whole coefficient
    groups: a LOAD with its ADD and digits), each ending with END"""
    groups, g = [], []
    for w in prog[:-1]:
        if (w >> 14) == 0 and g:
            groups.append(g); g = []
        g.append(w)
    groups.append(g)
    lanes = [[], []]
    for g in sorted(groups, key=len, reverse=True):     # greedy balance
        (lanes[0] if len(lanes[0]) <= len(lanes[1]) else lanes[1]).extend(g)
    return [l + [3 << 14] for l in lanes]

def write_progs(res):
    info = {}
    for name in ("hb_a", "hb_b", "hd_a", "hd_b", "dsd"):
        hq = res[name][0]
        interp = name.startswith("hb_")
        prog, starts = csd_program(hq, interp)
        if name == "dsd":                    # 352.8 kHz in, 256 MCLK per output: two lanes
            l0, l1 = split_lanes(prog)
            write_bin(f"{name}.prog", l0, 16)
            write_bin(f"{name}.prog2", l1, 16)
            info[name] = (len(l0), [0], len(l1))
        else:
            write_bin(f"{name}.prog", prog, 16)
            info[name] = (len(prog), starts, 0)
    return info

def check(res):
    for name in ("hb_a", "hb_b", "hd_a", "hd_b", "dsd"):
        hq, fs, fp, fst, gain = res[name]
        h = hq / (1 << CS) / gain
        fpass = np.linspace(0, fp, 400)
        fstop = np.linspace(fst, fs / 2, 4000)
        _, Hp = signal.freqz(h, worN=fpass / fs * 2 * np.pi)
        _, Hs = signal.freqz(h, worN=fstop / fs * 2 * np.pi)
        P = 20 * np.log10(np.abs(Hp))
        if name == "dsd":
            P = P + 20 * np.log10(cic_resp(fpass / fs))
        print(f"{name:5s} N={len(hq):3d} pass {P.min():+.4f}..{P.max():+.4f} dB "
              f"stop {20*np.log10(np.abs(Hs).max()):.1f} dB")
    hq = res["asrc"][0]
    print(f"asrc  table {len(hq)} entries (L={ASRC_L}, P={ASRC_P}) peak {hq.max()}")

if __name__ == "__main__":
    r = build()
    pi = write_progs(r)
    if "--check" in sys.argv:
        for n, (ln, st, l2) in pi.items():
            print(f"{n:5s} CSD program {ln} ops, phase starts {st}" + (f", lane 1 {l2} ops" if l2 else ""))
    if "--check" in sys.argv:
        check(r)
