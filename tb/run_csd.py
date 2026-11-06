#!/usr/bin/env python3
"""fir_csd (shift-and-add) vs fir_stage (multiplier): bit-exact for all
five filters at the highest input rate each one sees (MCLK clocks per
input at 45.1584 / 49.152 MHz)."""
import os, subprocess, sys
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
B = os.path.join(ROOT, "build")
os.makedirs(B, exist_ok=True)

def plen(f):
    return sum(1 for l in open(os.path.join(ROOT, "coef", f)) if l.strip())

CASES = [  # name, N, interp, AW, CAW, PW, P1, PER, lanes
    ("hb_a", 175, 1, 7, 8, 9, None, 1024, 1),
    ("hb_b", 91, 1, 6, 7, 8, None, 512, 1),
    ("hd_a", 23, 0, 5, 5, 6, None, 64, 1),
    ("hd_b", 31, 0, 5, 5, 7, None, 128, 1),
    ("dsd", 95, 0, 7, 7, 8, None, 128, 2),
]
ok = True
for name, n, interp, aw, caw, pw, _, per, lanes in CASES:
    pl = plen(f"{name}.prog")
    # phase-1 start = position after the first END
    words = [int(l, 2) for l in open(os.path.join(ROOT, "coef", f"{name}.prog")) if l.strip()]
    p1 = next(i for i, w in enumerate(words) if (w >> 14) == 3) + 1 if interp else 0
    params = [f"-Ptb_csd.N={n}", f"-Ptb_csd.INTERP={interp}", f"-Ptb_csd.AW={aw}",
              f"-Ptb_csd.CAW={caw}", f"-Ptb_csd.PW={pw}", f"-Ptb_csd.PLEN={pl}", f"-Ptb_csd.P1={p1}",
              f"-Ptb_csd.PER={per}", f'-Ptb_csd.FILE="coef/{name}.bin"', f'-Ptb_csd.PFILE="coef/{name}.prog"',
              f"-Ptb_csd.LANES={lanes}"]
    if lanes > 1:
        params += [f"-Ptb_csd.PLEN2={plen(name + '.prog2')}", f'-Ptb_csd.PFILE2="coef/{name}.prog2"']
    exe = os.path.join(B, f"csd_{name}.vvp")
    r = subprocess.run(["iverilog", "-g2005", "-o", exe, "-s", "tb_csd"] + params +
                       ["tb/tb_csd.v", "rtl/fir_stage.v", "rtl/fir_csd.v", "rtl/mem.v"],
                       cwd=ROOT, capture_output=True, text=True)
    if r.returncode:
        print(r.stdout, r.stderr); ok = False; continue
    r = subprocess.run(["vvp", "-n", exe], cwd=ROOT, capture_output=True, text=True)
    line = [l for l in r.stdout.splitlines() if l.startswith(("PASS", "FAIL"))]
    print(line[0] if line else r.stdout)
    ok &= bool(line) and line[0].startswith("PASS")
print("PASS csd" if ok else "FAIL csd")
sys.exit(0 if ok else 1)
