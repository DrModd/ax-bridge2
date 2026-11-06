#!/usr/bin/env python3
# ax-bridge2 block diagram -> SVG
import sys
W, H = 1700, 1140
o = []
def a(s): o.append(s)
def T(x, y, s, cls="", anc="start"):
    a(f'<text x="{x}" y="{y}" class="{cls}" text-anchor="{anc}">{s}</text>')
def box(x, y, w, h, title, lines=(), cls="mc", mod=""):
    a(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" class="{cls}"/>')
    ty = y + 22
    T(x + w / 2, ty, title, "t", "middle")
    if mod:
        ty += 16; T(x + w / 2, ty, mod, "m", "middle")
    for i, s in enumerate(lines):
        T(x + w / 2, ty + 18 + i * 16, s, "v", "middle")
def pl(pts, cls="w", arrow=True, mk="ar"):
    p = " ".join(f"{x},{y}" for x, y in pts)
    a(f'<polyline points="{p}" class="{cls}"' + (f' marker-end="url(#{mk})"' if arrow else "") + '/>')
def dot(x, y): a(f'<circle cx="{x}" cy="{y}" r="4" class="d"/>')

a(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
a('''<defs>
<marker id="ar" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" markerHeight="8" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="#222"/></marker>
<marker id="arb" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" markerHeight="8" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="#2e7d32"/></marker>
<marker id="arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" markerHeight="8" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="#b00020"/></marker>
</defs>
<style>
text{font-family:"DejaVu Sans",Arial,sans-serif;font-size:13px;fill:#111}
.h{font-size:21px;font-weight:bold}.g{fill:#555}.t{font-weight:bold;font-size:14px}
.m{fill:#1a5fb4;font-size:12px;font-family:"DejaVu Sans Mono",monospace}.v{fill:#333;font-size:12px}
.lab{font-size:12px;fill:#333}.lb{font-size:12px;fill:#1a5fb4}.lg{font-size:12px;fill:#2e7d32}.lr{font-size:12px;fill:#b00020}
.mc{fill:#eef4fb;stroke:#2b5797;stroke-width:2}.bc{fill:#fff1e0;stroke:#c66a00;stroke-width:2}
.sc{fill:#eef8ee;stroke:#2e7d32;stroke-width:2}.ex{fill:#f3f3f3;stroke:#555;stroke-width:2}
.new{fill:#fff7d6;stroke:#b8860b;stroke-width:2}
.w{fill:none;stroke:#222;stroke-width:2}.wg{fill:none;stroke:#2e7d32;stroke-width:1.6;stroke-dasharray:6 4}
.wr{fill:none;stroke:#b00020;stroke-width:2.4}
.d{fill:#222}.zone{fill:none;stroke-width:2;stroke-dasharray:10 6}
</style>''')
a(f'<rect width="{W}" height="{H}" fill="#fff"/>')
T(20, 34, "ax-bridge2 — блок-схема: два генератора, синхронный Фокс, обслуживание на 50 МГц", "h")
T(20, 56, "Три тактовых домена: BCLK входа (оранжевый), 50 МГц — обслуживание (зелёный), MCLK 45,1584 / 49,152 МГц — звук (синий). Красное — MCLK на плате.", "g")

# ---- external clock source ----
box(20, 90, 200, 120, "генераторы", ["45,1584 МГц (сетка 44,1)", "49,152 МГц (сетка 48)", "OE / мультиплексор"], "ex", "на плате D2")
T(232, 130, "MCLK", "lr")
pl([(220, 150), (300, 150), (300, 112), (1660, 112)], "wr", arrow=False)
dot(300, 150)
# MCLK to Fox
pl([(300, 150), (300, 205), (180, 205), (180, 262)], "wr", mk="arr")
# MCLK to core
dot(760, 112)
pl([(760, 112), (760, 138)], "wr", mk="arr")
T(768, 130, "MCLK → C11 (GCLKT)", "lr")

# ---- inputs ----
box(20, 262, 200, 66, "0  Luckfox (Фокс)", ["PCM / DoP / DSD, клок от", "нашего MCLK — синхронно"], "ex")
box(20, 342, 200, 52, "2  Bluetooth", ["I2S PCM"], "ex")
box(20, 408, 200, 52, "3  HDMI I2S", ["PCM / DoP / DSD"], "ex")
box(20, 474, 200, 52, "4  фонокорректор", ["AK5572, I2S 192 кГц"], "ex")
box(20, 560, 200, 84, "1  S/PDIF × 4", ["коакс, оптика,", "AES/EBU, HDMI ARC"], "ex", "логический уровень")
T(20, 666, "BCLK — на GCLKT", "lab")

# input BCLK domain
a('<rect x="250" y="240" width="270" height="310" rx="10" class="zone" stroke="#c66a00"/>')
T(262, 258, "домен BCLK входа", "lab")
for y in (295, 368, 434, 500):
    pl([(220, y), (270, y)])
box(270, 280, 100, 240, "in_mux", ["DCS / LUT", "4 BCLK", "", "флаги DSD", "(Фокс, HDMI)"], "bc")
pl([(370, 400), (395, 400)])
box(395, 350, 110, 100, "bclk_rx", ["I2S / DSD", "приём"], "bc")
pl([(505, 400), (545, 400)])

# FIFO
box(545, 360, 120, 80, "afifo", ["BCLK → MCLK"], "mc")
# S/PDIF path in sys domain
# sys domain zone
a('<rect x="250" y="560" width="680" height="560" rx="10" class="zone" stroke="#2e7d32"/>')
T(262, 578, "домен 50 МГц — обслуживание (не останавливается при смене генератора)", "lg")
box(20, 690, 200, 70, "генератор 50 МГц", ["на модуле, E2 (GCLKT)"], "ex")
pl([(220, 725), (270, 725)], "wg", mk="arb")
pl([(220, 602), (270, 602)])
box(270, 590, 150, 70, "spdif_rx", ["98 Мвыб/с, сетка", "по длине кадра"], "sc")
pl([(420, 625), (545, 625)])
box(545, 590, 120, 70, "afifo", ["50 МГц → MCLK"], "mc")
pl([(665, 625), (720, 625), (720, 470), (790, 470)])
pl([(665, 400), (720, 400), (720, 470)], arrow=False)
dot(720, 470)

box(270, 690, 190, 120, "частотомер", ["отсчёты входа за 5,2 мс", "→ частота и сетка", "44,1 / 48 кГц"], "sc")
pl([(450, 450), (450, 545), (330, 545), (330, 690)], "wg", mk="arb")
T(338, 560, "", "lg")
pl([(345, 660), (345, 690)], "wg", mk="arb")
box(500, 700, 410, 190, "менеджер клоков и входов", [
    "вход 0 (Фокс): mclk_sel = MCLK_SEL Фокса",
    "другие входы: mclk_sel — по частотомеру",
    "смена: стоп → флаг STM32 → подтверждение",
    "(или 300 мс) → сброс тракта → mclk_sel",
    "→ 20 мс на генератор → снова захват",
    "переключение входов и линий S/PDIF,",
    "задержки и сторожа"], "new")
pl([(460, 760), (500, 760)], "wg", mk="arb")
# fox_fsel
pl([(120, 328), (120, 336), (232, 336), (232, 860), (500, 860)], "wg", mk="arb")
T(270, 852, "MCLK_SEL Фокса", "lg")
# mclk_sel output to oscillators
pl([(560, 700), (560, 682), (242, 682), (242, 228), (120, 228), (120, 212)], "wg", mk="arb")
T(128, 224, "mclk_sel", "lg")

box(270, 930, 250, 150, "I2C 0x5A, регистры", ["вход, MUTE, ATT, DSD, EQ,", "S/PDIF 0x10–0x13,", "клоки 0x14–0x16, VERSION 0x20"], "sc")
a('<rect x="20" y="960" width="200" height="60" rx="6" class="ex"/>')
T(120, 986, "STM32H523", "t", "middle"); T(120, 1004, "I2C3", "v", "middle")
pl([(220, 990), (270, 990)], "w", arrow=False)
pl([(520, 1000), (700, 1000), (700, 890)], "wg", mk="arb")
T(540, 1020, "управление тракту — уровни через синхронизаторы", "lg")
pl([(520, 1040), (950, 1040), (950, 692)], "wg", mk="arb")

# ---- MCLK core ----
a('<rect x="760" y="140" width="920" height="550" rx="10" class="zone" stroke="#2b5797"/>')
T(772, 160, "домен MCLK — звуковой тракт (core.v), сбрасывается на время смены генератора", "lb")
box(790, 430, 120, 80, "приём", ["отсчётов,", "ATT 0…6 дБ"], "mc")
pl([(910, 470), (945, 470)])
# filters
fx = 945
rows = [("PCM ×4", ["hb_a ×2", "hb_b ×2"], 200), ("PCM ×2", ["hb_b ×2"], 260),
        ("PCM ×1", [], 320), ("PCM ÷2", ["hd_b ÷2"], 380), ("PCM ÷4", ["hd_a ÷2", "hd_b ÷2"], 440),
        ("DSD/DoP", ["CIC5", "dsd ÷2"], 510)]
line_x = 945
pl([(line_x, 200), (line_x, 510)], arrow=False)
dot(line_x, 470)
mx = 1265
for lab, st, y in rows:
    T(line_x + 6, y - 6, lab, "lb")
    x = line_x + 70
    pl([(line_x, y), (x, y)])
    for name in st:
        a(f'<rect x="{x}" y="{y-16}" width="80" height="32" rx="6" class="new"/>')
        T(x + 40, y + 5, name, "v", "middle")
        x += 80
        if name != st[-1]:
            pl([(x, y), (x + 10, y)]); x += 10
    pl([(x, y), (mx, y)])
pl([(mx, 200), (mx, 510)], arrow=False)
T(line_x + 6, 545, "жёлтые — КИХ на сдвигах и сложениях (CSD), без умножителей, бит в бит", "lab")
T(line_x + 6, 562, "с прежними; DSD-фильтр — две дорожки параллельно", "lab")
dot(mx, 355)
T(mx - 4, 590, "", "lab")
# split sync / asrc
pl([(mx, 355), (1300, 355)], arrow=False)
dot(1300, 355)
pl([(1300, 355), (1300, 260), (1330, 260)])
pl([(1300, 355), (1300, 450), (1330, 450)])
box(1330, 210, 170, 100, "буфер 16 отсч.", ["Фокс синхронно:", "без ASRC"], "new")
box(1330, 400, 170, 100, "ASRC", ["остальные входы:", "отношение ≈1"], "mc")
T(1338, 340, "176,4 / 192 кГц", "lab")
pl([(1500, 260), (1530, 260), (1530, 355)], arrow=False)
pl([(1500, 450), (1530, 450), (1530, 355)], arrow=False)
dot(1530, 355)
pl([(1530, 355), (1550, 355), (1550, 380), (1555, 380)], arrow=False)
box(1555, 340, 110, 80, "fade", ["плавный", "старт/стоп"], "mc")
pl([(1610, 420), (1610, 460)])
box(1555, 460, 110, 90, "EQ", ["10 биквадов", "на канал"], "mc")
pl([(1610, 550), (1610, 590)])
box(1555, 590, 110, 90, "I2S TX", ["MCLK/256:", "176,4 / 192"], "mc")
pl([(1600, 680), (1600, 790)])
a('<rect x="1500" y="790" width="190" height="80" rx="6" class="ex"/>')
T(1595, 820, "AX5689", "t", "middle"); T(1595, 840, "MCLK — тот же генератор", "v", "middle")
T(1595, 856, "STM32: переинициализация", "v", "middle")
pl([(1660, 112), (1660, 790)], "wr", mk="arr")
T(1595, 890, "I2S 32 бит, 176,4 / 192 кГц", "lab", "middle")

# legend
lx, ly = 1000, 820
T(lx, ly, "Обозначения", "t")
for i, (c, s) in enumerate((("bc", "домен BCLK входа"), ("sc", "домен 50 МГц"), ("mc", "домен MCLK"), ("new", "новое в ax-bridge2"), ("ex", "вне FPGA"))):
    a(f'<rect x="{lx}" y="{ly+14+i*26}" width="26" height="18" rx="4" class="{c}"/>'); T(lx + 34, ly + 28 + i * 26, s, "v")
a(f'<line x1="{lx+250}" y1="{ly+24}" x2="{lx+290}" y2="{ly+24}" class="wr"/>'); T(lx + 298, ly + 28, "MCLK на плате", "v")
a(f'<line x1="{lx+250}" y1="{ly+50}" x2="{lx+290}" y2="{ly+50}" class="wg"/>'); T(lx + 298, ly + 54, "управление", "v")
a('</svg>')
open(sys.argv[1], 'w').write('\n'.join(o))
