import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import matplotlib.lines as mlines

fig, ax = plt.subplots(figsize=(16, 10))
ax.set_xlim(0, 16)
ax.set_ylim(0, 10)
ax.set_aspect('equal')
ax.axis('off')
fig.patch.set_facecolor('#0d1117')
ax.set_facecolor('#0d1117')

def box(x, y, w, h, fc, ec, lw=1.5, radius=0.15):
    p = FancyBboxPatch((x, y), w, h,
                       boxstyle=f"round,pad=0,rounding_size={radius}",
                       facecolor=fc, edgecolor=ec, linewidth=lw, zorder=3)
    ax.add_patch(p)

def txt(x, y, s, fs=9, color='white', ha='center', va='center', bold=False, zorder=5):
    weight = 'bold' if bold else 'normal'
    ax.text(x, y, s, fontsize=fs, color=color, ha=ha, va=va,
            fontweight=weight, zorder=zorder,
            fontfamily='monospace')

def wire(x1, y1, x2, y2, color, lw=2.2, zorder=2, style='-'):
    ax.plot([x1, x2], [y1, y2], color=color, linewidth=lw,
            zorder=zorder, linestyle=style, solid_capstyle='round')

def pin_dot(x, y, color, zorder=6):
    ax.plot(x, y, 'o', color=color, markersize=6, zorder=zorder,
            markeredgecolor='white', markeredgewidth=0.5)

# ── Background title ────────────────────────────────────────────
txt(8, 9.55, 'Briter Wire Encoder  ←→  MCP2551  ←→  STM32 Nucleo-F746ZG',
    fs=12, color='#58a6ff', bold=True)
txt(8, 9.2, 'Wiring Diagram  |  Wire_encoder_ID project',
    fs=9, color='#8b949e')

# ═══════════════════════════════════════════════════════════════
# ENCODER  (left)
# ═══════════════════════════════════════════════════════════════
EX, EY, EW, EH = 0.3, 2.5, 2.8, 5.0
box(EX, EY, EW, EH, '#161b22', '#30363d', lw=2)

# encoder body top circle
circle = plt.Circle((EX + EW/2, EY + EH - 1.0), 0.85,
                     color='#21262d', ec='#58a6ff', lw=1.5, zorder=4)
ax.add_patch(circle)
txt(EX + EW/2, EY + EH - 1.0, '⊙', fs=18, color='#58a6ff')

txt(EX + EW/2, EY + EH - 2.2, 'BRITER', fs=10, color='#79c0ff', bold=True)
txt(EX + EW/2, EY + EH - 2.6, 'BRT38-C0M', fs=8, color='#8b949e')

# warning label
box(EX+0.2, EY+0.3, EW-0.4, 1.1, '#2d1f0e', '#ffa657', lw=1)
txt(EX + EW/2, EY + 0.88, '⚠', fs=11, color='#ffa657')
txt(EX + EW/2, EY + 0.52, 'Wire Encoder', fs=7.5, color='#ffa657')

txt(EX + EW/2, EY + EH + 0.2, 'Encoder', fs=10, color='#f0f6fc', bold=True)

# wire exit points (right side of encoder)
wire_y = {
    'red':    EY + 4.1,
    'black':  EY + 3.4,
    'green':  EY + 2.7,
    'white':  EY + 2.0,
    'yellow': EY + 1.3,
}
wire_colors = {
    'red':    '#f85149',
    'black':  '#8b949e',
    'green':  '#3fb950',
    'white':  '#e6edf3',
    'yellow': '#e3b341',
}
wire_labels = {
    'red':    'Улаан  — VCC 12~24V',
    'black':  'Хар    — GND',
    'green':  'Ногоон — CAN_H',
    'white':  'Цагаан — CAN_L',
    'yellow': 'Шар    — Функц (ХОЛБОХГҮЙ)',
}

EX_right = EX + EW
for k, wy in wire_y.items():
    pin_dot(EX_right, wy, wire_colors[k])

# ═══════════════════════════════════════════════════════════════
# MCP2551  (center)
# ═══════════════════════════════════════════════════════════════
MX, MY, MW, MH = 6.2, 3.2, 2.6, 4.2
box(MX, MY, MW, MH, '#161b22', '#30363d', lw=2)
txt(MX + MW/2, MY + MH - 0.45, 'MCP2551', fs=11, color='#79c0ff', bold=True)
txt(MX + MW/2, MY + MH - 0.85, 'CAN Transceiver', fs=8, color='#8b949e')

# IC body
box(MX+0.45, MY+0.8, MW-0.9, MH-1.3, '#21262d', '#444d56', lw=1)
txt(MX + MW/2, MY + MH/2 - 0.1, 'DIP-8', fs=9, color='#444d56')

# Left pins (input side)
left_pins = [
    ('TXD', 'pin1', MY + 3.3),
    ('VSS', 'pin2', MY + 2.7),
    ('VDD', 'pin3', MY + 2.1),
    ('RXD', 'pin4', MY + 1.5),
]
for name, pnum, py in left_pins:
    ax.plot([MX-0.35, MX], [py, py], color='#444d56', lw=1.5, zorder=2)
    pin_dot(MX, py, '#444d56')
    txt(MX-0.55, py, name, fs=7.5, color='#cdd9e5', ha='right')

# Right pins (output side)
right_pins = [
    ('Rs',   'pin8', MY + 3.3),
    ('CANH', 'pin7', MY + 2.7),
    ('CANL', 'pin6', MY + 2.1),
    ('Vref', 'pin5', MY + 1.5),
]
for name, pnum, py in right_pins:
    ax.plot([MX+MW, MX+MW+0.35], [py, py], color='#444d56', lw=1.5, zorder=2)
    pin_dot(MX+MW, py, '#444d56')
    txt(MX+MW+0.55, py, name, fs=7.5, color='#cdd9e5', ha='left')

MX_left  = MX
MX_right = MX + MW

# ═══════════════════════════════════════════════════════════════
# NUCLEO  (right)
# ═══════════════════════════════════════════════════════════════
NX, NY, NW, NH = 11.5, 1.5, 4.2, 7.0
box(NX, NY, NW, NH, '#161b22', '#30363d', lw=2)
txt(NX + NW/2, NY + NH - 0.45, 'STM32 Nucleo-F746ZG', fs=10, color='#79c0ff', bold=True)
txt(NX + NW/2, NY + NH - 0.85, 'Morpho Connectors', fs=8, color='#8b949e')

# Board outline inner
box(NX+0.3, NY+0.8, NW-0.6, NH-1.5, '#0d1117', '#21262d', lw=1)

# LEDs
for lx, lname, lcolor, ly in [
    (NX+0.7, 'LD1', '#3fb950', NY+5.0),
    (NX+1.2, 'LD2', '#58a6ff', NY+5.0),
    (NX+1.7, 'LD3', '#f85149', NY+5.0),
]:
    circle2 = plt.Circle((lx, ly), 0.18, color=lcolor, ec='white', lw=0.5, zorder=5)
    ax.add_patch(circle2)
    txt(lx, ly-0.38, lname, fs=6.5, color=lcolor)

txt(NX+1.2, NY+4.55, 'LEDs', fs=7.5, color='#8b949e')

# Nucleo pin labels
nucleo_pins = [
    ('PA12 CAN_TX', 'CN12-12', NY + 3.8, '#d2a8ff'),
    ('PA11 CAN_RX', 'CN12-10', NY + 3.1, '#79c0ff'),
    ('+5V',         'CN11-18', NY + 2.4, '#ffa657'),
    ('GND',         'CN12-7',  NY + 1.7, '#8b949e'),
]
NX_left = NX
for pname, cname, py, pc in nucleo_pins:
    pin_dot(NX_left, py, pc)
    txt(NX_left + 0.15, py + 0.18, cname, fs=7, color='#444d56', ha='left')
    txt(NX_left + 0.15, py - 0.18, pname, fs=8, color=pc, ha='left', bold=True)

# ═══════════════════════════════════════════════════════════════
# WIRES: Encoder → MCP2551
# ═══════════════════════════════════════════════════════════════

# Red — power (goes down to external PSU label)
wire(EX_right, wire_y['red'], 5.0, wire_y['red'], wire_colors['red'], lw=2.5)
wire(5.0, wire_y['red'], 5.0, 8.5, wire_colors['red'], lw=2.5)
box(4.2, 8.6, 1.6, 0.55, '#2d1212', '#f85149', lw=1.5, radius=0.08)
txt(5.0, 8.88, '12~24V PSU', fs=8, color='#f85149', bold=True)

# Black → VSS pin2 (MY+2.7) via bottom bus
wire(EX_right, wire_y['black'], 4.5, wire_y['black'], wire_colors['black'], lw=2.5)
wire(4.5, wire_y['black'], 4.5, MY + 2.7, wire_colors['black'], lw=2.5)
wire(4.5, MY + 2.7, MX_left, MY + 2.7, wire_colors['black'], lw=2.5)

# Green → CANH (right side pin7 MY+2.7)
wire(EX_right, wire_y['green'], MX_right + 0.35, wire_y['green'], wire_colors['green'], lw=2.5)
# bend to CANH
wire(MX_right + 0.35, wire_y['green'], MX_right + 0.35, MY + 2.7, wire_colors['green'], lw=2.5)

# White → CANL (right side pin6 MY+2.1)
wire(EX_right, wire_y['white'], MX_right + 0.6, wire_y['white'], wire_colors['white'], lw=2.5)
wire(MX_right + 0.6, wire_y['white'], MX_right + 0.6, MY + 2.1, wire_colors['white'], lw=2.5)

# Yellow — dashed, floating
wire(EX_right, wire_y['yellow'], EX_right + 0.7, wire_y['yellow'],
     wire_colors['yellow'], lw=2.0, style='--')
txt(EX_right + 1.1, wire_y['yellow'], '✕ ХОЛБОХГҮЙ', fs=7.5, color='#e3b341')

# ═══════════════════════════════════════════════════════════════
# WIRES: MCP2551 → Nucleo
# ═══════════════════════════════════════════════════════════════

# TXD (pin1, MY+3.3) ← PA12 (NY+3.8)  purple
wire(MX_left, MY + 3.3, NX_left, NY + 3.8, '#d2a8ff', lw=2.5)

# RXD (pin4, MY+1.5) → voltage divider → PA11 (NY+3.1)
# divider node at x=9.5
DIV_X = 9.5
DIV_Y = MY + 1.5

wire(MX_right + 0.35, MY + 1.5, DIV_X, DIV_Y, '#79c0ff', lw=2.5)

# 10k from divider to PA11
wire(DIV_X, DIV_Y, NX_left, NY + 3.1, '#79c0ff', lw=2.5)

# 20k from divider to GND
wire(DIV_X, DIV_Y, DIV_X, DIV_Y - 0.8, '#8b949e', lw=1.8)
# resistor symbol
box(DIV_X - 0.15, DIV_Y - 1.05, 0.3, 0.3, '#21262d', '#8b949e', lw=1.2, radius=0.04)
wire(DIV_X, DIV_Y - 1.05, DIV_X, DIV_Y - 1.3, '#8b949e', lw=1.8)
pin_dot(DIV_X, DIV_Y - 1.3, '#8b949e')
txt(DIV_X + 0.25, DIV_Y - 0.9, '20kΩ', fs=7.5, color='#e3b341', ha='left')

# 10k resistor symbol on wire between MCP RXD and divider
R10_X = (MX_right + 0.35 + DIV_X) / 2
box(R10_X - 0.18, DIV_Y - 0.15, 0.36, 0.3, '#21262d', '#79c0ff', lw=1.2, radius=0.04)
txt(R10_X, DIV_Y + 0.25, '10kΩ', fs=7.5, color='#e3b341')

# +5V (pin3, MY+2.1) ← CN11-18 (NY+2.4)
wire(MX_left, MY + 2.1, NX_left, NY + 2.4, '#ffa657', lw=2.5)

# GND (pin2, MY+2.7) connects to GND rail → CN12-7 (NY+1.7)
wire(4.5, MY + 2.7, 4.5, NY + 1.7 - 0.3, '#8b949e', lw=2.0)
wire(4.5, NY + 1.7 - 0.3, NX_left, NY + 1.7, '#8b949e', lw=2.0)

# GND bottom rail join (divider 20k to gnd bus)
wire(DIV_X, DIV_Y - 1.3, DIV_X, NY + 1.7 - 0.05, '#8b949e', lw=1.5)
# horizontal join
wire(DIV_X, NY + 1.7 - 0.05, 4.5, NY + 1.7 - 0.05, '#8b949e', lw=1.2, style=':')

# Rs pin8 (MY+3.3) → GND
wire(MX_right + 0.35, MY + 3.3, MX_right + 0.35, NY + 1.4, '#8b949e', lw=1.8)
wire(MX_right + 0.35, NY + 1.4, 4.5, NY + 1.4, '#8b949e', lw=1.5, style=':')
txt(MX_right + 0.7, MY + 3.3, 'GND', fs=7, color='#8b949e')

# ═══════════════════════════════════════════════════════════════
# Legend
# ═══════════════════════════════════════════════════════════════
LX, LY = 0.3, 0.05
legend_items = [
    ('#f85149', 'Улаан — VCC 12~24V'),
    ('#8b949e', 'Хар — GND'),
    ('#3fb950', 'Ногоон — CAN_H'),
    ('#e6edf3', 'Цагаан — CAN_L'),
    ('#e3b341', 'Шар — Функц (ХОЛБОХГҮЙ)'),
    ('#d2a8ff', 'PA12 CAN_TX'),
    ('#79c0ff', 'PA11 CAN_RX + хуваагч'),
    ('#ffa657', '+5V MCP2551'),
]
for i, (c, label) in enumerate(legend_items):
    xi = LX + (i % 4) * 4.0
    yi = LY + 0.45 - (i // 4) * 0.38
    ax.plot([xi, xi + 0.4], [yi, yi], color=c, lw=2.5, zorder=5)
    txt(xi + 0.5, yi, label, fs=7.5, color='#cdd9e5', ha='left')

plt.tight_layout(pad=0.2)
plt.savefig('/home/dosjan/Desktop/forklift 2/FORKLIFT/Wire_encoder_ID/wiring.png',
            dpi=150, bbox_inches='tight', facecolor='#0d1117')
print("wiring.png saved")
