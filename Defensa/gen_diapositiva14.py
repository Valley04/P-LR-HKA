# -*- coding: utf-8 -*-
"""Diapositiva 14 - Resultado 4: resiliencia.
Compuesto: captura real del Log Capture + flujo de recuperacion +
tabla de 4 escenarios de falla. Paleta monocromo morada del deck."""

from PIL import Image, ImageDraw, ImageFont
import math, os

OUT  = r"C:\Users\lrojas\Desktop\P-LR-HKA\Defensa"
SHOT = r"C:\Users\lrojas\Desktop\P-LR-HKA\Avances_Pasantias\Img\image35.png"

PURPLE  = "#3C0366"
ACCENT  = "#6100AC"
LILAC   = "#EFE8F8"
LILAC2  = "#C9B6E4"
BLACK   = "#111111"
GRAY    = "#666666"

FD = r"C:\Windows\Fonts"
def F(names, size):
    if isinstance(names, str):
        names = [names]
    for n in names:
        try:
            return ImageFont.truetype(os.path.join(FD, n), size)
        except OSError:
            continue
    return ImageFont.load_default()

f_h   = F(["segoeuib.ttf", "arialbd.ttf"], 40)
f_h2  = F(["segoeuib.ttf", "arialbd.ttf"], 32)
f_b   = F(["segoeui.ttf",  "arial.ttf"],   28)
f_s   = F(["segoeui.ttf",  "arial.ttf"],   25)
f_si  = F(["segoeuii.ttf", "ariali.ttf", "segoeui.ttf"], 25)
f_cap = F(["segoeuib.ttf", "arialbd.ttf"], 26)

def center_text(d, cx, y, s, font, fill):
    d.text((cx, y), s, font=font, fill=fill, anchor="mm")

def arrow(d, x1, y1, x2, y2, color=PURPLE, width=6, head=15):
    d.line([(x1, y1), (x2, y2)], fill=color, width=width)
    ang = math.atan2(y2 - y1, x2 - x1)
    d.polygon([(x2, y2),
               (x2 - head * math.cos(ang - 0.40), y2 - head * math.sin(ang - 0.40)),
               (x2 - head * math.cos(ang + 0.40), y2 - head * math.sin(ang + 0.40))],
              fill=color)

W, H = 1750, 980
img = Image.new("RGB", (W, H), "white")
d = ImageDraw.Draw(img)

# ================= IZQUIERDA: captura real Log Capture =================
shot = Image.open(SHOT).convert("RGB")
shot = shot.crop((0, 46, 1366, 688))          # quitar barra del navegador
sw = 700
sh = int(shot.height * sw / shot.width)
shot = shot.resize((sw, sh), Image.LANCZOS)

d.rounded_rectangle([30, 30, 750, 950], radius=20, fill=LILAC,
                    outline=PURPLE, width=5)
center_text(d, 390, 70, "Log Capture — historial persistido", f_h2, PURPLE)
img.paste(shot, (40, 110))
d.rounded_rectangle([40, 110, 40 + sw, 110 + sh], radius=8, outline=PURPLE,
                    width=4)

# anotacion bajo la captura (centrada en el espacio restante)
ay = 110 + sh + (900 - sh - 110 - 150) // 2
d.rounded_rectangle([60, ay, 720, ay + 150], radius=14, fill="white",
                    outline=ACCENT, width=4)
d.text((85, ay + 22), '«Desconexión Abrupta»', font=f_cap, fill=ACCENT)
d.text((85, ay + 62), "el worker intercepta la alerta offline y congela",
       font=f_s, fill="#333333")
d.text((85, ay + 96), "el último estado válido — sin alterar el histórico",
       font=f_s, fill="#333333")
center_text(d, 390, 915, "cada fila = evento persistido con su trama MQTT",
            f_si, GRAY)

# ================= DERECHA: flujo de recuperacion =================
X0 = 790
center_text(d, (X0 + W) / 2, 70, "Ciclo de recuperación ante pérdida de enlace",
            f_h2, PURPLE)

steps = [
    "enlace cae",
    "Keep-Alive\nexpira",
    "broker publica\nLWT «offline»",
    "registra «Desconexión\nAbrupta» + congela\núltimo estado",
    "túnel\nrestaurado",
]
# serpiente: 3 arriba, 2 abajo
bw, bh = 270, 120
ys_top, ys_bot = 140, 340
xs = [X0 + 10, X0 + 330, X0 + 650]
pos = [(xs[0], ys_top), (xs[1], ys_top), (xs[2], ys_top),
       (xs[1] + 165, ys_bot), (xs[0], ys_bot)]
order = [0, 1, 2, 3, 4]
for i, (sx, syy) in enumerate(pos):
    fill = PURPLE if i == 3 else "white"
    fg = "white" if i == 3 else PURPLE
    d.rounded_rectangle([sx, syy, sx + bw, syy + bh], radius=16,
                        fill=fill, outline=PURPLE, width=4)
    lines = steps[i].split("\n")
    for j, ln in enumerate(lines):
        center_text(d, sx + bw / 2, syy + bh / 2 - (len(lines) - 1) * 17 + j * 34,
                    ln, f_s, fg)
# flechas
arrow(d, xs[0] + bw + 8, ys_top + 60, xs[1] - 8, ys_top + 60)
arrow(d, xs[1] + bw + 8, ys_top + 60, xs[2] - 8, ys_top + 60)
arrow(d, xs[2] + bw / 2, ys_top + bh + 8, xs[1] + 165 + bw / 2, ys_bot - 8)
arrow(d, xs[1] + 165 - 8, ys_bot + 60, xs[0] + bw + 8, ys_bot + 60)

# ================= DERECHA: 4 escenarios =================
center_text(d, (X0 + W) / 2, 545, "Cuatro escenarios de falla evaluados",
            f_h2, PURPLE)

scen = [("corte de\nenergía"), ("micro-corte\nWi-Fi"), ("caída del\nbroker"),
        ("desconexión del\nperiférico")]
cx0 = X0 + 30
cw, ch, gap = 210, 160, 18
for i, s in enumerate(scen):
    x = cx0 + i * (cw + gap)
    d.rounded_rectangle([x, 575, x + cw, 575 + ch], radius=14,
                        fill=LILAC, outline=PURPLE, width=4)
    # circulo con check
    ccx, ccy = x + cw / 2, 575 + 48
    d.ellipse([ccx - 26, ccy - 26, ccx + 26, ccy + 26], fill=PURPLE)
    d.line([(ccx - 12, ccy), (ccx - 3, ccy + 10)], fill="white", width=5)
    d.line([(ccx - 3, ccy + 10), (ccx + 14, ccy - 11)], fill="white", width=5)
    for j, ln in enumerate(s.split("\n")):
        center_text(d, x + cw / 2, 575 + 102 + j * 30, ln, f_s, "#333333")

# resultado
d.rounded_rectangle([X0 + 10, 770, W - 30, 930], radius=20, fill=PURPLE)
center_text(d, (X0 + 10 + W - 30) / 2, 815, "pérdida de datos: NULA", f_h,
            "white")
center_text(d, (X0 + 10 + W - 30) / 2, 872,
            "el gateway es observador — la fuente de verdad vive en el periférico",
            f_si, "#D9C8EE")

img.save(os.path.join(OUT, "diapositiva_14_resiliencia.png"), dpi=(200, 200))
print("OK: diapositiva_14_resiliencia.png")
