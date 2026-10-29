# -*- coding: utf-8 -*-
"""Diapositiva 13 - Resultado 3: eficiencia de red.
Grafico vertical monocromatico (paleta del deck): HTTP lila vs MQTT morado."""

from PIL import Image, ImageDraw, ImageFont
import os

OUT = r"C:\Users\lrojas\Desktop\P-LR-HKA\Defensa"

PURPLE   = "#3C0366"   # morado profundo del deck
ACCENT   = "#6100AC"   # violeta nodos
LILAC    = "#C9B6E4"   # lila para HTTP (descartada)
BLACK    = "#111111"
GRAY     = "#666666"

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

f_h   = F(["segoeuib.ttf", "arialbd.ttf"], 46)
f_si  = F(["segoeuii.ttf", "ariali.ttf", "segoeui.ttf"], 28)
f_lab = F(["segoeuib.ttf", "arialbd.ttf"], 40)
f_b   = F(["segoeui.ttf",  "arial.ttf"],   34)
f_pct = F(["segoeuib.ttf", "arialbd.ttf"], 46)
f_big = F(["impact.ttf",   "segoeuib.ttf", "arialbd.ttf"], 120)

def center_text(d, cx, y, s, font, fill):
    d.text((cx, y), s, font=font, fill=fill, anchor="mm")

ROWS = [
    ("CABECERA",          450, 12,  "−97,33 %"),
    ("CUERPO DEL MENSAJE", 328, 142, "−56,70 %"),
    ("TOTAL POR TRAMA",    778, 154, "−80,2 %"),
]

W, H = 1150, 1500
img = Image.new("RGB", (W, H), "white")
d = ImageDraw.Draw(img)

center_text(d, W / 2, 60,  "Bytes por ciclo de telemetría", f_h, PURPLE)
center_text(d, W / 2, 112, "HTTP REST  vs  MQTT v5.0", f_si, GRAY)

# leyenda
ly = 165
d.rounded_rectangle([330, ly, 380, ly + 26], radius=6, fill=LILAC)
d.text((392, ly - 3), "HTTP", font=f_b, fill=GRAY)
d.rounded_rectangle([560, ly, 610, ly + 26], radius=6, fill=PURPLE)
d.text((622, ly - 3), "MQTT", font=f_b, fill=GRAY)

BAR_X = 60
BAR_MAX = 700
scale = BAR_MAX / 778
BH = 72                      # alto de barra

y = 250
for label, http, mqtt, pct in ROWS:
    center_text(d, W / 2, y, label, f_lab, BLACK)
    y += 62
    # barra HTTP
    w_http = http * scale
    d.rounded_rectangle([BAR_X, y, BAR_X + w_http, y + BH],
                        radius=10, fill=LILAC)
    d.text((BAR_X + w_http + 18, y + 14), f"{http} B", font=f_b, fill=GRAY)
    y += BH + 16
    # barra MQTT
    w_mqtt = max(mqtt * scale, 56)
    d.rounded_rectangle([BAR_X, y, BAR_X + w_mqtt, y + BH],
                        radius=10, fill=PURPLE)
    d.text((BAR_X + w_mqtt + 18, y + 14), f"{mqtt} B", font=f_b, fill=PURPLE)
    # porcentaje a la derecha
    d.text((W - 60, y + 10), pct, font=f_pct, fill=ACCENT, anchor="ra")
    y += BH + 70

# banda final con el numero estrella
d.rounded_rectangle([60, y + 10, W - 60, y + 190], radius=24, fill=PURPLE)
center_text(d, W / 2, y + 80, "−80,2 %", f_big, "white")
center_text(d, W / 2, y + 152,
            "menos consumo por trama → ahorro real del contribuyente",
            f_si, "#D9C8EE")

img.save(os.path.join(OUT, "diapositiva_13_vertical.png"), dpi=(200, 200))
print("OK: diapositiva_13_vertical.png")
