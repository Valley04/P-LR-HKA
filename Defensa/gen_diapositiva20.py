# -*- coding: utf-8 -*-
"""Diapositiva 20 - Aporte y beneficios para The Factory HKA.
Infografia vertical: 4 beneficios numerados, paleta morada del deck."""

from PIL import Image, ImageDraw, ImageFont
import os

OUT = r"C:\Users\lrojas\Desktop\P-LR-HKA\Defensa"

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

f_num  = F(["impact.ttf",  "segoeuib.ttf", "arialbd.ttf"], 74)
f_t    = F(["segoeuib.ttf", "arialbd.ttf"], 32)
f_s    = F(["segoeui.ttf",  "arial.ttf"],   25)

def center_text(d, cx, y, s, font, fill):
    d.text((cx, y), s, font=font, fill=fill, anchor="mm")

CARDS = [
    ("01", "Visibilidad centralizada",
     ["todo el parque en", "un solo panel"]),
    ("02", "Mantenimiento preventivo",
     ["bitácora de 90 días →", "anticipar fallos"]),
    ("03", "Ahorro operativo en red",
     ["−80,2 % de datos", "por trama"]),
    ("04", "Firmware sin visitas",
     ["gateway + impresora,", "desde la nube"]),
]

W, H = 900, 1250
img = Image.new("RGB", (W, H), PURPLE)
d = ImageDraw.Draw(img)

y = 55
for num, titulo, lineas in CARDS:
    ch = 290
    d.rounded_rectangle([50, y, W - 50, y + ch - 35], radius=20, fill="white")
    # numero grande
    center_text(d, 150, y + 85, num, f_num, ACCENT)
    # divisor
    d.line([(255, y + 35), (255, y + ch - 70)], fill=LILAC2, width=4)
    # titulo + lineas
    tx = 285
    ty = y + 45
    for i, word in enumerate(titulo.split("\n")):
        d.text((tx, ty), word, font=f_t, fill=PURPLE)
        ty += 40
    ty += 8
    for ln in lineas:
        d.text((tx, ty), ln, font=f_s, fill=GRAY)
        ty += 36
    y += ch

img.save(os.path.join(OUT, "diapositiva_20_beneficios.png"), dpi=(200, 200))
print("OK: diapositiva_20_beneficios.png")
