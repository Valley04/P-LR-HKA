# -*- coding: utf-8 -*-
"""Diapositiva 21 - Limitaciones y trabajo futuro.
Franja vertical derecha: 3 tarjetas de trabajo futuro, paleta morada del deck."""

from PIL import Image, ImageDraw, ImageFont
import os

OUT = r"C:\Users\lrojas\Desktop\P-LR-HKA\Defensa"

PURPLE = "#3C0366"
ACCENT = "#6100AC"
LILAC2 = "#C9B6E4"
GRAY   = "#666666"

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

f_hdr = F(["impact.ttf", "segoeuib.ttf", "arialbd.ttf"], 54)
f_num = F(["impact.ttf", "segoeuib.ttf", "arialbd.ttf"], 68)
f_t   = F(["segoeuib.ttf", "arialbd.ttf"], 32)
f_s   = F(["segoeui.ttf", "arial.ttf"], 26)

CARDS = [
    ("01", "Tracking de actualizaciones", ["monitoreo detallado del", "estado de cada OTA", "en la flota"]),
    ("02", "Analítica predictiva", ["anticipar fallos", "mecánicos sobre el", "histórico de errores"]),
    ("03", "Aprovisionamiento masivo", ["alta de dispositivos", "a escala de flota"]),
]

W, H = 900, 1250
img = Image.new("RGB", (W, H), PURPLE)
d = ImageDraw.Draw(img)

d.text((W // 2, 95), "TRABAJO FUTURO", font=f_hdr, fill="white", anchor="mm")
d.line([(170, 150), (W - 170, 150)], fill=LILAC2, width=4)

y = 195
ch = 340
for num, titulo, lineas in CARDS:
    d.rounded_rectangle([50, y, W - 50, y + ch - 40], radius=20, fill="white")
    d.text((150, y + 80), num, font=f_num, fill=ACCENT, anchor="mm")
    d.line([(255, y + 35), (255, y + ch - 75)], fill=LILAC2, width=4)
    tx, ty = 285, y + 45
    d.text((tx, ty), titulo, font=f_t, fill=PURPLE)
    ty += 55
    for ln in lineas:
        d.text((tx, ty), ln, font=f_s, fill=GRAY)
        ty += 38
    y += ch

img.save(os.path.join(OUT, "diapositiva_21_futuro.png"), dpi=(200, 200))
print("OK: diapositiva_21_futuro.png")
